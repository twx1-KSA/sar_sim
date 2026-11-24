#!/usr/bin/env python3
"""SAR mission coordinator: turns detections into targets and sends the confirmation drone.

- Listens to /sar/detections (JSON from every drone's thermal_detector).
- Merges sightings within `merge_radius_m` into one target (T1, T2, ...), even across drones.
- For new targets, launches the confirmation drone (PX4 instance `confirm_drone`):
    takeoff -> terrain-following transit at `transit_agl` -> descend to `confirm_agl` over the
    target -> hover `loiter_s` seconds -> climb back -> next target or home -> land.
  All targets waiting at launch time are visited in one sortie (nearest first).
- The target is "confirmed" when the confirmation drone's own detector reports a sighting within
  `confirm_radius_m` of it; otherwise "not_found" after the hover. A down-camera RGB close-up is
  saved while hovering.
- Publishes every target status change on /sar/targets (JSON).
- Watches the search drones (`primaries`). If one is lost (no heartbeat), leaves mission mode
  (failsafe RTL/LAND) or disarms before finishing, the backup drone (`backup_drone`) flies from
  its pad to the last waypoint the failed drone reached and completes the remaining sweeps.
  Fleet events are published on /sar/fleet (JSON).

Run (simulation, sar_bringup with the confirmation drone included, e.g. drones:=0,1,3):
    python3 ~/sar_sim/ros2/sar_coordinator.py
    python3 ~/sar_sim/ros2/sar_coordinator.py --ros-args -p confirm_drone:=3
"""

import json
import math
import os
import threading
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
import rclpy.parameter
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sar_geo import GeoRef, Terrain
from sar_mission import MAV, Vehicle, plan_route


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class Target:
    def __init__(self, num, east, north, alt, det):
        self.id = f"T{num}"
        self.east, self.north, self.alt = east, north, alt
        self.status = "unconfirmed"  # -> confirming -> confirmed | not_found
        self.reports = [det["detection_id"]]
        self.drones = {det["drone_id"]}
        self.first_detected = utc_now()
        self.confirmed_at = None
        self.confirm_detection = None
        self.thermal_image = det.get("image_ref", "")
        self.rgb_closeup = ""
        self.peak_c = det.get("peak_temperature_c")

    def to_json(self, geo):
        lat, lon = geo.geo_from_enu(self.east, self.north)
        return json.dumps({
            "target_id": self.id,
            "status": self.status,
            "latitude": round(lat, 7),
            "longitude": round(lon, 7),
            "altitude_amsl_m": round(self.alt, 1),
            "reported_by": self.reports,
            "drones": sorted(self.drones),
            "first_detected_utc": self.first_detected,
            "confirmed_utc": self.confirmed_at,
            "confirm_detection": self.confirm_detection,
            "thermal_image": self.thermal_image,
            "rgb_closeup": self.rgb_closeup,
            "peak_temperature_c": self.peak_c,
        })


class Coordinator(Node):
    def __init__(self):
        super().__init__("sar_coordinator",
                         parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        p = self.declare_parameter
        self.confirm_drone = p("confirm_drone", 3).value
        self.merge_radius = p("merge_radius_m", 10.0).value
        self.confirm_radius = p("confirm_radius_m", 6.0).value
        self.transit_agl = p("transit_agl", 25.0).value     # above the 12 m search layer
        self.confirm_agl = p("confirm_agl", 10.0).value
        self.loiter_s = p("loiter_s", 20.0).value
        self.speed = p("speed", 6.0).value
        self.out_dir = os.path.expanduser(p("output_dir", "~/sar_sim/detections").value)
        # Backup drone: takes over a search drone's remaining sweeps if that drone fails.
        self.backup_drone = p("backup_drone", 2).value          # -1 disables
        self.primaries = dict(item.split(":") for item in p("primaries", "0:west,1:east").value.split(","))
        self.search_agl = p("search_agl", 12.0).value
        self.search_speed = p("search_speed", 5.0).value
        self.hb_timeout = p("heartbeat_timeout_s", 5.0).value
        os.makedirs(self.out_dir, exist_ok=True)

        self.geo = GeoRef()
        self.terrain = Terrain()
        self.lock = threading.Lock()
        self.targets = []
        self.next_num = 1
        self.want_closeup = None      # Target to grab an RGB frame for (set by the sortie thread)
        self.closeup_sub = None

        self.create_subscription(String, "/sar/detections", self.on_detection, 50)
        self.pub = self.create_publisher(String, "/sar/targets", 10)
        self.create_timer(0.5, self.manage_closeup_sub)

        self.vehicle = Vehicle(self.confirm_drone, 14600 + self.confirm_drone, 14610 + self.confirm_drone)
        self.vehicle.wait_position()
        self.climb = self.vehicle.get_param("MPC_Z_V_AUTO_UP", 3.0)
        self.descent = self.vehicle.get_param("MPC_Z_V_AUTO_DN", 1.5)
        pos = self.vehicle.pos
        self.pad = self.geo.enu_from_geo(pos.lat * 1e-7, pos.lon * 1e-7)
        self.get_logger().info(f"confirmation drone {self.confirm_drone} ready on pad at "
                               f"E {self.pad[0]:.1f} N {self.pad[1]:.1f}")
        threading.Thread(target=self.sortie_loop, daemon=True).start()
        self.fleet_pub = self.create_publisher(String, "/sar/fleet", 10)
        self.takeovers = []          # (primary_id, region, last_seq, reason) waiting for the backup
        if self.backup_drone >= 0:
            self.backup = Vehicle(self.backup_drone, 14600 + self.backup_drone, 14610 + self.backup_drone)
            self.backup.wait_position()
            bp = self.backup.pos
            self.backup_pad = self.geo.enu_from_geo(bp.lat * 1e-7, bp.lon * 1e-7)
            self.get_logger().info(f"backup drone {self.backup_drone} ready on pad at "
                                   f"E {self.backup_pad[0]:.1f} N {self.backup_pad[1]:.1f}")
            threading.Thread(target=self.backup_loop, daemon=True).start()
            for pid, region in self.primaries.items():
                threading.Thread(target=self.watch_primary, args=(int(pid), region), daemon=True).start()

    # ------------------------------------------------------------------ targets

    def publish(self, t, note):
        self.pub.publish(String(data=t.to_json(self.geo)))
        self.get_logger().info(f"{t.id} {t.status.upper()}: {note}")

    def on_detection(self, msg):
        det = json.loads(msg.data)
        east, north = self.geo.enu_from_geo(det["latitude"], det["longitude"])
        alt = det["altitude_amsl_m"]
        with self.lock:
            near = [t for t in self.targets
                    if math.hypot(t.east - east, t.north - north) < self.merge_radius]
            t = min(near, key=lambda t: math.hypot(t.east - east, t.north - north)) if near else None

            if det["drone_id"] == self.confirm_drone:
                if t is not None and t.status in ("unconfirmed", "confirming"):
                    t.status = "confirmed"
                    t.confirmed_at = utc_now()
                    t.confirm_detection = det["detection_id"]
                    t.thermal_image = det.get("image_ref") or t.thermal_image
                    # The confirmation drone sees the target closer and from overhead: use its fix.
                    t.east, t.north, t.alt = east, north, alt
                    self.publish(t, f"seen by confirmation drone ({det['detection_id']}), "
                                    f"peak {det.get('peak_temperature_c')} C")
                elif t is None:
                    self.get_logger().info(f"confirmation drone saw a heat source away from any "
                                           f"target ({det['detection_id']}); ignored")
                return

            if t is None:
                t = Target(self.next_num, east, north, alt, det)
                self.next_num += 1
                self.targets.append(t)
                self.publish(t, f"new target from {det['detection_id']} at E {east:.1f} N {north:.1f}")
            elif det["detection_id"] not in t.reports:
                t.reports.append(det["detection_id"])
                t.drones.add(det["drone_id"])
                self.publish(t, f"also reported as {det['detection_id']} (merged)")

    # ------------------------------------------------------------------ RGB close-up

    def manage_closeup_sub(self):
        """Runs in the executor thread: subscribe to the down camera only while needed, so the
        lazy bridge does not make Gazebo render it the rest of the time."""
        topic = f"/drone{self.confirm_drone}/down_camera/image_raw"
        if self.want_closeup is not None and self.closeup_sub is None:
            self.closeup_sub = self.create_subscription(Image, topic, self.on_closeup, 10)
        elif self.want_closeup is None and self.closeup_sub is not None:
            self.destroy_subscription(self.closeup_sub)
            self.closeup_sub = None

    def on_closeup(self, msg):
        t = self.want_closeup
        if t is None or msg.encoding != "rgb8":
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        path = os.path.join(self.out_dir, f"closeup_{t.id}_drone{self.confirm_drone}.png")
        PILImage.fromarray(img.reshape(msg.height, msg.width, 3)).save(path)
        self.want_closeup = None
        with self.lock:
            t.rgb_closeup = path
            self.publish(t, f"RGB close-up saved to {path}")

    # ------------------------------------------------------------------ sorties

    def sortie_loop(self):
        while rclpy.ok():
            with self.lock:
                todo = [t for t in self.targets if t.status == "unconfirmed"]
            if not todo:
                time.sleep(1.0)
                self.vehicle.recv(timeout=0.2)   # keep GCS heartbeat going
                continue
            try:
                self.fly_sortie(todo)
            except Exception as e:  # keep coordinating even if one sortie fails
                self.get_logger().error(f"sortie failed: {e}")
                time.sleep(5.0)

    def order_targets(self, todo):
        order, here = [], self.pad
        rest = list(todo)
        while rest:
            t = min(rest, key=lambda t: math.hypot(t.east - here[0], t.north - here[1]))
            order.append(t)
            rest.remove(t)
            here = (t.east, t.north)
        return order

    def fly_sortie(self, todo):
        order = self.order_targets(todo)
        corners = [self.pad] + [(t.east, t.north) for t in order] + [self.pad]
        grad_up = 0.8 * self.climb / self.speed
        grad_down = 0.8 * self.descent / self.speed
        wps = plan_route(corners, self.terrain, self.transit_agl, grad_up, grad_down)

        def item_wp(e, n, alt, accept=3.0):
            lat, lon = self.geo.geo_from_enu(e, n)
            return dict(cmd=MAV.MAV_CMD_NAV_WAYPOINT, frame=MAV.MAV_FRAME_GLOBAL, p2=accept,
                        p4=float("nan"), x=int(lat * 1e7), y=int(lon * 1e7), z=alt)

        pad_lat, pad_lon = self.geo.geo_from_enu(*self.pad)
        items = [
            dict(cmd=MAV.MAV_CMD_NAV_TAKEOFF, frame=MAV.MAV_FRAME_GLOBAL, p4=float("nan"),
                 x=int(pad_lat * 1e7), y=int(pad_lon * 1e7), z=wps[0][2]),
            dict(cmd=MAV.MAV_CMD_DO_CHANGE_SPEED, frame=MAV.MAV_FRAME_MISSION, p1=1, p2=self.speed, p3=-1),
        ]
        hover = {}   # target id -> (seq of descend item, seq of loiter item)
        pending = list(order)
        for e, n, alt in wps[1:-1]:
            items.append(item_wp(e, n, alt))
            if pending and math.isclose(e, pending[0].east, abs_tol=1e-3) and \
                    math.isclose(n, pending[0].north, abs_tol=1e-3):
                t = pending.pop(0)
                low = self.terrain.height_amsl(e, n) + self.confirm_agl
                lat, lon = self.geo.geo_from_enu(e, n)
                items.append(item_wp(e, n, low, accept=1.0))
                descend_seq = len(items) - 1
                items.append(dict(cmd=MAV.MAV_CMD_NAV_LOITER_TIME, frame=MAV.MAV_FRAME_GLOBAL,
                                  p1=self.loiter_s, x=int(lat * 1e7), y=int(lon * 1e7), z=low))
                hover[t.id] = (descend_seq, len(items) - 1)
                items.append(item_wp(e, n, alt))     # climb back to transit height
        items.append(item_wp(self.pad[0], self.pad[1], wps[-1][2]))
        items.append(dict(cmd=MAV.MAV_CMD_NAV_LAND, frame=MAV.MAV_FRAME_GLOBAL, p4=float("nan"),
                          x=int(pad_lat * 1e7), y=int(pad_lon * 1e7), z=0))

        names = ", ".join(t.id for t in order)
        self.get_logger().info(f"launching confirmation drone to {names} ({len(items)} mission items)")
        # Right after a landing PX4 still treats the previous mission as finished; arming then makes
        # it report "Mission finished" and auto-disarm. Go to Hold first, upload, arm, and start the
        # mission explicitly from item 0.
        self.vehicle.command(MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3)
        if not self.vehicle.upload(items):
            raise RuntimeError("mission upload rejected")
        if self.vehicle.command(MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4) \
                != MAV.MAV_RESULT_ACCEPTED:
            raise RuntimeError("could not switch to mission mode")
        if self.vehicle.command(MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1) != MAV.MAV_RESULT_ACCEPTED:
            raise RuntimeError("arming rejected")
        self.vehicle.command(MAV.MAV_CMD_MISSION_START, 0, len(items) - 1)

        by_id = {t.id: t for t in order}
        current, was_armed = 0, False
        while True:
            msg = self.vehicle.recv(["MISSION_CURRENT", "HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"])
            if msg is not None and msg.get_type() == "MISSION_CURRENT":
                current = msg.seq
            for tid, (d_seq, l_seq) in hover.items():
                t = by_id[tid]
                with self.lock:
                    if current == d_seq and t.status == "unconfirmed":
                        t.status = "confirming"
                        self.publish(t, "confirmation drone arriving overhead")
                    if current == l_seq and self.vehicle.pos is not None and not t.rgb_closeup \
                            and self.want_closeup is None:
                        self.want_closeup = t
                    if current > l_seq and t.status == "confirming":
                        t.status = "not_found"
                        self.publish(t, f"nothing seen during {self.loiter_s:.0f} s hover")
            if self.vehicle.armed:
                was_armed = True
            elif was_armed:
                break
        self.want_closeup = None
        self.get_logger().info(f"confirmation drone back on its pad after {names}")


    # ------------------------------------------------------------------ backup drone

    def sim_now(self):
        """Simulation time in seconds (from /clock via use_sim_time), or None before /clock arrives."""
        t = self.get_clock().now().nanoseconds
        return t * 1e-9 if t > 0 else None

    def fleet_event(self, event, **fields):
        data = dict(event=event, utc=utc_now(), **fields)
        self.fleet_pub.publish(String(data=json.dumps(data)))
        self.get_logger().info(f"FLEET {event}: " + ", ".join(f"{k}={v}" for k, v in fields.items()))

    def plan_csv(self, drone, region):
        path = os.path.expanduser(f"~/sar_sim/missions/plan_drone{drone}_{region}_{int(self.search_agl)}m.csv")
        rows = []
        with open(path) as f:
            next(f)
            for line in f:
                e, n, a, _t = map(float, line.split(","))
                rows.append((e, n, a))
        return rows

    def watch_primary(self, drone, region):
        """Watch one search drone on its own MAVLink link; queue a takeover if it fails mid-search."""
        v = Vehicle(drone, 14620 + drone, 14630 + drone)
        # Heartbeat timeout is measured in SIMULATION time: PX4 sends heartbeats per sim second, so
        # when Gazebo stalls every drone goes quiet at once. Wall-clock timeouts gave false alarms.
        state, seq, last_hb, n_wps = "idle", 0, None, None
        self.get_logger().info(f"watching search drone {drone} ({region} half)")
        while rclpy.ok():
            msg = v.recv(["HEARTBEAT", "MISSION_CURRENT", "GLOBAL_POSITION_INT"], timeout=0.5)
            now = self.sim_now()
            if now is None:
                continue
            mode = None
            if msg is not None:
                t = msg.get_type()
                if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    last_hb = now
                    mode = ((msg.custom_mode >> 16) & 0xFF, (msg.custom_mode >> 24) & 0xFF)
                elif t == "MISSION_CURRENT":
                    seq = msg.seq
            if state == "idle":
                if v.armed and mode == (4, 4) and seq >= 2:
                    n_wps = len(self.plan_csv(drone, region))
                    state = "searching"
                    self.fleet_event("search_started", drone=drone, region=region, waypoints=n_wps)
                continue
            if state != "searching":
                continue
            reason = None
            if seq >= n_wps:                       # flying the home waypoint / landing: search done
                state = "done"
                self.fleet_event("search_finished", drone=drone, region=region)
                continue
            if last_hb is not None and now - last_hb > self.hb_timeout:
                reason = f"no heartbeat for {self.hb_timeout:.0f} s of sim time (drone lost)"
            elif mode is not None and v.armed and mode != (4, 4):
                names = {(4, 5): "RTL", (4, 6): "LAND", (4, 3): "HOLD"}
                reason = f"left mission mode for {names.get(mode, mode)} (failsafe?)"
            elif mode is not None and not v.armed:
                reason = "disarmed before finishing the search"
            if reason:
                state = "failed"
                self.fleet_event("search_drone_failed", drone=drone, region=region, at_item=seq,
                                 reason=reason)
                with self.lock:
                    self.takeovers.append((drone, region, seq, reason))

    def backup_loop(self):
        while rclpy.ok():
            with self.lock:
                job = self.takeovers.pop(0) if self.takeovers else None
            if job is None:
                self.backup.recv(timeout=0.5)       # keep GCS heartbeat going while idle
                continue
            try:
                self.fly_takeover(*job)
            except Exception as e:
                self.get_logger().error(f"backup takeover failed: {e}")

    def fly_takeover(self, drone, region, seq, reason):
        wps = self.plan_csv(drone, region)
        # Mission item k (k >= 2) flies to wps[k - 1]; the last item reached was seq - 1 -> wps[seq - 2].
        start = max(1, seq - 2)
        remaining = wps[start:-1]                   # exclude the failed drone's own return-home point
        if not remaining:
            self.fleet_event("takeover_skipped", failed_drone=drone, note="nothing left to search")
            return
        grad_up = 0.8 * self.climb / self.search_speed
        grad_down = 0.8 * self.descent / self.search_speed
        pad = self.backup_pad
        r0, rN = remaining[0], remaining[-1]
        out = plan_route([pad, r0[:2]], self.terrain, self.transit_agl, grad_up, grad_down)
        back = plan_route([rN[:2], pad], self.terrain, self.transit_agl, grad_up, grad_down)

        def wp(e, n, alt):
            lat, lon = self.geo.geo_from_enu(e, n)
            return dict(cmd=MAV.MAV_CMD_NAV_WAYPOINT, frame=MAV.MAV_FRAME_GLOBAL, p2=3.0,
                        p4=float("nan"), x=int(lat * 1e7), y=int(lon * 1e7), z=alt)

        pad_lat, pad_lon = self.geo.geo_from_enu(*pad)
        items = [dict(cmd=MAV.MAV_CMD_NAV_TAKEOFF, frame=MAV.MAV_FRAME_GLOBAL, p4=float("nan"),
                      x=int(pad_lat * 1e7), y=int(pad_lon * 1e7), z=out[0][2]),
                 dict(cmd=MAV.MAV_CMD_DO_CHANGE_SPEED, frame=MAV.MAV_FRAME_MISSION, p1=1,
                      p2=self.search_speed, p3=-1)]
        items += [wp(*w) for w in out[1:]]          # transit at transit_agl, ends above r0
        items += [wp(*w) for w in remaining]        # descend at r0, then the remaining sweeps
        items += [wp(*w) for w in back]             # climb at the last sweep point, transit home
        items.append(dict(cmd=MAV.MAV_CMD_NAV_LAND, frame=MAV.MAV_FRAME_GLOBAL, p4=float("nan"),
                          x=int(pad_lat * 1e7), y=int(pad_lon * 1e7), z=0))

        self.fleet_event("takeover_started", backup_drone=self.backup_drone, failed_drone=drone,
                         region=region, resume_at=f"E {r0[0]:.0f} N {r0[1]:.0f}",
                         remaining_waypoints=len(remaining), reason=reason)
        v = self.backup
        v.command(MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3)
        if not v.upload(items):
            raise RuntimeError("backup mission upload rejected")
        if v.command(MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4) \
                != MAV.MAV_RESULT_ACCEPTED:
            raise RuntimeError("backup could not switch to mission mode")
        if v.command(MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1) != MAV.MAV_RESULT_ACCEPTED:
            raise RuntimeError("backup arming rejected")
        v.command(MAV.MAV_CMD_MISSION_START, 0, len(items) - 1)

        was_armed, last_note = False, 0.0
        while True:
            msg = v.recv(["MISSION_CURRENT", "HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"])
            if msg is not None and msg.get_type() == "MISSION_CURRENT" and time.time() - last_note > 60:
                last_note = time.time()
                self.get_logger().info(f"backup drone {self.backup_drone}: item {msg.seq}/{len(items) - 1}")
            if v.armed:
                was_armed = True
            elif was_armed:
                break
        self.fleet_event("takeover_finished", backup_drone=self.backup_drone, failed_drone=drone,
                         region=region)


def main():
    rclpy.init()
    node = Coordinator()     # started with use_sim_time (see __init__)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
