#!/usr/bin/env python3
"""Terrain-following lawnmower search mission for one SAR drone (PX4 SITL, MAVLink).

Plans parallel north-south sweep lines over one half of the circular search area, sets every
waypoint's altitude from the terrain DEM + a fixed height above ground (AGL), uploads it to PX4
as a normal mission (takeoff -> sweeps -> land at home), flies it and logs ground clearance.

Usage (simulation running; detector optional, in another terminal):
    python3 ~/sar_sim/ros2/sar_mission.py --plan-only          # plan + map image, no flight
    python3 ~/sar_sim/ros2/sar_mission.py                      # drone 0, west half, 12 m AGL
    python3 ~/sar_sim/ros2/sar_mission.py --drone 1 --region east

Connection: the script opens its own MAVLink link on the PX4 instance (px4-mavlink start),
so it does not take over QGroundControl's port (14550) or the detector's port (14540+N).
"""

import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np
from pymavlink import mavutil

from sar_geo import PX4_GZ_DIR, GeoRef, Terrain

PX4_BIN = os.path.join(os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot")),
                       "build/px4_sitl_default/bin")
MAV = mavutil.mavlink


# =========================================================================== planning

def sample_segment(p0, p1, step):
    """Points every `step` metres from p0 to p1 (both included), as (east, north) arrays."""
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(math.ceil(length / step)))
    t = np.linspace(0.0, 1.0, n + 1)
    return np.column_stack([p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])])


def rate_limit(desired, dist, grad_up, grad_down):
    """Raise a dense altitude profile so it never climbs steeper than grad_up or descends
    steeper than grad_down (m per m). dist[i] = horizontal distance from sample i to i+1.
    Only ever raises altitudes, so clearance never drops.
    Backward pass = start climbing early before a hill; forward pass = descend late after it."""
    alt = desired.copy()
    for i in range(len(alt) - 2, -1, -1):
        alt[i] = max(alt[i], alt[i + 1] - grad_up * dist[i])
    for i in range(1, len(alt)):
        alt[i] = max(alt[i], alt[i - 1] - grad_down * dist[i - 1])
    return alt


def simplify(cum, alt, i0, i1, low_tol, high_tol, max_seg_len):
    """Indices in [i0, i1] to keep so that straight lines between kept samples stay within
    [alt - low_tol, alt + high_tol] at every sample in between."""
    keep = [i0]
    i = i0
    while i < i1:
        best = i + 1
        for j in range(i + 2, i1 + 1):
            if cum[j] - cum[i] > max_seg_len:
                break
            k = np.arange(i + 1, j)
            line = alt[i] + (alt[j] - alt[i]) * (cum[k] - cum[i]) / (cum[j] - cum[i])
            if np.all(line >= alt[k] - low_tol) and np.all(line <= alt[k] + high_tol):
                best = j
            else:
                break
        keep.append(best)
        i = best
    return keep


def sweep_lines(region, radius, spacing, edge_margin, dem_half):
    """North-south lines covering one half of the circle. Returns list of (x, y_south, y_north)."""
    sign = -1.0 if region == "west" else 1.0
    lines = []
    k = 0
    while True:
        x = sign * (spacing / 2 + k * spacing)
        if abs(x) >= radius:
            break
        half = math.sqrt(radius ** 2 - x ** 2)
        half = min(half, dem_half - edge_margin)
        if abs(x) <= dem_half - edge_margin and half > 5.0:
            lines.append((x, -half, half))
        k += 1
    return lines


def plan_mission(home_en, region, radius, spacing, agl, terrain, grad_up, grad_down, step=2.0,
                 low_tol=2.0, high_tol=5.0, max_seg_len=60.0, edge_margin=8.0):
    """Waypoint list [(east, north, alt_amsl)] starting and ending above home."""
    lines = sweep_lines(region, radius, spacing, edge_margin, terrain.size / 2)
    # Boustrophedon: start at the line end nearest home, alternate direction.
    first_x, ys, yn = lines[0]
    north_first = abs(home_en[1] - yn) < abs(home_en[1] - ys)
    corners = [home_en]
    for i, (x, y_s, y_n) in enumerate(lines):
        a, b = (y_n, y_s) if (i % 2 == 0) == north_first else (y_s, y_n)
        corners += [(x, a), (x, b)]
    corners.append(home_en)
    wps = plan_route(corners, terrain, agl, grad_up, grad_down, step, low_tol, high_tol, max_seg_len)
    return wps, lines


def plan_route(corners, terrain, agl, grad_up, grad_down, step=2.0, low_tol=2.0, high_tol=5.0,
               max_seg_len=60.0):
    """Terrain-following waypoints [(east, north, alt_amsl)] along straight legs between the
    given (east, north) corners, at `agl` above terrain, rate-limited for climb/descent."""
    # Dense samples along the whole route; corner indices are always kept as waypoints.
    pts, corner_idx = [np.array([corners[0]])], [0]
    for p0, p1 in zip(corners[:-1], corners[1:]):
        seg = sample_segment(p0, p1, step)[1:]
        pts.append(seg)
        corner_idx.append(corner_idx[-1] + len(seg))
    pts = np.vstack(pts)
    dist = np.hypot(*np.diff(pts, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(dist)])
    terrain_agl = np.array([terrain.height_amsl(e, n) + agl for e, n in pts])
    # Rate-limit over the WHOLE route so there are no altitude jumps at corners.
    alt = rate_limit(terrain_agl, dist, grad_up, grad_down)

    keep = [0]
    for i0, i1 in zip(corner_idx[:-1], corner_idx[1:]):
        keep += simplify(cum, alt, i0, i1, low_tol, high_tol, max_seg_len)[1:]
    return [(pts[k][0], pts[k][1], alt[k]) for k in keep]


def path_stats(wps, terrain, agl, step=1.0):
    """Length and min/max clearance of the planned straight-line path, sampled every metre."""
    length, clear = 0.0, []
    max_climb, max_desc = 0.0, 0.0
    for (e0, n0, a0), (e1, n1, a1) in zip(wps[:-1], wps[1:]):
        d = math.hypot(e1 - e0, n1 - n0)
        length += d
        if d > 0:
            max_climb = max(max_climb, (a1 - a0) / d)
            max_desc = max(max_desc, (a0 - a1) / d)
        for t in np.linspace(0, 1, max(2, int(d / step) + 1)):
            e, n, a = e0 + t * (e1 - e0), n0 + t * (n1 - n0), a0 + t * (a1 - a0)
            clear.append(a - terrain.height_amsl(e, n))
    clear = np.array(clear)
    near = float(np.mean(clear <= agl + 5.0))   # fraction of the route within 5 m of search height
    return length, clear.min(), clear.max(), max_climb, max_desc, near


def save_plan_image(path, wps, terrain, radius, home_en, title):
    from PIL import Image, ImageDraw
    size = 800
    dem = terrain.dem
    img = Image.fromarray((dem / dem.max() * 200 + 30).astype(np.uint8)).convert("RGB").resize((size, size))
    draw = ImageDraw.Draw(img)

    def px(e, n):
        return ((e + terrain.size / 2) / terrain.size * size, (terrain.size / 2 - n) / terrain.size * size)

    c = px(0, 0)
    r = radius / terrain.size * size
    draw.ellipse([c[0] - r, c[1] - r, c[0] + r, c[1] + r], outline=(80, 160, 255), width=2)
    draw.line([px(e, n) for e, n, _ in wps], fill=(255, 200, 0), width=2)
    for e, n, _ in wps:
        x, y = px(e, n)
        draw.ellipse([x - 1.5, y - 1.5, x + 1.5, y + 1.5], fill=(255, 80, 0))
    hx, hy = px(*home_en)
    draw.rectangle([hx - 5, hy - 5, hx + 5, hy + 5], outline=(0, 255, 0), width=2)
    draw.text((10, 10), title, fill=(255, 255, 255))
    img.save(path)


# =========================================================================== MAVLink

class Vehicle:
    def __init__(self, drone, local_port, remote_port):
        self.sysid = drone + 1
        self._ensure_link(drone, local_port, remote_port)
        self.m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{remote_port}", source_system=245,
                                            source_component=MAV.MAV_COMP_ID_MISSIONPLANNER)
        self.last_hb = 0.0
        self.pos = None
        self.armed = False
        self.wait_heartbeat()

    @staticmethod
    def _ensure_link(drone, local_port, remote_port):
        cmd = [f"{PX4_BIN}/px4-mavlink", "--instance", str(drone), "start", "-u", str(local_port),
               "-o", str(remote_port), "-t", "127.0.0.1", "-m", "onboard", "-r", "400000"]
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        text = (out.stdout + out.stderr).strip()
        if out.returncode != 0 and "already" not in text and "in use" not in text:
            print(f"warning: px4-mavlink start returned {out.returncode}: {text}")

    def heartbeat(self):
        now = time.time()
        if now - self.last_hb > 0.5:   # identify as a GCS so PX4's "no GCS" arming check passes
            self.m.mav.heartbeat_send(MAV.MAV_TYPE_GCS, MAV.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_hb = now

    def recv(self, types=None, timeout=0.5):
        self.heartbeat()
        msg = self.m.recv_match(type=types, blocking=True, timeout=timeout)
        if msg is None or msg.get_srcSystem() != self.sysid:
            return None
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            self.pos = msg
        elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
            self.armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "STATUSTEXT":
            print(f"  PX4: {msg.text}")
        return msg

    def wait_heartbeat(self, timeout=20):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.recv(["HEARTBEAT"], 1.0)
            if msg is not None:
                return
        sys.exit(f"no heartbeat from PX4 instance (system {self.sysid})")

    def wait_position(self, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            self.recv(["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"])
            if self.pos is not None:
                return self.pos
        sys.exit("no position from PX4")

    def get_param(self, name, default):
        for _ in range(3):
            self.m.mav.param_request_read_send(self.sysid, 1, name.encode(), -1)
            end = time.time() + 2
            while time.time() < end:
                msg = self.recv(["PARAM_VALUE"])
                if msg is not None and msg.param_id == name:
                    return msg.param_value
        print(f"warning: could not read {name}, assuming {default}")
        return default

    def command(self, cmd, *params, timeout=5):
        p = list(params) + [0] * (7 - len(params))
        for _ in range(3):
            self.m.mav.command_long_send(self.sysid, 1, cmd, 0, *p)
            end = time.time() + timeout
            while time.time() < end:
                msg = self.recv(["COMMAND_ACK", "STATUSTEXT", "HEARTBEAT", "GLOBAL_POSITION_INT"])
                if msg is not None and msg.get_type() == "COMMAND_ACK" and msg.command == cmd:
                    return msg.result
        return None

    def upload(self, items):
        """items: list of dicts for MISSION_ITEM_INT. Returns True on MAV_MISSION_ACCEPTED."""
        self.m.mav.mission_count_send(self.sysid, 1, len(items), MAV.MAV_MISSION_TYPE_MISSION)
        end = time.time() + 30
        while time.time() < end:
            msg = self.recv(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK", "STATUSTEXT"], 1.0)
            if msg is None:
                continue
            t = msg.get_type()
            if t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                it = items[msg.seq]
                self.m.mav.mission_item_int_send(
                    self.sysid, 1, msg.seq, it["frame"], it["cmd"], 0, 1,
                    it.get("p1", 0), it.get("p2", 0), it.get("p3", 0), it.get("p4", 0),
                    it.get("x", 0), it.get("y", 0), it.get("z", 0), MAV.MAV_MISSION_TYPE_MISSION)
                end = time.time() + 30
            elif t == "MISSION_ACK":
                if msg.type != MAV.MAV_MISSION_ACCEPTED:
                    print(f"mission rejected, MAV_MISSION_RESULT={msg.type}")
                return msg.type == MAV.MAV_MISSION_ACCEPTED
        return False


def build_items(wps, geo, speed, accept_radius):
    home_e, home_n, _ = wps[0]
    lat0, lon0 = geo.geo_from_enu(home_e, home_n)
    items = [
        # Takeoff straight up at home to the first waypoint's altitude (AMSL)
        dict(cmd=MAV.MAV_CMD_NAV_TAKEOFF, frame=MAV.MAV_FRAME_GLOBAL, p4=float("nan"),
             x=int(lat0 * 1e7), y=int(lon0 * 1e7), z=wps[0][2]),
        dict(cmd=MAV.MAV_CMD_DO_CHANGE_SPEED, frame=MAV.MAV_FRAME_MISSION, p1=1, p2=speed, p3=-1),
    ]
    for e, n, alt in wps[1:-1]:
        lat, lon = geo.geo_from_enu(e, n)
        items.append(dict(cmd=MAV.MAV_CMD_NAV_WAYPOINT, frame=MAV.MAV_FRAME_GLOBAL, p2=accept_radius,
                          p4=float("nan"), x=int(lat * 1e7), y=int(lon * 1e7), z=alt))
    # Back above home at search height, then land there.
    items.append(dict(cmd=MAV.MAV_CMD_NAV_WAYPOINT, frame=MAV.MAV_FRAME_GLOBAL, p2=accept_radius,
                      p4=float("nan"), x=int(lat0 * 1e7), y=int(lon0 * 1e7), z=wps[-1][2]))
    items.append(dict(cmd=MAV.MAV_CMD_NAV_LAND, frame=MAV.MAV_FRAME_GLOBAL, p4=float("nan"),
                      x=int(lat0 * 1e7), y=int(lon0 * 1e7), z=0))
    return items


# =========================================================================== main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drone", type=int, default=0, help="PX4 instance / drone index")
    ap.add_argument("--region", choices=["west", "east"], default="west")
    ap.add_argument("--agl", type=float, default=12.0, help="search height above terrain, m")
    ap.add_argument("--spacing", type=float, default=22.0,
                    help="sweep line spacing, m (thermal footprint ~28 m wide at 12 m AGL)")
    ap.add_argument("--speed", type=float, default=5.0, help="search ground speed, m/s")
    ap.add_argument("--radius", type=float, default=250.0, help="search area radius, m")
    ap.add_argument("--climb-rate", type=float, default=3.0,
                    help="PX4 MPC_Z_V_AUTO_UP for --plan-only (read from PX4 when flying), m/s")
    ap.add_argument("--descent-rate", type=float, default=1.5,
                    help="PX4 MPC_Z_V_AUTO_DN for --plan-only (read from PX4 when flying), m/s")
    ap.add_argument("--min-clearance", type=float, default=6.0,
                    help="warn when the drone gets closer than this to the terrain, m")
    ap.add_argument("--plan-only", action="store_true", help="plan and save the map, do not fly")
    ap.add_argument("--home", type=str, default=None,
                    help="'east,north' world position for --plan-only (default: pad at -55,-20)")
    args = ap.parse_args()

    geo = GeoRef()
    terrain = Terrain()
    out_dir = os.path.expanduser("~/sar_sim/missions")
    os.makedirs(out_dir, exist_ok=True)

    vehicle = None
    if args.plan_only:
        home_en = tuple(float(v) for v in args.home.split(",")) if args.home else (-55.0, -20.0)
    else:
        vehicle = Vehicle(args.drone, 14600 + args.drone, 14610 + args.drone)
        pos = vehicle.wait_position()
        home_en = geo.enu_from_geo(pos.lat * 1e-7, pos.lon * 1e-7)
        print(f"drone {args.drone}: at world E {home_en[0]:.1f} N {home_en[1]:.1f}, "
              f"{pos.relative_alt * 1e-3:.1f} m above home")
        if pos.relative_alt > 1000:
            sys.exit("drone is already flying; land it first")
        args.climb_rate = vehicle.get_param("MPC_Z_V_AUTO_UP", args.climb_rate)
        args.descent_rate = vehicle.get_param("MPC_Z_V_AUTO_DN", args.descent_rate)

    # Plan for 80% of the vehicle's vertical speed limits at search speed.
    grad_up = 0.8 * args.climb_rate / args.speed
    grad_down = 0.8 * args.descent_rate / args.speed
    print(f"vertical limits: climb {args.climb_rate:.1f} m/s, descent {args.descent_rate:.1f} m/s "
          f"-> max {math.degrees(math.atan(grad_up)):.0f} deg up, "
          f"{math.degrees(math.atan(grad_down)):.0f} deg down at {args.speed} m/s")
    wps, lines = plan_mission(home_en, args.region, args.radius, args.spacing, args.agl, terrain,
                              grad_up, grad_down)
    length, cmin, cmax, max_climb, max_desc, near = path_stats(wps, terrain, args.agl)
    minutes = length / args.speed / 60
    print(f"plan: {len(lines)} sweep lines, {len(wps)} waypoints, {length / 1000:.2f} km, "
          f"~{minutes:.0f} min at {args.speed} m/s")
    print(f"      planned clearance {cmin:.1f}..{cmax:.1f} m above terrain; steepest climb "
          f"{max_climb * args.speed:.1f} m/s, steepest descent {max_desc * args.speed:.1f} m/s at search speed")
    print(f"      {near * 100:.0f}% of the route is within 5 m of the {args.agl:.0f} m search height")
    tag = f"drone{args.drone}_{args.region}_{int(args.agl)}m"
    img = os.path.join(out_dir, f"plan_{tag}.png")
    save_plan_image(img, wps, terrain, args.radius, home_en,
                    f"{tag}: {len(wps)} wps, {length / 1000:.2f} km, {cmin:.0f}-{cmax:.0f} m AGL")
    with open(os.path.join(out_dir, f"plan_{tag}.csv"), "w") as f:
        f.write("east,north,alt_amsl,terrain_amsl\n")
        for e, n, a in wps:
            f.write(f"{e:.2f},{n:.2f},{a:.2f},{terrain.height_amsl(e, n):.2f}\n")
    print(f"      map: {img}")
    if args.plan_only:
        return

    items = build_items(wps, geo, args.speed, accept_radius=3.0)
    print(f"uploading {len(items)} mission items ...")
    if not vehicle.upload(items):
        sys.exit("upload failed")
    print("mission accepted by PX4")

    # AUTO.MISSION, then arm: PX4 starts the mission on arming.
    res = vehicle.command(MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4)
    if res != MAV.MAV_RESULT_ACCEPTED:
        sys.exit(f"could not switch to mission mode (result {res})")
    res = vehicle.command(MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if res != MAV.MAV_RESULT_ACCEPTED:
        sys.exit(f"arming rejected (result {res}); see PX4 messages above")
    print("armed, mission started")

    log_path = os.path.join(out_dir, f"flight_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    log = open(log_path, "w")
    log.write("t,item,east,north,alt_amsl,terrain_amsl,agl\n")
    t0, last_print, current, was_armed = time.time(), 0.0, 0, False
    min_agl, min_agl_where = float("inf"), None
    while True:
        msg = vehicle.recv(["GLOBAL_POSITION_INT", "MISSION_CURRENT", "HEARTBEAT", "STATUSTEXT"])
        if msg is not None and msg.get_type() == "MISSION_CURRENT":
            current = msg.seq
        if msg is not None and msg.get_type() == "GLOBAL_POSITION_INT":
            e, n = geo.enu_from_geo(msg.lat * 1e-7, msg.lon * 1e-7)
            alt = msg.alt * 1e-3
            ground = terrain.height_amsl(e, n)
            agl = alt - ground
            t = time.time() - t0
            log.write(f"{t:.2f},{current},{e:.2f},{n:.2f},{alt:.2f},{ground:.2f},{agl:.2f}\n")
            airborne = 2 <= current < len(items) - 1   # between takeoff and final approach
            if airborne and agl < min_agl:
                min_agl, min_agl_where = agl, (e, n)
            if airborne and agl < args.min_clearance:
                print(f"  WARNING: only {agl:.1f} m above terrain at E {e:.0f} N {n:.0f}")
            if t - last_print > 15:
                last_print = t
                print(f"  t={t:4.0f}s item {current}/{len(items) - 1}  E {e:6.1f} N {n:6.1f}  "
                      f"{agl:5.1f} m above terrain")
        if vehicle.armed:
            was_armed = True
        elif was_armed:
            break
    log.close()
    if current < len(items) - 1:
        print(f"MISSION ENDED EARLY: drone disarmed at mission item {current}/{len(items) - 1} "
              f"(failsafe or manual mode change), at E {e:.0f} N {n:.0f}")
    print(f"mission ended after {(time.time() - t0) / 60:.1f} min; lowest clearance during search "
          f"{min_agl:.1f} m at E {min_agl_where[0]:.0f} N {min_agl_where[1]:.0f}" if min_agl_where else
          "mission ended")
    print(f"flight log: {log_path}")


if __name__ == "__main__":
    main()
