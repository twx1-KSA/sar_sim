#!/usr/bin/env python3
"""Thermal person detector for the SAR simulation.

Finds warm blobs in a drone's downward thermal camera, works out where they are on the
ground (lat/lon/alt of the TARGET, not of the drone), and publishes detection events.

Inputs
  /droneN/thermal_camera/image_raw    sensor_msgs/Image, mono16, 1 count = 0.01 K (ROS bridge)
  /droneN/thermal_camera/camera_info  sensor_msgs/CameraInfo (intrinsics)
  MAVLink from PX4 instance N on udp 14540+N: GLOBAL_POSITION_INT + ATTITUDE (pymavlink)

Outputs
  /sar/detections                     std_msgs/String, one JSON object per event (see Detection.to_json)
  /droneN/thermal_detections/image    sensor_msgs/Image rgb8, annotated (only built when someone listens)
  <output_dir>/det_<id>.png           annotated thermal frame for each new detection

Geolocation: pixel -> ray in camera frame -> camera mount -> vehicle attitude (NED) -> ray-march
against the terrain DEM (the world's heightmap). Without a DEM it falls back to flat ground at
the home altitude.

Run (simulation and camera_bridge.launch.py already running):
    source /opt/ros/humble/setup.bash
    python3 ~/sar_sim/ros2/thermal_detector.py                      # drone 0
    python3 ~/sar_sim/ros2/thermal_detector.py --ros-args -r __node:=thermal_detector_1 -p drone_id:=1
    (or start everything with sar_bringup.launch.py)
"""

import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime, timezone

import numpy as np
import rclpy
from PIL import Image as PILImage
from PIL import ImageDraw
from pymavlink import mavutil
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from sar_geo import DEFAULT_DEM, GeoRef, Terrain


# --------------------------------------------------------------------------- geometry

def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def ned_from_frd(roll, pitch, yaw):
    """Rotation matrix taking body FRD vectors to NED (ZYX Euler, as in MAVLink ATTITUDE)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


# --------------------------------------------------------------------------- blobs

def find_blobs(mask, min_pixels):
    """4-connected components of a boolean mask. Returns lists of (rows, cols) index arrays."""
    seen = np.zeros_like(mask, dtype=bool)
    h, w = mask.shape
    blobs = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        stack = [(r0, c0)]
        seen[r0, c0] = True
        rows, cols = [], []
        while stack:
            r, c = stack.pop()
            rows.append(r)
            cols.append(c)
            for rr, cc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    stack.append((rr, cc))
        if len(rows) >= min_pixels:
            blobs.append((np.array(rows), np.array(cols)))
    return blobs


# --------------------------------------------------------------------------- detections

class Detection:
    def __init__(self, det_id, drone_id, east, north, alt, lat, lon, conf, peak_k, stamp):
        self.id = det_id
        self.label = f"D{drone_id}-{det_id:03d}"
        self.drone_id = drone_id
        self.positions = deque(maxlen=50)
        self.positions.append((east, north, alt))
        self.lat, self.lon, self.alt = lat, lon, alt
        self.confidence = conf
        self.peak_k = peak_k
        self.hits = 1
        self.first_stamp = stamp
        self.last_stamp = stamp
        self.published = False
        self.image_ref = ""

    def mean_enu(self):
        a = np.array(self.positions)
        return a.mean(axis=0)

    def to_json(self, event):
        return json.dumps({
            "event": event,                     # "new" or "update"
            "detection_id": self.label,         # unique across drones, e.g. "D1-003"
            "drone_id": self.drone_id,
            "timestamp_sim": self.last_stamp,   # simulation seconds of the latest sighting
            "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "object_class": "person_thermal",
            "confidence": round(self.confidence, 2),  # heuristic (temperature + persistence), not a model score
            "latitude": round(self.lat, 7),
            "longitude": round(self.lon, 7),
            "altitude_amsl_m": round(self.alt, 1),
            "peak_temperature_c": round(self.peak_k - 273.15, 1),
            "sightings": self.hits,
            "image_ref": self.image_ref,
            "confirmation_status": "unconfirmed",
        })


# --------------------------------------------------------------------------- node

class ThermalDetector(Node):
    def __init__(self):
        super().__init__("thermal_detector")
        p = self.declare_parameter
        self.drone_id = p("drone_id", 0).value
        self.kelvin_per_count = p("kelvin_per_count", 0.01).value
        self.threshold_k = p("threshold_k", 300.0).value       # ~27 C: well above ground, below body heat
        self.min_pixels = p("min_pixels", 3).value
        self.min_hits = p("min_hits", 3).value                 # sightings before a detection is reported
        self.merge_radius = p("merge_radius_m", 8.0).value     # sightings closer than this are one target
        self.mount_pitch = p("camera_mount_pitch", 1.5707).value  # thermal_cam_down pose in x500_dual_cam
        self.max_range = p("max_ray_range_m", 400.0).value
        self.origin_lat = p("origin_lat", 24.7136).value       # sar_mountain.sdf spherical_coordinates
        self.origin_lon = p("origin_lon", 46.6753).value
        self.origin_alt = p("origin_alt", 600.0).value
        dem_path = p("dem_path", DEFAULT_DEM).value
        dem_size = p("dem_size_m", 500.0).value
        dem_height = p("dem_height_m", 90.0).value
        self.output_dir = os.path.expanduser(p("output_dir", "~/sar_sim/detections").value)
        mav_url = p("mavlink_url", f"udpin:0.0.0.0:{14540 + self.drone_id}").value
        os.makedirs(self.output_dir, exist_ok=True)

        self.lock = threading.Lock()
        self.state = None        # (lat, lon, alt_amsl, home_alt, roll, pitch, yaw, t)
        self.att = None
        self.K = None
        self.detections = []
        self.next_id = 1

        self.geo = GeoRef(self.origin_lat, self.origin_lon, self.origin_alt)
        self.terrain = Terrain(dem_path, dem_size, dem_height, self.origin_alt, self._home_alt)
        self.get_logger().info(
            f"drone {self.drone_id}: terrain model = "
            f"{'DEM ' + dem_path if self.terrain.dem is not None else 'flat ground at home altitude'}")

        ns = f"/drone{self.drone_id}"
        self.create_subscription(Image, f"{ns}/thermal_camera/image_raw", self.on_image, 10)
        self.create_subscription(CameraInfo, f"{ns}/thermal_camera/camera_info", self.on_info, 10)
        self.pub_det = self.create_publisher(String, "/sar/detections", 10)
        self.pub_dbg = self.create_publisher(Image, f"{ns}/thermal_detections/image", 2)

        self.mav = mavutil.mavlink_connection(mav_url, source_system=254, source_component=191)
        threading.Thread(target=self.mavlink_loop, daemon=True).start()
        self.get_logger().info(f"listening for PX4 MAVLink on {mav_url}")

    # ---- inputs

    def mavlink_loop(self):
        pos = None
        while rclpy.ok():
            msg = self.mav.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE"], blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == "GLOBAL_POSITION_INT":
                pos = (msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3, (msg.alt - msg.relative_alt) * 1e-3)
            else:
                with self.lock:
                    self.att = (msg.roll, msg.pitch, msg.yaw)
            if pos and self.att:
                with self.lock:
                    self.state = pos + self.att + (time.monotonic(),)

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"thermal intrinsics fx={self.K[0, 0]:.1f} cx={self.K[0, 2]:.1f} cy={self.K[1, 2]:.1f}")

    def _home_alt(self):
        with self.lock:
            return self.state[3] if self.state else self.origin_alt

    # ---- geolocation

    def enu_from_geo(self, lat, lon):
        return self.geo.enu_from_geo(lat, lon)

    def geo_from_enu(self, east, north):
        return self.geo.geo_from_enu(east, north)

    def locate(self, u, v, state):
        """Ground point (east, north, alt_amsl) seen at pixel (u, v), or None."""
        lat, lon, alt, _home, roll, pitch, yaw, _t = state
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        # Gazebo camera link: looks along +X, image right = -Y, image down = -Z.
        ray_link = np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])
        ray_flu = rot_y(self.mount_pitch) @ ray_link                    # camera mount on the airframe
        ray_frd = np.array([ray_flu[0], -ray_flu[1], -ray_flu[2]])
        ray_ned = ned_from_frd(roll, pitch, yaw) @ ray_frd
        ray = np.array([ray_ned[1], ray_ned[0], -ray_ned[2]])           # ENU
        ray /= np.linalg.norm(ray)
        if ray[2] >= -1e-3:
            return None  # pointing at or above the horizon
        e0, n0 = self.enu_from_geo(lat, lon)
        start = np.array([e0, n0, alt])

        def above(t):
            pt = start + t * ray
            return pt[2] - self.terrain.height_amsl(pt[0], pt[1])

        step, t_prev = 0.5, 0.0
        t = step
        while t <= self.max_range:
            if above(t) <= 0:
                lo, hi = t_prev, t
                for _ in range(20):
                    mid = 0.5 * (lo + hi)
                    lo, hi = (mid, hi) if above(mid) > 0 else (lo, mid)
                pt = start + hi * ray
                return pt[0], pt[1], self.terrain.height_amsl(pt[0], pt[1])
            t_prev, t = t, t + step
        return None

    # ---- main callback

    def on_image(self, msg):
        if msg.encoding != "mono16" or self.K is None:
            return
        with self.lock:
            state = self.state
        if state is None or time.monotonic() - state[7] > 1.0:
            return  # no fresh vehicle state: cannot geolocate

        dtype = ">u2" if msg.is_bigendian else "<u2"
        counts = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 2)[:, :msg.width]
        kelvin = counts.astype(np.float32) * self.kelvin_per_count
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        blobs = find_blobs(kelvin > self.threshold_k, self.min_pixels)
        boxes = []
        for rows, cols in blobs:
            temps = kelvin[rows, cols]
            w = temps - self.threshold_k
            v, u = float((rows * w).sum() / w.sum()), float((cols * w).sum() / w.sum())
            peak = float(temps.max())
            ground = self.locate(u, v, state)
            boxes.append((cols.min(), rows.min(), cols.max(), rows.max(), peak))
            if ground is None:
                continue
            self.add_sighting(ground, peak, len(rows), stamp, kelvin, boxes[-1])

        if self.pub_dbg.get_subscription_count() > 0:
            self.pub_dbg.publish(self.annotated_msg(msg.header, kelvin, boxes))

    def add_sighting(self, ground, peak, npix, stamp, kelvin, box):
        east, north, alt = ground
        # Heuristic confidence: how far above threshold toward body temperature, and blob size.
        temp_score = min(max((peak - self.threshold_k) / (309.0 - self.threshold_k), 0.0), 1.0)
        size_score = min(npix / 20.0, 1.0)
        conf = 0.6 * temp_score + 0.4 * size_score

        det = None
        for d in self.detections:
            me, mn, _ = d.mean_enu()
            if math.hypot(me - east, mn - north) < self.merge_radius:
                det = d
                break
        if det is None:
            lat, lon = self.geo_from_enu(east, north)
            det = Detection(self.next_id, self.drone_id, east, north, alt, lat, lon, conf, peak, stamp)
            self.next_id += 1
            self.detections.append(det)
        else:
            det.positions.append((east, north, alt))
            det.hits += 1
            det.last_stamp = stamp
            det.peak_k = max(det.peak_k, peak)
            det.confidence = max(det.confidence, conf)
            me, mn, ma = det.mean_enu()
            det.lat, det.lon = self.geo_from_enu(me, mn)
            det.alt = ma

        # Persistence: increases confidence, and gates reporting.
        det.confidence = min(1.0, det.confidence * (0.7 + 0.1 * min(det.hits, 3)))
        if not det.published and det.hits >= self.min_hits:
            det.published = True
            det.image_ref = os.path.join(self.output_dir, f"det_{det.label}.png")
            self.annotated_pil(kelvin, [box]).save(det.image_ref)
            self.pub_det.publish(String(data=det.to_json("new")))
            self.get_logger().info(
                f"NEW detection {det.label}: lat {det.lat:.7f} lon {det.lon:.7f} alt {det.alt:.1f} m "
                f"(local E {east:.1f} N {north:.1f}), peak {peak - 273.15:.1f} C")
        elif det.published and det.hits % 30 == 0:
            self.pub_det.publish(String(data=det.to_json("update")))  # refined position, ~1/s at 30 Hz

    # ---- visualisation

    def annotated_pil(self, kelvin, boxes):
        lo, hi = 285.0, 315.0
        grey = (np.clip((kelvin - lo) / (hi - lo), 0, 1) * 255).astype(np.uint8)
        img = PILImage.fromarray(grey).convert("RGB")
        draw = ImageDraw.Draw(img)
        for c0, r0, c1, r1, peak in boxes:
            draw.rectangle([c0 - 2, r0 - 2, c1 + 2, r1 + 2], outline=(255, 40, 40))
            draw.text((c1 + 4, r0), f"{peak - 273.15:.1f}C", fill=(255, 40, 40))
        return img

    def annotated_msg(self, header, kelvin, boxes):
        img = self.annotated_pil(kelvin, boxes)
        out = Image()
        out.header = header
        out.height, out.width = img.height, img.width
        out.encoding = "rgb8"
        out.step = img.width * 3
        out.data = img.tobytes()
        return out


def main():
    rclpy.init()
    node = ThermalDetector()
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
