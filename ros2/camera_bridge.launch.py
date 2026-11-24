"""Bridge the SAR drones' Gazebo cameras into ROS 2 (Humble + Gazebo Harmonic).

Run (after the simulation is up):
    source /opt/ros/humble/setup.bash
    ros2 launch ~/sar_sim/ros2/camera_bridge.launch.py              # drone 0 only
    ros2 launch ~/sar_sim/ros2/camera_bridge.launch.py drones:=0,1  # several drones

ROS 2 topics per drone N:
    /droneN/front_camera/image_raw    sensor_msgs/Image (rgb8, 1280x960)
    /droneN/front_camera/camera_info  sensor_msgs/CameraInfo
    /droneN/down_camera/image_raw     sensor_msgs/Image (rgb8, 1280x960)
    /droneN/down_camera/camera_info   sensor_msgs/CameraInfo
    /droneN/thermal_camera/image_raw  sensor_msgs/Image (mono16, 320x240, 0.01 K per count)
    /droneN/thermal_camera/camera_info
plus /clock (simulation time) so nodes can use use_sim_time:=true.

Camera topics are bridged lazily: the bridge subscribes to a Gazebo camera only while
some ROS 2 node reads that topic. Gazebo only renders a camera while it has a
subscriber, so cameras nobody uses cost no rendering time.
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD = "sar_mountain"
MODEL = "x500_dual_cam"

# camera name in ROS -> (Gazebo link, Gazebo sensor)
CAMERAS = {
    "front_camera": ("front_camera_link", "front_camera"),
    "down_camera": ("down_camera_link", "down_camera"),
    "thermal_camera": ("thermal_camera_link", "thermal_camera"),
}


def entry(gz_topic, ros_topic, ros_type, gz_type, lazy):
    return (
        f"- gz_topic_name: \"{gz_topic}\"\n"
        f"  ros_topic_name: \"{ros_topic}\"\n"
        f"  ros_type_name: \"{ros_type}\"\n"
        f"  gz_type_name: \"{gz_type}\"\n"
        f"  direction: GZ_TO_ROS\n"
        f"  lazy: {'true' if lazy else 'false'}\n"
    )


def make_bridge(context):
    world = LaunchConfiguration("world").perform(context)
    drones = [d.strip() for d in LaunchConfiguration("drones").perform(context).split(",") if d.strip()]

    # /clock is small and always needed, so it is never lazy.
    yaml = entry(f"/world/{world}/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", False)

    for n in drones:
        for ros_name, (link, sensor) in CAMERAS.items():
            base = f"/world/{world}/model/{MODEL}_{n}/link/{link}/sensor/{sensor}"
            yaml += entry(f"{base}/image", f"/drone{n}/{ros_name}/image_raw",
                          "sensor_msgs/msg/Image", "gz.msgs.Image", True)
            yaml += entry(f"{base}/camera_info", f"/drone{n}/{ros_name}/camera_info",
                          "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", True)

    fd, config_path = tempfile.mkstemp(prefix="sar_camera_bridge_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        f.write(yaml)

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="sar_camera_bridge",
            parameters=[{"config_file": config_path, "use_sim_time": True}],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drones", default_value="0", description="Comma-separated drone indices, e.g. 0,1,2,3"),
        DeclareLaunchArgument("world", default_value=WORLD),
        OpaqueFunction(function=make_bridge),
    ])
