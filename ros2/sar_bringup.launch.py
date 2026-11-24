"""Start the ROS 2 side of the SAR simulation for one or more drones:
the Gazebo camera bridge plus one thermal detector per drone.

    source /opt/ros/humble/setup.bash
    ros2 launch ~/sar_sim/ros2/sar_bringup.launch.py drones:=0,1

Start the PX4 instances first (PX4 instance N = drone N, model x500_dual_cam_N).
Detections from all drones arrive on /sar/detections.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.realpath(__file__))


def detectors(context):
    drones = [d.strip() for d in LaunchConfiguration("drones").perform(context).split(",") if d.strip()]
    return [
        ExecuteProcess(
            cmd=["python3", os.path.join(HERE, "thermal_detector.py"), "--ros-args",
                 "-r", f"__node:=thermal_detector_{n}", "-p", f"drone_id:={n}", "-p", "use_sim_time:=true"],
            name=f"thermal_detector_{n}",
            output="screen",
        )
        for n in drones
    ]


def generate_launch_description():
    drones = LaunchConfiguration("drones")
    return LaunchDescription([
        DeclareLaunchArgument("drones", default_value="0,1", description="Comma-separated drone indices"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, "camera_bridge.launch.py")),
            launch_arguments={"drones": drones}.items(),
        ),
        OpaqueFunction(function=detectors),
    ])
