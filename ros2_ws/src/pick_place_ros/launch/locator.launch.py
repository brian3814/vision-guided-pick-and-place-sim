"""The ROS 2 side of the cell: the wrist camera's mount as a static transform (from the hand-eye
calibration, as on a real robot), the object_locator node, and RViz2.

    ros2_ws\\ros2env.bat ros2 launch pick_place_ros locator.launch.py
    ros2_ws\\ros2env.bat ros2 launch pick_place_ros locator.launch.py rviz:=false

Arguments: project_dir (the omniverse_hookup folder), handeye (calibration file, relative to it),
mount_frame (the TF frame the camera is mounted on), rviz (true/false).
"""

import json
import pathlib

import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

USD_TO_CV = np.diag([1.0, -1.0, -1.0, 1.0])  # a USD camera looks down -z with y up; OpenCV/ROS optical: z forward, y down


def quaternion(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def nodes(context):
    project = pathlib.Path(LaunchConfiguration("project_dir").perform(context))
    calibration = json.loads((project / LaunchConfiguration("handeye").perform(context)).read_text(encoding="utf-8"))
    optical = np.array(calibration["mount_usd"]) @ USD_TO_CV  # the camera's optical frame in the gripper frame
    w, x, y, z = quaternion(optical[:3, :3])
    tx, ty, tz = optical[:3, 3]
    mount = Node(
        package="tf2_ros", executable="static_transform_publisher", name="wrist_camera_mount",
        arguments=["--x", f"{tx:.6f}", "--y", f"{ty:.6f}", "--z", f"{tz:.6f}",
                   "--qx", f"{x:.8f}", "--qy", f"{y:.8f}", "--qz", f"{z:.8f}", "--qw", f"{w:.8f}",
                   "--frame-id", LaunchConfiguration("mount_frame").perform(context), "--child-frame-id", "wrist_camera_optical"],
    )
    locator = Node(package="pick_place_ros", executable="object_locator", output="screen",
                   parameters=[{"project_dir": str(project)}])
    rviz = Node(package="rviz2", executable="rviz2", output="log",
                arguments=["-d", str(pathlib.Path(get_package_share_directory("pick_place_ros")) / "rviz" / "cell.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")))
    return [mount, locator, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("project_dir", default_value=r"D:\code\sideproject\omniverse_hookup"),
        DeclareLaunchArgument("handeye", default_value="results/perception/handeye.json"),
        DeclareLaunchArgument("mount_frame", default_value="wrist_3_link"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
