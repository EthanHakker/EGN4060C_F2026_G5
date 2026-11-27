"""Launch Gazebo with the HSR test world, spawn the robot and bridge topics to ROS 2.

    ros2 launch hsr_description sim.launch.py
    ros2 launch hsr_description sim.launch.py headless:=true
    ros2 launch hsr_description sim.launch.py world:=/abs/path/my_world.sdf x:=1.0 y:=-2.0 yaw:=1.57
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("hsr_description")
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    x, y, yaw = LaunchConfiguration("x"), LaunchConfiguration("y"), LaunchConfiguration("yaw")

    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([pkg, "urdf", "hsr.urdf.xacro"])]), value_type=str)

    # Let Gazebo resolve package://hsr_description/meshes/... (as model://hsr_description/...)
    resource_path = os.path.dirname(pkg) + os.pathsep + os.environ.get("GZ_SIM_RESOURCE_PATH", "")

    gz_args_gui = PythonExpression(["'-r ", world, "'"])
    gz_args_headless = PythonExpression(["'-r -s --headless-rendering ", world, "'"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(pkg, "worlds", "hsr_test_world.sdf")),
        DeclareLaunchArgument("headless", default_value="false", description="true = server only, no GUI"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": gz_args_gui, "on_exit_shutdown": "true"}.items(),
                                 condition=UnlessCondition(headless)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={"gz_args": gz_args_headless, "on_exit_shutdown": "true"}.items(),
                                 condition=IfCondition(headless)),

        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),

        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-name", "hsr", "-topic", "robot_description",
                        "-x", x, "-y", y, "-z", "0.01", "-Y", yaw]),

        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(pkg, "config", "ros_gz_bridge.yaml"),
                          "use_sim_time": True}]),
    ])
