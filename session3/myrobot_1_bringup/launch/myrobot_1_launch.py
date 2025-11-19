from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    # Get the path to the URDF file and RViz config file
    urdf_path = os.path.join(get_package_share_path('myrobot_1_description'),'urdf', 'myrobot_1.urdf')
    rviz_config_path = os.path.join(get_package_share_path('myrobot_1_description'),'rviz', 'default.rviz')
    # Read the URDF file
    with open(urdf_path, 'r') as f:robot_description = f.read()
    # Create the robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}])
    # Open RViz with the specified configuration
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path])
    # Launch Gazebo with an empty world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'myrobot_1'],
        output='screen')
    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node,
        gazebo,
        spawn_robot])