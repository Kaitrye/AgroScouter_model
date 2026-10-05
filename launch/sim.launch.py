from pathlib import Path
import json
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    params = json.loads((root / 'config/parameters.json').read_text())
    camera = params['camera']
    pose = camera['pose_m_rad']
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        # The standalone Gazebo command is started by a tiny wrapper, so gui:=false
        # uses server-only mode without changing the model or world.
        ExecuteProcess(cmd=['python3', str(root / 'tools/run_gazebo.py'), LaunchConfiguration('gui')], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='robot_bridge',
             parameters=[{'config_file': str(root / 'config/bridge.yaml'), 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
             parameters=[{'config_file': str(root / 'config/camera_bridge.yaml'), 'use_sim_time': True,
                          'override_frame_id': 'camera_optical_frame'}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge',
             parameters=[{'config_file': str(root / 'config/lidar_bridge.yaml'), 'use_sim_time': True,
                          'override_frame_id': 'lidar_link'}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='imu_bridge',
             parameters=[{'config_file': str(root / 'config/imu_bridge.yaml'), 'use_sim_time': True,
                          'override_frame_id': 'imu_link'}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='imu_tf',
             arguments=['--x', str(params['imu']['pose_m'][0]),
                        '--y', str(params['imu']['pose_m'][1]),
                        '--z', str(params['imu']['pose_m'][2]),
                        '--roll', str(params['imu']['rpy_rad'][0]),
                        '--pitch', str(params['imu']['rpy_rad'][1]),
                        '--yaw', str(params['imu']['rpy_rad'][2]),
                        '--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='lidar_tf',
             arguments=['--x', str(params['underbody']['lidar']['pose_m'][0]),
                        '--y', str(params['underbody']['lidar']['pose_m'][1]),
                        '--z', str(params['underbody']['lidar']['pose_m'][2]),
                        '--frame-id', 'base_link', '--child-frame-id', 'lidar_link']),
        ExecuteProcess(cmd=['python3', str(root / 'tools/command_relay.py'), '--ros-args',
                            '-p', 'use_sim_time:=true', '-p', f"timeout:={params['command_timeout_s']}"], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_mount_tf',
             arguments=['--x', str(pose[0]), '--y', str(pose[1]), '--z', str(pose[2]),
                        '--roll', str(pose[3]), '--pitch', str(pose[4]), '--yaw', str(pose[5]),
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_optical_tf',
             arguments=['--roll', str(-math.pi/2), '--pitch', '0', '--yaw', str(-math.pi/2),
                        '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame']),
    ])
