from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('ros_gz_h1_description'), 'models/h1_ign', 'h1_2_handless.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    rviz_config = os.path.join(
        get_package_share_directory('h1_mocap_retarget'), 'config', 'mocap_view.rviz')

    args = [
        DeclareLaunchArgument('trial', default_value='',
            description='Substring to pick a trial, e.g. bar_01 / dumbells_01 / dumbells_02'),
        DeclareLaunchArgument('fps', default_value='100.0',
            description='Mocap capture rate in Hz -- confirm against your Vicon setup'),
        DeclareLaunchArgument('rate', default_value='1.0', description='Playback speed multiplier'),
        DeclareLaunchArgument('loop', default_value='true', description='Loop the trial at the end'),
        DeclareLaunchArgument('legs_only', default_value='true',
            description='Freeze both arms at neutral so only the squat is being tested'),
        DeclareLaunchArgument('flip_hip_yaw', default_value='false'),
        DeclareLaunchArgument('flip_hip_pitch', default_value='false'),
        DeclareLaunchArgument('flip_hip_roll', default_value='false'),
        DeclareLaunchArgument('flip_knee', default_value='false'),
        DeclareLaunchArgument('flip_ankle_pitch', default_value='false'),
        DeclareLaunchArgument('flip_ankle_roll', default_value='false'),
        DeclareLaunchArgument('flip_torso', default_value='false'),
    ]

    retarget_params = {
        'trial': LaunchConfiguration('trial'),
        'fps': LaunchConfiguration('fps'),
        'rate': LaunchConfiguration('rate'),
        'loop': LaunchConfiguration('loop'),
        'legs_only': LaunchConfiguration('legs_only'),
        'flip_hip_yaw': LaunchConfiguration('flip_hip_yaw'),
        'flip_hip_pitch': LaunchConfiguration('flip_hip_pitch'),
        'flip_hip_roll': LaunchConfiguration('flip_hip_roll'),
        'flip_knee': LaunchConfiguration('flip_knee'),
        'flip_ankle_pitch': LaunchConfiguration('flip_ankle_pitch'),
        'flip_ankle_roll': LaunchConfiguration('flip_ankle_roll'),
        'flip_torso': LaunchConfiguration('flip_torso'),
    }

    return LaunchDescription(args + [
        Node(
            package='h1_mocap_retarget',
            executable='retarget_node',
            name='mocap_retarget_publisher',
            output='screen',
            parameters=[retarget_params],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            arguments=[urdf]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
