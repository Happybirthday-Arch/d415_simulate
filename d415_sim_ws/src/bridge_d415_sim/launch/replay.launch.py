from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    session = Path(LaunchConfiguration('session').perform(context)).resolve()
    if not (session / 'bag/metadata.yaml').is_file():
        raise FileNotFoundError('A closed bag session is required')
    share = Path(get_package_share_directory('bridge_d415_sim'))
    result = [
        Node(package='bridge_d415_sim', executable='camera_support.py',
             parameters=[{'use_sim_time': True, 'config_file': str(session / 'run_config.yaml')}]),
        Node(package='depth_image_proc', executable='point_cloud_xyz_node',
             parameters=[{'use_sim_time': True}],
             remappings=[('image_rect','/d415/depth/image_raw'),
                         ('camera_info','/d415/depth/camera_info'),('points','/d415/depth/points')]),
        ExecuteProcess(cmd=['ros2','bag','play',str(session/'bag'),'--delay','3',
                            '--qos-profile-overrides-path',str(session/'qos.yaml')], output='screen'),
    ]
    if LaunchConfiguration('rviz').perform(context) == 'true':
        result.append(Node(package='rviz2', executable='rviz2',
                           arguments=['-d',str(share/'rviz/d415.rviz')],
                           parameters=[{'use_sim_time':True}]))
    return result


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session'),
                              DeclareLaunchArgument('rviz',default_value='true'),
                              OpaqueFunction(function=setup)])
