"""Official RealSense geometry + adapted Locus RGB/depth macros on Fortress."""
import os
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('bridge_d415_sim'))
    cfg_path = Path(get('config')).resolve()
    cfg = yaml.safe_load(cfg_path.read_text())
    calibration = get('world') == 'calibration'
    if get('world') not in ('calibration', 'bridge_camera'):
        raise ValueError('world must be calibration or bridge_camera')
    pose = [cfg[k] for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
    if calibration:
        pose = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    description = xacro.process_file(str(share / 'urdf/test_camera.urdf.xacro'),
                                     mappings={'config': str(cfg_path)}).toxml()
    bridge_models = Path(get('bridge_models')).resolve()
    if not (bridge_models / 'bridge/model.sdf').is_file():
        raise FileNotFoundError(f'Bridge model not found: {bridge_models}')
    paths = [str(bridge_models), str(Path(get_package_share_directory('realsense2_description')).parent)]
    old = os.environ.get('IGN_GAZEBO_RESOURCE_PATH')
    if old:
        paths.append(old)
    sim_cmd = ['ign', 'gazebo', '-r', '-v', '2']
    if get('gui') != 'true':
        sim_cmd.append('-s')
    sim_cmd.append(str(share / 'worlds' / (get('world') + '.sdf')))
    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(paths)),
        ExecuteProcess(cmd=sim_cmd, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', str(pose[0]), '--y', str(pose[1]), '--z', str(pose[2]),
                        '--roll', str(pose[3]), '--pitch', str(pose[4]), '--yaw', str(pose[5]),
                        '--frame-id', 'world', '--child-frame-id', 'camera_mount']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', get('world'), '-string', description, '-name', 'd415_test_mount',
                        '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
                        '-R', str(pose[3]), '-P', str(pose[4]), '-Y', str(pose[5])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='d415_bridge',
             parameters=[{'config_file': str(share / 'config/bridge.yaml')}], output='screen'),
        Node(package='bridge_d415_sim', executable='camera_support.py', name='camera_support',
             parameters=[{'use_sim_time': True, 'config_file': str(cfg_path)}], output='screen'),
        Node(package='depth_image_proc', executable='point_cloud_xyz_node', name='depth_points',
             parameters=[{'use_sim_time': True}],
             remappings=[('image_rect', '/d415/depth/image_raw'),
                         ('camera_info', '/d415/depth/camera_info'), ('points', '/d415/depth/points')]),
    ]
    if get('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(share / 'rviz/d415.rviz')],
                            parameters=[{'use_sim_time': True}]))
    if get('record') == 'true':
        actions.append(Node(package='bridge_d415_sim', executable='record.py',
                            arguments=['--root', get('recordings'), '--config', str(cfg_path),
                                       '--world', get('world'), '--bridge-models', str(bridge_models)],
                            output='screen', sigterm_timeout='30', sigkill_timeout='10'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('bridge_d415_sim'))
    # Source checkout location remains stable under colcon --symlink-install.
    workspace = Path(__file__).resolve().parents[3]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/camera.yaml')),
        DeclareLaunchArgument('world', default_value='bridge_camera'),
        DeclareLaunchArgument('bridge_models', default_value=str(workspace.parent / 'gazebo_bridge/models')),
        DeclareLaunchArgument('recordings', default_value=str(workspace / 'recordings')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='true'),
        OpaqueFunction(function=setup),
    ])
