"""One world, dynamic camera TF, keyboard and selectable recording."""
import os,subprocess,tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,SetEnvironmentVariable,RegisterEventHandler,EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    get=lambda k:LaunchConfiguration(k).perform(context)
    repo=Path(get('repo')).resolve();share=Path(get_package_share_directory('bridge_robot_sim'))
    camera=Path(get_package_share_directory('bridge_d415_sim'));lib=Path(get_package_prefix('bridge_robot_sim'))/'lib/bridge_robot_sim'
    mode=get('record_mode');world=get('world')
    if mode not in ('rgb','full','off'):raise ValueError('record_mode must be rgb, full or off')
    allowed=('bridge_robot','robot_bench','adhesion_wall','adhesion_ceiling','adhesion_slope','bridge_left_wall','bridge_right_wall','bridge_ceiling')
    if world not in allowed:raise ValueError(f'world must be one of {allowed}')
    runtime=repo/'d415_sim_ws/runtime';runtime.mkdir(exist_ok=True)
    generated=Path(tempfile.mkdtemp(prefix='robot_',dir=runtime))
    subprocess.run(['python3',str(lib/'generate_assembly.py'),'--repo',str(repo),'--config',str(share/'config/assembly.yaml'),
                    '--output',str(generated),'--world',world],check=True)
    if not (repo/'gazebo_robot/build/libMagneticRobot.so').exists():raise FileNotFoundError('先运行 ./build_robot.sh')
    simcmd=['ign','gazebo','-r','-v','2',str(generated/'world.sdf')]
    if get('gui')!='true':simcmd.append('-s')
    sim=ExecuteProcess(cmd=simcmd,output='screen')
    envpaths=[str(repo/'gazebo_robot/models'),str(repo/'gazebo_bridge/models'),str(Path(get_package_share_directory('realsense2_description')).parent)]
    old=os.environ.get('IGN_GAZEBO_RESOURCE_PATH')
    if old:envpaths.append(old)
    actions=[SetEnvironmentVariable('MAGNETIC_ROBOT_RESOURCE_ROOT',str(repo/'gazebo_robot')),
             SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',os.pathsep.join(envpaths)),
             SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',str(repo/'gazebo_robot/build')+os.pathsep+os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH','')),
             sim]
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='robot_camera_bridge',
                parameters=[{'config_file':str(generated/'bridge.yaml')}],output='screen')
    tf=Node(package='bridge_robot_sim',executable='state_tf.py',arguments=['--manifest',str(generated/'assembly.json')],
            parameters=[{'use_sim_time':True}],output='screen')
    actions.extend([bridge,tf,
        Node(package='bridge_d415_sim',executable='camera_support.py',parameters=[{'use_sim_time':True,'config_file':str(camera/'config/camera.yaml')}],output='screen'),
        Node(package='depth_image_proc',executable='point_cloud_xyz_node',name='depth_points',parameters=[{'use_sim_time':True}],
             remappings=[('image_rect','/d415/depth/image_raw'),('camera_info','/d415/depth/camera_info'),('points','/d415/depth/points')])])
    critical=[sim,bridge,tf]
    if get('keyboard')=='true':
        keyboard=Node(package='bridge_robot_sim',executable='keyboard_drive.py',output='screen');actions.append(keyboard);critical.append(keyboard)
    if get('rviz')=='true':actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/robot_camera.rviz')],parameters=[{'use_sim_time':True}]))
    if mode!='off':
        args=['--root',get('recordings')]
        if mode=='rgb':args+=['--config',str(share/'config/recording.yaml')]
        else:args+=['--config',str(camera/'config/camera.yaml'),'--world','bridge_robot_camera',
                    '--bridge-models',str(repo/'gazebo_bridge/models'),'--manifest',str(generated/'assembly.json')]
        recorder=Node(package='bridge_robot_sim',executable=f'record_{mode}.py',arguments=args,output='screen',sigterm_timeout='35',sigkill_timeout='5')
        actions.append(recorder);critical.append(recorder)
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='联合仿真组件退出'))])))
    return actions

def generate_launch_description():
    repo=Path(__file__).resolve().parents[4]
    return LaunchDescription([
        DeclareLaunchArgument('repo',default_value=str(repo)),
        DeclareLaunchArgument('world',default_value='bridge_robot'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('keyboard',default_value='true'),DeclareLaunchArgument('record_mode',default_value='rgb'),
        DeclareLaunchArgument('recordings',default_value=str(repo/'d415_sim_ws/recordings_robot')),
        OpaqueFunction(function=setup)])
