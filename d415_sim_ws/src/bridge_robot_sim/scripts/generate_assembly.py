#!/usr/bin/env python3
"""Generate a fused rigid camera/support on the existing dynamic SDF robot."""
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4); t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix(); t[:3, 3] = xyz
    return t


def origin(element):
    if element is None: return np.eye(4)
    return transform([float(x) for x in element.get('xyz', '0 0 0').split()],
                     [float(x) for x in element.get('rpy', '0 0 0').split()])


def pose(element, t):
    old = element.find('pose')
    if old is not None: element.remove(old)
    ET.SubElement(element, 'pose').text = ' '.join(map(str, [*t[:3, 3], *Rotation.from_matrix(t[:3, :3]).as_euler('xyz')]))


def generate(repo, config, output, world='bridge_robot'):
    repo, output = Path(repo), Path(output); output.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load(Path(config).read_text())
    if not 0 < cfg['support_length'] < .2: raise ValueError('Invalid support length')
    sys.path.insert(0, str(repo/'gazebo_robot/scripts'))
    from mesh_utils import read_stl, mass_properties, combine
    camera_share = Path(get_package_share_directory('bridge_d415_sim'))
    camera_cfg = camera_share/'config/camera.yaml'
    # Expand the existing camera macro only; do not import the old static stand.
    wrapper = f'''<robot name="camera" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="cfg" value="${{xacro.load_yaml('{camera_cfg}')}}"/>
    <xacro:include filename="{camera_share}/urdf/d415_module.xacro"/>
    <link name="camera_support_link"/>
    <xacro:bridge_d415 parent="camera_support_link" cfg="${{cfg}}">
      <origin xyz="0 0 {cfg['support_length']}" rpy="0 0 0"/>
    </xacro:bridge_d415></robot>'''
    description = xacro.parse(wrapper); xacro.process_doc(description)
    urdf = ET.fromstring(description.toxml())
    bottom = np.array(cfg['support_bottom'])
    transforms = {'camera_support_link': transform(bottom)}
    static = [{'parent': 'base_link', 'child': 'camera_support_link', 'xyz': bottom.tolist(), 'quat': [0,0,0,1]}]
    pending = list(urdf.findall('joint'))
    while pending:
        progress = False
        for j in pending[:]:
            parent = j.find('parent').get('link'); child = j.find('child').get('link')
            if parent not in transforms: continue
            local = origin(j.find('origin')); transforms[child] = transforms[parent]@local
            static.append({'parent': parent, 'child': child, 'xyz': local[:3,3].tolist(),
                           'quat': Rotation.from_matrix(local[:3,:3]).as_quat().tolist()})
            pending.remove(j); progress = True
        if not progress: raise ValueError('Disconnected camera TF')
    tree = ET.parse(repo/f'gazebo_robot/worlds/{world}.sdf'); w = tree.getroot().find('world')
    w.set('name', 'bridge_robot_camera')
    robot = w.find("model[@name='magnetic_robot']"); base = robot.find("link[@name='base_link']")
    for link in urdf.findall('link'):
        if link.get('name') not in transforms: continue
        for kind in ('visual', 'collision'):
            for index, item in enumerate(link.findall(kind)):
                t = transforms[link.get('name')]@origin(item.find('origin'))
                out = ET.SubElement(base, kind, name=f'{link.get("name")}_{kind}_{index}')
                pose(out,t); geom = copy.deepcopy(item.find('geometry'))
                box = geom.find('box')
                if box is not None:
                    size = box.attrib.pop('size')
                    ET.SubElement(box, 'size').text = size
                mesh = geom.find('mesh')
                if mesh is not None:
                    uri = mesh.attrib.pop('filename')
                    if uri.startswith('package://'):
                        package, rel = uri[len('package://'):].split('/',1)
                        uri = str(Path(get_package_share_directory(package))/rel)
                    mesh.attrib.clear();ET.SubElement(mesh,'uri').text=uri
                ET.SubElement(out,'geometry').extend(list(geom))
                if kind=='visual':
                    material=ET.SubElement(out,'material')
                    ET.SubElement(material,'ambient').text='0.25 0.28 0.3 1'
                    ET.SubElement(material,'diffuse').text='0.35 0.38 0.4 1'
    for gazebo in urdf.findall('gazebo'):
        frame = gazebo.get('reference')
        for sensor in gazebo.findall('sensor'):
            sensor=copy.deepcopy(sensor);pose(sensor,transforms[frame]);base.append(sensor)
    length=cfg['support_length'];radius=cfg['support_diameter']/2
    rod_center=bottom+np.array([0,0,length/2]);rod_mass=math.pi*radius**2*length*cfg['support_density']
    for kind in ('visual','collision'):
        out=ET.SubElement(base,kind,name='camera_support_'+kind);pose(out,transform(rod_center))
        cylinder=ET.SubElement(ET.SubElement(out,'geometry'),'cylinder')
        ET.SubElement(cylinder,'radius').text=str(radius);ET.SubElement(cylinder,'length').text=str(length)
        if kind=='visual':
            mat=ET.SubElement(out,'material');ET.SubElement(mat,'ambient').text='0.55 0.55 0.55 1'
            ET.SubElement(mat,'diffuse').text='0.65 0.65 0.65 1'
    # Recompute chassis + motor + camera + support inertias; mass stays 2.70 kg.
    camera_mass=cfg['camera_mass'];deduction=rod_mass+camera_mass if cfg['preserve_total_mass'] else 0
    chassis_mass=.4862190308204766-deduction
    if chassis_mass<=0: raise ValueError('Accessories exceed available chassis mass budget')
    meshroot=repo/'gazebo_robot/models/magnetic_robot/meshes'
    chassis=mass_properties(read_stl(meshroot/'robot_chassis.stl'),chassis_mass)
    motor=mass_properties(read_stl(meshroot/'rear_motor_collision_0.stl'),.31)
    rod_I=np.diag([rod_mass*(3*radius**2+length**2)/12]*2+[rod_mass*radius**2/2])
    camera_center=bottom+np.array([-.000155,0,length+.0115])
    dims=np.array([.02005,.099,.023]);camera_I=np.diag(camera_mass*(np.sum(dims**2)-dims**2)/12)
    mass,com,inertia=combine([chassis,motor,(rod_mass,rod_center,rod_I),(camera_mass,camera_center,camera_I)])
    old=base.find('inertial');base.remove(old);inertial=ET.SubElement(base,'inertial')
    pose(inertial,transform(com));ET.SubElement(inertial,'mass').text=str(mass)
    tensor=ET.SubElement(inertial,'inertia')
    for tag,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:ET.SubElement(tensor,tag).text=str(inertia[i,j])
    plugin=ET.SubElement(robot,'plugin',filename='libMagneticRobot.so',name='magnetic_robot::RobotDriveController')
    drive=yaml.safe_load((Path(config).parent/'drive.yaml').read_text())
    for k,v in drive.items():ET.SubElement(plugin,k).text=str(v)
    sensors=ET.SubElement(w,'plugin',filename='ignition-gazebo-sensors-system',name='ignition::gazebo::systems::Sensors')
    ET.SubElement(sensors,'render_engine').text='ogre2'
    ET.indent(tree,space='  ');tree.write(output/'world.sdf',encoding='utf-8',xml_declaration=True)
    manifest=json.loads((repo/'gazebo_robot/validation/model_manifest.json').read_text())
    links={k:v['origin_m'] for k,v in manifest['links'].items()}
    report={'assembly':cfg,'rod_mass':rod_mass,'camera_mass':camera_mass,'base_mass':mass,
            'total_mass':mass+sum(float(l.findtext('inertial/mass')) for l in robot.findall('link') if l is not base),
            'camera_bottom_xyz':transforms['d415_bottom_screw_frame'][:3,3].tolist(),
            'base_com':com.tolist(),'inertia_eigenvalues':np.linalg.eigvalsh(inertia).tolist(),
            'static_tf':static,'link_origins':links,'drive':drive,'source_world':world,
            'rigid_fusion':'Camera/support visual, collision, sensors and inertia fused into base_link; static TF retained.'}
    (output/'assembly.json').write_text(json.dumps(report,indent=2))
    bridge=yaml.safe_load((camera_share/'config/bridge.yaml').read_text())
    for ros,gz,rt,gt,direction in [
        ('/robot/drive_command','/robot/drive_command','geometry_msgs/msg/Vector3','ignition.msgs.Vector3d','ROS_TO_GZ'),
        ('/robot/drive_status','/robot/drive_status','geometry_msgs/msg/Vector3','ignition.msgs.Vector3d','GZ_TO_ROS'),
        ('/joint_states','/robot/joint_states','sensor_msgs/msg/JointState','ignition.msgs.Model','GZ_TO_ROS'),
        ('/robot/ground_truth','/robot/ground_truth','geometry_msgs/msg/PoseStamped','ignition.msgs.Pose','GZ_TO_ROS')]:
        bridge.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction=direction))
    (output/'bridge.yaml').write_text(yaml.safe_dump(bridge))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',required=True);p.add_argument('--config',required=True)
    p.add_argument('--output',required=True);p.add_argument('--world',default='bridge_robot')
    a=p.parse_args();generate(a.repo,a.config,a.output,a.world)
