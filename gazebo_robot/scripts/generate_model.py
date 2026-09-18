#!/usr/bin/env python3
"""Generate reproducible Fortress model, finite steel surfaces and test worlds."""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from mesh_utils import read_stl, write_stl, split_mesh, hull_mesh, mass_properties, combine

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent/'basic_model/robot_simplified'
MODEL = ROOT/'models/magnetic_robot'
CFG = yaml.safe_load((ROOT/'config/robot.yaml').read_text())
R = np.array([[-1., 0, 0], [0, 0, 1.], [0, 1., 0]])
ORIGIN = np.array([0.030999997946, 0.007000005936, 0.005886110239])
STEER_CAD = np.array([0.004999989209, 0.054, 0.005886110239])


def vec(x):
    return ' '.join(f'{v:.16g}' for v in np.ravel(x))


def sub(e, tag, text=None, **attrs):
    c = ET.SubElement(e, tag, attrs)
    if text is not None:
        c.text = str(text)
    return c


def write_xml(path, tree):
    ET.indent(tree, space='  ')
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)


def pose(e, xyz=(0, 0, 0), rpy=(0, 0, 0), **attrs):
    return sub(e, 'pose', vec([*xyz, *rpy]), **attrs)


def material(e, color):
    m = sub(e, 'material')
    sub(m, 'ambient', color)
    sub(m, 'diffuse', color)
    sub(m, 'specular', '0.15 0.15 0.15 1')


def mesh_geom(e, filename):
    m = sub(sub(e, 'geometry'), 'mesh')
    sub(m, 'uri', f'model://magnetic_robot/meshes/{filename}')


def surface(c):
    s = sub(c, 'surface')
    ode = sub(sub(s, 'friction'), 'ode')
    sub(ode, 'mu', CFG['wheel']['friction'])
    sub(ode, 'mu2', CFG['wheel']['friction'])


def inertia(link, props, origin):
    mass, center, I = props
    el = sub(link, 'inertial')
    pose(el, center-origin)
    sub(el, 'mass', f'{mass:.16g}')
    tensor = sub(el, 'inertia')
    for key, i, j in [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
                      ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]:
        sub(tensor, key, f'{I[i,j]:.16g}')


def box_triangles(size):
    x, y, z = np.asarray(size)/2
    p = np.array([[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],
                  [-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]])
    faces = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
             [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
    return p[np.array(faces)]


def world_base(name, step=None):
    root = ET.Element('sdf', version='1.7')
    w = sub(root, 'world', name=name)
    sub(w, 'gravity', '0 0 -9.81')
    physics = sub(w, 'physics', name='dart', type='ignored')
    sub(physics, 'max_step_size', step or CFG['physics']['step'])
    sub(physics, 'real_time_factor', 1)
    for system in ['Physics', 'Contact', 'UserCommands', 'SceneBroadcaster']:
        p = sub(w, 'plugin', filename='ignition-gazebo-'+{'UserCommands':'user-commands','SceneBroadcaster':'scene-broadcaster'}.get(system,system.lower())+'-system',
                name='ignition::gazebo::systems::'+system)
        if system == 'Physics':
            sub(p, 'engine', 'ignition-physics-dartsim-plugin')
    scene = sub(w, 'scene')
    sub(scene, 'ambient', '0.65 0.65 0.65 1')
    sub(scene, 'background', '0.15 0.18 0.22 1')
    light = sub(w, 'light', name='sun', type='directional')
    pose(light, [0,0,10])
    sub(light,'diffuse','0.9 0.9 0.9 1')
    sub(light,'direction','-0.5 -0.4 -1')
    # Start near this 13 cm vehicle, not the multi-meter default Gazebo view.
    gui=sub(w,'gui',fullscreen='false')
    scene_gui=sub(gui,'plugin',filename='MinimalScene',name='3D View')
    ui=sub(scene_gui,'ignition-gui');sub(ui,'title','3D View')
    sub(ui,'property','false',type='bool',key='showTitleBar')
    sub(ui,'property','docked',type='string',key='state')
    sub(scene_gui,'engine','ogre2');sub(scene_gui,'scene','scene')
    sub(scene_gui,'ambient_light','0.65 0.65 0.65')
    sub(scene_gui,'background_color','0.15 0.18 0.22')
    sub(scene_gui,'camera_pose','0.3 -1.8 0.24 0 0.4 2.35619449' if name=='bridge_robot' else '0.25 -0.3 0.23 0 0.43 2.2655346')
    clip=sub(scene_gui,'camera_clip');sub(clip,'near',.005);sub(clip,'far',100)
    for filename,title in [('GzSceneManager','Scene Manager'),('InteractiveViewControl','View Control'),
                           ('CameraTracking','Camera Tracking'),('EntityContextMenuPlugin','Context Menu'),
                           ('WorldControl','World Control'),('WorldStats','World Stats'),('EntityTree','Entity Tree')]:
        plug=sub(gui,'plugin',filename=filename,name=title)
        ui=sub(plug,'ignition-gui')
        sub(ui,'property','false',type='bool',key='showTitleBar')
        if filename!='EntityTree':
            sub(ui,'property','floating',type='string',key='state')
            sub(ui,'property','false',type='bool',key='resizable')
            sub(ui,'property',120 if filename=='WorldControl' else 260 if filename=='WorldStats' else 5,type='double',key='width')
            sub(ui,'property',70 if filename=='WorldControl' else 110 if filename=='WorldStats' else 5,type='double',key='height')
            if filename in ['WorldControl','WorldStats']:
                sub(ui,'property',1,type='double',key='z')
                anchors=sub(ui,'anchors',target='3D View')
                side='left' if filename=='WorldControl' else 'right'
                sub(anchors,'line',own=side,target=side);sub(anchors,'line',own='bottom',target='bottom')
        if filename=='WorldControl':
            sub(plug,'play_pause','true');sub(plug,'step','true');sub(plug,'start_paused','false');sub(plug,'use_event','true')
        if filename=='WorldStats':
            for key in ['sim_time','real_time','real_time_factor','iterations']:sub(plug,key,'true')
    return root, w


def add_robot(w, position, rpy, surface_name, surface_file, fixed=False):
    # Inline model permits world-specific magnetic targets without mutating the asset.
    model = copy.deepcopy(ET.parse(MODEL/'model.sdf').getroot().find('model'))
    pose(model, position, rpy)
    p = model.find("plugin[@name='magnetic_robot::MagneticAdhesion']")
    s = sub(p, 'surface')
    sub(s, 'model', surface_name)
    sub(s, 'file', surface_file)
    if fixed:
        joint = sub(model, 'joint', name='test_fixture', type='fixed')
        sub(joint, 'parent', 'world')
        sub(joint, 'child', 'base_link')
    w.append(model)
    return model


def make_plane_world(name, rpy=(0,0,0), magnetic=True, gap=0.0001, step=None,
                     fixed=False, position=(0,0,0), hole=False):
    from scipy.spatial.transform import Rotation
    root,w=world_base(name,step)
    mat=Rotation.from_euler('xyz',rpy).as_matrix()
    steel=sub(w,'model',name='steel_plate' if magnetic else 'nonmagnetic_plate')
    sub(steel,'static','true')
    pose(steel,position,rpy)
    link=sub(steel,'link',name='plate')
    pieces=[([2,2,.02],[0,0,-.01])]
    if hole:
        pieces=[([.8,2,.02],[-.6,0,-.01]),([.8,2,.02],[.6,0,-.01]),
                ([.4,.8,.02],[0,-.6,-.01]),([.4,.8,.02],[0,.6,-.01])]
    triangles=[]
    for i,(size,center) in enumerate(pieces):
        triangles.append(box_triangles(size)+center)
        for tag in ['visual','collision']:
            e=sub(link,tag,name=f'{tag}_{i}')
            pose(e,center)
            sub(sub(sub(e,'geometry'),'box'),'size',vec(size))
            if tag=='visual':material(e,'0.4 0.47 0.55 1')
            else:surface(e)
    filename=f'surfaces/{"hole" if hole else "plate"}.stl'
    write_stl(ROOT/filename,np.concatenate(triangles))
    add_robot(w,np.array(position)+mat@np.array([0,0,.0315+gap]),rpy,
              steel.attrib['name'] if magnetic else '__no_magnetic_target__',filename,fixed)
    write_xml(ROOT/'worlds'/f'{name}.sdf',root)


def generate():
    for p in [MODEL/'meshes', ROOT/'surfaces', ROOT/'validation', ROOT/'worlds']:
        p.mkdir(parents=True, exist_ok=True)
    manifest=json.loads((SOURCE/'export_manifest.json').read_text(encoding='utf-8-sig'))
    source={e['Name']:e for e in manifest['Files']}
    total=CFG['mass']['total'];wheel=CFG['mass']['wheel'];motor=CFG['mass']['motor']
    remainder=total-2*wheel-2*motor
    if remainder<=0:raise ValueError('Mass budget must leave positive structural mass')
    vc=source['robot_chassis']['VolumeM3'];vs=source['front_steering']['VolumeM3']
    masses={'robot_chassis':remainder*vc/(vc+vs),'front_steering':remainder*vs/(vc+vs),
            'front_motor':motor,'rear_motor':motor,'front_wheel':wheel,'rear_wheel':wheel}
    meshes={};props={};hashes={};collision={};origins={}
    for name in masses:
        path=SOURCE/'stl'/f'{name}.STL'
        hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest()
        meshes[name]=(read_stl(path)*.001-ORIGIN)@R.T
        if 'wheel' in name:
            b=np.array(source[name]['Bodies'][0]['BoundsM'])
            origins[name+'_link']=((b[:3]+b[3:])/2-ORIGIN)@R.T
        # Closed wheel and structure meshes preserve holes in mass integrals.
        # Non-manifold motors use their exterior convex hull for an explicit approximation.
        inertia_mesh=hull_mesh(meshes[name]) if 'motor' in name else meshes[name]
        props[name]=mass_properties(inertia_mesh,masses[name])
        collision[name]=[hull_mesh(t) for t in split_mesh(meshes[name])] if 'motor' not in name else [hull_mesh(meshes[name])]
        write_stl(ROOT/'validation'/f'{name}_collision_model.stl',np.concatenate(collision[name]))
    origins['base_link']=np.zeros(3)
    origins['front_steering_link']=(STEER_CAD-ORIGIN)@R.T
    groups={'base_link':['robot_chassis','rear_motor'],
            'front_steering_link':['front_steering','front_motor'],
            'front_wheel_link':['front_wheel'],'rear_wheel_link':['rear_wheel']}
    root=ET.Element('sdf',version='1.7');model=sub(root,'model',name='magnetic_robot')
    sub(model,'static','false');sub(model,'self_collide','true')
    report={'source_sha256':hashes,'cad_to_robot_rotation':R.tolist(),
            'cad_origin_m':ORIGIN.tolist(),'steering_axis_cad_m':STEER_CAD.tolist(),
            'mass_assumptions':'Remaining 0.68 kg by CAD volume; motors use exterior convex hull for inertia.',
            'links':{},'parts':{},'collision_approximation':'Per connected structural body convex hull; exterior motor hull; three cylindrical wheel bands.'}
    for link_name,parts in groups.items():
        link=sub(model,'link',name=link_name);o=origins[link_name]
        pose(link,o);sub(link,'self_collide','true')
        mp=combine([props[p] for p in parts]);inertia(link,mp,o)
        report['links'][link_name]={'origin_m':o.tolist(),'mass_kg':mp[0],
                                  'com_model_m':mp[1].tolist(),'inertia_kg_m2':mp[2].tolist()}
        for name in parts:
            write_stl(MODEL/'meshes'/f'{name}.stl',meshes[name]-o)
            # Model-space copies are used only by the independent geometry validator.
            write_stl(ROOT/'validation'/f'{name}_model.stl',meshes[name])
            visual=sub(link,'visual',name=name+'_visual');mesh_geom(visual,name+'.stl')
            material(visual,'0.12 0.14 0.16 1' if 'wheel' in name else '0.25 0.32 0.4 1' if 'motor' in name else '0.7 0.72 0.76 1')
            report['parts'][name]={'mass_kg':masses[name],'triangles':len(meshes[name])}
            if 'wheel' in name:
                # The source rear wheel axial coordinate runs in the opposite direction.
                sign=1 if name=='front_wheel' else -1
                for i,(a,b,radius) in enumerate([(0,.0135,.0315),(.0135,.0185,.03),(.0185,.0325,.0315)]):
                    c=sub(link,'collision',name=f'{name}_band_{i}')
                    pose(c,[0,sign*((a+b)/2-.01625),0],[math.pi/2,0,0])
                    cyl=sub(sub(c,'geometry'),'cylinder');sub(cyl,'radius',radius);sub(cyl,'length',b-a);surface(c)
                    sensor=sub(link,'sensor',name=f'{name}_contact_{i}',type='contact')
                    sub(sensor,'always_on','true');sub(sensor,'update_rate',0)
                    sub(sub(sensor,'contact'),'collision',c.attrib['name'])
            else:
                for i,t in enumerate(collision[name]):
                    fn=f'{name}_collision_{i}.stl';write_stl(MODEL/'meshes'/fn,t-o)
                    c=sub(link,'collision',name=f'{name}_collision_{i}');mesh_geom(c,fn);surface(c)
    specs=[('front_steering_joint','base_link','front_steering_link','0 0 1'),
           ('front_wheel_joint','front_steering_link','front_wheel_link','0 1 0'),
           ('rear_wheel_joint','base_link','rear_wheel_link','0 1 0')]
    for name,parent,child,axis in specs:
        # Fortress DART does not implement SDF continuous joints. A revolute
        # joint without finite position limits is the supported rolling hinge.
        j=sub(model,'joint',name=name,type='revolute')
        sub(j,'parent',parent);sub(j,'child',child);a=sub(j,'axis');sub(a,'xyz',axis)
        dynamics=sub(a,'dynamics');sub(dynamics,'damping',CFG['steering' if 'steering' in name else 'wheel']['damping'])
        if 'steering' in name:
            lim=sub(a,'limit');limit=math.radians(CFG['steering']['stop_limit_deg'])
            if not 0<CFG['steering']['stop_limit_deg']<=CFG['steering']['hard_limit_deg']<=15:
                raise ValueError('Steering limits must remain within +/-15 degrees')
            sub(lim,'lower',-limit);sub(lim,'upper',limit)
    p=sub(model,'plugin',filename='libMagneticRobot.so',name='magnetic_robot::MagneticAdhesion')
    for key,value in CFG['magnet'].items():sub(p,key,value)
    # Asset defaults to the existing bridge. Worlds can override with explicit surfaces.
    sub(p,'default_surface_model','bridge');sub(p,'default_surface_file','surfaces/bridge.stl')
    p=sub(model,'plugin',filename='libMagneticRobot.so',name='magnetic_robot::SteeringLimitMonitor')
    sub(p,'hard_limit',math.radians(CFG['steering']['hard_limit_deg']))
    write_xml(MODEL/'model.sdf',root)
    config=ET.Element('model');sub(config,'name','Magnetic inspection robot');sub(config,'version','1.0')
    sub(config,'sdf','model.sdf',version='1.7');sub(config,'description','2.7 kg, two 200 N magnetic wheels, front steering bounded by 15 degrees.')
    write_xml(MODEL/'model.config',config)
    report['total_mass_kg']=sum(x['mass_kg'] for x in report['links'].values())
    report['steering_stop_deg']=CFG['steering']['stop_limit_deg']
    (ROOT/'validation/model_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    # Magnetic geometry is the identical collision STL in bridge-local meters.
    bridge_paths=sorted((ROOT.parent/'gazebo_bridge/models/bridge/meshes').glob('*.stl'))
    write_stl(ROOT/'surfaces/bridge.stl',np.concatenate([read_stl(p)*.001 for p in bridge_paths]))
    (ROOT/'surfaces/bridge_manifest.json').write_text(json.dumps({str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in bridge_paths},indent=2)+'\n')
    for name,rpy in [('robot_bench',(0,0,0)),('adhesion_wall',(0,math.pi/2,0)),
                     ('adhesion_ceiling',(math.pi,0,0)),('adhesion_slope',(0,math.pi/4,0))]:
        make_plane_world(name,rpy)
    make_plane_world('nonmagnetic',magnetic=False)
    make_plane_world('out_of_range',gap=.005)
    make_plane_world('hole',hole=True)
    make_plane_world('steering_fixture',fixed=True,gap=.3)
    single=ET.parse(ROOT/'worlds/robot_bench.sdf').getroot()
    single.find('world').set('name','single_wheel_fixture')
    for shape in single.findall("world/model[@name='steel_plate']/link/*"):
        if shape.tag in ['visual','collision']:
            shape.find('geometry/box/size').text='1 2 0.02'
            shape.find('pose').text='0.5 0 -0.01 0 0 0'
    write_stl(ROOT/'surfaces/front_only.stl',box_triangles([1,2,.02])+[.5,0,-.01])
    sm=single.find("world/model[@name='magnetic_robot']")
    sm.find("plugin[@name='magnetic_robot::MagneticAdhesion']/surface/file").text='surfaces/front_only.stl'
    joint=sub(sm,'joint',name='single_wheel_test_fixture',type='fixed')
    sub(joint,'parent','world');sub(joint,'child','base_link')
    write_xml(ROOT/'worlds/single_wheel_fixture.sdf',single)
    root,w=world_base('bridge_robot')
    inc=sub(w,'include');sub(inc,'uri','model://bridge');pose(inc,[0,0,0],[math.pi/2,0,0])
    # Center channel floor: final height is verified against the actual bridge mesh.
    add_robot(w,[0,-1.5,.0416],[0,0,math.pi/2],'bridge','surfaces/bridge.stl')
    light=sub(w,'light',name='inspection_light',type='point');pose(light,[0,-1.5,1])
    sub(light,'diffuse','0.9 0.9 0.9 1')
    att=sub(light,'attenuation');sub(att,'range',10);sub(att,'constant',.5);sub(att,'linear',.01);sub(att,'quadratic',.001)
    write_xml(ROOT/'worlds/bridge_robot.sdf',root)
    bridge_cases={
        'bridge_ceiling':([0,-1.5,2.98799766-.0316],[math.pi,0,math.pi/2]),
        'bridge_roof':([0,-1.5,3+.0316],[0,0,math.pi/2]),
        'bridge_underside':([0,-1.5,-.0316],[math.pi,0,math.pi/2]),
        'bridge_left_wall':([-8.995+.0316,-1.5,1.5],[0,math.pi/2,0]),
        'bridge_right_wall':([8.995-.0316,-1.5,1.5],[0,-math.pi/2,0]),
        'bridge_panel':([5,.392-.0316,1.5],[math.pi/2,0,0]),
        'bridge_hole':([0,.392-.0316,1.5],[math.pi/2,0,0]),
        'bridge_seam':([-8.995+.0316,-2.608,1.5],[0,math.pi/2,0]),
    }
    for name,(position,rpy) in bridge_cases.items():
        clone=copy.deepcopy(root);cw=clone.find('world');cw.set('name',name)
        cw.find("model[@name='magnetic_robot']/pose").text=vec([*position,*rpy])
        # Inspection view approaches the robot from outside the supporting surface.
        from scipy.spatial.transform import Rotation
        rot=Rotation.from_euler('xyz',rpy).as_matrix()
        target=np.array(position)+rot@np.array([0,0,.04])
        eye=np.array(position)+rot@np.array([.25,-.3,.23])
        look=target-eye
        yaw=math.atan2(look[1],look[0]);pitch=math.atan2(-look[2],np.linalg.norm(look[:2]))
        cw.find("gui/plugin[@filename='MinimalScene']/camera_pose").text=vec([*eye,0,pitch,yaw])
        write_xml(ROOT/'worlds'/f'{name}.sdf',clone)
    print(f'Generated model: {report["total_mass_kg"]:.9f} kg, steering stops +/-{report["steering_stop_deg"]} deg')


if __name__=='__main__':generate()
