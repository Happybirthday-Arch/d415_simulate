#!/usr/bin/env python3
"""Conservative geometric separation proof for the added camera/support."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
repo=Path(__file__).resolve().parents[4];sys.path.insert(0,str(repo/'gazebo_robot/scripts'))
from mesh_utils import read_stl
cfg=yaml.safe_load((repo/'d415_sim_ws/src/bridge_robot_sim/config/assembly.yaml').read_text())
bottom=np.array(cfg['support_bottom']);length=cfg['support_length'];radius=cfg['support_diameter']/2
chassis=read_stl(repo/'gazebo_robot/models/magnetic_robot/meshes/robot_chassis.stl')
tri=chassis[np.max(np.abs(chassis[:,:,2]-bottom[2]),axis=1)<1e-7,:,:2]
# Check a dense footprint lattice against the actual top-face triangles, including holes.
points=np.array([[x,y] for x in np.linspace(-radius,radius,31) for y in np.linspace(-radius,radius,31) if x*x+y*y<=radius*radius])+bottom[:2]
a=tri[:,0];u=tri[:,1]-a;v=tri[:,2]-a;det=u[:,0]*v[:,1]-u[:,1]*v[:,0]
good=np.abs(det)>1e-15;a,u,v,det=a[good],u[good],v[good],det[good]
covered=[]
for p in points:
 d=p-a;b=(d[:,0]*v[:,1]-d[:,1]*v[:,0])/det;c=(u[:,0]*d[:,1]-u[:,1]*d[:,0])/det
 covered.append(bool(np.any((b>=-1e-7)&(c>=-1e-7)&(b+c<=1+1e-7))))
gaps={}
for name in ('front_wheel','front_motor','front_steering','rear_wheel','rear_motor'):
 t=read_stl(repo/f'gazebo_robot/validation/{name}_model.stl')
 # Rotation about the Z steering axis preserves every vertex's Z coordinate.
 gaps[name]=float(bottom[2]-t[:,:,2].max())
camera=read_stl('/opt/ros/humble/share/realsense2_description/meshes/d415.stl')
camera=camera@Rotation.from_euler('xyz',[np.pi/2,0,np.pi/2]).as_matrix().T
camera+=bottom+np.array([.00987,0,length+.0115])
report={'support_length_m':length,'camera_front_x_m':float(camera[:,:,0].max()),
        'front_flush_error_m':float(abs(camera[:,:,0].max()-.052)),
        'camera_bottom_z_m':float(camera[:,:,2].min()),'camera_top_z_m':float(camera[:,:,2].max()),
        'support_footprint_samples':len(points),'support_footprint_covered':all(covered),
        'minimum_vertical_separation_m':gaps,
        'separation_method':'Positive Z interval separation for all moving meshes, invariant under every steering angle; rod/chassis attachment contact intentional.'}
report['passed']=all(covered) and min(gaps.values())>0 and report['front_flush_error_m']<.0002 and abs(length-.03)<1e-9
out=repo/'gazebo_robot/validation/integration/attachment_report.json';out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
assert report['passed']
