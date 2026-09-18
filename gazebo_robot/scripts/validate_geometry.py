#!/usr/bin/env python3
"""Check source alignment, bearing axis, mass budget and inertia; run FCL sweeps."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from mesh_utils import read_stl, split_mesh

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT.parent/'basic_model/robot_simplified'


def main():
    manifest=json.loads((ROOT/'validation/model_manifest.json').read_text())
    R=np.array(manifest['cad_to_robot_rotation']);origin=np.array(manifest['cad_origin_m'])
    model=ET.parse(ROOT/'models/magnetic_robot/model.sdf').getroot().find('model')
    errors={};inertias={};total=0
    for link in model.findall('link'):
        offset=np.fromstring(link.findtext('pose'),sep=' ')[:3]
        for visual in link.findall('visual'):
            name=visual.attrib['name'].removesuffix('_visual')
            source=(read_stl(SOURCE/'stl'/f'{name}.STL')*.001-origin)@R.T
            derived=read_stl(ROOT/'models/magnetic_robot/meshes'/f'{name}.stl')+offset
            errors[name]=float(np.max(np.abs(source-derived)))
        mass=float(link.findtext('inertial/mass'));total+=mass
        values={e.tag:float(e.text) for e in link.find('inertial/inertia')}
        I=np.array([[values['ixx'],values['ixy'],values['ixz']],
                    [values['ixy'],values['iyy'],values['iyz']],
                    [values['ixz'],values['iyz'],values['izz']]])
        eig=np.linalg.eigvalsh(I)
        inertias[link.attrib['name']]={'mass_kg':mass,'principal_inertias':eig.tolist(),
                                     'valid':bool(eig.min()>0 and eig.max()<=sum(eig)-eig.max()+1e-12)}
    parts=split_mesh(read_stl(SOURCE/'stl/front_steering.STL')*.001)
    bearing=max(parts,key=lambda t:t[:,:,1].mean())
    v=np.unique(bearing.reshape(-1,3),axis=0)
    p=v[np.abs(v[:,1]-.05)<1e-7][:,[0,2]]
    approximate=np.array(manifest['steering_axis_cad_m'])[[0,2]]
    radii=np.linalg.norm(p-approximate,axis=1);p=p[radii>radii.max()-.0001]
    fit=np.linalg.lstsq(np.column_stack([2*p,np.ones(len(p))]),(p*p).sum(1),rcond=None)[0]
    radius=float(np.sqrt(fit[2]+sum(fit[:2]**2)))
    residual=float(np.max(np.abs(np.linalg.norm(p-fit[:2],axis=1)-radius)))
    report={'alignment_max_error_m':errors,'total_mass_kg':total,'inertias':inertias,
            'bearing_axis_fit':{'cad_x_z_m':fit[:2].tolist(),'outer_radius_m':radius,
                                'points':len(p),'max_radial_residual_m':residual},
            'passed':bool(max(errors.values())<1e-7 and abs(total-2.7)<1e-12 and
                          all(v['valid'] for v in inertias.values()) and
                          np.max(np.abs(fit[:2]-approximate))<1e-6)}
    (ROOT/'validation/geometry_integrity_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['passed']:raise RuntimeError(report)
    subprocess.run([str(ROOT/'build/validate_geometry'),str(ROOT/'validation'),str(ROOT/'validation/geometry_report.json')],check=True)
    subprocess.run([str(ROOT/'build/validate_geometry'),str(ROOT/'validation'),str(ROOT/'validation/collision_sweep_report.json'),'collision'],check=True)
    print('Geometry alignment, bearing axis, 2.70 kg mass budget and inertia checks passed.')


if __name__=='__main__':main()
