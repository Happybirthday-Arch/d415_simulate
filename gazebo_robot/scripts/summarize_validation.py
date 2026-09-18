#!/usr/bin/env python3
"""Collect current experiment reports without rerunning or hiding failures."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def main():
    reports={p.parent.name:json.loads(p.read_text()) for p in sorted((ROOT/'validation/physics').glob('*/report.json'))}
    expected={'floor','wall','ceiling','slope','nonmagnetic','hole','out_of_range','pull_off','single_wheel',
              'bridge_floor','bridge_ceiling','bridge_roof','bridge_underside','bridge_left_wall','bridge_right_wall',
              'bridge_panel','bridge_seam','bridge_hole','roll_floor','roll_ceiling','roll_wall','roll_slope','roll_bridge',
              'limit_stress_-1','limit_stress_1',*[f'limit_{s}_{dt}' for s in [-1,1] for dt in [.001,.0005]]}
    missing=sorted(expected-reports.keys())
    geometric={p:json.loads((ROOT/'validation'/p).read_text()) for p in
               ['geometry_integrity_report.json','geometry_report.json','collision_sweep_report.json']}
    reset=json.loads((ROOT/'validation/reset/report.json').read_text())
    long_hold=all(reports.get(name,{}).get('final_time',0)>=60 for name in ['floor','wall','ceiling','slope'])
    screenshots=all((ROOT/f'validation/gui/{name}.png').exists() for name in ['robot_bench','bridge_robot'])
    summary={'passed':not missing and all(r['passed'] for r in reports.values()) and
                       all(r['passed'] for r in geometric.values()) and reset['passed'] and long_hold and screenshots,
             'physics_case_count':len(reports),'missing_cases':missing,'all_hold_tests_at_least_60s':long_hold,
             'physics_cases':reports,'geometry':geometric,'reset':reset,'gui_screenshots_present':screenshots,
             'max_abs_steering_deg':max(r['max_steering_deg'] for r in reports.values()),
             'runtime':'Gazebo Sim Fortress 6.18.0 / ignition-physics5 DART / sdformat12',
             'current_artifact_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in [ROOT/'models/magnetic_robot/model.sdf',ROOT/'config/robot.yaml',ROOT/'build/libMagneticRobot.so']},
             'scope':'Representative bridge locations; mass distribution and inertias approximate; no production controller.'}
    (ROOT/'validation/summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({'passed':summary['passed'],'physics_cases':len(reports),'max_steering_deg':summary['max_abs_steering_deg'],'missing':missing}))
    return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
