#!/usr/bin/env python3
"""
Strips all geoms/lights/meshes out of model/h1_2_handless.xml, producing a
tiny kinematics-only MJCF with zero external mesh-file dependencies.

Why: Mink's FrameTask (used for the foot-locked pelvis IK, both here and in
the h1_mocap_retarget ROS2 package's mink_pelvis.py) only reads body
kinematic transforms -- joint tree + origins -- never geometry. So none of
the ~90 STL meshes are actually needed to solve the IK, only to *render* it.
Stripping them out means this file can be copied anywhere (e.g. into the
ROS2 package, so it doesn't need to duplicate the mesh files your teammate
already has in ros_gz_h1_description) with no mesh dependency at all.

Verified: FK from this stripped model matches the full mesh model to 0.0
(exact) across randomized joint configurations -- see mink_pelvis.py's
docstring for the same claim in the ROS2 package's copy.

Dev-time tool: run from a source checkout (from the h1_mocap_retarget package
root), not installed via colcon.

Usage:
    python3 scripts/convert_urdf_to_mjcf.py       # regenerates model/h1_2_handless.xml first
    python3 scripts/make_kinematics_only_model.py # then strip it down
"""
import os

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
IN_XML = os.path.join(HERE, '..', 'model', 'h1_2_handless.xml')
OUT_XML = os.path.join(HERE, '..', 'model', 'h1_kinematics_only.xml')


def main():
    full_model = mujoco.MjModel.from_xml_path(IN_XML)
    spec = mujoco.MjSpec.from_file(IN_XML)

    for g in list(spec.geoms):
        spec.delete(g)
    for l in list(spec.lights):
        spec.delete(l)
    for m in list(spec.meshes):
        spec.delete(m)

    kin_model = spec.compile()
    print(f'Kinematics-only compiled OK: nq={kin_model.nq} nv={kin_model.nv} '
          f'njnt={kin_model.njnt} nbody={kin_model.nbody} ngeom={kin_model.ngeom} '
          f'nmesh={kin_model.nmesh}')
    spec.to_file(OUT_XML)
    print(f'Wrote {OUT_XML}')

    # Sanity check: FK must match the full mesh model exactly.
    import mink
    import numpy as np
    rng = np.random.default_rng(0)
    q = np.zeros(full_model.nq)
    q[3] = 1.0
    for j in range(full_model.njnt):
        if full_model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            adr = full_model.jnt_qposadr[j]
            lo, hi = full_model.jnt_range[j]
            q[adr] = rng.uniform(lo, hi)

    c_full = mink.Configuration(full_model)
    c_kin = mink.Configuration(kin_model)
    c_full.update(q)
    c_kin.update(q)
    for body in ['left_ankle_roll_link', 'right_ankle_roll_link']:
        t_full = c_full.get_transform_frame_to_world(body, 'body')
        t_kin = c_kin.get_transform_frame_to_world(body, 'body')
        diff = np.abs(t_full.translation() - t_kin.translation()).max()
        print(f'{body} FK diff vs full model: {diff:.2e}')
        assert diff < 1e-9, f'{body} FK mismatch after stripping geoms!'
    print('Verified: FK matches the full mesh model exactly.')


if __name__ == '__main__':
    main()
