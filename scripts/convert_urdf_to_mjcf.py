#!/usr/bin/env python3
"""
One-time conversion: ros_gz_h1_description's h1_2_handless.urdf -> a standalone
MuJoCo MJCF model, for use with Mink-based retargeting (h1_mocap_retarget/retarget_mink.py).

Dev-time tool: run from a source checkout (`python3 scripts/convert_urdf_to_mjcf.py`
from the h1_mocap_retarget package root), not installed via colcon. Regenerates
model/h1_2_handless.xml from ../ros_gz_h1_description's URDF.

Two fixes were needed beyond a plain `mujoco.MjSpec.from_file()`:
1. The URDF's mesh <geometry> tags use ROS `package://ros_gz_h1_description/...`
   URIs, which MuJoCo's URDF importer doesn't resolve. Rewritten to bare
   filenames (the URDF already has a `<mujoco><compiler meshdir="meshes"/>`
   block prepared by the original authors for exactly this conversion).
2. MuJoCo's URDF importer welds the root body (pelvis) rigidly to the world
   by default -- URDF's `floating_base_joint` (type="floating") isn't
   recognized. A freejoint is added explicitly to the pelvis body so it can
   actually move in space (needed for the pelvis IK / Mink tasks).

Verified against the hand-written FK in h1_mocap_retarget/retarget_core.py
(leg_forward_kinematics) -- foot positions matched to ~1e-16 for several
random joint-angle configurations.
"""
import os
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_URDF = os.path.join(
    HERE, '..', '..', 'ros_gz_h1_description', 'models', 'h1_ign', 'h1_2_handless.urdf')
SRC_MESHES = os.path.join(HERE, '..', '..', 'ros_gz_h1_description', 'models', 'h1_ign', 'meshes')
OUT_DIR = os.path.join(HERE, '..', 'model')
OUT_XML = os.path.join(OUT_DIR, 'h1_2_handless.xml')
OUT_MESHES = os.path.join(OUT_DIR, 'meshes')


def main():
    with open(SRC_URDF) as f:
        text = f.read()
    fixed = text.replace('package://ros_gz_h1_description/models/h1_ign/meshes/', '')

    tmp_urdf = os.path.join(OUT_DIR, '_tmp_fixed.urdf')
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(tmp_urdf, 'w') as f:
        f.write(fixed)

    if not os.path.isdir(OUT_MESHES):
        import shutil
        shutil.copytree(SRC_MESHES, OUT_MESHES)

    spec = mujoco.MjSpec.from_file(tmp_urdf)
    pelvis = spec.body('pelvis')
    pelvis.add_freejoint()
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], pos=[0, 0, 0],
                             rgba=[0.5, 0.5, 0.55, 1.0])

    # The URDF had no <light> elements at all, and the default headlight
    # alone rendered nearly black -- add explicit lights so the model is
    # actually visible in the viewer.
    spec.visual.headlight.ambient = [0.4, 0.4, 0.4]
    spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
    spec.visual.headlight.specular = [0.3, 0.3, 0.3]
    spec.worldbody.add_light(pos=[1, 1, 3], dir=[-0.3, -0.3, -1],
                              type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                              diffuse=[0.7, 0.7, 0.7], ambient=[0.2, 0.2, 0.2])
    spec.worldbody.add_light(pos=[-1, -1, 3], dir=[0.3, 0.3, -1],
                              type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                              diffuse=[0.5, 0.5, 0.5])

    # Default offscreen framebuffer is 640x480 -- bump it so
    # mujoco.Renderer(width=1280, height=720) (used for --save_video demo
    # exports) doesn't fail with a "framebuffer too small" error.
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720

    # sanity compile before writing out
    model = spec.compile()
    print(f'Compiled OK: nq={model.nq} nv={model.nv} njnt={model.njnt} nbody={model.nbody}')

    spec.to_file(OUT_XML)
    with open(tmp_urdf, 'w') as f:
        f.write('<!-- scratch file from conversion, safe to ignore -->')
    print(f'Wrote {OUT_XML}')


if __name__ == '__main__':
    main()
