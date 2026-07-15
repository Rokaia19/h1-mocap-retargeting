#!/usr/bin/env python3
"""
Foot-locked pelvis solving via Mink (differential IK / QP), reused here so
the ROS2/Rviz pipeline can play back the same, more accurate pelvis motion
already validated in retarget_mink.py (worst-case foot error
0.087 vs the older hand-rolled Levenberg-Marquardt solver's 0.16).

Uses model/h1_kinematics_only.xml -- a copy of the full MJCF H1_2 model with
every geom/light/mesh asset stripped out via the MjSpec API. Mink's
FrameTask only reads body kinematic transforms (joint tree + origins), never
geometry, so this strips out the 90-file mesh dependency entirely while
producing bit-identical forward kinematics to the full model (verified: 0.0
max difference in foot position across randomized joint configurations).
"""
import os

import mujoco
import mink
import numpy as np
from ament_index_python.packages import get_package_share_directory

from h1_mocap_retarget.retarget_core import H1_JOINT_ORDER

_FOOT_BODIES = {'left': 'left_ankle_roll_link', 'right': 'right_ankle_roll_link'}


def _default_model_path():
    # Uses the package share directory (not a source-tree-relative path) so
    # this resolves correctly whether run from source or after colcon
    # install -- same pattern as csv_path/rviz_config elsewhere in this node.
    share = get_package_share_directory('h1_mocap_retarget')
    return os.path.join(share, 'model', 'h1_kinematics_only.xml')


def load_kinematics_model(model_path=None):
    return mujoco.MjModel.from_xml_path(model_path or _default_model_path())


def build_qpos_index(model):
    """joint name -> qpos address, for the 27 H1_JOINT_ORDER hinge joints."""
    idx = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name in H1_JOINT_ORDER:
            idx[name] = model.jnt_qposadr[i]
    return idx


def solve_pelvis_trajectory_mink(model, angles_rad, joint_names, reference_frame=0, verbose=True):
    """Foot-locked pelvis IK via Mink. Returns (xyz, quat_wxyz) arrays of
    shape (n_frames, 3) and (n_frames, 4) -- the pelvis freejoint's position
    and orientation (MuJoCo quaternion convention: w,x,y,z) for every frame.
    """
    configuration = mink.Configuration(model)
    qpos_idx = build_qpos_index(model)
    n_frames = angles_rad.shape[0]

    def set_leg_arm_angles(q, frame):
        for j, jname in enumerate(joint_names):
            if jname in qpos_idx:
                q[qpos_idx[jname]] = angles_rad[frame, j]
        return q

    q_ref = configuration.q.copy()
    q_ref[3] = 1.0  # identity quaternion -- pelvis at world origin for the reference frame
    q_ref = set_leg_arm_angles(q_ref, reference_frame)
    configuration.update(q_ref)
    target_left = configuration.get_transform_frame_to_world(_FOOT_BODIES['left'], 'body')
    target_right = configuration.get_transform_frame_to_world(_FOOT_BODIES['right'], 'body')

    # The URDF's pelvis body has no inherent standing height (body_pos =
    # [0,0,0]), so with the pelvis at the world origin the feet land far
    # below z=0. Shift the whole reference pose up so the lower foot sits
    # at z=0 (matches Rviz's ground/grid plane) before locking targets.
    foot_z = min(target_left.translation()[2], target_right.translation()[2])
    q_ref[2] -= foot_z
    configuration.update(q_ref)
    target_left = configuration.get_transform_frame_to_world(_FOOT_BODIES['left'], 'body')
    target_right = configuration.get_transform_frame_to_world(_FOOT_BODIES['right'], 'body')

    left_task = mink.FrameTask(_FOOT_BODIES['left'], 'body', position_cost=1.0, orientation_cost=1.0)
    right_task = mink.FrameTask(_FOOT_BODIES['right'], 'body', position_cost=1.0, orientation_cost=1.0)
    left_task.set_target(target_left)
    right_task.set_target(target_right)

    # Freeze every DOF except the pelvis freejoint's 6 -- the leg/arm angles
    # are already fully determined by the mocap data, only the pelvis is
    # unknown and needs solving.
    freeze_dofs = list(range(6, model.nv))
    freeze_task = mink.DofFreezingTask(model, freeze_dofs, gain=1.0)
    tasks = [left_task, right_task, freeze_task]

    xyz_traj = np.zeros((n_frames, 3))
    quat_traj = np.zeros((n_frames, 4))
    q = q_ref.copy()
    dt = 0.05
    max_err = 0.0
    for f in range(n_frames):
        q = set_leg_arm_angles(q, f)
        configuration.update(q)
        for _ in range(30):
            vel = mink.solve_ik(configuration, tasks, dt, solver='daqp', damping=1e-6)
            configuration.integrate_inplace(vel, dt)
            err = np.concatenate([left_task.compute_error(configuration),
                                   right_task.compute_error(configuration)])
            if np.max(np.abs(err)) < 1e-6:
                break
        max_err = max(max_err, float(np.max(np.abs(err))))
        q = configuration.q.copy()
        xyz_traj[f] = q[0:3]
        quat_traj[f] = q[3:7]

    if verbose:
        print(f'[mink_pelvis] Worst-case foot position/orientation error across all frames: {max_err:.4f} '
              f'(known limitation: this rises to ~0.087 at deep-squat frames where the knee hits its '
              f'URDF limit -- see body/limb scaling task, not yet applied)')
    return xyz_traj, quat_traj
