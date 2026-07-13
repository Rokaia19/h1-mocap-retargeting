#!/usr/bin/env python3
"""
Phase 1 (squat-only) retargeting using Mink's foot-locked IK on a MuJoCo model
of the H1_2 (handless), replacing the earlier hand-rolled Levenberg-Marquardt
pelvis solver used in the ROS2/Rviz pipeline (h1_mocap_retarget package).

Reuses the same already-validated joint-angle mapping (retarget_core's
RETARGET_MAP, including the hip_pitch/knee/ankle_pitch sign fixes worked out
via forward-kinematics checks against the URDF earlier) -- only the
pelvis-solving method changed, not the human->H1 angle correspondence.

Usage (no ROS2 node/graph needed, just the installed package):
    pip install mujoco mink --break-system-packages
    ros2 run h1_mocap_retarget retarget_mink --trial bar_01
    ros2 run h1_mocap_retarget retarget_mink --trial dumbells_01 --rate 0.5 --legs_only false

    Save an MP4 instead of (or alongside) opening the live viewer:
    pip install imageio imageio-ffmpeg --break-system-packages
    ros2 run h1_mocap_retarget retarget_mink --trial bar_01 --save_video demo_bar_01.mp4
"""
import argparse
import os
import time

import mujoco
import mujoco.viewer
import mink
import numpy as np
from ament_index_python.packages import get_package_share_directory

from h1_mocap_retarget.retarget_core import (
    parse_vicon_model_output, retarget_trial, ARM_JOINTS, H1_JOINT_ORDER)

_SHARE = get_package_share_directory('h1_mocap_retarget')
MODEL_PATH = os.path.join(_SHARE, 'model', 'h1_2_handless.xml')
CSV_PATH = os.path.join(_SHARE, 'data', 'JointAngles.txt')

_FOOT_BODIES = {'left': 'left_ankle_roll_link', 'right': 'right_ankle_roll_link'}


def build_qpos_index(model):
    """joint name -> qpos address, for the 27 H1_JOINT_ORDER hinge joints."""
    idx = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name in H1_JOINT_ORDER:
            idx[name] = model.jnt_qposadr[i]
    return idx


def solve_pelvis_trajectory_mink(model, angles_rad, joint_names, reference_frame=0, verbose=True):
    """Same job as h1_mocap_retarget's solve_pelvis_trajectory (foot-locked
    pelvis IK), but using Mink's QP-based differential IK + DofFreezingTask
    instead of a hand-rolled Levenberg-Marquardt solve. Returns the full
    (n_frames, model.nq) qpos trajectory (not just the 6 pelvis params),
    since that's what MuJoCo needs for playback."""
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
    # [0,0,0]), so with the pelvis at the world origin the feet land ~0.97m
    # *below* z=0 -- underneath the ground plane, which is why only the
    # upper body was visible. Shift the whole reference pose up so the
    # lower foot sits at z=0 (on the ground plane) before locking targets.
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

    qpos_traj = np.zeros((n_frames, model.nq))
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
        qpos_traj[f] = q

    if verbose:
        print(f'Pelvis IK done. Worst-case foot position/orientation error across all frames: {max_err:.4f}')
    return qpos_traj


def render_video(model, qpos_traj, out_path, video_fps=30.0, source_fps=60.0,
                  width=1280, height=720):
    """Render the solved qpos trajectory offscreen and save it as an MP4,
    instead of (or in addition to) the interactive viewer -- useful for
    sharing a demo clip. Subsamples frames if source_fps > video_fps so the
    output plays back at the correct (real-time) speed."""
    import imageio

    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    cam = mujoco.MjvCamera()
    cam.distance = 2.8
    cam.azimuth = 120
    cam.elevation = -15
    cam.lookat[:] = [0.0, 0.0, 0.7]

    stride = max(1, round(source_fps / video_fps))
    actual_fps = source_fps / stride
    n_frames = qpos_traj.shape[0]

    frames = []
    for f in range(0, n_frames, stride):
        data.qpos[:] = qpos_traj[f]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render())

    imageio.mimsave(out_path, frames, fps=actual_fps)
    print(f'Saved {len(frames)} frames @ {actual_fps:.1f} fps -> {out_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial', default='', help='substring: bar_01 / dumbells_01 / dumbells_02')
    ap.add_argument('--rate', type=float, default=1.0, help='playback speed multiplier')
    ap.add_argument('--fps', type=float, default=60.0, help='mocap capture rate -- confirmed from the .c3d files ROTATION:RATE')
    ap.add_argument('--legs_only', type=lambda s: s.lower() != 'false', default=True,
                     help='Phase 1: freeze arms, squat only (default true)')
    ap.add_argument('--save_video', default='', help='Path to save an MP4 demo instead of the live viewer, e.g. demo.mp4')
    ap.add_argument('--video_fps', type=float, default=30.0, help='Output video frame rate (default 30)')
    args = ap.parse_args()

    print(f'Loading {CSV_PATH}')
    trials = parse_vicon_model_output(CSV_PATH)
    trial_names = list(trials.keys())
    chosen = trial_names[0]
    if args.trial:
        matches = [t for t in trial_names if args.trial in t]
        if matches:
            chosen = matches[0]
        else:
            print(f'No trial matched "{args.trial}", available: {trial_names}. Using first trial.')
    print(f'Using trial: {chosen}')

    freeze_joints = ARM_JOINTS if args.legs_only else []
    if args.legs_only:
        print('legs_only=true (Phase 1) -- both arms held at neutral, squat only.')
    n_frames, joint_names, angles_rad, clipped = retarget_trial(trials[chosen], freeze_joints=freeze_joints)
    if clipped:
        print(f'WARNING: these joints were clamped to their URDF limits: {sorted(clipped)}')

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)

    print(f'Solving foot-locked pelvis motion via Mink for {n_frames} frames...')
    t0 = time.time()
    qpos_traj = solve_pelvis_trajectory_mink(model, angles_rad, joint_names)
    print(f'Done in {time.time() - t0:.1f}s')

    if args.save_video:
        render_video(model, qpos_traj, args.save_video,
                     video_fps=args.video_fps, source_fps=args.fps * args.rate)
        return

    data = mujoco.MjData(model)
    period = 1.0 / (args.fps * args.rate)
    print('Opening viewer -- close the window or Ctrl+C in this terminal to stop.')
    with mujoco.viewer.launch_passive(model, data) as viewer:
        frame = 0
        while viewer.is_running():
            data.qpos[:] = qpos_traj[frame]
            mujoco.mj_forward(model, data)
            viewer.sync()
            frame = (frame + 1) % n_frames
            time.sleep(period)


if __name__ == '__main__':
    main()
