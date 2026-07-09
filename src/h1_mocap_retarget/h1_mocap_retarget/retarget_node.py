#!/usr/bin/env python3
"""
ROS2 node that plays back a retargeted mocap trial as /joint_states, plus a
world->pelvis TF solved so both feet stay planted (foot-locked IK), so it can
be viewed on the H1_2 (handless) model in Rviz via robot_state_publisher.

Parameters
----------
csv_path : string
    Path to the Vicon "Model Outputs" joint-angle export.
trial : string
    Substring to match against the trial's source .c3d path (e.g. "bar_01",
    "dumbells_01", "dumbells_02"). Empty string = first trial in the file.
fps : float
    Mocap capture frame rate in Hz. Confirmed from the raw .c3d files'
    ROTATION:RATE -- 60.0 Hz for this dataset (not stored in the CSV "Model
    Outputs" export, which is why this was wrongly assumed 100.0 before).
    Override with `-p fps:=<value>` if a different trial has a different rate.
rate : float
    Playback speed multiplier (1.0 = real time).
loop : bool
    Loop the trial when it reaches the end.
legs_only : bool
    Freeze both arms at 0.0 (neutral) so only hips/knees/ankles/torso move.
use_root_motion : bool
    Broadcast a world->pelvis TF solved via foot-locked IK, so the body sinks
    into the squat with both feet held at their frame-0 ground position,
    instead of the pelvis staying pinned in place with the legs swinging
    freely underneath it. Disable to fall back to a fixed pelvis.
use_mink : bool
    Solve the foot-locked pelvis IK with Mink (QP-based differential IK,
    same method validated in mujoco_retarget/retarget_mink.py, worst-case
    foot error 0.087) instead of the original hand-rolled Levenberg-Marquardt
    solver in retarget_core (worst-case foot error 0.16). Requires
    `pip install mujoco mink --break-system-packages` in the ROS2 Python
    environment. Default true; set false to fall back to the old solver.
flip_hip_yaw, flip_hip_pitch, flip_hip_roll, flip_knee, flip_ankle_pitch,
flip_ankle_roll, flip_torso : bool
    Flip that joint's sign (applied symmetrically to left+right) for fast
    direction testing without touching code.
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

from h1_mocap_retarget.retarget_core import (
    parse_vicon_model_output, retarget_trial, solve_pelvis_trajectory,
    rpy_to_quaternion, RETARGET_MAP, ARM_JOINTS)

_FLIP_PARAM_TO_JOINTS = {
    'flip_hip_yaw':     ['left_hip_yaw_joint', 'right_hip_yaw_joint'],
    'flip_hip_pitch':   ['left_hip_pitch_joint', 'right_hip_pitch_joint'],
    'flip_hip_roll':    ['left_hip_roll_joint', 'right_hip_roll_joint'],
    'flip_knee':        ['left_knee_joint', 'right_knee_joint'],
    'flip_ankle_pitch': ['left_ankle_pitch_joint', 'right_ankle_pitch_joint'],
    'flip_ankle_roll':  ['left_ankle_roll_joint', 'right_ankle_roll_joint'],
    'flip_torso':       ['torso_joint'],
}


class MocapRetargetPublisher(Node):
    def __init__(self):
        super().__init__('mocap_retarget_publisher')

        share = get_package_share_directory('h1_mocap_retarget')
        self.declare_parameter('csv_path', os.path.join(share, 'data', 'JointAngles.txt'))
        self.declare_parameter('trial', '')
        self.declare_parameter('fps', 60.0)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('legs_only', True)
        self.declare_parameter('use_root_motion', True)
        self.declare_parameter('use_mink', True)
        for p in _FLIP_PARAM_TO_JOINTS:
            self.declare_parameter(p, False)

        csv_path = self.get_parameter('csv_path').value
        trial_filter = self.get_parameter('trial').value
        self.fps = float(self.get_parameter('fps').value)
        rate = float(self.get_parameter('rate').value)
        self.loop = bool(self.get_parameter('loop').value)
        legs_only = bool(self.get_parameter('legs_only').value)
        self.use_root_motion = bool(self.get_parameter('use_root_motion').value)
        self.use_mink = bool(self.get_parameter('use_mink').value)

        sign_overrides = {}
        flipped = []
        for pname, joints in _FLIP_PARAM_TO_JOINTS.items():
            if bool(self.get_parameter(pname).value):
                flipped.append(pname)
                for jname in joints:
                    if jname in RETARGET_MAP:
                        default_sign = RETARGET_MAP[jname][2]
                        sign_overrides[jname] = -1.0 * default_sign
        if flipped:
            self.get_logger().info(f'Sign flips active: {flipped}')

        freeze_joints = ARM_JOINTS if legs_only else []
        if legs_only:
            self.get_logger().info(
                'legs_only=true -- both arms held at neutral (0.0), testing the squat only.')

        self.get_logger().info(f'Loading mocap angle data from {csv_path}')
        trials = parse_vicon_model_output(csv_path)
        trial_names = list(trials.keys())

        chosen = trial_names[0]
        if trial_filter:
            matches = [t for t in trial_names if trial_filter in t]
            if not matches:
                self.get_logger().warn(
                    f'No trial matched "{trial_filter}", available: {trial_names}. '
                    f'Falling back to first trial.')
            else:
                chosen = matches[0]

        self.get_logger().info(f'Using trial: {chosen}')
        n_frames, joint_names, angles_rad, clipped = retarget_trial(
            trials[chosen], sign_overrides=sign_overrides, freeze_joints=freeze_joints)
        self.joint_names = joint_names
        self.angles = angles_rad
        self.n_frames = n_frames

        if clipped:
            self.get_logger().warn(
                f'These joints hit their URDF limits and were clamped -- likely a sign/scale '
                f'mismatch worth checking visually: {sorted(clipped)}')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pelvis_xyz = None
        self.pelvis_quat_xyzw = None

        # Publish the starting pose immediately, before the (several-second)
        # pelvis solve below, so Rviz shows the robot right away instead of a
        # blank scene during the wait -- makes it obvious nothing has hung.
        self._publish_frame(0)

        if self.use_root_motion:
            self.get_logger().info(
                'Solving foot-locked pelvis motion (both feet held at their frame-0 '
                f'position) for {n_frames} frames -- this takes roughly '
                f'{n_frames * 0.010:.0f}-{n_frames * 0.015:.0f}s, the robot in Rviz will '
                f'hold its starting pose until this finishes...')
            if self.use_mink:
                from h1_mocap_retarget.mink_pelvis import (
                    load_kinematics_model, solve_pelvis_trajectory_mink)
                self.get_logger().info('use_mink=true -- solving via Mink (QP differential IK).')
                mink_model = load_kinematics_model()
                xyz, quat_wxyz = solve_pelvis_trajectory_mink(mink_model, angles_rad, joint_names)
                self.pelvis_xyz = xyz
                # MuJoCo quaternion order is (w,x,y,z); ROS TransformStamped wants (x,y,z,w).
                self.pelvis_quat_xyzw = quat_wxyz[:, [1, 2, 3, 0]]
            else:
                self.get_logger().info('use_mink=false -- solving via the original Levenberg-Marquardt solver.')
                pelvis_traj = solve_pelvis_trajectory(angles_rad, joint_names)
                self.pelvis_xyz = pelvis_traj[:, 0:3]
                self.pelvis_quat_xyzw = np.array([
                    rpy_to_quaternion(roll, pitch, yaw)
                    for _, _, _, roll, pitch, yaw in pelvis_traj])
            self.get_logger().info(
                f'Pelvis IK done. Z range {self.pelvis_xyz[:,2].min():.3f} to '
                f'{self.pelvis_xyz[:,2].max():.3f} m.')

        self.get_logger().info(
            f'Loaded {n_frames} frames across {len(joint_names)} joints. '
            f'Assuming capture rate = {self.fps} Hz (confirm this matches your mocap setup).')

        self.frame_idx = 0
        self.finished_logged = False
        period = 1.0 / (self.fps * rate)
        self.timer = self.create_timer(period, self.update)

    def _publish_frame(self, idx):
        now = self.get_clock().now().to_msg()

        msg = JointState()
        msg.header.stamp = now
        msg.name = self.joint_names
        msg.position = self.angles[idx].tolist()
        self.pub.publish(msg)

        if self.pelvis_xyz is not None:
            x, y, z = self.pelvis_xyz[idx]
            qx, qy, qz, qw = self.pelvis_quat_xyzw[idx]
        else:
            # Pelvis IK hasn't run yet (this is the immediate frame-0 publish
            # before the solve) -- hold at the origin rather than skip the
            # transform entirely, so robot_state_publisher/Rviz always has a
            # valid world->pelvis frame to render against.
            x, y, z = 0.0, 0.0, 0.0
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'pelvis'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)

    def update(self):
        if self.frame_idx >= self.n_frames:
            if self.loop:
                self.frame_idx = 0
            else:
                if not self.finished_logged:
                    self.get_logger().info('Trial finished (loop=false).')
                    self.finished_logged = True
                return

        self._publish_frame(self.frame_idx)
        self.frame_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = MocapRetargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
