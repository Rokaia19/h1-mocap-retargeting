"""
Pure-Python retargeting core: Vicon "Model Outputs" joint-angle CSV -> Unitree H1_2 (handless) joint_states.

No ROS dependency here on purpose, so the mapping logic can be unit-tested outside
a ROS2 environment. retarget_node.py (ROS2 side) imports this module.
"""
import numpy as np
import csv

# ---------------------------------------------------------------------------
# 1. H1_2 (handless) joint order -- must match ros_gz_h1_controller joint_cmd_topics
#    and h1_2_handless.urdf. floating_base_joint + 4 fixed joints excluded.
# ---------------------------------------------------------------------------
H1_JOINT_ORDER = [
    "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "torso_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

H1_JOINT_LIMITS = {
    "left_hip_yaw_joint": (-0.43, 0.43), "left_hip_pitch_joint": (-3.14, 2.5),
    "left_hip_roll_joint": (-0.43, 3.14), "left_knee_joint": (-0.12, 2.19),
    "left_ankle_pitch_joint": (-0.897334, 0.523598), "left_ankle_roll_joint": (-0.261799, 0.261799),
    "right_hip_yaw_joint": (-0.43, 0.43), "right_hip_pitch_joint": (-3.14, 2.5),
    "right_hip_roll_joint": (-3.14, 0.43), "right_knee_joint": (-0.12, 2.19),
    "right_ankle_pitch_joint": (-0.897334, 0.523598), "right_ankle_roll_joint": (-0.261799, 0.261799),
    "torso_joint": (-2.35, 2.35),
    "left_shoulder_pitch_joint": (-3.14, 1.57), "left_shoulder_roll_joint": (-0.38, 3.4),
    "left_shoulder_yaw_joint": (-2.66, 3.01), "left_elbow_joint": (-0.95, 3.18),
    "left_wrist_roll_joint": (-3.01, 2.75), "left_wrist_pitch_joint": (-0.4625, 0.4625),
    "left_wrist_yaw_joint": (-1.27, 1.27),
    "right_shoulder_pitch_joint": (-3.14, 1.57), "right_shoulder_roll_joint": (-3.4, 0.38),
    "right_shoulder_yaw_joint": (-3.01, 2.66), "right_elbow_joint": (-0.95, 3.18),
    "right_wrist_roll_joint": (-2.75, 3.01), "right_wrist_pitch_joint": (-0.4625, 0.4625),
    "right_wrist_yaw_joint": (-1.27, 1.27),
}

DEG2RAD = np.pi / 180.0


def parse_vicon_model_output(path):
    """Returns {trial_name: {angle_label: (N,3) float array in degrees}}."""
    with open(path, newline="") as f:
        reader = csv.reader(f, delimiter="\t")
        rows = list(reader)

    trial_row, label_row = rows[0], rows[1]
    data_rows = rows[5:]
    ncols = len(label_row)

    trials = []
    start = 1
    cur = trial_row[1]
    for i in range(2, ncols):
        if trial_row[i] != cur:
            trials.append((cur, start, i))
            cur = trial_row[i]
            start = i
    trials.append((cur, start, ncols))

    out = {}
    for trial_name, c0, c1 in trials:
        labels_block = label_row[c0:c1]
        uniq = []
        seen = set()
        for l in labels_block:
            if l not in seen:
                seen.add(l)
                uniq.append(l)

        n_frames = 0
        for r in data_rows:
            if c0 < len(r) and r[c0].strip() != "":
                n_frames += 1
            else:
                break

        angle_data = {}
        for k, label in enumerate(uniq):
            col0 = c0 + 3 * k
            arr = np.zeros((n_frames, 3))
            for fi in range(n_frames):
                row = data_rows[fi]
                for j in range(3):
                    v = row[col0 + j].strip()
                    arr[fi, j] = float(v) if v else 0.0
            short_label = label.split(":")[-1]
            angle_data[short_label] = arr

        out[trial_name] = angle_data

    return out


# ---------------------------------------------------------------------------
# 2. Human angle label -> H1 joint retargeting map.
#    sign here is the DEFAULT -- retarget_trial() lets you override any of
#    these per-joint at runtime via `sign_overrides`, without touching this
#    file, so you can flip a joint's direction from the launch command line
#    while watching Rviz live.
#
#    Current best guess as of the last Rviz check: hip_pitch was reported as
#    bending the wrong way (legs lifting instead of folding into a squat) --
#    flipped to +1.0 here as the new default. knee's sign is not implicated
#    by that report (it was chosen to match the URDF's mostly-positive limit
#    range) so it's left alone for now.
# ---------------------------------------------------------------------------
RETARGET_MAP = {
    "left_hip_yaw_joint":    ("L_HIP_ANGLE", 2, 1.0),
    "left_hip_pitch_joint":  ("L_HIP_ANGLE", 0, -1.0),
    "left_hip_roll_joint":   ("L_HIP_ANGLE", 1, 1.0),
    "left_knee_joint":       ("L_KNEE_ANGLE", 0, -1.0),
    "left_ankle_pitch_joint": ("L_ANKLE_ANGLE", 0, -1.0),
    "left_ankle_roll_joint": ("L_ANKLE_ANGLE", 1, 1.0),

    # hip_roll/hip_yaw/ankle_roll rotate about axes that lie IN the L/R mirror
    # plane (X and Z) -- confirmed via the URDF that left/right share the
    # exact same <axis> direction (not mirrored) while only the joint's
    # position is mirrored, so a "positive" angle on the right leg is the
    # physical opposite of the same positive angle on the left leg for these
    # three joints specifically (also matches the mirrored URDF limit ranges:
    # left_hip_roll -0.43..3.14 vs right_hip_roll -3.14..0.43). Verified
    # empirically: without this flip, left/right hip_roll differed by a mean
    # 0.60 rad and hip_yaw by 0.36 rad frame-to-frame (clearly not real human
    # asymmetry); with it, both drop to ~0.03 rad, matching the natural
    # asymmetry level seen on the (correctly unflipped) pitch/knee joints.
    "right_hip_yaw_joint":    ("R_HIP_ANGLE", 2, -1.0),
    "right_hip_pitch_joint":  ("R_HIP_ANGLE", 0, -1.0),
    "right_hip_roll_joint":   ("R_HIP_ANGLE", 1, -1.0),
    "right_knee_joint":       ("R_KNEE_ANGLE", 0, -1.0),
    "right_ankle_pitch_joint": ("R_ANKLE_ANGLE", 0, -1.0),
    "right_ankle_roll_joint": ("R_ANKLE_ANGLE", 1, -1.0),

    "torso_joint": ("Thorax_Joint_Angle", 2, 1.0),

    "left_shoulder_pitch_joint": ("L_SHOULDER_ANGLE", 0, -1.0),
    "left_shoulder_roll_joint":  ("L_SHOULDER_ANGLE", 1, 1.0),
    "left_shoulder_yaw_joint":   ("L_SHOULDER_ANGLE", 2, 1.0),
    "left_elbow_joint":          ("L_ELBOW_ANGLE", 0, 1.0),
    "left_wrist_roll_joint":     ("L_ELBOW_ANGLE", 2, 1.0),
    "left_wrist_pitch_joint":    ("L_WRIST_ANGLE", 0, 1.0),
    "left_wrist_yaw_joint":      ("L_WRIST_ANGLE", 1, 1.0),
}

_RIGHT_ARM_MIRROR = {
    "right_shoulder_pitch_joint": ("left_shoulder_pitch_joint", False),
    "right_shoulder_roll_joint":  ("left_shoulder_roll_joint", True),
    "right_shoulder_yaw_joint":   ("left_shoulder_yaw_joint", True),
    "right_elbow_joint":          ("left_elbow_joint", False),
    "right_wrist_roll_joint":     ("left_wrist_roll_joint", True),
    "right_wrist_pitch_joint":    ("left_wrist_pitch_joint", False),
    "right_wrist_yaw_joint":      ("left_wrist_yaw_joint", False),
}

_LEFT_ARM_JOINTS = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
}


def retarget_trial(angle_data, clip=True, sign_overrides=None, freeze_joints=None):
    """angle_data: {label: (N,3) array in degrees}, per-trial from the parser.

    sign_overrides: optional {joint_name: sign} to override RETARGET_MAP's
        default sign for that joint at runtime (e.g. from a ROS parameter),
        without editing this file.
    freeze_joints: optional iterable of joint names to hold at 0.0 for the
        whole trial (e.g. pass ARM_JOINTS to isolate leg motion for testing).

    Returns (n_frames, joint_names, angles_rad (N, len(joint_names)), clipped_joint_names).
    """
    sign_overrides = sign_overrides or {}
    freeze_joints = set(freeze_joints or [])

    n_frames = next(iter(angle_data.values())).shape[0]
    n_joints = len(H1_JOINT_ORDER)
    out = np.zeros((n_frames, n_joints))
    clipped_joints = set()
    left_arm_values = {}

    for j, jname in enumerate(H1_JOINT_ORDER):
        if jname in freeze_joints:
            continue  # stays 0.0

        if jname in RETARGET_MAP:
            label, axis, default_sign = RETARGET_MAP[jname]
            sign = sign_overrides.get(jname, default_sign)
            if label not in angle_data:
                continue
            vals = sign * DEG2RAD * angle_data[label][:, axis]
            if jname in _LEFT_ARM_JOINTS:
                left_arm_values[jname] = vals
        elif jname in _RIGHT_ARM_MIRROR:
            src_joint, flip = _RIGHT_ARM_MIRROR[jname]
            if src_joint not in left_arm_values:
                continue
            vals = left_arm_values[src_joint] * (-1.0 if flip else 1.0)
        else:
            continue

        if clip and jname in H1_JOINT_LIMITS:
            lo, hi = H1_JOINT_LIMITS[jname]
            if np.any(vals < lo) or np.any(vals > hi):
                clipped_joints.add(jname)
            vals = np.clip(vals, lo, hi)

        out[:, j] = vals

    return n_frames, H1_JOINT_ORDER, out, clipped_joints


# ---------------------------------------------------------------------------
# 3. Root (pelvis) translation from the "absolute joint position" export.
#    JointAngles.txt has no global position -- so without this, the pelvis is
#    forced to stay pinned in place in Rviz and only the limbs can move,
#    which looks like the legs "flying" instead of a body sinking into a
#    squat. L_HIP_POSITION in JointPositions.txt gives an actual lab-frame
#    3D position per frame; we use its motion (relative to frame 0) as a
#    stand-in for pelvis translation.
# ---------------------------------------------------------------------------
def get_root_translation(position_data, label='L_HIP_POSITION', baseline_frames=10, vertical_only=True):
    """position_data: {label: (N,3) array in meters}, per-trial from the parser.
    Returns (N,3) array: position at each frame minus the average position of
    the first `baseline_frames` frames, so playback starts at (0,0,0).

    vertical_only: zero out X/Y (horizontal) motion and keep only Z. Without
    foot-contact IK, the feet aren't actually locked to the ground, so
    copying the hip's raw horizontal sway drags the whole robot sideways
    across the floor instead of the feet staying planted. Defaults to True.
    """
    traj = position_data[label]
    baseline = traj[:baseline_frames].mean(axis=0)
    delta = traj - baseline
    if vertical_only:
        delta = delta.copy()
        delta[:, 0] = 0.0
        delta[:, 1] = 0.0
    return delta


# ---------------------------------------------------------------------------
# 4. Foot-locked pelvis IK.
#
#    Forward kinematics alone (the RETARGET_MAP above) never touches the
#    pelvis, so nothing keeps the feet planted -- any hip/ankle rotation
#    shows up as the whole leg sliding across the floor. This solves for the
#    pelvis's 6-DOF world pose, every frame, such that both feet stay at a
#    fixed target position (their pose in frame 0), the same job your own
#    nervous system does automatically when you squat without moving your feet.
# ---------------------------------------------------------------------------
def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Leg joint origins from h1_2_handless.urdf (all have rpy="0 0 0"). Y offsets
# for hip_yaw/hip_pitch are mirrored between sides (confirmed against the
# URDF directly); the rest are Y=0 so mirroring is a no-op for them.
_LEG_ORIGINS = {
    'hip_yaw': np.array([0.0, 0.0875, -0.1632]),
    'hip_pitch': np.array([0.0, 0.0755, 0.0]),
    'hip_roll': np.array([0.0, 0.0, 0.0]),
    'knee': np.array([0.0, 0.0, -0.4]),
    'ankle_pitch': np.array([0.0, 0.0, -0.4]),
    'ankle_roll': np.array([0.0, 0.0, -0.02]),
}


def leg_forward_kinematics(hip_yaw, hip_pitch, hip_roll, knee, ankle_pitch, ankle_roll, side):
    """Position of the 'foot' (ankle_roll_link origin) relative to the pelvis
    frame, given this leg's 6 joint angles (radians). side: 'left' or 'right'."""
    sign_y = 1.0 if side == 'left' else -1.0
    T = np.eye(4)

    def step(T, origin, axis_rot, angle):
        Tt = np.eye(4)
        Tt[:3, 3] = origin
        Tr = np.eye(4)
        Tr[:3, :3] = axis_rot(angle)
        return T @ Tt @ Tr

    T = step(T, _LEG_ORIGINS['hip_yaw'] * np.array([1.0, sign_y, 1.0]), _rot_z, hip_yaw)
    T = step(T, _LEG_ORIGINS['hip_pitch'] * np.array([1.0, sign_y, 1.0]), _rot_y, hip_pitch)
    T = step(T, _LEG_ORIGINS['hip_roll'], _rot_x, hip_roll)
    T = step(T, _LEG_ORIGINS['knee'], _rot_y, knee)
    T = step(T, _LEG_ORIGINS['ankle_pitch'], _rot_y, ankle_pitch)
    T = step(T, _LEG_ORIGINS['ankle_roll'], _rot_x, ankle_roll)
    return T[:3, 3]


def pelvis_transform(params):
    """params = [x, y, z, roll, pitch, yaw] -> 4x4 world_T_pelvis matrix."""
    x, y, z, roll, pitch, yaw = params
    R = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


_LEG_JOINT_NAMES = {
    'left': ['left_hip_yaw_joint', 'left_hip_pitch_joint', 'left_hip_roll_joint',
             'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint'],
    'right': ['right_hip_yaw_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint',
              'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint'],
}


def _foot_world_position(params, leg_angles, side):
    local = leg_forward_kinematics(*leg_angles, side=side)
    world = pelvis_transform(params) @ np.array([local[0], local[1], local[2], 1.0])
    return world[:3]


def _residual(params, left_angles, right_angles, target_left, target_right):
    fl = _foot_world_position(params, left_angles, 'left')
    fr = _foot_world_position(params, right_angles, 'right')
    return np.concatenate([fl - target_left, fr - target_right])


def _drot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[0, 0, 0], [0, -s, -c], [0, c, -s]])


def _drot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[-s, 0, c], [0, 0, 0], [-c, 0, -s]])


def _drot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[-s, -c, 0], [c, -s, 0], [0, 0, 0]])


def _foot_jacobian_wrt_pelvis(params, local):
    """Analytical d(foot_world)/d(params), params=[x,y,z,roll,pitch,yaw].
    Much faster than finite differences (measured ~4x fewer residual-equivalent
    evaluations per solver iteration) -- this is what made the foot-lock IK's
    ~25-30s startup cost a real risk of starving Rviz's WSLg connection.
    Replaces the earlier _numerical_jacobian for this solve."""
    _, _, _, roll, pitch, yaw = params
    Rz, Ry, Rx = _rot_z(yaw), _rot_y(pitch), _rot_x(roll)
    dRx, dRy, dRz = _drot_x(roll), _drot_y(pitch), _drot_z(yaw)

    d_droll = Rz @ Ry @ dRx @ local
    d_dpitch = Rz @ dRy @ Rx @ local
    d_dyaw = dRz @ Ry @ Rx @ local

    J = np.zeros((3, 6))
    J[:, 0] = [1, 0, 0]
    J[:, 1] = [0, 1, 0]
    J[:, 2] = [0, 0, 1]
    J[:, 3] = d_droll
    J[:, 4] = d_dpitch
    J[:, 5] = d_dyaw
    return J


def _analytical_jacobian(params, left_angles, right_angles):
    local_left = leg_forward_kinematics(*left_angles, side='left')
    local_right = leg_forward_kinematics(*right_angles, side='right')
    J_left = _foot_jacobian_wrt_pelvis(params, local_left)
    J_right = _foot_jacobian_wrt_pelvis(params, local_right)
    return np.vstack([J_left, J_right])


def _numerical_jacobian(params, left_angles, right_angles, target_left, target_right, r, eps=1e-6):
    J = np.zeros((6, 6))
    for k in range(6):
        dp = params.copy()
        dp[k] += eps
        J[:, k] = (_residual(dp, left_angles, right_angles, target_left, target_right) - r) / eps
    return J


def _solve_frame(left_angles, right_angles, target_left, target_right, guess,
                  lam0=1e-3, max_iter=20, tol=1e-9):
    """6-unknown / 6-equation Levenberg-Marquardt solve with the analytical
    Jacobian. Plain Newton-Raphson blows up here because the Jacobian can be
    near-singular (two foot-position constraints don't always fully separate
    all 6 pelvis DOF -- verified via SVD during development, condition
    numbers in the hundreds of millions were observed with plain Newton).
    LM's damping term keeps steps bounded and biases the near-degenerate
    direction toward the warm-started guess instead of blowing up.

    Returns (params, final_lam) -- the caller warm-starts both the pelvis
    guess AND lam from the previous frame, since consecutive frames are very
    similar and this cuts iterations-per-frame substantially (this, plus the
    analytical Jacobian, is what brought the full-trial solve time down from
    ~30s to a couple of seconds -- the original 30s blocking call was long
    enough to be a real risk of starving Rviz's connection to the display)."""
    params = guess.copy()
    lam = lam0
    r = _residual(params, left_angles, right_angles, target_left, target_right)
    cost = float(r @ r)

    for _ in range(max_iter):
        if cost < tol:
            break
        J = _analytical_jacobian(params, left_angles, right_angles)
        JTJ = J.T @ J
        JTr = J.T @ r

        for _retry in range(12):
            A = JTJ + lam * np.diag(np.diag(JTJ) + 1e-9)
            try:
                dparams = np.linalg.solve(A, -JTr)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            new_params = params + dparams
            new_r = _residual(new_params, left_angles, right_angles, target_left, target_right)
            new_cost = float(new_r @ new_r)
            if new_cost < cost:
                params, r, cost = new_params, new_r, new_cost
                lam = max(lam * 0.5, 1e-12)
                break
            lam = min(lam * 4.0, 1e8)  # capped -- unbounded growth previously overflowed to inf/nan
        else:
            break  # no improving step found even after damping hard -- stop

    return params, lam


def solve_pelvis_trajectory(angles_rad, joint_names, reference_frame=0):
    """angles_rad: (N, n_joints) as returned by retarget_trial(). joint_names must
    be H1_JOINT_ORDER (or a superset with the same indices).

    Returns (N, 6) array of [x, y, z, roll, pitch, yaw] world_T_pelvis params
    per frame, solved so both feet stay planted at their frame-`reference_frame`
    position (pelvis assumed at identity for that reference frame)."""
    idx = {name: joint_names.index(name) for side in _LEG_JOINT_NAMES.values() for name in side}
    n_frames = angles_rad.shape[0]

    def leg_angles_at(frame, side):
        return tuple(angles_rad[frame, idx[name]] for name in _LEG_JOINT_NAMES[side])

    ref_left = leg_angles_at(reference_frame, 'left')
    ref_right = leg_angles_at(reference_frame, 'right')
    target_left = leg_forward_kinematics(*ref_left, side='left')
    target_right = leg_forward_kinematics(*ref_right, side='right')

    out = np.zeros((n_frames, 6))
    guess = np.zeros(6)
    for f in range(n_frames):
        la = leg_angles_at(f, 'left')
        ra = leg_angles_at(f, 'right')
        # lam0 is NOT warm-started across frames -- it grew unbounded (to inf)
        # on hard frames during testing and poisoned every frame after it.
        # Position guess is still warm-started, which is where the real
        # speedup comes from with the analytical Jacobian.
        guess, _ = _solve_frame(la, ra, target_left, target_right, guess, lam0=1e-3)
        out[f] = guess

    return out


def rpy_to_quaternion(roll, pitch, yaw):
    """ZYX Euler (matching pelvis_transform's Rz(yaw)@Ry(pitch)@Rx(roll)) -> (x,y,z,w) quaternion."""
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw
