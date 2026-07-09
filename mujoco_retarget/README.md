# mujoco_retarget

Phase 1 (squat-only) retargeting on a MuJoCo model of the H1_2 (handless),
using Mink for foot-locked pelvis IK. This is a from-scratch parallel path to
the ROS2/Rviz pipeline in `../src/h1_mocap_retarget/` -- that one still works
and is a useful cross-check, but this one:

- Locks both feet's **position AND orientation** (via `mink.FrameTask`), not
  just position like the custom solver did -- should further help with the
  earlier heel-lifting complaint.
- Uses a proper QP-based differential IK solver (`daqp` backend) instead of a
  hand-rolled Levenberg-Marquardt loop -- more numerically robust, no risk of
  the divergence/overflow bugs hit while building the custom version.
- Needs **no ROS2 at all** for this step -- just `pip install mujoco mink`
  and run the script directly. No colcon build, no `source install/setup.bash`.

The human->H1 joint-angle mapping itself (`retarget_core.py`, copied from
`h1_mocap_retarget`) is unchanged -- same `RETARGET_MAP`, same hip_pitch/
knee/ankle_pitch sign fixes already worked out and verified there.

## Setup

    pip install mujoco mink --break-system-packages

(`--break-system-packages` needed on the same externally-managed Ubuntu
Python install as before.)

`data/` and `model/` are not tracked in this repo -- they're either raw mocap
data or assets regenerable from `../src/ros_gz_h1_description`, which you
already have, so there's no point duplicating them here. Before running:

1. Drop your copy of `JointAngles.txt` into `data/` (same format as the
   Vicon "Model Outputs" CSV export used throughout).
2. Generate the MuJoCo model from your local URDF:

       python3 convert_urdf_to_mjcf.py        # -> model/h1_2_handless.xml + model/meshes/
       python3 make_kinematics_only_model.py  # -> model/h1_kinematics_only.xml (optional, only
                                               #    needed if you're also wiring up the ROS2 bridge)

## Run

    cd mujoco_retarget
    python3 retarget_mink.py

Options:

    python3 retarget_mink.py --trial dumbells_01 --rate 0.5
    python3 retarget_mink.py --legs_only false   # bring the arms back in

A viewer window should open showing the H1 model on a ground plane. Same
caveat as Rviz: this needs a GUI on your WSL (WSLg on Windows 11, or an X
server on Windows 10) -- if the window doesn't appear, that's the first thing
to check.

## The MuJoCo model (`model/h1_2_handless.xml`)

Converted from `../src/ros_gz_h1_description/models/h1_ign/h1_2_handless.urdf`
by `convert_urdf_to_mjcf.py`. Two fixes were needed beyond a plain import:

1. The URDF's mesh paths use ROS `package://` URIs, which MuJoCo doesn't
   resolve -- rewritten to bare filenames (the URDF already had a
   `<mujoco><compiler meshdir="meshes"/></mujoco>` block prepared by the
   original authors for exactly this conversion, marked "uncomment when
   convert to mujoco").
2. MuJoCo's URDF importer welds the root body (pelvis) rigidly to the world
   by default -- URDF's `floating_base_joint` (type="floating") isn't
   recognized by it. A freejoint was added explicitly to the pelvis body.

Verified: with the freejoint added, forward kinematics from this MJCF model
matched the hand-written FK in `retarget_core.leg_forward_kinematics` to
~1e-16 (floating point noise) across several random joint-angle test cases.

Re-run the conversion if the source URDF ever changes:

    python3 convert_urdf_to_mjcf.py

## Sign bug found and fixed: right_hip_roll/hip_yaw/ankle_roll

The dominant cause of the visible foot slide (not scaling, not the knee
limit -- see below) was a retargeting bug: `right_hip_roll_joint`,
`right_hip_yaw_joint`, and `right_ankle_roll_joint` were copying the same
sign as their `L_HIP_ANGLE`/`L_ANKLE_ANGLE` counterparts, but the URDF gives
left and right identical `<axis>` directions (`1 0 0` for roll, `0 0 1` for
yaw) while only mirroring the joint's *position*. Reflecting a rotation about
an axis that lies in the mirror plane (X or Z, here) flips its sign; an axis
perpendicular to the mirror plane (Y, i.e. hip_pitch/knee/ankle_pitch) does
not. Confirmed both empirically (left vs right hip_roll differed by a mean
0.60 rad before the fix, 0.03 rad after -- matching the natural asymmetry
level already seen on the correctly-signed pitch joints) and by conjugating
the rotation with the mirror reflection matrix by hand.

Effect of the fix, measured on `bar_01`:

| | worst-case foot error | mean foot position error |
|---|---|---|
| before | 0.087 | 1.69 cm |
| after | 0.010 | 0.38 cm |

The error's previously strong growth with squat depth (correlation 0.86
against knee angle) also flattened out almost entirely.

## Body/limb scaling: measured, documented, not applied

Parsed the actual `.c3d` files (Theia3D markerless mocap -- stores full
per-segment 4x4 transforms, not individual markers) to get real segment
lengths directly from segment-origin distances (near-zero frame-to-frame
variance, as expected for rigid segments):

| | human (avg of 3 trials) | H1 (URDF) |
|---|---|---|
| thigh | ~0.372 m | 0.4 m |
| shank | ~0.361 m | 0.4 m |

H1's legs are ~6-9% longer than the subject's. Important correction to an
earlier assumption: this mismatch does **not** meaningfully explain the foot
slide above. The foot-locked pelvis IK gives the pelvis a full 6 DOF to plant
both feet using H1's own real (fixed, symmetric) leg length and the copied
angles -- that solve is self-consistent regardless of absolute leg length, as
long as both legs are equal length (they are) and correctly signed (now they
are). The remaining small residual (~0.03-0.9 cm after the sign fix) is
ordinary human asymmetry and IK convergence limits, not a scale effect.

What the length mismatch *does* affect: copying the same joint angle onto a
longer leg produces a proportionally deeper squat on H1 than the human
actually performed (same angle x longer segment = more vertical travel).
That's a motion-realism/proportionality issue worth knowing about for the
Phase 1 feasibility analysis -- and it makes the knee-limit clipping (next
section) very slightly worse -- but it is not applied as a code fix here by
design; the measured mismatch is documented as-is.

## Known limitation: knee joint limit

At the bottom of the squat, `left_knee_joint`/`right_knee_joint` hit the
URDF's `2.19 rad` limit and get clamped -- the subject squats deeper than
H1's knee can physically bend. No pelvis pose can compensate for a clamped
knee angle; this is a hard kinematic limit, not a solver issue. Confirmed the
foot-error growth correlates strongly with knee angle (0.86) even before
clipping kicks in. Options if this needs addressing later: rescale the knee
trajectory to stay within limits (shallower squat, no plateau artifact), or
accept as a known Phase 1 direct-copy-retargeting limitation to be resolved
by the optimization-based retargeting planned for Step 3.1.

## Fixed: capture rate was wrong

`--fps` defaulted to 100.0 everywhere (this pipeline and the ROS2 one), but
the actual `ROTATION:RATE` stored in the `.c3d` files is **60 Hz** --
playback was running ~1.67x too fast. The default is now 60.0 in both
pipelines; override with `--fps <value>` only if a different trial turns out
to have a different rate.
