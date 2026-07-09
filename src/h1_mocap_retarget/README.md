# h1_mocap_retarget

Plays back a Vicon mocap trial on the H1_2 (handless) model in Rviz -- joint
angles drive the limbs, and the pelvis itself now moves (driven by real
lab-frame position data) instead of staying pinned in place.

## Build & run

    colcon build --packages-select h1_mocap_retarget
    source install/setup.bash
    ros2 launch h1_mocap_retarget mocap_rviz.launch.py

`legs_only:=true` by default -- arms stay neutral so the squat can be judged
on its own. Add `legs_only:=false` once the squat looks right.

## Root motion (pelvis actually sinking into the squat)

The pelvis is not driven by raw mocap position data anymore -- that approach
caused the feet to slide (copying `L_HIP_POSITION` sway directly doesn't keep
feet planted). Instead the node solves the pelvis's 6-DOF pose per frame so
both feet stay locked to their frame-0 ground position (foot-locked IK), and
broadcasts the result as a `world -> pelvis` TF. Rviz's Fixed Frame is `world`
now (was `pelvis`). Disable with `use_root_motion:=false` if this ever needs
to be ruled out while debugging.

Two solvers are available for that pelvis IK:

- `use_mink:=true` (**default**) -- Mink's QP-based differential IK, same
  method used in `mujoco_retarget/retarget_mink.py`. More accurate
  (worst-case foot error 0.087 vs the old solver's 0.16). Requires
  `pip install mujoco mink --break-system-packages` in the ROS2 Python
  environment.
- `use_mink:=false` -- the original hand-rolled Levenberg-Marquardt solver in
  `retarget_core.py`. No extra dependencies, kept as a fallback.

**Fixed:** the dominant cause of the visible foot slide was a sign bug, not
scaling. `right_hip_roll_joint`, `right_hip_yaw_joint`, and
`right_ankle_roll_joint` copied the same sign as their left counterparts, but
the URDF gives left/right identical `<axis>` directions while only mirroring
the joint's *position* -- so a rotation about an axis lying in the mirror
plane (roll about X, yaw about Z) needs a sign flip between legs that hip_
pitch/knee/ankle_pitch (axis Y, perpendicular to the mirror plane) don't.
Confirmed both empirically (left vs right hip_roll differed by a mean 0.60
rad before the fix, 0.03 rad after) and by conjugating the rotation with the
mirror reflection matrix by hand. Effect on `bar_01`: worst-case foot error
0.087 -> 0.010, mean foot position error 1.69cm -> 0.38cm, and the error's
strong growth with squat depth (correlation 0.86 against knee angle) flattened
out almost entirely.

**Known limitation (measured, not fixed):** the H1 URDF's leg segments (0.4m
thigh, 0.4m shank) are longer than the mocap subject's real segments (parsed
from the `.c3d` files directly: ~0.372m thigh, ~0.361m shank -- Theia3D
markerless mocap stores full per-segment transforms, not markers). This does
**not** meaningfully affect the foot slide above -- the foot-locked pelvis IK
is self-consistent regardless of absolute leg length as long as both legs are
equal (they are) and correctly signed (now they are). What it does affect:
copying the same joint angle onto a longer leg produces a proportionally
deeper squat on H1 than the subject actually performed -- a motion-realism
concern for the Phase 1 feasibility analysis, documented here rather than
corrected in code.

**Known limitation (hard hardware limit, not fixed):** at peak squat depth,
`left_knee_joint`/`right_knee_joint` hit the URDF's `2.19 rad` limit and get
clamped -- the subject squats deeper than H1's knee can bend, so no pelvis
pose can keep the foot exactly on target at those frames. Confirmed the foot
error still correlates strongly with knee angle (0.86) even before clipping
kicks in, so the residual isn't purely the plateau frames. Not a solver tuning
issue.

**Fixed:** `fps` defaulted to 100.0 here and in `mujoco_retarget`, but the
`.c3d` files' actual `ROTATION:RATE` is **60 Hz** -- playback was running
~1.67x too fast. The default is now 60.0 in both pipelines; override with
`fps:=<value>` only if a different trial turns out to have a different
rate.

## Fixing a joint that moves the wrong way

    ros2 launch h1_mocap_retarget mocap_rviz.launch.py flip_hip_pitch:=true

Available: `flip_hip_yaw`, `flip_hip_pitch`, `flip_hip_roll`, `flip_knee`,
`flip_ankle_pitch`, `flip_ankle_roll`, `flip_torso`. Tell me which
combination (if any) looks right and I'll bake it into `RETARGET_MAP` as the
new default.

Other arguments: `trial:=dumbells_01` / `dumbells_02`, `rate:=0.3` (slow
motion), `fps:=<value>` (capture rate, now correctly defaults to 60 Hz --
confirmed from the raw `.c3d` files' `ROTATION:RATE`).

## Session log (things already found and fixed)

- hip_pitch's sign was verified with actual forward kinematics against the
  URDF's joint origins/axes (not guesswork): `+X` is forward (confirmed via
  the camera mount's offset), and a negative hip_pitch swings the leg
  forward -- i.e. hip flexion, correct for a squat. The default in
  `RETARGET_MAP` reflects this.
- The originally-copied `check_joints.rviz` crashed Rviz on load (stale
  hardcoded path from the original author's machine). Replaced with our own
  minimal `config/mocap_view.rviz`.
- Root motion (this section) added after confirming the "legs flying, hip
  fixed" look was an inherent limitation of pure joint-angle playback with no
  root translation, not a retargeting bug.
