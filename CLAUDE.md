# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

MATLAB simulation and control stack for a **PhantomX Pincher 4-DOF robot arm** performing vision-guided pick-and-place. There is no build system — all code runs directly in MATLAB.

## Running the code

```matlab
% From MATLAB with working directory set to src/
run('driver.m')                   % full demo: circle trajectory + pick-and-place

% Run perception on a saved point cloud
ptCloud = pcread('../data/pointclouds/ptCloud5.ply');
[seg_rgb, labels] = vision_pipeline(ptCloud);
```

There are no automated tests. Verify changes by running `driver.m` in sim mode (`arm = Arm('sim')`) and observing the live visualizer.

## Architecture

All active code lives under `src/`. There are two independent subsystems:

### Robot control — `src/@Arm/`

MATLAB classdef (`handle` subclass). Every `.m` file in `@Arm/` is a method.

**Key data flow for a Cartesian move:**
```
moveByCoordinates(x,y,z,phi)
  → findJointAngles()       analytical IK → up to 4 candidate configs (2 theta1 × 2 theta3)
  → findValidSolution()     filters by joint limits / self-collision / floor
  → findSolution()          picks lowest weighted-delta cost from current config
  → animateInterpolation()  or updatePlot()
```

**DH parameters** (all distances in cm):
| Joint | a     | α    | d    |
|-------|-------|------|------|
| 1     | 0     | π/2  | 13.7 |
| 2     | 10.5  | 0    | 0    |
| 3     | 10.5  | 0    | 0    |
| 4     | 11    | 0    | 0    |

`DH.m` returns cumulative transforms — `Ts{i}` is the world-frame transform to the origin of frame `i`.

**IK geometry:** joints 2/3/4 are all coplanar (α=0), so the arm reduces to a planar 3-link problem in the vertical plane defined by θ₁. The two θ₁ candidates (`atan2(y,x)` and `atan2(y,x)+π`) produce a "front" and "back" configuration. In practice, back solutions almost always fail joint limits (θ₂_back ≈ π − θ₂_front, which exceeds the ±150° limit for this geometry), so only the two front solutions (elbow-up / elbow-down) survive filtering for typical reachable targets.

**Joint limit check** (`checkJointLimits.m`): `mod(abs(wrapToPi(θ)), π) ≥ deg2rad(150)` rejects any joint angle within 30° of ±180°.

**Self-collision check** (`checkSelfCollision.m`): interpolates 20 waypoints along the joint-space path and calls MATLAB's `checkCollision()` on a `rigidBodyTree`. The tree includes gripper palm + two finger bodies (fixed joints at nominal open position).

**Auto-pitch:** if `phi = NaN` is passed to `moveByCoordinates`, pitch is computed as `atan2(z − 24.2, sqrt(x²+y²))`. The constant 24.2 cm is the nominal shoulder height for the working configuration.

**Hardware mode:** `Arm('real', '/dev/ttyUSB0')` communicates via the Arbotix controller class. Joint 2 has a firmware offset: `setpos` receives `θ₂ + π/2`. Sim mode skips all hardware calls.

**Visualizer:** the figure has three regions — 3D view (top-left), top-down view (top-right), and an IK solution annotation panel (bottom). The annotation panel is populated when `arm.show_four_IK = true`; it lists all raw IK candidates in degrees with valid/invalid/selected status. The annotation handle is stored as `simIKAnnotation`; update it by setting `.String` to a cell array.

**Pick-and-place sequence** (`pickAndPlace.m` / `place.m`): both use a fixed approach direction (phi is resolved once from the grasp z, then held constant for pregrasp/grasp/retract) so descent is a straight vertical line. `grip()` auto-attaches `cubePos` to the end-effector if `norm(ee − cubePos) < gripperOpenWidth + 2`. `ungrip()` drops the cube at the current end-effector position.

### Vision pipeline — `src/perception/`

Standalone functions (no class):

```
vision_pipeline(ptCloud)
  → find_plane()        RANSAC plane fit; returns RGB image of above-plane points only
  → segment()           k-means (6 clusters) in RGB space
  → mergeSegments()     maps clusters to {Yellow, Red, Green, Blue} by mean RGB distance
  → watershed           per-color instance separation using LAB-channel gradients
  → regionprops         centroid + orientation → 4×4 SE(3) pose per object
```

Output poses are in the depth camera frame. Coordinate frame transformation to the robot base frame is not yet implemented.

## Important constraints

- All distances are in **centimetres** throughout (DH params, IK, visualizer).
- `jointAngles` is `1×5`; index 5 is reserved for a gripper servo (currently unused in IK).
- The cost function in `findSolution.m` uses raw (unwrapped) angle differences — this is intentional because the ±150° joint limits prevent crossing the ±π boundary, so `wrapToPi` on deltas would give incorrect physical distances.
- `animateInterpolation` temporarily overwrites `obj.jointAngles(1:4)` at each interpolation step, triggering `updatePlot` — don't read `jointAngles` during animation.
