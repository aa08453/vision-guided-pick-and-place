# vision-guided-pick-and-place

Phantom x Pincher 4-DOF vision-guided pick-and-place pipeline.

This repository contains code, data, reports and a MATLAB digital twin for a complete vision-guided pick-and-place project built for a 4-degree-of-freedom [Phantom x Pincher robot](https://www.researchgate.net/publication/322222351_PhantomX_Pincher_Specifications) using [Peter Corke's toolbox](https://petercorke.com/toolboxes/robotics-toolbox/). The project is organized around three primary stages: perception, motion control & kinematics, and simulation (digital twin). Key implementation and analysis artifacts are linked below.

Watch the [Robotics_Demo](Robotics_Demo.mp4) video to see how it works.

## Project overview

- **Perception:** image and point-cloud processing to detect, segment and estimate 6-DOF object poses from camera data.
- **Motion control & kinematics:** inverse kinematics, collision checks, and motion execution for pick-and-place tasks.
- **Digital twin & simulation:** a MATLAB-based digital twin (simulation mode) that mirrors the real robot behaviour for verification and visualization. A picture of the twin is included in the repository: [twin.jpg](twin.jpg).

See the reports folder for writeups and method descriptions: [reports](reports), especially the Architecture Brief for the complete overview.

## Stages and key files

**Perception**
- Purpose: detect objects on the workspace, segment clusters, compute object poses and feed those poses to the motion planner.
- Main scripts and functions:
	- [src/run_vision_pipeline.m](src/run_vision_pipeline.m) — top-level pipeline driver for perception experiments.
	- [src/perception/run_vision_pipeline.m](src/perception/run_vision_pipeline.m) — pipeline entry for the perception submodule.
	- [src/perception/segment.m](src/perception/segment.m) and [src/perception/segment_cluster.m](src/perception/segment_cluster.m) — segmentation helpers.
	- [src/perception/compute_object_pose.m](src/perception/compute_object_pose.m) — computes object pose from segmented data.
	- [src/perception/vision_functions.m](src/perception/vision_functions.m) — utility functions for visualization and processing.
	- Data samples: [data/images](data/images) and [data/pointclouds](data/pointclouds).

**Motion control & kinematics**
- Purpose: plan joint movements, compute inverse kinematics and ensure collision-free trajectories for pick-and-place.
- Main scripts and functions:
	- [src/vision_pick_and_place.m](src/vision_pick_and_place.m) — integrated pick-and-place demo combining perception and control.
	- [src/kinematics/Inverse/moveIK.m](src/kinematics/Inverse/moveIK.m) — inverse-kinematics motion executor.
	- [src/kinematics/Inverse/checkIK.m](src/kinematics/Inverse/checkIK.m) — IK validity checks.
	- [src/kinematics/Inverse/checkSelfCollision.m](src/kinematics/Inverse/checkSelfCollision.m) — self-collision checks.
	- High-level motion helpers in [src/driver.m](src/driver.m) and [final.m](final.m).

**Arm MATLAB class**
- The project includes an `Arm` MATLAB class that encapsulates robot geometry, forward/inverse routines and helper utilities. See the implementation and evaluation files in [reports/@Arm/Arm.m](src/@Arm/Arm.m) and supporting scripts in [src/@Arm](src/@Arm).

**Digital twin & simulation**
- A MATLAB digital twin is provided that runs in `sim` mode and reproduces the robot's kinematics and planned motions for offline testing and visualization.
- The twin is used for validating trajectories and visualizing interpolation; see the picture at [twin.jpg](twin.jpg).

## Reports and analysis
- All formal reports, experiment logs and some analysis scripts are in the `reports` folder: [reports](reports). 

## Getting started
Prerequisites
- MATLAB (R2018b or newer recommended) with Image Processing and Computer Vision toolboxes for some scripts. The code was developed and tested in MATLAB; adaptors would be needed for other environments.

Basic usage
- Calibrate cameras (if using real hardware): run `src/run_calibrate_search.m` or `src/calibrate_camera.m`.
- Run the perception pipeline: open MATLAB and run `src/run_vision_pipeline.m` or call `src/perception/run_vision_pipeline.m` from the MATLAB path.
- Run the integrated demo: run `src/vision_pick_and_place.m` or `final.m` to execute the full pipeline (perception → planning → simulated execution).

Notes on simulation
- To use the digital twin, start the control scripts in `sim` mode (see the top of `src/vision_pick_and_place.m` for the mode toggle). The twin visualization and an example image are available at [twin.jpg](twin.jpg).

## Project structure (high level)
- [src](src) — MATLAB source code for perception, calibration, kinematics and demos.
- [data](data) — imagery and point-cloud samples used for testing.
- [reports](reports) — writeups and analysis notebooks/scripts including the `Arm` class evaluation.

## Reproducing experiments
- Use the scripts in `src` and the sample data in `data` to reproduce perception and motion experiments. See the individual reports in `reports` for experiment parameters and detailed procedures.

## License
- This project abides by the MIT License.
