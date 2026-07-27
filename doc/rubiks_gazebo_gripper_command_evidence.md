# Gazebo no-contact gripper command evidence

## Scope

This gate verifies that the Lime gripper command path works through Gazebo Classic physics and `gazebo_ros2_control` before any cube contact or grasp trajectory is attempted.

The commanded sequence is:

```text
open (0.019 m) -> close (-0.010 m) -> open (0.019 m)
```

The accepted evidence is the ROS 2 action result, both gripper joints from `/joint_states`, the robot model pose and twist from `/model_states`, and a deterministic telemetry-rendered MP4.

## Test-only model

`project/resource/turtlebot3_lime_gripper_test.urdf.xacro` retains:

- the full TurtleBot3 Lime physical link and collision model;
- the OpenManipulator SARA physical model;
- passive wheel and caster contact settings;
- manipulator Gazebo joint properties;
- the existing `turtlebot3_lime_system` ROS 2 Control definition with `GazeboSystem`;
- `libgazebo_ros2_control.so`.

It deliberately excludes:

- all RGB, infrared, depth, lidar, and Gazebo IMU sensor elements;
- `librealsense_gazebo_plugin.so`;
- `libgazebo_ros_ray_sensor.so`;
- `libgazebo_ros_diff_drive.so`;
- the absent `libgazebo_grasp_fix.so`;
- `gzclient`, RViz, and every GUI recording dependency.

The production `turtlebot3_lime.urdf.xacro` and normal launch path are not modified by this gate.

## Execution

Inside the completed Lime Docker image:

```bash
bash /root/bin/run_gazebo_gripper_command_evidence /artifacts
```

The runner:

1. expands the test-only Xacro and rejects forbidden plugins;
2. launches headless `gzserver` in the isolated calibration world;
3. spawns the robot without a cube;
4. loads only `joint_state_broadcaster` and `gripper_controller`;
5. lets the robot settle;
6. sends the three gripper goals;
7. records left and right joints independently;
8. records robot model position and velocity;
9. validates command, mimic, separation, and base-stability bounds;
10. writes telemetry, summary, manifest, expanded URDF, and logs.

## Acceptance conditions

The gate passes only if:

- all three goals are accepted and reach the target without stalling;
- both gripper joints are independently present in every accepted sample;
- held target error is at most 1.5 mm;
- left/right mimic disagreement is at most 0.5 mm;
- link-frame separation reaches approximately 22 mm closed and 80 mm open;
- the robot moves less than 50 mm after the settle reference;
- final robot linear speed is below 0.05 m/s;
- no cube is spawned;
- `inner_face_opening_m` remains `null`;
- no grasp-success claim is made.

## Evidence meaning

The rendered video visualizes only observed joint positions and link-frame separation. It does not display the Gazebo mesh and must not be interpreted as a measurement of the physical finger inner-face opening.

Passing this gate permits a separate gauge-contact calibration trial in an actual Lime simulation session. It does not yet permit a Rubik grasp trajectory, lift, IFRA attachment, or physics grasp-success claim.
