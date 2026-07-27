# Gazebo no-contact gripper command evidence

## Scope

This gate verifies that the Lime gripper command path works through Gazebo Classic physics and `gazebo_ros2_control` before any cube contact or grasp trajectory is attempted.

The commanded sequence is:

```text
open (0.019 m) -> close (-0.010 m) -> open (0.019 m)
```

The accepted evidence consists of the ROS 2 action result, the commanded left gripper joint from `/joint_states`, both physical gripper-link poses from Gazebo `/link_states`, the robot model pose and twist from `/model_states`, backend introspection, and a deterministic telemetry-rendered MP4.

## Test-only model

`project/resource/turtlebot3_lime_gripper_test.urdf.xacro` retains:

- the full TurtleBot3 Lime physical link and collision model;
- the OpenManipulator SARA physical model;
- the physical right-joint `<mimic>` relation;
- the ROS 2 Control `mimic` and `multiplier` parameters;
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

## Gazebo backend observation

In the accepted Humble/Gazebo Classic run, `/joint_states` and `/dynamic_joint_states` contained `gripper_left_joint` and the upstream fixed workaround joint `gripper_right_joint_mimic`, but not the physical prismatic `gripper_right_joint`.

The missing physical-right JointState is preserved as `null`. It is never copied from the left joint. The physical right finger is instead validated independently from the world-space pose of `turtlebot3_lime_gripper_test::gripper_right_link` published by `/link_states`.

The physical distance between the left and right gripper link frames reached the expected approximately 22 mm closed and 80 mm open. This directly verifies that the Gazebo mimic link moved even though the corresponding physical joint was absent from the broadcaster output.

## Execution

Inside the completed Lime Docker image:

```bash
bash /root/bin/run_gazebo_gripper_command_evidence /artifacts
```

The runner:

1. expands the test-only Xacro and rejects forbidden plugins;
2. starts `robot_state_publisher` for the `gazebo_ros2_control` parameter contract;
3. launches headless `gzserver` in the isolated calibration world;
4. spawns the robot without a cube;
5. loads only `joint_state_broadcaster` and `gripper_controller`;
6. records hardware interfaces plus initial joint, dynamic-joint, and link states;
7. lets the robot settle;
8. sends the three gripper goals;
9. records left-joint, left/right physical-link, and robot-model telemetry;
10. validates action, physical mimic, separation, and base-stability bounds;
11. writes telemetry, summary, manifest, expanded URDF, video, and logs.

## Accepted run

Workflow run `30270928900` completed successfully.

Recorded values:

- 618 joint-state samples over 6.1846 s;
- 616 model-state samples;
- 616 physical gripper-link samples;
- left joint range: -0.0100000003 m to 0.0190000052 m;
- physical link-frame separation: 0.0219999950 m to 0.0800000054 m;
- robot displacement after the settle reference: 0.00000914 m;
- final robot linear speed: 0.0000509 m/s;
- all three action goals reached the target with `stalled=false`;
- generated MP4 duration: 6.2 s.

## Acceptance conditions

The gate passes only if:

- all three goals are accepted and reach the target without stalling;
- the commanded left joint reaches each target within 1.5 mm;
- both physical gripper links are independently present in `/link_states`;
- physical link-frame separation reaches approximately 22 mm closed and 80 mm open;
- a missing right physical JointState remains missing rather than being synthesized;
- the robot moves less than 50 mm after the settle reference;
- final robot linear speed is below 0.05 m/s;
- no cube is spawned;
- `inner_face_opening_m` remains `null`;
- no grasp-success claim is made.

## Evidence meaning

The rendered video visualizes the physical left/right link-frame separation observed by Gazebo. It does not display the mesh surfaces and must not be interpreted as a measurement of the finger inner-face opening.

Passing this gate permits a separate gauge-contact calibration trial in an actual Lime simulation session. It does not yet permit a Rubik grasp trajectory, lift, IFRA attachment, or physics grasp-success claim.
