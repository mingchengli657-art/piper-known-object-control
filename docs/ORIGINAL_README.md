# Piper Control — known-object grasp

This project is the robot-control layer between FoundationPose and Piper X. It
currently implements one deliberately bounded trial for `object_003`:

1. verify that FoundationPose is running the expected repaired model;
2. wait for a stable object pose in `base_link` and a stationary TCP;
3. freeze that pose before motion so later occlusion cannot move the target;
4. normalize the calibrated gripper to 80 mm, then command the configured
   95 mm daily opening and verify real directional feedback;
5. solve and preflight a joint-limit-safe Piper X IK branch;
6. move in joint space to a 150 mm pre-grasp, then advance linearly to the
   validated 100 mm standoff;
7. close to 40 mm; accept target width, a sustained force threshold, or a
   stable closing plateau as physical contact;
8. move linearly upward by exactly 50 mm in robot-base +Z;
9. hold the object. It does not automatically return or release.

Dry-run is the default. Real commands are impossible unless `--execute` is
supplied. The controller never calls `/control_enable` and never enables the
robot itself.

## Interfaces

Inputs:

```text
/foundationpose/object_pose_base  geometry_msgs/msg/PoseStamped
/foundationpose/object_pose_json  std_msgs/msg/String
/foundationpose/recovery_required std_msgs/msg/Bool
/feedback/tcp_pose                geometry_msgs/msg/PoseStamped
/feedback/joint_states            sensor_msgs/msg/JointState
/feedback/arm_status              agx_arm_msgs/msg/AgxArmStatus
/feedback/gripper_status          agx_arm_msgs/msg/GripperStatus
```

Commands, sent only with `--execute`:

```text
/control/move_j       sensor_msgs/msg/JointState
/control/move_l       geometry_msgs/msg/PoseStamped
/control/joint_states sensor_msgs/msg/JointState
```

Inspection outputs are transient-local, so a late `ros2 topic echo --once`
still receives the last value:

```text
/piper_control/frozen_object_pose
/piper_control/grasp_target_pose
/piper_control/pregrasp_joint_target
/piper_control/grasp_status
```

The AGX ROS driver is the only CAN owner. The vendored
`vendor/piper_ros2_control/gripper_execution.py` validates feedback and sends
one absolute hardware target, with a bounded retry of that same target. It
does not synthesize intermediate positions and its hardware node must not be
started in parallel with `agx_arm_ctrl`.

## Full object_003 trial

Use separate terminals and keep the first six processes running.

### 1. Activate CAN

```bash
cd /path/to/agx_arm_ws/src/agx_arm_ros/scripts
bash can_activate.sh can0 1000000
ip -details link show can0
```

Expected: `UP`, `LOWER_UP`, `ERROR-ACTIVE`, and bitrate `1000000`.

### 2. Start Piper and AGX gripper feedback

```bash
cd /path/to/Piper_Control
CAN_PORT=can0 SPEED_PERCENT=10 ./scripts/start_piper.sh
```

This launch explicitly uses `effector_type:=agx_gripper` and
`control_enabled:=true`. Do not run a second SDK/CAN driver.

### 3. Start D405

```bash
bash /path/to/PBVS_test/start_d405.sh
```

### 4. Start FoundationPose ROS I/O

```bash
cd "/path/to/Foundation Pose"
./scripts/start_ros_io.sh
```

### 5. Start FoundationPose for object_003

Activate the configured `foundationpose` environment, then:

```bash
cd "/path/to/Foundation Pose"
export CUDA_HOME=/usr/local/cuda-12.8
./scripts/start_worker.sh \
  ../object_modeling/datasets/object_003/model_plane_cleaned_fixed
```

Draw a tight polygon around only object 003 and press Enter. Wait for a stable,
correct upright box before continuing.

### 6. Transform camera pose into robot-base pose

```bash
cd /path/to/Piper_Control
./scripts/start_handeye.sh
```

### 7. Run read-only preflight and dry-run

```bash
cd /path/to/Piper_Control
./scripts/check_ready.sh
./scripts/start_grasp_003.sh
```

The dry-run may spend roughly 8--10 seconds searching and sampling IK. It must
print `IK preflight passed` followed by `DRY RUN COMPLETE`; it generates the
frozen grasp/lift and pre-grasp joint targets but publishes no hardware
commands. Inspect the latched result in another sourced ROS terminal if desired:

```bash
ros2 topic echo /piper_control/grasp_status --once
ros2 topic echo /piper_control/grasp_target_pose --once
ros2 topic echo /piper_control/pregrasp_joint_target --once
```

Stop the dry-run with Ctrl+C after inspection.

### 8. Execute once

Clear the robot workspace, keep the physical emergency stop reachable, keep
object 003 fully visible and upright, and then run:

```bash
cd /path/to/Piper_Control
./scripts/start_grasp_003.sh --execute
```

Expected terminal sequence is `target frozen`, `IK preflight passed`,
`gripper opening`, `validated pre-grasp
MoveJ command sent`, `final linear grasp approach`, `gripper close`, `linear
lift command sent`, then `GRASP SUCCESS`.
On success the node remains in `holding` and the gripper stays closed.

The calibrated v189 add-on reports `homing_status=false` even after a verified
zero operation and power-cycle persistence, so that bit is advisory for this
device profile. The three-second unverified lift fallback is disabled in the
normal object configurations: lifting requires target width, sustained force,
or a measured closing-motion plateau.

For an emergency software stop from a correctly sourced ROS terminal:

```bash
ros2 service call /emergency_stop std_srvs/srv/Empty "{}"
```

The physical emergency stop remains the primary emergency action.

## Configuration and portability

The object-specific values are in `config/object_003_grasp.yaml`; hand-eye
calibration is in `config/online_handeye.yaml`. Paths inside the project are
relative. On another computer, set either `AGX_ARM_WS=/path/to/agx_arm_ws` or
`AGX_ARM_SETUP=/path/to/install/setup.bash` before running the scripts.

The current object identity contract is:

```text
object.id: object_003
FoundationPose model_id: model_plane_cleaned_fixed
```

The controller rejects a different model rather than silently grasping with
the wrong geometry.

The kinematic preflight loads the installed Piper X URDF, searches multiple IK
seeds, reserves a five-degree joint-limit margin, samples the pre-grasp-to-grasp
and grasp-to-lift Cartesian paths, and checks the joint-interpolated MoveJ TCP
against the configured workspace. This removes the previous dependence on the
firmware selecting a usable `move_p` IK branch. It is not a general obstacle or
self-collision planner, so the physical workspace must still be kept clear.

## Tests

Pure tests:

```bash
source scripts/env.sh
PYTHONPATH="$PWD:${PYTHONPATH:-}" python3 -m pytest -q \
  tests/test_grasp_geometry.py tests/test_piper_kinematics.py
PYTHONPATH="$PWD/vendor" python3 -m pytest -q \
  /path/to/piper_ros2_control/test/test_gripper_execution.py
```

ROS integration dry-run with synthetic feedback:

```bash
source scripts/env.sh
ROS_DOMAIN_ID=42 ROS_LOG_DIR=/tmp/piper_control_ros_test \
  python3 tests/ros_dry_run.py
```

The integration test fails if any arm or gripper command is published.

The full no-hardware execution simulation additionally covers IK preflight,
joint-space pre-grasp, the physical opening plateau, linear approach, closing
and the exact 50 mm lift:

```bash
source scripts/env.sh
ROS_DOMAIN_ID=43 ROS_LOG_DIR=/tmp/piper_control_execute_sim \
  python3 tests/ros_execute_sim.py
```
