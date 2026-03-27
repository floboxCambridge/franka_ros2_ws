# mujoco_test

`mujoco_test` contains two Python ROS 2 nodes that use a MuJoCo model of the Panda/Franka arm together with the real robot joint states.

## Nodes

### `panda_vmc_controller`

This is the controller node.

It:
- subscribes to `/franka/joint_states`
- writes the robot joint positions and velocities into the MuJoCo model
- runs `mujoco.mj_forward(...)`
- computes the end-effector translational Jacobian in MuJoCo
- computes a Cartesian PD wrench to move the end effector toward a desired position
- maps that wrench to joint torques with `tau = J^T F`
- publishes the 7 joint torques to `/joint_torque_controller/torques`

Important behavior:
- It does not publish any torque until a valid joint-state message has been received and MuJoCo forward kinematics has been updated.
- If `desired_position` is not set at startup, it initializes the target to the current end-effector position plus `initial_target_offset`.
- The default `initial_target_offset` is `[0.0, 0.0, -0.05]`, which means 5 cm below the current end-effector position.
- It does not add MuJoCo `qfrc_bias`. The published torques are only the Jacobian-mapped task-space torques.

### `mujoco_bridge`

This is a visualization/debug node.

It:
- subscribes to `/franka/joint_states`
- mirrors the robot state into the MuJoCo model
- opens a MuJoCo passive viewer
- publishes the MuJoCo end-effector position on `/mujoco/ee_position`

This node does not command the robot.

## Topics

### `panda_vmc_controller`

Subscriptions:
- `/franka/joint_states` (`sensor_msgs/msg/JointState`)
- `/desired_ee_position` (`geometry_msgs/msg/PointStamped`)

Publications:
- `/joint_torque_controller/torques` (`std_msgs/msg/Float64MultiArray`)
- `/mujoco/debug_joint_states` (`sensor_msgs/msg/JointState`, optional)

### `mujoco_bridge`

Subscriptions:
- `/franka/joint_states` (`sensor_msgs/msg/JointState`)

Publications:
- `/mujoco/ee_position` (`geometry_msgs/msg/PointStamped`)

## Parameters

### `panda_vmc_controller`

Common parameters:
- `model_path`: path to the MuJoCo XML model
- `ee_body_name`: MuJoCo body name used as the end effector, default `hand`
- `joint_state_topic`: default `/franka/joint_states`
- `torque_topic`: default `/joint_torque_controller/torques`
- `desired_position_topic`: default `/desired_ee_position`
- `mujoco_joint_names`: MuJoCo joint names for the 7 arm joints
- `ros_joint_names`: ROS joint names expected from the real robot
- `kp_cartesian`: Cartesian proportional gains `[kx, ky, kz]`
- `kd_cartesian`: Cartesian damping gains `[dx, dy, dz]`
- `max_joint_torque`: per-joint saturation limits
- `desired_position`: optional fixed target `[x, y, z]`
- `initial_target_offset`: default `[0.0, 0.0, -0.05]`
- `publish_debug_joint_states`: whether to publish MuJoCo debug joint states
- `debug_joint_state_topic`: debug topic name

### `mujoco_bridge`

Common parameters:
- `model_path`
- `ee_body_name`
- `joint_state_topic`
- `mujoco_joint_names`
- `ros_joint_names`
- `ee_position_topic`

## Build

From the workspace root:

```bash
cd /home/flobox/franka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_test
source install/setup.bash
```

## Run

### 1. Start the Franka torque controller

The current bringup is configured so this one command starts the robot with `joint_torque_controller` by default:

```bash
ros2 launch franka_bringup example.launch.py
```

### 2. Start the MuJoCo torque node

```bash
ros2 run mujoco_test panda_vmc_controller
```

### 3. Optionally start the MuJoCo viewer bridge

```bash
ros2 run mujoco_test mujoco_bridge
```

## Changing the target

You can send a new desired end-effector position with:

```bash
ros2 topic pub /desired_ee_position geometry_msgs/msg/PointStamped "{point: {x: 0.4, y: 0.0, z: 0.5}}"
```

## Notes

- The controller currently uses only the translational Jacobian, not orientation control.
- The controller assumes the MuJoCo model and the real robot joint naming correspond closely enough for the configured name mapping.
- The `mujoco` Python package must be installed in the environment where these nodes are run.
- Since the Franka torque controller has a 10 ms timeout, the controller node should keep receiving fresh joint states and publishing torques continuously during operation.
