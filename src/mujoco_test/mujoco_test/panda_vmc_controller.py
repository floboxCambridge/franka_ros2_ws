#!/usr/bin/env python3
import math
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import mujoco
except ImportError as exc:  # pragma: no cover - runtime dependency check
    mujoco = None
    MUJOCO_IMPORT_ERROR = exc
else:
    MUJOCO_IMPORT_ERROR = None


class PandaVMCController(Node):
    def __init__(self):
        super().__init__('panda_vmc_controller')

        if mujoco is None:
            raise RuntimeError(
                'The Python package "mujoco" is required for panda_vmc_controller.'
            ) from MUJOCO_IMPORT_ERROR

        self.declare_parameter('model_path', '/home/flobox/mujoco_menagerie/franka_emika_panda/scene.xml')
        self.declare_parameter('ee_body_name', 'hand')
        self.declare_parameter('joint_state_topic', '/franka/joint_states')
        self.declare_parameter('torque_topic', '/joint_torque_controller/torques')
        self.declare_parameter(
            'mujoco_joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        )
        self.declare_parameter(
            'ros_joint_names',
            ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'],
        )
        self.declare_parameter('kp_cartesian', [100.0, 100.0, 100.0])
        self.declare_parameter('kd_cartesian', [30.0, 30.0, 30.0])
        self.declare_parameter('max_joint_torque', [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
        self.declare_parameter('initial_target_offset', [0.0, 0.0, -0.05])
        self.declare_parameter('target_reached_log_epsilon', 0.005)

        self.model_path = self.get_parameter('model_path').value
        self.ee_body_name = self.get_parameter('ee_body_name').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.torque_topic = self.get_parameter('torque_topic').value
        self.mujoco_joint_names = list(self.get_parameter('mujoco_joint_names').value)
        self.ros_joint_names = list(self.get_parameter('ros_joint_names').value)
        self.kp_cartesian = np.asarray(self.get_parameter('kp_cartesian').value, dtype=float)
        self.kd_cartesian = np.asarray(self.get_parameter('kd_cartesian').value, dtype=float)
        self.max_joint_torque = np.asarray(self.get_parameter('max_joint_torque').value, dtype=float)
        self.initial_target_offset = np.asarray(
            self.get_parameter('initial_target_offset').value,
            dtype=float,
        )
        self.target_reached_log_epsilon = float(
            self.get_parameter('target_reached_log_epsilon').value
        )

        if len(self.mujoco_joint_names) != 7 or len(self.ros_joint_names) != 7:
            raise ValueError(
                'mujoco_joint_names and ros_joint_names must both contain exactly 7 names.'
            )
        if self.kp_cartesian.shape != (3,) or self.kd_cartesian.shape != (3,):
            raise ValueError('kp_cartesian and kd_cartesian must each contain exactly 3 values.')
        if self.max_joint_torque.shape != (7,):
            raise ValueError('max_joint_torque must contain exactly 7 values.')
        if self.initial_target_offset.shape != (3,):
            raise ValueError('initial_target_offset must contain exactly 3 values.')

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        self.ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.ee_body_name)
        if self.ee_body_id == -1:
            raise RuntimeError(
                f'Body "{self.ee_body_name}" not found in MuJoCo model: {self.model_path}'
            )

        self.qpos_adr = []
        self.dof_adr = []
        for name in self.mujoco_joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id == -1:
                raise RuntimeError(
                    f'Joint "{name}" not found in MuJoCo model: {self.model_path}'
                )
            self.qpos_adr.append(self.model.jnt_qposadr[joint_id])
            self.dof_adr.append(self.model.jnt_dofadr[joint_id])

        self.desired_position: Optional[np.ndarray] = None
        self.current_joint_names: List[str] = []
        self.last_joint_state_time = None
        self.last_logged_target_distance = math.inf
        self.state_initialized = False

        self.torque_publisher = self.create_publisher(Float64MultiArray, self.torque_topic, 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.get_logger().info(f'Loaded MuJoCo model: {self.model_path}')
        self.get_logger().info(f'Using EE body: {self.ee_body_name}')
        self.get_logger().info(f'Subscribing to joint states on: {self.joint_state_topic}')
        self.get_logger().info(f'Publishing torques on: {self.torque_topic}')
        self.get_logger().info(
            'Initial desired position will be set to current pose plus offset [%.4f, %.4f, %.4f].'
            % tuple(self.initial_target_offset.tolist())
        )

    def joint_state_callback(self, msg: JointState) -> None:
        if len(msg.position) < len(msg.name) or len(msg.velocity) < len(msg.name):
            self.get_logger().error('JointState message is missing position or velocity entries.')
            return

        if not self.write_state_into_mujoco(msg):
            return

        mujoco.mj_forward(self.model, self.data)
        self.state_initialized = True

        if self.desired_position is None:
            self.desired_position = self.current_ee_position().copy() + self.initial_target_offset
            self.get_logger().info(
                'Initialized desired EE position to current pose plus offset [%.4f, %.4f, %.4f]: [%.4f, %.4f, %.4f].'
                % (
                    self.initial_target_offset[0],
                    self.initial_target_offset[1],
                    self.initial_target_offset[2],
                    self.desired_position[0],
                    self.desired_position[1],
                    self.desired_position[2],
                )
            )

        torque = self.compute_joint_torque()
        self.publish_torque_command(torque)
        self.last_joint_state_time = self.get_clock().now()

    def write_state_into_mujoco(self, msg: JointState) -> bool:
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        q = []
        qd = []
        resolved_names = []

        for preferred_name, mujoco_name in zip(self.ros_joint_names, self.mujoco_joint_names):
            candidate_names = [preferred_name, mujoco_name]
            if preferred_name.startswith('fr3_'):
                candidate_names.append(preferred_name.replace('fr3_', ''))
            if preferred_name.startswith('panda_'):
                candidate_names.append(preferred_name.replace('panda_', ''))

            match_index = next(
                (name_to_index[name] for name in candidate_names if name in name_to_index),
                None,
            )
            if match_index is None:
                self.get_logger().error(
                    f'Could not find any ROS joint state for MuJoCo joint "{mujoco_name}". Candidates: {candidate_names}'
                )
                return False

            q.append(float(msg.position[match_index]))
            qd.append(float(msg.velocity[match_index]))
            resolved_names.append(msg.name[match_index])

        self.current_joint_names = resolved_names

        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        for index, adr in enumerate(self.qpos_adr):
            self.data.qpos[adr] = q[index]
        for index, adr in enumerate(self.dof_adr):
            self.data.qvel[adr] = qd[index]
        return True

    def current_ee_position(self) -> np.ndarray:
        return self.data.xpos[self.ee_body_id].copy()

    def compute_joint_torque(self) -> np.ndarray:
        if not self.state_initialized:
            raise RuntimeError(
                'compute_joint_torque called before a valid joint state was written into MuJoCo and mj_forward was run.'
            )

        x = self.current_ee_position()
        jacp = np.zeros((3, self.model.nv), dtype=float)
        jacr = np.zeros((3, self.model.nv), dtype=float)
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.ee_body_id)

        jacobian = jacp[:, self.dof_adr]
        joint_velocity = self.data.qvel[self.dof_adr].copy()
        ee_velocity = jacobian @ joint_velocity

        position_error = self.desired_position - x
        wrench = self.kp_cartesian * position_error - self.kd_cartesian * ee_velocity
        torque = jacobian.T @ wrench
        torque = np.clip(torque, -self.max_joint_torque, self.max_joint_torque)

        error_norm = float(np.linalg.norm(position_error))
        if error_norm < self.target_reached_log_epsilon <= self.last_logged_target_distance:
            self.get_logger().info(
                'End effector is within %.4f m of the desired position.' % error_norm
            )
        self.last_logged_target_distance = error_norm
        return torque

    def publish_torque_command(self, torque: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = torque.astype(float).tolist()
        self.torque_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PandaVMCController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
