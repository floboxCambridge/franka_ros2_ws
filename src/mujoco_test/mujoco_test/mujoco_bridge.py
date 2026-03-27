#!/usr/bin/env python3
import time
from typing import List

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

try:
    import mujoco
    import mujoco.viewer
except ImportError as exc:  # pragma: no cover - runtime dependency check
    mujoco = None
    MUJOCO_IMPORT_ERROR = exc
else:
    MUJOCO_IMPORT_ERROR = None


class MujocoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')

        if mujoco is None:
            raise RuntimeError(
                'The Python package "mujoco" is required for mujoco_bridge.'
            ) from MUJOCO_IMPORT_ERROR

        self.declare_parameter('model_path', '/home/flobox/mujoco_menagerie/franka_emika_panda/scene.xml')
        self.declare_parameter('ee_body_name', 'hand')
        self.declare_parameter('joint_state_topic', '/franka/joint_states')
        self.declare_parameter('mujoco_joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'])
        self.declare_parameter('ros_joint_names', ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'])
        self.declare_parameter('ee_position_topic', '/mujoco/ee_position')

        self.model_path = self.get_parameter('model_path').value
        self.ee_body_name = self.get_parameter('ee_body_name').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.mujoco_joint_names = list(self.get_parameter('mujoco_joint_names').value)
        self.ros_joint_names = list(self.get_parameter('ros_joint_names').value)
        self.ee_position_topic = self.get_parameter('ee_position_topic').value

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        self.ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.ee_body_name)
        if self.ee_body_id == -1:
            raise RuntimeError(f'Body "{self.ee_body_name}" not found in MuJoCo model: {self.model_path}')

        self.qpos_adr = []
        self.dof_adr = []
        for name in self.mujoco_joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id == -1:
                raise RuntimeError(f'Joint "{name}" not found in MuJoCo model: {self.model_path}')
            self.qpos_adr.append(self.model.jnt_qposadr[joint_id])
            self.dof_adr.append(self.model.jnt_dofadr[joint_id])

        self.ee_position_publisher = self.create_publisher(PointStamped, self.ee_position_topic, 10)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )
        self.latest_joint_state = None

        self.get_logger().info(f'Loaded MuJoCo model: {self.model_path}')
        self.get_logger().info(f'Subscribing to joint states on: {self.joint_state_topic}')
        self.get_logger().info(f'Publishing MuJoCo EE position on: {self.ee_position_topic}')

    def joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        for preferred_name, mujoco_name, qpos_adr, dof_adr in zip(
            self.ros_joint_names, self.mujoco_joint_names, self.qpos_adr, self.dof_adr
        ):
            candidate_names = [preferred_name, mujoco_name]
            if preferred_name.startswith('fr3_'):
                candidate_names.append(preferred_name.replace('fr3_', ''))
            match_index = next((name_to_index[name] for name in candidate_names if name in name_to_index), None)
            if match_index is None:
                self.get_logger().warning(
                    f'Could not resolve joint for MuJoCo joint "{mujoco_name}" from candidates {candidate_names}.',
                    throttle_duration_sec=2.0,
                )
                return

            self.data.qpos[qpos_adr] = float(msg.position[match_index])
            self.data.qvel[dof_adr] = float(msg.velocity[match_index])

        mujoco.mj_forward(self.model, self.data)
        self.publish_ee_position(msg.header.stamp)

    def publish_ee_position(self, stamp) -> None:
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        ee_position = self.data.xpos[self.ee_body_id]
        msg.point.x = float(ee_position[0])
        msg.point.y = float(ee_position[1])
        msg.point.z = float(ee_position[2])
        self.ee_position_publisher.publish(msg)

    def run(self) -> None:
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.get_logger().info('MuJoCo viewer launched.')
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0.0)
                viewer.sync()
                time.sleep(self.model.opt.timestep)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
