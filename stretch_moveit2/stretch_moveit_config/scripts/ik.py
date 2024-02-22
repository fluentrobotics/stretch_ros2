#!/usr/bin/env python3

import time
import rclpy
import urdfpy
import pathlib
import warnings
import ikpy.chain
import numpy as np
import ikpy.urdf.utils
import stretch_body.hello_utils as hu
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import euler_from_quaternion
from hello_helpers.hello_misc import HelloNode
from geometry_msgs.msg import PoseStamped, Pose


class InverseKinematics(HelloNode):
    def __init__(self, pose_goal):
        HelloNode.__init__(self)
        HelloNode.main(self, 'ik', 'ik', wait_for_first_pointcloud=False)
        self.chain = self.load_urdf()
        self.target_point = [pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z]
        orientation_list = [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.target_orientation = ikpy.utils.geometry.rpy_matrix(roll, pitch, yaw)
        self.pretarget_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
        
    def load_urdf(self):
        urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
        original_urdf = urdfpy.URDF.load(urdf_path)
        modified_urdf = original_urdf.copy()
        names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'laser', 'respeaker_base']
        links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]

        for lr in links_to_remove:
            modified_urdf._links.remove(lr)
        names_of_joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'joint_laser', 'joint_respeaker']
        joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
        for jr in joints_to_remove:
            modified_urdf._joints.remove(jr)
    
        # Save it to the "/tmp" directory.
        iktuturdf_path = "/tmp/iktutorial/stretch.urdf"
        modified_urdf.save(iktuturdf_path)
        chain = ikpy.chain.Chain.from_urdf_file(iktuturdf_path)

        return chain
    
    def get_current_configuration(self):
        def bound_range(name, value):
            names = [l.name for l in self.chain.links]
            index = names.index(name)
            bounds = self.chain.links[index].bounds
            return min(max(value, bounds[0]), bounds[1])
        
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue

        self.get_logger().info('Get current joints ...')
        joint_state = self.joint_state

        lift_index = joint_state.name.index('joint_lift')
        arm_index = joint_state.name.index('wrist_extension')
        wrist_yaw_index = joint_state.name.index('joint_wrist_yaw')
        wrist_pitch_index = joint_state.name.index('joint_wrist_pitch')
        wrist_roll_index = joint_state.name.index('joint_wrist_roll')

        joint_lift = joint_state.position[lift_index]
        wrist_extension = joint_state.position[arm_index]
        joint_wrist_yaw = joint_state.position[wrist_yaw_index]
        joint_wrist_pitch = joint_state.position[wrist_pitch_index]
        joint_wrist_roll = joint_state.position[wrist_roll_index]

        q_lift = bound_range('joint_lift', joint_lift)
        q_arml = bound_range('joint_arm_l0', wrist_extension / 4.0)
        q_yaw = bound_range('joint_wrist_yaw', joint_wrist_yaw)
        q_pitch = bound_range('joint_wrist_pitch', joint_wrist_pitch)
        q_roll = bound_range('joint_wrist_roll', joint_wrist_roll)

        return [0.0, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]
    
    def get_goal_solution(self, target_point, target_orientation, pretarget_orientation=None):
        q_init = self.get_current_configuration()

        if pretarget_orientation is not None:
            q_init = self.chain.inverse_kinematics(target_point, pretarget_orientation, orientation_mode='all', initial_position=q_init)
        q_soln = self.chain.inverse_kinematics(target_point, target_orientation, orientation_mode='all', initial_position=q_init)
        print('Solution:', q_soln)

        err = np.linalg.norm(self.chain.forward_kinematics(q_soln)[:3, 3] - target_point)
        # if not np.isclose(err, 0.0, atol=5e-2):
        #     print("error is %f, IKPy did not find a valid solution" % (err))
        #     return

        return q_soln

    def issue_ik_command(self):
        solution = self.get_goal_solution(self.target_point, self.target_orientation, self.pretarget_orientation)

        joint_point = JointTrajectoryPoint()
        joint_point.positions = [solution[2], solution[4] + solution[5] + solution[6] + solution[7], solution[8], solution[10], solution[11], 0.1]

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_gripper_finger_left']
        trajectory_goal.trajectory.points = [joint_point]
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")
        rclpy.shutdown()
    
    def main(self):
        self.issue_ik_command()

def main():
    warnings.filterwarnings("ignore")
    time.sleep(5)

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = 0.06
    pose_goal.pose.orientation.y = 0.43
    pose_goal.pose.orientation.z = -0.346
    pose_goal.pose.orientation.w = 0.938
    pose_goal.pose.position.x = 0.135
    pose_goal.pose.position.y = -0.436
    pose_goal.pose.position.z = 0.746

    try:
        node = InverseKinematics(pose_goal)
        node.main()
        node.new_thread.join()
    except:
        node.get_logger().info("Exiting")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
