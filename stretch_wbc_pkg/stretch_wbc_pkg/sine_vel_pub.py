import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import csv
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class SineVelPublisher(Node):
    def __init__(self):
        super().__init__('sine_vel_publisher')

        # Load the configuration from the YAML file
        package_share_directory = get_package_share_directory('stretch_wbc_pkg')
        config_file = os.path.join(package_share_directory, 'config', 'joint_vel_pub_config.yaml')

        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        # Sanity check the config file
        # self.print_joint_limits_from_config()

        # command velcoity publisher
        self.base_publisher_ = self.create_publisher(TwistStamped, '/base_cmd_vel', 10)
        self.arm_publisher_ = self.create_publisher(TwistStamped, '/arm_cmd_vel', 10)
        self.lift_publisher_ = self.create_publisher(TwistStamped, '/lift_cmd_vel', 10)
        self.wrist_publisher_ = self.create_publisher(TwistStamped, '/wrist_cmd_vel', 10)

        self.pub_base = False
        self.pub_arm = False
        self.pub_wrist_yaw = False
        self.pub_lift = False

        self.plot = False
        self.save = True
        self.freq = self.config["full_body"]["freq"]
        if self.config["full_body"]["pub"]:
            self.full_body_mode = self.config["full_body"]["mode"]
            self.pub_base = True
            self.pub_arm = True
            self.pub_lift = True
            self.pub_wrist_yaw = True
            if self.full_body_mode == "default":
                self.base_mode = "default_both"
                self.arm_mode = "default"
                self.lift_mode = "default"
                self.wrist_yaw_mode = "default"
            elif self.full_body_mode =="slow":
                self.base_mode = "slow_both"
                self.arm_mode = "slow"
                self.lift_mode = "slow"
                self.wrist_yaw_mode = "slow"
            elif self.full_body_mode == "fast":
                self.base_mode = "fast_both"
                self.arm_mode = "fast"
                self.lift_mode = "fast"
                self.wrist_yaw_mode = "fast"
        else:
            self.base_mode = self.config["base"]["mode"]
            self.arm_mode = self.config["arm"]["mode"]
            self.wrist_yaw_mode = self.config["wrist_yaw"]["mode"]
            self.lift_mode = self.config["lift"]["mode"]

            self.pub_base = self.config["base"]["pub"]
            self.pub_arm = self.config["arm"]["pub"]
            self.pub_lift = self.config["lift"]["pub"]
            self.pub_wrist_yaw = self.config["wrist_yaw"]["pub"]

        # hacky thing to publish zero on all topics
        _start = time.time()
        while time.time() - _start < 3.0:
            zero_msg = TwistStamped()
            zero_msg.header.stamp = self.get_clock().now().to_msg()
            self.base_publisher_.publish(zero_msg)
            self.arm_publisher_.publish(zero_msg)
            self.lift_publisher_.publish(zero_msg)
            self.wrist_publisher_.publish(zero_msg)
            time.sleep(0.2)

        self.time_start = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.1, self.timer_callback)

    def plot_init(self):
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-2, 2)
        return self.ln,

    def update_plot(self):
        # self.ax.set_xlim(0, max(self.x_data[-1], 10))  # Dynamically adjust x limit
        self.ln.set_data(self.x_data, self.linear_data)
        return self.ln,

    def l_vel_profile(self, time_elapsed, freq, limits):
        min, max = limits
        amp = (max - min) / 2
        offset = (max + min) / 2
        raw_vel = np.sin(time_elapsed * 2 * np.pi * freq)
        l_vel = amp * raw_vel + offset
        return l_vel

    def ang_vel_profile(self, time_elapsed, freq, limits):
        min, max = limits
        amp = (max - min) / 2
        offset = (max + min) / 2
        raw_vel = np.sin(time_elapsed * 2 * np.pi * freq)
        a_vel = amp * raw_vel + offset
        # may need to do clamping or somehting here
        return a_vel

    def print_joint_limits_from_config(self,):
        print("Joint Limits:")
        for joint, params in self.config.items():
            if 'limits' in params:
                print(f"{joint.capitalize()} Limits:")
                for limit_type, values in params['limits'].items():
                    print(f"  {limit_type.capitalize()}: {values}")
            else:
                print(f"{joint.capitalize()} has no limits defined.")

    def timer_callback(self,):

        current_time = self.get_clock().now()
        current_time_sec = (self.get_clock().now().nanoseconds - self.time_start) / 1e9
        time_elapsed = (current_time.nanoseconds - self.time_start) / 1e9

        if self.pub_base:
            base_msg = TwistStamped()
            base_freq = self.freq

            base_mode = self.base_mode
            if base_mode == "default_linear":
                l_limits = self.config["base"]["limits"]["linear"]["default"]
                a_limits = [0.0, 0.0]
            elif base_mode == "default_both":
                l_limits = self.config["base"]["limits"]["linear"]["default"]
                a_limits = self.config["base"]["limits"]["angular"]["default"]
            elif base_mode == "default_angular":
                l_limits = [0.0, 0.0]
                a_limits = self.config["base"]["limits"]["angular"]["default"]
            elif base_mode == "slow_linear":
                l_limits = self.config["base"]["limits"]["linear"]["slow"]
                a_limits = [0.0, 0.0]
            elif base_mode == "slow_both":
                l_limits = self.config["base"]["limits"]["linear"]["slow"]
                a_limits = self.config["base"]["limits"]["angular"]["slow"]
            elif base_mode == "slow_angular":
                l_limits = [0.0, 0.0]
                a_limits = self.config["base"]["limits"]["angular"]["slow"]
            elif base_mode == "fast_linear":
                l_limits = self.config["base"]["limits"]["linear"]["fast"]
                a_limits = [0.0, 0.0]
            elif base_mode == "fast_both":
                l_limits = self.config["base"]["limits"]["linear"]["fast"]
                a_limits = self.config["base"]["limits"]["angular"]["fast"]
            elif base_mode == "fast_angular":
                l_limits = [0.0, 0.0]
                a_limits = self.config["base"]["limits"]["angular"]["fast"]

            # base_amp = self.config["base"]["amp"]
            # l_limits = self.config["base"]["limits"]["linear"]
            # a_limits = self.config["base"]["limits"]["angular"]

            # compute the linear and angular velocity for base joint
            linear_vel = self.l_vel_profile(time_elapsed, base_freq, l_limits)
            angular_vel = self.ang_vel_profile(time_elapsed, base_freq, a_limits)

            # if self.config["base"]["limits"]["linear"][0] == 0.0 and self.config["base"]["limits"]["linear"][1] == 0.0:
            #     linear_vel = 0.0
            # if self.config["base"]["limits"]["angular"][0] == 0.0 and self.config["base"]["limits"]["angular"][1] == 0.0:
            #     angular_vel = 0.0

            base_msg.header.stamp = current_time.to_msg()
            base_msg.header.frame_id = 'base_link'
            base_msg.twist.linear.x = linear_vel
            base_msg.twist.linear.y = 0.0
            base_msg.twist.linear.z = 0.0
            base_msg.twist.angular.x = 0.0
            base_msg.twist.angular.y = 0.0
            base_msg.twist.angular.z = angular_vel

            # Publish base cmd vel to /base_cmd_vel topic
            self.base_publisher_.publish(base_msg)

        if self.pub_lift:
            lift_msg = TwistStamped()
            lift_freq = self.freq
            # lift_amp = self.config["base"]["amp"]\
            # lift_limits = self.config["lift"]["limits"]["linear"]
            lift_mode = self.lift_mode
            if lift_mode == "default":
                lift_limits = self.config["lift"]["limits"]["default"]
            elif lift_mode == "slow":
                lift_limits = self.config["lift"]["limits"]["slow"]
            elif lift_mode == "fast":
                lift_limits = self.config["lift"]["limits"]["fast"]

            # compute the linear velocity for lift joint
            linear_vel = self.l_vel_profile(time_elapsed, lift_freq, lift_limits)

            lift_msg.header.stamp = current_time.to_msg()
            lift_msg.header.frame_id = 'base_link'
            lift_msg.twist.linear.x = linear_vel
            lift_msg.twist.linear.y = 0.0
            lift_msg.twist.linear.z = 0.0
            lift_msg.twist.angular.x = 0.0
            lift_msg.twist.angular.y = 0.0
            lift_msg.twist.angular.z = 0.0

            # Publish base cmd vel to /base_cmd_vel topic
            self.lift_publisher_.publish(lift_msg)

        if self.pub_arm:
            arm_msg = TwistStamped()
            arm_freq = self.freq
            # arm_amp = self.config["base"]["amp"]
            arm_mode = self.arm_mode
            if arm_mode == "default":
                arm_limits = self.config["arm"]["limits"]["default"]
            elif arm_mode == "slow":
                arm_limits = self.config["arm"]["limits"]["slow"]
            elif arm_mode == "fast":
                arm_limits = self.config["arm"]["limits"]["fast"]

            # compute the linear velocity for lift joint
            linear_vel = self.l_vel_profile(time_elapsed, arm_freq, arm_limits)

            arm_msg.header.stamp = current_time.to_msg()
            arm_msg.header.frame_id = 'base_link'
            arm_msg.twist.linear.x = linear_vel
            arm_msg.twist.linear.y = 0.0
            arm_msg.twist.linear.z = 0.0
            arm_msg.twist.angular.x = 0.0
            arm_msg.twist.angular.y = 0.0
            arm_msg.twist.angular.z = 0.0

            self.arm_publisher_.publish(arm_msg)

        if self.pub_wrist_yaw:
            wrist_msg = TwistStamped()
            wrist_freq = self.freq
            wrist_yaw_mode = self.wrist_yaw_mode
            if wrist_yaw_mode == "default":
                wrist_limits = self.config["wrist_yaw"]["limits"]["default"]
            elif wrist_yaw_mode == "slow":
                wrist_limits = self.config["wrist_yaw"]["limits"]["slow"]
            elif wrist_yaw_mode == "fast":
                wrist_limits = self.config["wrist_yaw"]["limits"]["fast"]

            # compute the angular velocity for wrist_yaw
            ang_vel = self.ang_vel_profile(time_elapsed, wrist_freq, wrist_limits)

            wrist_msg.header.stamp = current_time.to_msg()
            wrist_msg.header.frame_id = 'base_link'
            wrist_msg.twist.linear.x = 0.0
            wrist_msg.twist.linear.y = 0.0
            wrist_msg.twist.linear.z = 0.0
            wrist_msg.twist.angular.x = 0.0
            wrist_msg.twist.angular.y = 0.0
            wrist_msg.twist.angular.z = ang_vel

            # Publish base cmd vel to /base_cmd_vel topic
            self.wrist_publisher_.publish(wrist_msg)


def main(args=None):

    rclpy.init(args=args)
    node = SineVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
