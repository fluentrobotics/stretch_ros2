import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
from concurrent.futures import ThreadPoolExecutor

class BaseArc(Node):

    def __init__(self):
        super().__init__('BaseArc')
        self.base_publisher = self.create_publisher(TwistStamped, '/base_cmd_vel', 1)
        self.ee_publisher = self.create_publisher(TwistStamped, '/wrist_cmd_vel', 1)
        
        self.latest_joint_state: JointState | None = None
        
        self.joint_state_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_state_callback, 1)
        
        self.done = False
        self.timer = self.create_timer(1 , self.null_space_control_align)


    def joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

        
    def null_space_control_align(self) -> bool:
        if self.done:
            return True 
        while self.latest_joint_state is None:
            print("WAITING")
            time.sleep(0.5)
        
        idx = self.latest_joint_state.name.index("joint_wrist_yaw")
        theta = self.latest_joint_state.position[idx]
        
        ANGULAR_SPEED = 0.2
        duration = abs(theta) / ANGULAR_SPEED
        
        BASE_ROTATION_RADIUS = 0.24
        
        wrist_twist_msg = TwistStamped()
        wrist_twist_msg.twist.angular.z = math.copysign(1.0, -theta) * ANGULAR_SPEED
        
        base_twist_msg = TwistStamped()
        base_twist_msg.twist.angular.z = math.copysign(1.0, theta) * ANGULAR_SPEED
        base_twist_msg.twist.linear.x = BASE_ROTATION_RADIUS * ANGULAR_SPEED * math.copysign(1.0, -theta)
        
        
        start = time.time()
        while time.time() - start < duration:
            self.base_publisher.publish(base_twist_msg)
            self.ee_publisher.publish(wrist_twist_msg)
            time.sleep(1 / 20)
        
        self.done = True
        return True
        
        
    
    def move_ee(self, wz, duration):
        """
        Rotates the joint at a specified angular velocity for a given duration.

        Parameters:
        wz (float): Angular velocity in radians per second.
        duration (float): Duration in seconds to apply the angular velocity.
        """
        # Create a Twist message
        twist_msg = TwistStamped()        
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = wz

        # Publish the velocity command at a rate of 10 Hz
        end_time = time.time() + duration
        self.get_logger().info(f"Starting joint rotation at {wz} rad/s for {duration:.2f} seconds.")

        while time.time() < end_time:
            self.ee_publisher.publish(twist_msg)
            time.sleep(0.1)  # Publish at 10 Hz

        # Stop the joint after rotation
        stop_msg = TwistStamped()
        self.ee_publisher.publish(stop_msg)
        self.get_logger().info("Joint rotation completed and stopped.")

    def move_base_arc(self, vx, wz, alpha_deg, full_circle_time):
        """
        Moves the robot to make an arc of alpha_deg degrees.

        Parameters:
        vx (float): Linear velocity in m/s.
        wz (float): Angular velocity in rad/s.
        alpha_deg (float): Angle of the arc in degrees.
        full_circle_time (float): Time to complete a full circle in seconds.
        """
        # Compute the time to make the arc
        T_alpha = self.compute_arc_time(alpha_deg, full_circle_time)
        
        # Create a TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = wz
        
        print("Total time to make arc: ", T_alpha)

        # Publish the velocity command at a rate of 10 Hz
        end_time = time.time() + T_alpha
        while time.time() < end_time:
            twist_msg.header.stamp = self.get_clock().now().to_msg()  # Update the timestamp
            self.base_publisher.publish(twist_msg)
            self.get_logger().info(f'Publishing: {twist_msg}')
            time.sleep(0.1)  # 10 Hz
        
        # Stop the robot after the arc is completed
        stop_msg = TwistStamped()  # Zero velocities to stop the robot
        stop_msg.header.stamp = self.get_clock().now().to_msg()  # Update the timestamp
        self.base_publisher.publish(stop_msg)
        self.get_logger().info('Stopping the robot.')
        
    def compute_base_vel(self, l, T):
        vx = (2*math.pi*l) / T
        wz = (2*math.pi) / T
        return vx, wz * -1

def main(args=None):
    rclpy.init(args=args)
    base_arc = BaseArc()
    
    rclpy.spin(base_arc)

    
    # l = 0.24
    # T = 40
    # ee_wz = 0.2
    # beta_deg = 90
    
    # # vx_, wz_ = base_arc.compute_base_vel(l, T)
    # # T_ee = base_arc.compute_ee_rotation_time(beta_deg, ee_wz)
    
    # # print(T_ee)
    # # print(vx_, wz_)
    
    # # base_arc.move_base_arc(vx_, wz_, 180, T)

    # # base_arc.destroy_node()
    # # rclpy.shutdown()
    # vx_, wz_ = base_arc.compute_base_vel(l, T)
    # T_ee = base_arc.compute_ee_rotation_time(beta_deg, ee_wz)
    # # base_arc.move_ee(ee_wz, T_ee)
    
    # print(f"EE Rotation Time: {T_ee:.2f} seconds")
    # print(f"Base Velocities: vx={vx_:.4f} m/s, wz={wz_:.4f} rad/s")
    
    # with ThreadPoolExecutor(max_workers=2) as executor:
    #     executor.submit(base_arc.move_base_arc, vx_, wz_, 180, T)
    #     executor.submit(base_arc.move_ee, ee_wz, T_ee)

    base_arc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
