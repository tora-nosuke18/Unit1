import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from osr_interfaces.msg import CommandDrive


pi=3.14159265
class Rover(Node):
    """Math and motor control algorithms to move the rover"""

    def __init__(self):
        super().__init__("rover")
        self.log = self.get_logger()
        self.declare_parameter('tread', '0.5')

        self.left_pub = self.create_publisher(Float64MultiArray,"/rover/left_targets", 1)
        self.right_pub = self.create_publisher(Float64MultiArray,"/rover/right_targets", 1)       


        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel",self.cmd_cb, 1)

    def cmd_cb(self, twist_msg):

        drive_cmd_msg = CommandDrive()
        drive_cmd_msg = self.calculatedrive_velocities_lrp_unit1(twist_msg)
        
        left_msg = [drive_cmd_msg.left_front_vel]
        left_msg.append(drive_cmd_msg.left_back_vel)

        right_msg = [drive_cmd_msg.right_front_vel]
        right_msg.append(drive_cmd_msg.right_back_vel)

        left_msg = Float64MultiArray(data=left_msg)
        right_msg = Float64MultiArray(data=right_msg)
        
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.log.info(f'Left wheel velocity : {drive_cmd_msg.left_front_vel:.2f} m/s')
        self.log.info(f'Right wheel velocity : {drive_cmd_msg.right_front_vel:.2f} m/s')

    def calculatedrive_velocities_lrp_unit1(self, twist):

        tread =  self.get_parameter('tread').get_parameter_value().double_value

        left_wheel_direct = twist.linear.x - 0.5 * twist.angular.z * tread
        right_wheel_direct = twist.linear.x + 0.5 * twist.angular.z * tread

        cmd_msg = CommandDrive()
        cmd_msg.left_front_vel =  left_wheel_direct
        cmd_msg.left_back_vel =  left_wheel_direct
        cmd_msg.right_back_vel =  right_wheel_direct
        cmd_msg.right_front_vel =  right_wheel_direct

        return cmd_msg


def main():
    rclpy.init()
    rover = Rover()
    try:
        rclpy.spin(rover)
    except:
        return 
    rover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()