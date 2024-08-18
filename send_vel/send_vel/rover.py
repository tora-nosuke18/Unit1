import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class Rover(Node):
    """Math and motor control algorithms to move the rover"""

    def __init__(self):
        super().__init__("rover")
        self.log = self.get_logger()
        self.declare_parameter('tread', 0.5)

        self.targets_pub = self.create_publisher(Float64MultiArray, "/rover_targets", 1)

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 1)

    def cmd_cb(self, twist):
        wheel_msg = Float64MultiArray()

        tread = self.get_parameter('tread').get_parameter_value()._double_value
        self.log.info(f'tread : {tread} m/s')

        left_wheel = twist.linear.x - twist.angular.z * 0.5 * tread
        right_wheel = twist.linear.x + twist.angular.z * 0.5 * tread    

        wheel_msg.data = [left_wheel, left_wheel, right_wheel, right_wheel] 

        self.targets_pub.publish(wheel_msg)

        # self.log.info(f'Left wheel velocity : {left_wheel:.2f} m/s')


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
