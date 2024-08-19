import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_transformations

class WheelVelToPose(Node):
    def __init__(self):
        super().__init__('wheel_vel_to_pose')
        self.left_wheel_vel_sub = self.create_subscription(
            Float64,
            '/C620/left_wheel_vel',
            self.left_wheel_vel_callback,
            10)
        self.right_wheel_vel_sub = self.create_subscription(
            Float64,
            '/C620/right_wheel_vel',
            self.right_wheel_vel_callback,
            10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)

        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

    def left_wheel_vel_callback(self, msg):
        self.left_wheel_vel = msg.data
        self.publish_pose()

    def right_wheel_vel_callback(self, msg):
        self.right_wheel_vel = msg.data
        self.publish_pose()

    def publish_pose(self):
        # Placeholder logic to compute yaw from wheel velocities
        yaw = (self.right_wheel_vel - self.left_wheel_vel) / 2.0

        # Convert yaw to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelVelToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
