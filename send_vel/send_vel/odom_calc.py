import math
from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdometryCalc(Node):
    def __init__(self):
        super().__init__('odom_calc')

        self.r = 0.055  # 車輪半径
        self.TREAD = 1  # トレッド幅

        self.x = 0.0  # x座標
        self.y = 0.0  # y座標
        self.th = 0.0  # 姿勢

        self.vx = 0.0  # x方向速度
        self.vy = 0.0  # y方向速度
        self.vth = 0.0  # 角速度

        self.prev_left_enc = 0.0  # 前回の左エンコーダ値
        self.prev_right_enc = 0.0  # 前回の右エンコーダ値

        self.prev_time = self.get_clock().now()  # 前回の時間
        
        self.current_left_enc = 0.0  # 現在の左エンコーダ値
        self.current_right_enc = 0.0  # 現在の右エンコーダ値

        self.odom_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float64, '/C620/left_wheel_vel', self.callback_l, 10)
        self.create_subscription(Float64, '/C620/right_wheel_vel', self.callback_r, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.timer = self.create_timer(0.1, self.publish_odom)  # 10Hzで実行

    def callback_l(self, msg):
        # エンコーダ値を取得
        self.get_logger().info(f"l {msg.data:.3f}")
        self.current_left_enc = msg.data  # 左エンコーダ値

    def callback_r(self, msg):
        # エンコーダ値を取得
        self.get_logger().info(f"r {msg.data:.3f}")
        self.current_right_enc = msg.data  # 右エンコーダ値
    
    def publish_odom(self):
        # エンコーダ値を取得
        left_enc = self.current_left_enc
        right_enc = self.current_right_enc

        # 経過時間を計算
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # ナノ秒を秒に変換

        # 角速度から速度を計算
        self.vx = self.r * (right_enc + left_enc) / 2
        self.vy = 0.0
        self.vth = self.r * (right_enc - left_enc) / (dt * self.TREAD)
        
        # 速度からオドメトリを計算
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.get_logger().info(f"dt {dt:.3f}")
        self.x += delta_x
        self.y += delta_y  
        self.th += delta_th

        # クオータニオンを計算
        odom_quat = quaternion_from_euler(0, 0, self.th)

        # tf変換を送信
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation.x = odom_quat[0]
        odom_trans.transform.rotation.y = odom_quat[1]
        odom_trans.transform.rotation.z = odom_quat[2]
        odom_trans.transform.rotation.w = odom_quat[3]

        self.odom_broadcaster.sendTransform(odom_trans)

        # オドメトリメッセージをROS 2で配信
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"

        # 位置を設定
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))

        # 速度を設定
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0.0), Vector3(0, 0, self.vth))

        # オドメトリを配信
        self.odom_pub.publish(odom)
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    odom_calc = OdometryCalc()
    rclpy.spin(odom_calc)

    odom_calc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
