import cv2
import mediapipe as mp
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Mediapipe設定
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# グローバル変数
decide = True
count = 0
ptime = 0
close_check = False
open_check = False

# ROS2ノードの初期化
class HandControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)
        self.pTime = 0
        self.timer = self.create_timer(0.1,self.run)

    def run(self):
        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5) as hands:

            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break

                # BGR to RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_rgb.flags.writeable = False

                results = hands.process(frame_rgb)

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                        keypoints = self.take_coordinates([hand_landmarks])
                        self.calculate(keypoints)

                ctime = time.time()
                fps = 1 / (ctime - self.pTime)
                self.pTime = ctime

                cv2.putText(frame, f'FPS: {int(fps)}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Frame", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.cap.release()
        cv2.destroyAllWindows()

    def calculate(self, keypoints):
        if not keypoints:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            print("Publishing: Stopped")
            self.cmd_vel_pub.publish(twist)
            return

        center = self.centroid_palm(keypoints)
        angle = self.get_angle(keypoints[0], center)
        twist = Twist()

        if self.open_check_by_distance(keypoints, center):
            if -0.03 < keypoints[12][2] < 0.03:
                twist.linear.x = 0.0  # 停止
            else:
                if -0.03 >= keypoints[12][2]:
                    twist.linear.x = -10 * keypoints[12][2] # 前進
                elif keypoints[12][2] >= 0.03:
                    twist.linear.x = -10 * keypoints[12][2]  # 後退

            if 80 <= angle <= 100:
                twist.angular.z = 0.0  # 正面を向いているときハンドルは切らない
            else:
                twist.angular.z = -1 * (angle - 90) / 90.0  # 左右の回転を設定
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        print(f"Publishing Twist: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")
        self.cmd_vel_pub.publish(twist)

    def open_check_by_distance(self, keypoints, center):
        def thumb_open_check(keypoints, center):
            d4 = np.sqrt(np.square(keypoints[4][0] - center[0]) + np.square(keypoints[4][1] - center[1]))
            d3 = np.sqrt(np.square(keypoints[3][0] - center[0]) + np.square(keypoints[3][1] - center[1]))
            if d4 > d3:
                return True
            else:
                return False

        def index_open_check(keypoints, center):
            d5 = np.sqrt(np.square(keypoints[5][0] - center[0]) + np.square(keypoints[5][1] - center[1]))
            d6 = np.sqrt(np.square(keypoints[6][0] - center[0]) + np.square(keypoints[6][1] - center[1]))
            d7 = np.sqrt(np.square(keypoints[7][0] - center[0]) + np.square(keypoints[7][1] - center[1]))
            d8 = np.sqrt(np.square(keypoints[8][0] - center[0]) + np.square(keypoints[8][1] - center[1]))
            if d8 > d7 > d6 > d5:
                return True
            else:
                return False

        def middle_open_check(keypoints, center):
            d9 = np.sqrt(np.square(keypoints[9][0] - center[0]) + np.square(keypoints[9][1] - center[1]))
            d10 = np.sqrt(np.square(keypoints[10][0] - center[0]) + np.square(keypoints[10][1] - center[1]))
            d11 = np.sqrt(np.square(keypoints[11][0] - center[0]) + np.square(keypoints[11][1] - center[1]))
            d12 = np.sqrt(np.square(keypoints[12][0] - center[0]) + np.square(keypoints[12][1] - center[1]))
            if d12 > d11 > d10 > d9:
                return True
            else:
                return False

        def ring_open_check(keypoints, center):
            d13 = np.sqrt(np.square(keypoints[13][0] - center[0]) + np.square(keypoints[13][1] - center[1]))
            d14 = np.sqrt(np.square(keypoints[14][0] - center[0]) + np.square(keypoints[14][1] - center[1]))
            d15 = np.sqrt(np.square(keypoints[15][0] - center[0]) + np.square(keypoints[15][1] - center[1]))
            d16 = np.sqrt(np.square(keypoints[16][0] - center[0]) + np.square(keypoints[16][1] - center[1]))
            if d16 > d15 > d14 > d13:
                return True
            else:
                return False

        def pinky_open_check(keypoints, center):
            d17 = np.sqrt(np.square(keypoints[17][0] - center[0]) + np.square(keypoints[17][1] - center[1]))
            d18 = np.sqrt(np.square(keypoints[18][0] - center[0]) + np.square(keypoints[18][1] - center[1]))
            d19 = np.sqrt(np.square(keypoints[19][0] - center[0]) + np.square(keypoints[19][1] - center[1]))
            d20 = np.sqrt(np.square(keypoints[20][0] - center[0]) + np.square(keypoints[20][1] - center[1]))
            if d20 > d19 > d18 > d17:
                return True
            else:
                return False

        thumb = thumb_open_check(keypoints, center)
        index = index_open_check(keypoints, center)
        middle = middle_open_check(keypoints, center)
        ring = ring_open_check(keypoints, center)
        pinky = pinky_open_check(keypoints, center)
        if thumb == True and index == True and middle == True and ring == True and pinky == True:
            return True
        else:
            return False

    def close_check_by_distance(self, keypoints, center):
        d3 = np.sqrt(np.square(keypoints[3][0] - center[0]) + np.square(keypoints[3][1] - center[1]))
        d4 = np.sqrt(np.square(keypoints[4][0] - center[0]) + np.square(keypoints[4][1] - center[1]))
        d5 = np.sqrt(np.square(keypoints[5][0] - keypoints[0][0]) + np.square(keypoints[5][1] - keypoints[0][1]))
        d8 = np.sqrt(np.square(keypoints[8][0] - keypoints[0][0]) + np.square(keypoints[8][1] - keypoints[0][1]))
        d9 = np.sqrt(np.square(keypoints[9][0] - keypoints[0][0]) + np.square(keypoints[9][1] - keypoints[0][1]))
        d12 = np.sqrt(np.square(keypoints[12][0] - keypoints[0][0]) + np.square(keypoints[12][1] - keypoints[0][1]))
        d13 = np.sqrt(np.square(keypoints[13][0] - keypoints[0][0]) + np.square(keypoints[13][1] - keypoints[0][1]))
        d16 = np.sqrt(np.square(keypoints[16][0] - keypoints[0][0]) + np.square(keypoints[16][1] - keypoints[0][1]))
        d17 = np.sqrt(np.square(keypoints[17][0] - keypoints[0][0]) + np.square(keypoints[17][1] - keypoints[0][1]))
        d20 = np.sqrt(np.square(keypoints[20][0] - keypoints[0][0]) + np.square(keypoints[20][1] - keypoints[0][1]))

        if d8 < d5 and d12 < d9 and d16 < d13 and d20 < d17 and d4 < d3:
            return True
        else:
            return False


    def take_coordinates(self, coordinates):
        if coordinates is None:
            return 0
        keypoints = []
        for data_point in coordinates:
            xyz_datapoints = data_point.landmark
            for xyz in xyz_datapoints:
                X_value = round(xyz.x*10000, 2)
                Y_value = round(xyz.y*10000, 2)
                Z_value = round(xyz.z, 3)
                xy = [X_value,Y_value, Z_value]
                keypoints.append(xy)
        return keypoints

    def centroid_palm(self, keypoints):
        if keypoints == 0:
            return 0
        x_bar = (keypoints[0][0] + keypoints[9][0]) / 2
        x_bar = round(x_bar, 2)
        y_bar = (keypoints[0][1] + keypoints[9][1]) / 2
        y_bar = round(y_bar, 2)
        return x_bar, y_bar

    def get_angle(self, keypoints, center):
        if keypoints == 0:
            return 0

        center = list(center)
        wrist = list(keypoints)
        wrist[1] = 10000 - wrist[1]
        center[1] = 10000 - center[1]
        Y = center[1] - wrist[1]
        X = center[0] - wrist[0]
        try:
            m = Y / X
        except ZeroDivisionError:
            m = 0
        angle = np.arctan(m) * 180 / np.pi
        if X > 0 and Y < 0:
            angle = angle + 360
        elif X < 0 and Y > 0:
            angle = angle + 180
        elif X < 0 and Y < 0:
            angle = angle + 180
        return round(angle, 1)

def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
         print('Ctrl + C が押されました')
    node.run()
    rclpy.shutdown()
    print('プログラム終了')

if __name__ == '__main__':
    main()