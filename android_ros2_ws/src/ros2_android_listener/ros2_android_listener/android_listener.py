import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data, qos_profile_services_default, qos_profile_parameters
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2 
import time


class ListenerQos(Node):
    def __init__(self, qos_profile, msgtype):
        super().__init__('listener')
        if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('QOS Reliable Listener')
        else:
            self.get_logger().info('QOS Sensor data/ Best effort listener')

        if msgtype == "text":
            self.sub = self.create_subscription(String, 'chatter', self.chatter_callback_text, qos_profile=qos_profile)
        else:
            self.sub = self.create_subscription(Image, 'chatter', self.chatter_callback, qos_profile=qos_profile)

        self.i = 1
        self.start_time = time.time()
        self.fps_interval = 1
        self.fps_count = 0
        self.realtime_fps = 0
        self.windows = 0
        self.window_height = 480
        self.window_width = 853
        self.window_name = "ROS2 Stream"

    def chatter_callback(self, msg):
        self.get_logger().info('I heard')
        self.fps_count += 1
        buf = np.asarray(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        img = cv2.transpose(img)

        if (time.time() - self.start_time) > self.fps_interval:
            self.realtime_fps = self.fps_count / (time.time() - self.start_time)
            self.fps_count = 0
            self.start_time = time.time()
        
        fps_label = 'FPS:{0:.2f}'.format(self.realtime_fps)
        cv2.putText(img, fps_label, (img.shape[1] - 160, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        if self.windows != img.shape[1]:
            if self.i != 1:
                cv2.destroyAllWindows()

            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
            self.windows = img.shape[1]
            self.i += 1

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def chatter_callback_text(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.data)


def main(args=None):
    parser = argparse.ArgumentParser(description="Ros2 listener")
    parser.add_argument("--qos", default="default", help="Setting ROS 2 Quality of Service policies (default, services, parameters, sensor).")
    parser.add_argument("--msg", default="camera", help="Setting ROS 2 message (camera, text).")
    args = parser.parse_args()
    print(args.qos)

    rclpy.init(args=None)
    if args.qos == "default":
        # custom_qos_profile = qos_profile_default
        custom_qos_profile = qos_profile_system_default
    elif args.qos == "services":
        custom_qos_profile = qos_profile_services_default
    elif args.qos == "parameters":
        custom_qos_profile = qos_profile_sensor_data
    else:
        custom_qos_profile = qos_profile_system_default
    
    node = ListenerQos(custom_qos_profile, args.msg)

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()