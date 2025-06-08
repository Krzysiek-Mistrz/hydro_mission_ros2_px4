#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2


class RealSenseCVPublisher(Node):
    """
    ROS 2 node publikujący obraz RGB z Intel RealSense (videodevice #4) 
    oraz odpowiadające CameraInfo (dla kamery D435i) na topici:
    /camera/image       (sensor_msgs/Image)
    /camera/image_info  (sensor_msgs/CameraInfo)
    """

    def __init__(self):
        super().__init__('camera_image')

        self.br = CvBridge()

        self.device_index = 4
        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera on /dev/video{self.device_index}')
            raise RuntimeError(f'Cannot open video device {self.device_index}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.get_logger().info(f'Camera opened Intel RealSense as /dev/video{self.device_index} (640x480 @30fps)')

        self.image_pub = self.create_publisher(Image, '/camera/image', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/image_info', 10)

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = 'camera_rgb_frame'
        self.camera_info_msg.height = 480
        self.camera_info_msg.width = 640
        self.camera_info_msg.distortion_model = 'plumb_bob'
        fx = 615.0
        fy = 615.0
        cx = 320.0
        cy = 240.0
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [fx, 0.0, cx,
                                  0.0, fy, cy,
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.r = [1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.p = [fx, 0.0, cx, 0.0,
                                  0.0, fy, cy, 0.0,
                                  0.0, 0.0, 1.0, 0.0]
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Co ~33 ms: odczyt klatki BGR z cv2, konwersja do ROS Image i publikacja.
        Następnie w tej samej funkcji publikujemy odpowiednie CameraInfo.
        """
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Empty frame -> obsolete')
            return
        try:
            ros_img = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            stamp = self.get_clock().now().to_msg()
            ros_img.header.stamp = stamp
            ros_img.header.frame_id = 'camera_rgb_frame'
            self.image_pub.publish(ros_img)
            self.camera_info_msg.header.stamp = stamp
            self.info_pub.publish(self.camera_info_msg)
            self.get_logger().debug('Image and CameraInfo sent successfully')
        except Exception as e:
            self.get_logger().error(f'Error during image conversion: {e}')

    def destroy_node(self):
        """
        Przy zamykaniu: zwalniamy VideoCapture i niszczymy node.
        """
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealSenseCVPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt -> closing node')
    except Exception as e:
        print(f'Error during init.: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
