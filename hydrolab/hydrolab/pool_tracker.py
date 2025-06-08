import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class PoolTracker(Node):
    def __init__(self):
        super().__init__('pool_tracker')
        self.bridge = CvBridge()
        # self.camera_matrix = None
        # self.dist_coeffs = None
        # subsykrypcje
        self.image_sub = self.create_subscription(
            Image, '/world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image', self.image_callback, 10)
        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo, '/world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info', self.camera_info_callback, 10)
        # publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.get_logger().info('pool_tracker node initialized.')

    # def camera_info_callback(self, msg: CameraInfo):
    #     """camera info params"""
    #     self.camera_matrix = np.array(msg.k).reshape((3,3))
    #     self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        """
        proba wykrycia srodka obrazu
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cvBridge error: {e}')
            return
        center = self.find_red_center(frame)
        if center is not None:
            cx, cy = center
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position.x = float(cx)
            pose.pose.position.y = float(cy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            #self.get_logger().info(f'red pool center: ({cx:.1f}, {cy:.1f})')

    def find_red_center(self, frame: np.ndarray):
        """
        zwraca srodek wykrytego okregu.
        konwersja do hsv, maskowanie, morfologia, kontury, filtr minim obszaru.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1, upper1 = np.array([0,70,50]), np.array([10,255,255])
        lower2, upper2 = np.array([170,70,50]), np.array([180,255,255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 500:
            return None
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        return (M['m10']/M['m00'], M['m01']/M['m00'])

def main(args=None):
    rclpy.init(args=args)
    node = PoolTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
