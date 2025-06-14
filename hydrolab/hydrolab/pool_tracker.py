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

        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.get_logger().info('pool_tracker node initialized.')

    def image_callback(self, msg: Image):
        """
        Wywoływane przy każdej nowej klatce: próba wykrycia środka niebieskiego basenu.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cvBridge error: {e}')
            return
        center = self.find_blue_center(frame)
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
            self.get_logger().info(f'Blue pool center: ({cx:.1f}, {cy:.1f})')

    def find_blue_center(self, frame: np.ndarray):
        """
        Wyszukuje obszar niebieskiego basenu (w oparciu o RGB-thresholdy), 
        zwraca środek masy wykrytego konturu lub None, jeśli brak.
        Thresholdy w przestrzeni RGB: 
          R ∈ [0, 67], G ∈ [52, 141], B ∈ [83, 255].
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        R = rgb[:, :, 0]
        G = rgb[:, :, 1]
        B = rgb[:, :, 2]
        mask = (
            (R >= 0)   & (R <= 67)  &
            (G >= 52)  & (G <= 141) &
            (B >= 83)  & (B <= 255)
        ).astype(np.uint8) * 255
        kernel = np.ones((5, 5), np.uint8)
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
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        return (cx, cy)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoolTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()