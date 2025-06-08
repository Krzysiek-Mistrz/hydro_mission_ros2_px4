import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        # parametry
        self.declare_parameter('aruco_id', 0)
        self.declare_parameter('dictionary', 2)
        self.declare_parameter('marker_size', 0.5)

        self.aruco_id = self.get_parameter('aruco_id').value
        self.dictionary_id = self.get_parameter('dictionary').value
        self.marker_size = self.get_parameter('marker_size').value

        # inicjalizacja detektora aruko za pomoca cv2
        self.dictionary = aruco.getPredefinedDictionary(self.dictionary_id)
        self.detector_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.detector_params)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # subskrypcje
        self.image_sub = self.create_subscription(
            Image,
            '/world/betteraruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image',
            self.image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/world/betteraruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info',
            self.camera_info_callback,
            10
        )

        # publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

    # k - macierz kalibracji, d - macierz znieksztalcen 
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        #self.get_logger().info('Otrzymano dane kalibracyjne kamery.')

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Brak danych kalibracyjnych kamery.')
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Konwersja obrazu nie powiodla sie: {e}")
            return

        # detekcja cornerow aruko
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        if corners:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != self.aruco_id:
                    continue
                # robi krotki 4 pozycji punktow cornerow aruko
                marker_corners = corners[i].reshape((4, 2))
                half_size = self.marker_size / 2.0
                object_points = np.array([
                    [-half_size,  half_size, 0],
                    [ half_size,  half_size, 0],
                    [ half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)

                image_points = marker_corners.astype(np.float32)
                # mussimy wykorzystac camera info zeby rzutowac punkty z 3d na 2d
                # zwraca macierze rotacji i translacji tych punktow na 2D
                ret, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
                if ret:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = msg.header.stamp
                    pose_msg.header.frame_id = "camera_frame"
                    pose_msg.pose.position.x = float(tvec[0])
                    pose_msg.pose.position.y = float(tvec[1])
                    pose_msg.pose.position.z = float(tvec[2])
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = 0.0
                    pose_msg.pose.orientation.w = 1.0

                    self.pose_pub.publish(pose_msg)
                    self.get_logger().info(f"Marker {marker_id} wykryty, wspolrzedne: {tvec.flatten()}")
                    break
        else:
            self.get_logger().info("Nie wykryto markerow.")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()