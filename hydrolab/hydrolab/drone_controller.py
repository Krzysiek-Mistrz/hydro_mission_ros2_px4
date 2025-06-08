import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import PoseStamped
import time, math
from sensor_msgs.msg import CameraInfo
import numpy as np

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pos_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile_sub
        )
        self.pool_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.pool_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar/distance',
            self.lidar_callback,
            qos_profile_sub
        )
        
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        self.pool_positions = []
        self.collecting_pool = False
        self.pool_collection_done = False
        self.offboard_setpoint_counter = 0
        self.current_drone_pos = (0.0, 0.0, 0.0)
        
        # Zmienna określająca aktualny stan misji
        # Nowa sekwencja:
        # 1: punkt A (start, takeoff do 2 m)
        # 2: punkt B (przelot do (10,0,-2))
        # 3: punkt C - pool marker (uśrednione dane)
        # 4: obnizenie (do 0 m) przy pozycji markera
        # 5: wzniesienie (do 2.0 m) przy pozycji markera
        # 6: powrot do punktu A (0,0,-2)
        # 7: ladowanie (0,0,0)
        self.current_state = 1
        self.reach_tolerance = 0.1
        self.current_height = None
        
        # lista punktow docelowych
        self.target_positions = [
            None,
            (0.0, 0.0, -2.0),
            (10, 0, -2.0),
            None,                  
            None,                  
            None,                  
            (0.0, 0.0, -2.0),      
            (0.0, 0.0,  0.0)       
        ]
        
        # Zestaw 1 → Zestaw 2 (w ned):
        #     ΔX ≈ −24.1 m (na wschód)
        #     ΔY ≈ +127.7 m (na południe)
        # Zestaw 2 → Zestaw 3:
        #     ΔX ≈ +55.3 m (na zachód)
        #     ΔY ≈ −113.8 m (na północ)

        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Arm command sent")
        
        self.publish_offboard_control_mode()
        self.publish_setpoint()
        
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def position_callback(self, msg: VehicleLocalPosition):
        current_x = msg.x
        current_y = msg.y
        current_z = msg.z
        #potrzebne do zamiany pozycji relatywnej na globalna do aruko
        self.current_drone_pos = (current_x, current_y, current_z)
        
        if self.current_state in [1,2,3,4,5,6,7]:
            target = self.target_positions[self.current_state]
            # dla st. 3,4,5 pomijamy sprawdzenie bo pozycja pool
            if target is None:
                return
            
            target_x, target_y, target_z = target
            dx = current_x - target_x
            dy = current_y - target_y
            dz = current_z - target_z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        
            if distance < self.reach_tolerance:
                if self.current_state == 4:
                    self.get_logger().info("Doleciano do z=0 nad pool; czekam 20 s…")
                    time.sleep(20)
                # gdy dolecimy do b rozpoczynamy detekcje pozycji aruko
                if self.current_state == 2:
                    if not self.collecting_pool and not self.pool_collection_done:
                        self.get_logger().info("Osiagnieto PUNKT B; rozpoczynam detekcje basenu.")
                        self.collecting_pool = True
                    if self.pool_collection_done:
                        self.current_state = 3
                        self.get_logger().info("pool detection complete. Lecimy do punktu C")
                else:
                    self.current_state += 1
                    if self.current_state == 1:
                        self.get_logger().info("Osiagnięto PUNKT A (0,0,-2); start misji.")
                    elif self.current_state == 3:
                        self.get_logger().info("Osiagnieto PUNKT C (pozycja pool).")
                    elif self.current_state == 4:
                        self.get_logger().info("Obnizanie do 0 m przy pozycji pool.")
                        desired_down = 0
                        if self.current_height is not None:
                            desired_down = msg.z + (self.current_height - 0)
                        self.target_positions[4] = (
                            self.target_positions[3][0],
                            self.target_positions[3][1],
                            desired_down
                        )
                    elif self.current_state == 5:
                        self.get_logger().info("Wznoszenie do 2.0 m przy p 0 pozycji pool.")
                        desired_up = -2.0
                        if self.current_height is not None:
                            desired_up = msg.z + (self.current_height - 2.0)
                        self.target_positions[5] = (
                            self.target_positions[3][0],
                            self.target_positions[3][1],
                            desired_up
                        )
                    elif self.current_state == 6:
                        self.get_logger().info("Powrot do PUNKTU A (0,0,-2).")
                    elif self.current_state == 7:
                        self.get_logger().info("Ladowanie (0,0,0).")
                    elif self.current_state > 7:
                        self.get_logger().info("Misja zakonczona.")
                        self.timer.cancel()
                        rclpy.shutdown()

    def lidar_callback(self, msg: LaserScan):
        if len(msg.ranges) > 0:
            self.current_height = msg.ranges[0]
            #logging co 10 pomiarow
            if self.current_state in [4,5]:
                self.get_logger().info(f"Lidar height: {self.current_height:.2f} m")

    def camera_info_callback(self, msg: CameraInfo):
        """camera info params"""
        self.camera_matrix = np.array(msg.k).reshape((3,3))
        self.dist_coeffs = np.array(msg.d)

    def pool_callback(self, msg: PoseStamped):
        if self.collecting_pool and not self.pool_collection_done:
            pos = msg.pose.position
            #pos.x,pos.y to piksele (u,v)
            self.pool_positions.append((pos.x, pos.y))
            if len(self.pool_positions) == 100:
                #uwaga! pozycja wykrycia na plaszczyznie kamery
                sum_x = sum(p[0] for p in self.pool_positions)
                sum_y = sum(p[1] for p in self.pool_positions)
                avg_x = sum_x / 100.0
                avg_y = sum_y / 100.0
                current_x, current_y, _ = self.current_drone_pos

                fx = self.camera_matrix[0,0]
                fy = self.camera_matrix[1,1]
                cx = self.camera_matrix[0,2]
                cy = self.camera_matrix[1,2]
                Z = 2.0
                
                #przeliczamy piksele na metry
                X_offset = (avg_x - cx) * Z / fx
                Y_offset = (avg_y - cy) * Z / fy
                global_marker_x = current_x + X_offset
                global_marker_y = current_y + Y_offset

                self.target_positions[3] = (global_marker_x, global_marker_y, -2.0)
                self.target_positions[4] = (global_marker_x, global_marker_y, 0)
                self.target_positions[5] = (global_marker_x, global_marker_y, -2.0)
                
                self.pool_collection_done = True
                self.collecting_pool = False

                self.get_logger().info(
                    f"Pixel center avg: ({avg_x:.1f}, {avg_y:.1f}), "
                    f"metric offset: ({X_offset:.2f} m, {Y_offset:.2f} m), "
                    f"global target: ({global_marker_x:.2f}, {global_marker_y:.2f})"
                )

    def publish_setpoint(self):
        if self.current_state in [1,2,3,4,5,6,7]:
            target = self.target_positions[self.current_state]
            # jesli pozycja nie jest okreslona to nie publikujemy pozycji
            if target is None:
                return
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = list(target)
            trajectory_msg.yaw = -3.14
            trajectory_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            self.trajectory_setpoint_publisher.publish(trajectory_msg)
        
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")
    
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()