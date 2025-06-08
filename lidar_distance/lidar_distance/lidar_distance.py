import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import sys
from collections import deque
from smbus2 import SMBus


class TFLunaI2CToLaserScan(Node):
    """
    ROS2 node odczytujący odległość z TF-Luna (I2C) i publikujący
    jako sensor_msgs/LaserScan (with single-range array).
    Uśrednianie 5 ostatnich pomiarów (metry).
    """

    def __init__(self):
        super().__init__('lidar_distance')

        self._i2c_bus_num = 7
        self._i2c_address = 0x10
        try:
            self._bus = SMBus(self._i2c_bus_num)
            self.get_logger().info(f'I2C bus #{self._i2c_bus_num} opened, address 0x{self._i2c_address:02X}')
        except Exception as e:
            self.get_logger().error(f'Cannot open I2C bus: {e}')
            sys.exit(1)
        self._window_size = 5
        self._buffer = deque(maxlen=self._window_size)
        self._topic = '/lidar/distance'
        self._publisher = self.create_publisher(LaserScan, self._topic, 10)
        self._scan_msg = LaserScan()
        self._scan_msg.header.frame_id = 'lidar_link'
        self._scan_msg.angle_min = 0.0
        self._scan_msg.angle_max = 0.0
        self._scan_msg.angle_increment = 0.0
        self._scan_msg.time_increment = 0.0
        self._scan_msg.scan_time = 0.1
        self._scan_msg.range_min = 0.03
        self._scan_msg.range_max = 8.0
        timer_period = 0.1
        self._timer = self.create_timer(timer_period, self._timer_callback)

    def _timer_callback(self):
        """
        Wywoływane co 0.1 s: odczyt 6 bajtów z TF-Luna, obliczenie odległości (cm→m),
        uśrednianie oraz publikacja w LaserScan.ranges = [avg_dist].
        """
        try:
            raw = self._bus.read_i2c_block_data(self._i2c_address, 0x00, 6)
            dist_cm = (raw[1] << 8) | raw[0]
            dist_m = dist_cm / 100.0
            self._buffer.append(dist_m)
            if len(self._buffer) == self._window_size:
                avg_dist = sum(self._buffer) / self._window_size
            else:
                avg_dist = sum(self._buffer) / len(self._buffer)
            self._scan_msg.header.stamp = self.get_clock().now().to_msg()
            self._scan_msg.ranges = [float(avg_dist)]
            self._publisher.publish(self._scan_msg)
            self.get_logger().debug(f'Publishing LaserScan: distance={avg_dist:.3f} m')
        except Exception as e:
            self.get_logger().warn(f'Cannot read I2C/LaserScan: {e}')

    def destroy_node(self):
        """Zatrzymanie I2C i zamknięcie node."""
        try:
            self._bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFLunaI2CToLaserScan()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard interrupt -> closing node')
    except Exception as e:
        print(f'Cannot initialize node: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
