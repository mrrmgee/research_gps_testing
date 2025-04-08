import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import ActuatorControlsStatus
from px4_msgs.msg import SensorGps

class GpsLogger(Node):

    def __init__(self):
        super().__init__('gps_logger')

        # Create custom QoS profile compatible with PX4 GPS topic
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscriptions
        self.servo_sub = self.create_subscription(
            ActuatorControlsStatus,
            '/fmu/out/actuator_controls_status_0',
            self.servo_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile
        )

        # Store latest GPS message
        self.latest_gps = None
        self.servo_triggered = False

    def gps_callback(self, msg):
        self.latest_gps = True
        self.latitude = msg.latitude_deg / 1e7
        self.longitude = msg.longitude_deg / 1e7

    def servo_callback(self, msg):
        # Check if control_power[4] is high (adjust index based on your setup)
        if msg.control_power[1] > 0.5 and not self.servo_triggered:
            self.servo_triggered = True
            self.get_logger().info('Servo activated. Logging GPS...')
            self.log_gps()

    def log_gps(self):
        if self.latest_gps is not None:
            lat = self.latitude
            lon = self.longitude
            self.get_logger().info(f'Logged Coordinates: Latitude={lat:.7f}, Longitude={lon:.7f}')
        else:
            self.get_logger().warn('No GPS data received yet.')

def main(args=None):
    rclpy.init(args=args)
    node = GpsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
