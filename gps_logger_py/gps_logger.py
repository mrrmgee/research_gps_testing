import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorControlsStatus
from px4_msgs.msg import SensorGps

class GpsLogger(Node):

    def __init__(self):
        super().__init__('gps_logger')

        # Subscriptions
        self.servo_sub = self.create_subscription(
            ActuatorControlsStatus,
            '/fmu/out/actuator_controls_status_0',
            self.servo_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            SensorGps,
            '/fmu/out/sensor_gps',
            self.gps_callback,
            10
        )

        # Store latest GPS message
        self.latest_gps = None
        self.servo_triggered = False

    def gps_callback(self, msg):
        self.latest_gps = msg

    def servo_callback(self, msg):
        # Check if control_power[4] is high (adjust index based on your setup)
        if msg.control_power[1] > 0.5 and not self.servo_triggered:
            self.servo_triggered = True
            self.get_logger().info('Servo activated. Logging GPS...')
            self.log_gps()

    def log_gps(self):
        if self.latest_gps is not None:
            lat = self.latest_gps.lat / 1e7
            lon = self.latest_gps.lon / 1e7
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
