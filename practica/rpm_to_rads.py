#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class RpmToRadsConverter(Node):

    def __init__(self):
        super().__init__('rpm_to_rads_converter')
        self.subscription = self.create_subscription(
            Float32,
            'rpm',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'rads', 10)
        self.subscription
        self.conversion_factor = 2 * 3.14159265 / 60.0

    def listener_callback(self, msg):
        rpm_value = msg.data
        rads_value = rpm_value * self.conversion_factor
        
        rads_msg = Float32()
        rads_msg.data = rads_value
        self.publisher_.publish(rads_msg)
        self.get_logger().info('Converted: "%.4f RPM" to "%.4f rad/s"' % (rpm_value, rads_value))

def main(args=None):
    rclpy.init(args=args)

    rpm_to_rads_converter = RpmToRadsConverter()

    try:
        rclpy.spin(rpm_to_rads_converter)
    except KeyboardInterrupt:
        rpm_to_rads_converter.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        rpm_to_rads_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
