#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class RpmPublisher(Node):

    def __init__(self):
        super().__init__('rpm_publisher')
        self.publisher_ = self.create_publisher(Float32, 'rpm', 10)
        timer_period = 1.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # Señal senoidal
        self.amplitude = 50.0
        self.frequency = 0.5  
        self.offset = 100.0
        self.start_time = time.time()

    def timer_callback(self):
        current_time = time.time() - self.start_time
        rpm_value = self.offset + self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)
        
        msg = Float32()
        msg.data = rpm_value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s" RPM' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    rpm_publisher = RpmPublisher()

    try:
        rclpy.spin(rpm_publisher)
    except KeyboardInterrupt:
        rpm_publisher.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        rpm_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
