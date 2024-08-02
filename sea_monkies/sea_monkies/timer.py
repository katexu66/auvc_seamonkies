#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class Timer(Node):
    
    def __init__(self):
        super().__init__("timer_node")
    
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("hi")
        
def main(args=None):
    rclpy.init(args=args)
    timer_test = Timer()

    try:
        rclpy.spin(timer_test)
    except KeyboardInterrupt:
        pass
    finally:
        timer_test.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()