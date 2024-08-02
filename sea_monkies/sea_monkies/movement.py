#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
import numpy as np
import random as rand
import matplotlib.pyplot as plt
from std_msgs.msg import Int16
import time

class Random_move(Node):
    def __init__(self):

        super().__init__("random_move")

        self.heading_publisher = self.create_publisher(
            Int16,
            'bluerov2/desired_heading',
            10
        )
        self.get_logger().info('starting random heading publisher')

        self.movement_publisher = self.create_publisher(
            ManualControl,
            'bluerov2/manual_control',
            10
        )
        self.get_logger().info('starting random movement publisher')
        
        # self.publisher_timer = self.create_timer(5.0, self.angles)
        # time.sleep(2)
        self.publisher_timer = self.create_timer(1.0, self.movements)
        self.get_logger().info("starting timers")

        # self.heading_subscriber = self.create_subscription(
        #     Int16,
        #     'bluerov2/heading',
        #     self.heading_callback,
        #     10
        # )

    def angles(self):
        msg = Int16()
        d = rand.randint(1, 5)
        msg.data = (d * 90)
        self.heading_publisher.publish(msg)

    def movements(self):
        msg = ManualControl()
        i = rand.randint(0, 2)
        i *= 50.0
        msg.x = i
        self.movement_publisher.publish(msg)
        self.angles()

def main(args=None):
    rclpy.init(args=args)
    random_move = Random_move()

    try:
        rclpy.spin(random_move)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        random_move.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()