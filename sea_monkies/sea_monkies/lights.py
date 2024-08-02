#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Int16

#65535

class Light(Node):
    def __init__(self):
        super().__init__("light_node")

        self.command_pub = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", 10
        )

        self.light_sub = self.create_subscription(
            Int16, "bluerov2/lights", self.light_callback, 10
        )
        
    def light_callback(self, msg):
        light_val = OverrideRCIn()
        light_val.channels = [65535] * 10
        if msg.data == 0: #turning the lights on
            light_val.channels[8] = 2000
            light_val.channels[9] = 2000
            self.command_pub.publish(light_val)
        else: #turning the lights off
            light_val.channels[8] = 1000
            light_val.channels[9] = 1000
            self.command_pub.publish(light_val)

def main(args=None):
    rclpy.init(args=args)
    light = Light()

    try:
        rclpy.spin(light)
    except KeyboardInterrupt:
        pass
    finally:
        light.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()