#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mr_indiana_jana.move import MRMove

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MRMove()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
