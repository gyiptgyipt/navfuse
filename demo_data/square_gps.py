#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header


class GpsNode(Node):

    def __init__(self):
        super().__init__('fake_gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/micro', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.initial_latitude = 22.121512
        self.intiial_longtitude = 95.153647
        self.counter = 0

        self.square_size_length = 3.0

    def timer_callback(self):
            msg = NavSatFix()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            # if self.counter % 4 == 0:
            #     msg.latitude = self.initial_latitude + self.square_size_length
            #     msg.longitude = self.intiial_longtitude

            # if self.counter %4 == 1:
            #     msg.latitude = self.initial_latitude
            #     msg.longitude = self.intiial_longtitude + self.square_size_length

            # if self.counter % 4 == 2:
            #     msg.latitude = self.initial_latitude - self.square_size_length
            #     msg.longitude = self.intiial_longtitude

            # if self.counter %4 == 3:
            #     msg.latitude = self.initial_latitude
            #     msg.longitude = self.intiial_longtitude - self.square_size_length

            msg.latitude = self.initial_latitude
            msg.longitude = self.intiial_longtitude

            msg.altitude = 1.00 # Altitude in metres.

            self.publisher_.publish(msg)
            self.counter += 1


def main():
    rclpy.init()

    gps_node = GpsNode()

    rclpy.spin(gps_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()