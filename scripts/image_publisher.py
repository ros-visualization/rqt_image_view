#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from copy import deepcopy

from ament_index_python.resources import get_resource
from rclpy.qos import QoSProfile

from cv2 import imread

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class ImagePublisher(Node):
    """A publisher that publishes an image on /images"""

    def __init__(self):
        """Initializer"""
        super().__init__('image_publisher')
        self._i = 0

        self.pub = self.create_publisher(Image, 'images', QoSProfile(depth=100))
        timer_period = 0.10
        self.tmr = self.create_timer(timer_period, self.__timer_callback)
        _, package_path = get_resource('packages', 'rqt_image_view')
        example_image_file = os.path.join(
            package_path, 'share', 'rqt_image_view', 'resource', 'lena.png')

        self._img = imread(example_image_file)
        self._msg = Image()
        self._msg.data = [int(b) for b in list(self._img.flatten())]
        self._msg.height = self._img.shape[0]
        self._msg.width = self._img.shape[1]
        self._msg.encoding = 'rgb8'
        self._msg.step = self._img.shape[1] * self._img.shape[2]

        # The amount by which to shift the image each cycle
        # so that it looks animated
        self._image_shift = 6

    def __timer_callback(self):
        self._i += 1
        self.get_logger().debug('Publishing Image: "{0}"'.format(self._i))
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.data = self._msg.data[self._image_shift:] + self._msg.data[:self._image_shift]
        self.pub.publish(self._msg)


def main(args=None):
    """Entry point for cross-platform executable."""
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
