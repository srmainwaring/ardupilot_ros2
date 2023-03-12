# Copyright (C) 2023 Rhys Mainwaring
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
"""

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time


class TimeListener(Node):
    def __init__(self):
        super().__init__("time_listener")

        # Declare and acquire `topic` parameter
        self.declare_parameter("topic", "ROS2_Time")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

        # Subscriptions
        self.subscription = self.create_subscription(Time, self.topic, self.cb, 1)
        self.subscription

    def cb(self, msg):
        if msg.sec:
            self.get_logger().info(
                "From AP : True [sec:{}, nsec: {}]".format(msg.sec, msg.nanosec)
            )
        else:
            self.get_logger().info("From AP : False")


def main(args=None):
    rclpy.init(args=args)
    node = TimeListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
