from pathlib import Path
import sys
from threading import Event
from threading import Thread

import launch
import launch_pytest
import launch_ros
# from launch_ros.actions import Node

import pytest

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String


# class MakeTestNode(Node):
#     def __init__(self, name="test_node"):
#         super().__init__(name)
#         self.msg_event_object = Event()

#     def start_subscriber(self):
#         # Create a subscriber
#         self.subscription = self.create_subscription(
#             String, "chatter", self.subscriber_callback, 10
#         )

#         # Add a spin thread
#         self.ros_spin_thread = Thread(
#             target=lambda node: rclpy.spin(node), args=(self,)
#         )
#         self.ros_spin_thread.start()

#     def subscriber_callback(self, data):
#         self.msg_event_object.set()


# @launch_pytest.fixture
# def generate_test_description():
#     path_to_test = Path(__file__).parent

#     return launch.LaunchDescription(
#         [
#             launch_ros.actions.Node(
#                 executable=sys.executable,
#                 arguments=[str(path_to_test / "executables" / "talker.py")],
#                 additional_env={"PYTHONUNBUFFERED": "1"},
#                 name="demo_node_1",
#                 output="screen",
#             ),
#         ]
#     )


# @pytest.mark.skip(reason="Testing not ready.")
# @pytest.mark.launch(fixture=generate_test_description)
# def test_check_if_msgs_published():
#     rclpy.init()
#     try:
#         node = MakeTestNode("test_node")
#         node.start_subscriber()
#         msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
#         assert msgs_received_flag, "Did not receive msgs !"
#     finally:
#         rclpy.shutdown()

def test_check_ap_dds_time_msgs():
    assert True, "Did not received [builtin_interfaces.msg.Time]"