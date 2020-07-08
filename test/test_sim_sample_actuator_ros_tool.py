#!/usr/bin/env python
from dynamic_reconfigure.client import Client
from simulation_only_msgs.msg import DeltaTrajectoryWithID
import time
import rospy
import unittest
import sys
PKG = "sim_sample_actuator_ros_tool"


class Listener:
    """
    Can be used to block and wait for a message
    """

    def __init__(self, topic, msg):
        """
        Create a Listener
        :param topic: string: name of the topic
        :param msg: message object for this topic
        """
        self.subscriber = rospy.Subscriber(topic, msg, self._on_message, queue_size=5)

    def wait_for_message(self, timeout=rospy.Duration(5)):
        """
        Blocks until a message is received on the subscribed topic or timeout is reached
        :param timeout: Duration: maximum time to wait
        :return: message, if received - otherwise None
        """
        self.msg = None
        waitcount = 0
        while waitcount < 20 and not self.msg and not rospy.is_shutdown():
            waitcount += 1
            time.sleep(timeout.to_sec() / 20.)
        return self.msg

    def _on_message(self, msg):
        self.msg = msg


class TestActuatorPython(unittest.TestCase):

    def test_whether_delta_trajectory_is_sent(self):

        topic = rospy.get_param("~subscriber_topic")

        listener = Listener(topic, DeltaTrajectoryWithID)

        self.assertIsNotNone(listener.wait_for_message(), "Did not receive message on topic \"" + topic + "\"")


if __name__ == '__main__':
    import rostest

    rospy.init_node('actuator_test')
    rostest.rosrun(PKG, 'actuator_test', TestActuatorPython)
