#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from rpi_led_consensus.srv import SetValue, SetValueResponse

from rpi_ws281x_pylib import LEDClient


class SimpleConsensusNode(object):
    def __init__(self):
        # Initialize class from ROS parameters.
        self.config = rospy.get_param('/consensus_params')

        name = rospy.get_namespace().strip('/')
        index = int(self.config['mapping'].index(name))
        self.alpha = self.config['alpha']
        self.fix_until = rospy.get_time()
        self.value = 0
        self.set_random()

        # Value visualization with LEDs
        self.leds = LEDClient()
        self.leds.connect()
        self.leds.set_all((int(self.value / 360 * 255), 255, 20), color_space='hsv')

        # Create a publisher.
        pub = rospy.Publisher('value', Float32, queue_size=1)

        # Create subscribers.
        for connected, to in zip(self.config['adjacency'][index], self.config['mapping']):
            if connected:
                rospy.Subscriber('/{}/value'.format(to), Float32, self.callback, queue_size=3)

        # Create services.
        rospy.Service('set_value', SetValue, self.set_value)
        rospy.Service('set_random', Empty, self.set_random)

        # Main loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub.publish(self.value)
            self.leds.set_all((int(self.value / 360 * 255), 255, 20), color_space='hsv')
            rate.sleep()

    def callback(self, msg):
        if rospy.get_time() > self.fix_until:
            self.value = self.value + self.alpha * (msg.data - self.value)

    def set_value(self, req):
        self.value = req.value
        if req.fix_duration == -1:
            self.fix_until = float('inf')
        else:
            self.fix_until = rospy.get_time() + req.fix_duration
        return SetValueResponse()

    def set_random(self, req=None):
        if self.config['initial_values']['range'][2] == 0:
            self.value = random.uniform(*self.config['initial_values']['range'][0:2])
        else:
            self.value = random.randrange(*self.config['initial_values']['range'])
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("node")

    try:
        node = SimpleConsensusNode()
    except rospy.ROSInterruptException:
        pass
