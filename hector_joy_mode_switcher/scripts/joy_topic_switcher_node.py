#!/usr/bin/env python
from __future__ import print_function, division

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import copy


def load_mandatory_parameter(name):
    try:
        return rospy.get_param(name)
    except KeyError:
        rospy.logerr("Missing parameter '" + name + "'")
        exit(0)


class Mode:
    def __init__(self, _name):
        self.name = _name
        self.topic = ""
        self.publisher = None


class ModeSwitcher:
    def __init__(self):
        # Initialize variables
        self.current_mode = 0
        self.button_pressed = False
        self.modes = []

        # Load parameters
        self.switch_mode_button = load_mandatory_parameter("~switch_mode_button")
        if self.switch_mode_button < 0:
            rospy.logerr("'switch_mode_button' must be positive.")
            exit(0)
        mode_dict = load_mandatory_parameter("~modes")

        # Subscribers and publishers
        # Publisher for each mode
        for key, value in mode_dict.viewitems():
            mode = Mode(key)
            mode.topic = value
            mode.publisher = rospy.Publisher(mode.topic, Joy, queue_size=10)
            self.modes.append(mode)

        self.current_mode_pub = rospy.Publisher("~current_mode", String, queue_size=100, latch=True)
        self.publish_current_mode()
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=10)

    def joy_cb(self, joy_msg):
        if self.switch_mode_button >= len(joy_msg.buttons):
            rospy.logerr("joy_msgs.buttons out of bounds.")
            return
        pressed = (joy_msg.buttons[self.switch_mode_button] == 1)

        if pressed:
            # check if button is still pressed
            if not self.button_pressed:
                # new button press
                self.button_pressed = True
                self.switch_mode()
        else:
            self.button_pressed = False

        # Republish to topic of current mode
        joy_msg_copy = copy.deepcopy(joy_msg)
        joy_msg_copy.buttons = list(joy_msg_copy.buttons)  # needed to make buttons mutable
        joy_msg_copy.buttons[self.switch_mode_button] = 0  # Do not forward button press for mode switching
        self.modes[self.current_mode].publisher.publish(joy_msg_copy)

    def switch_mode(self):
        self.current_mode = (self.current_mode + 1) % len(self.modes)
        self.publish_current_mode()

    def publish_current_mode(self):
        self.current_mode_pub.publish(self.modes[self.current_mode].name)


if __name__ == "__main__":
    rospy.init_node("joy_mode_switcher")
    mode_switcher = ModeSwitcher()
    rospy.spin()