#!/usr/bin/env python
from __future__ import print_function, division

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Joy


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
        # Publisher for enable commands for each mode
        for key, value in mode_dict.viewitems():
            mode = Mode(key)
            mode.topic = value
            mode.publisher = rospy.Publisher(mode.topic, Bool, queue_size=100, latch=True)
            self.modes.append(mode)

        self.current_mode_pub = rospy.Publisher("~current_mode", String, queue_size=100, latch=True)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=100)

        self.init_mode()

    def joy_cb(self, joy_msg):
        if self.switch_mode_button >= len(joy_msg.buttons):
            rospy.logerr("joy_msgs.buttons out of bounds.")
            return
        pressed = (joy_msg.buttons[self.switch_mode_button] == 1)

        if pressed:
            # check if button is still pressed
            if self.button_pressed:
                return

            # new button press
            self.button_pressed = True
            self.switch_mode()

        else:
            self.button_pressed = False

    def init_mode(self):
        first_pub = True
        for mode in self.modes:
            if first_pub:
                mode.publisher.publish(True)
                first_pub = False
            else:
                mode.publisher.publish(False)

        self.publish_current_mode()

    def switch_mode(self):
        # send disable to current mode
        self.modes[self.current_mode].publisher.publish(False)

        # send enable to new mode
        self.current_mode = (self.current_mode + 1) % len(self.modes)
        self.modes[self.current_mode].publisher.publish(True)

        self.publish_current_mode()

    def publish_current_mode(self):
        self.current_mode_pub.publish(self.modes[self.current_mode].name)


if __name__ == "__main__":
    rospy.init_node("joy_mode_switcher")
    mode_switcher = ModeSwitcher()
    rospy.spin()