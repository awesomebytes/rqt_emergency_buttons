#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# rqt_emergency_buttons: test_plugin.py
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Sammy Pfeiffer
"""
Created on 6/8/15

@author: sampfeiffer

test_plugin.py contains...
"""
__author__ = 'sampfeiffer'

import rospy
from std_msgs.msg import Float32, Bool
import random

# rosparam set /emergency_buttons_dashboard/emergency_buttons "{'emergency1': {'pressed_topic': '/emergency1', 'tooltip_name': 'EMERGENCY1'}}"

NUM_EMERGENCY_B = 2

rospy.init_node('test_rqt_emergency')

emergency_list = []
for i in range(NUM_EMERGENCY_B):
    emergency_list.append({'emergency'+str(i) : {'pressed_topic': '/emergency'+str(i), 'tooltip_name': 'EMERGENCY'+str(i)}})

rospy.loginfo("Setting params.")
rospy.set_param('/emergency_buttons_dashboard/emergency_buttons', emergency_list)
own_params = rospy.get_param('/emergency_buttons_dashboard')
rospy.loginfo("We set the params:\n" + str(own_params))


for emer_list_elem in emergency_list:
    for emer_name in emer_list_elem.keys():
        emer_list_elem[emer_name].update({'pressed_pub': rospy.Publisher(emer_list_elem[emer_name]['pressed_topic'], Bool)})

print "emergency_list looks like:\n" + str(emergency_list)

battery_status = [i for i in range(1, 100, 10)]
battery_status.reverse()

hz = 50.0

# Publish battery levels going down simulating a robot
while not rospy.is_shutdown():
    for status in battery_status:
        rospy.loginfo("Publishing status: " + str(status))
        for i in range(int(hz)): # One second with every status at hz
            for emer_list_elem in emergency_list:
                for emer_name in emer_list_elem.keys():
                    emer_list_elem[emer_name]['pressed_pub'].publish(random.choice([True, False]))
            rospy.sleep(1.0/hz)
