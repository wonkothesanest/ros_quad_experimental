#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import std_msgs.msg
from ros_quad_experimental.msg import Barometer
import Adafruit_BMP.BMP085 as BMP085


def talker():
    pub = rospy.Publisher('barometer', Barometer, queue_size=10)
    rospy.init_node('barometersensor', anonymous=True)
    r = rospy.Rate(10) # 10hz
    sensor = BMP085.BMP085()
    while not rospy.is_shutdown():
        bar = Barometer()
        h =  std_msgs.msg.Header()
        h.frame_id="quad_test"
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

        bar.header = h
        bar.pressure.header = h
        bar.pressure_sea_level.header = h
        bar.temperature.header = h
        bar.pressure.fluid_pressure = sensor.read_pressure() #Pa
        bar.temperature.temperature = sensor.read_temperature() #celcius
        bar.altitude = sensor.read_altitude() #meters
        bar.pressure_sea_level.fluid_pressure = sensor.read_sealevel_pressure() #Pa
        
        #rospy.get_time()
        #rospy.loginfo(bar)
        pub.publish(bar)
        r.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
