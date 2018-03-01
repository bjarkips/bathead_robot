#!/usr/bin/env python
# license removed for brevity
import rospy
import RPi.GPIO as gpio
import time
from std_msgs.msg import Float64

chirp_trigger_pin = 4

gpio.setmode(gpio.BCM)
gpio.setup(chirp_trigger_pin, gpio.OUT)

def bathead_range():
    pub_left = rospy.Publisher('bathead/range/left', Float64, queue_size=1)
    pub_right = rospy.Publisher('bathead/range/right', Float64, queue_size=1)
    rospy.init_node('bathead_range')
    rate = rospy.Rate(20) # 20 Hz
    
    # TODO Sleep x ms while waiting for chirp, then read chirp data and estimate range
    
    while not rospy.is_shutdown():
    
        # Trigger chirp emitter with a short pulse
        gpio.output(chirp_trigger_pin, gpio.HIGH)
        time.sleep(.0001)
        gpio.output(chirp_trigger_pin, gpio.LOW)
        pub_left.publish(1.0) # TODO use estimated range values
        pub_right.publish(1.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        bathead_range()
    except rospy.ROSInterruptException:
        pass
