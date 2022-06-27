#!/usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
from comm_tcp.msg import BatteryStateMelodic

battery_pub = rospy.Publisher("battery_sensor_msg", BatteryState, queue_size= 1, latch=True)


def convert_msgs(incoming_msg):

    converted_msg = BatteryState()

    # assign melodic msgs = noetic msgs 
    converted_msg.header.stamp = rospy.Time.now()

    converted_msg.voltage = incoming_msg.voltage
    converted_msg.current = incoming_msg.current
    converted_msg.charge = incoming_msg.charge
    converted_msg.capacity = incoming_msg.capacity
    converted_msg.percentage = incoming_msg.percentage
    converted_msg.present = incoming_msg.present    

    battery_pub.publish(converted_msg)
    #rospy.loginfo('[battery_pub_melodic] incoming_msg: %s', incoming_msg)

    
def listener():

    rospy.init_node('battery_pub_melodic', anonymous=True)
    rospy.Subscriber("/battery", BatteryStateMelodic, convert_msgs)

    rospy.spin()

if __name__ == '__main__':
    listener()