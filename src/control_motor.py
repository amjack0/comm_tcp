#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool

cmd_motor_pub = rospy.Publisher("cmd_motor", String, queue_size = 1)

pwmLeft = 50
pwmRight = -50

def motor_befehl(button_pressed):

    motor_msg = String()

    if(button_pressed.data):
        motor_msg.data = 'GO;' + str(pwmLeft) + ';' + str(pwmRight)
    else:
        motor_msg.data = 'GO;' + '0' + ';' + '0'

    cmd_motor_pub.publish(motor_msg)
    rospy.loginfo('[motor_befehl] button status: %s', button_pressed.data)

    
def listener():

    rospy.init_node('control_motor', anonymous=True)
    rospy.Subscriber("/btn_press", Bool, motor_befehl)

    rospy.spin()

if __name__ == '__main__':
    listener()