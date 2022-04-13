#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32MultiArray

K_P = 2205
b = 1
# Initial Turning PWM output
PWM_TURN = 5
# min and max PWM output
PWM_MAX = 1100 # For velocity of 0.49 m/sec for safety
PWM_MIN = 5 

rotation_radius = 0.274 # The distance from the center to the wheel in meters
# wheel_diameter: 0.153932

# Correction multiplier for drift. Chosen through experimentation TODO
DRIFT_MULTIPLIER = 120
# Set linear velocity and PWM variable values for each wheel TODO
velLeftWheel = 0
velRightWheel = 0

cmd_motor_pub = rospy.Publisher("cmd_motor", String, queue_size = 1)
cmd_vel_pub = rospy.Publisher("cmd_vel_gui", Twist, queue_size = 1)

def calc_pwm(msg):
    
    # Calculate the PWM value given the desired velocity 

    pwmLeftReq = K_P * abs(msg.linear.x) + b
    pwmRightReq = K_P * abs(msg.linear.x) + b

    if(msg.linear.x < 0):
        pwmLeftReq = -1 * pwmLeftReq
        pwmRightReq = -1 * pwmRightReq

    # Calculate the PWM value given the angular velocity

    wheelVel = rotation_radius * abs(msg.angular.z)
    PWM_TURN = K_P * wheelVel + b
    #print('[CMD_to_TICKS] pwmAngular: ', PWM_TURN)

    # Check if we need to turn 
    if (msg.angular.z != 0.0):

        # Turn left
        if (msg.angular.z > 0.0):
            print('[CMD_to_TICKS] Turning left..')
            pwmLeftReq = -PWM_TURN
            pwmRightReq = PWM_TURN
        # Turn right    
        else:
            print('[CMD_to_TICKS] Turning right..')
            pwmLeftReq = PWM_TURN
            pwmRightReq = -PWM_TURN
            
    # Go straight
    else:
        # Remove any differences in wheel velocities to make sure the robot goes straight
        prevDiff = 0
        prevPrevDiff = 0
        currDifference = velLeftWheel - velRightWheel
        avgDifference = (prevDiff+prevPrevDiff+currDifference)/3
        prevPrevDiff = prevDiff
        prevDiff = currDifference

        # Correct PWM values of both wheels to make the vehicle go straight
        #pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER)
        #pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER)

    # Handle low and Hight PWM values
    if (abs(pwmLeftReq) < PWM_MIN or abs(pwmLeftReq) > PWM_MAX):
        pwmLeftReq = 0
        rospy.loginfo("[CMD_to_TICKS] Left PWM data out of limits ! ")

    if (abs(pwmRightReq) < PWM_MIN or abs(pwmRightReq) > PWM_MAX):
        pwmRightReq = 0
        rospy.loginfo("[CMD_to_TICKS] Right PWM data out of limits ! ")

    pwmRightReq = int(pwmRightReq)
    pwmLeftReq = int(pwmLeftReq)
    print('[CMD_to_TICKS] pwmRightReq: ', pwmRightReq)
    print('[CMD_to_TICKS] pwmLefttReq: ', pwmLeftReq)    
    str_msg = String()
    str_msg.data = 'GO;' + str(pwmLeftReq) + ';' + str(pwmRightReq)
    cmd_motor_pub.publish(str_msg)
    
    # publish the same cmd_vel for ROS Mobile gui
    cmd_vel_msg = Twist()
    cmd_vel_msg = msg
    cmd_vel_pub.publish(cmd_vel_msg)

    
def listener():

    rospy.init_node('cmd_to_ticks', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, calc_pwm)

    rospy.spin()

if __name__ == '__main__':
    listener()