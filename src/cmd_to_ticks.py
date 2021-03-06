#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from comm_tcp.msg import BatteryStateMelodic
import math

# slope of the Motor PWM - Velocity (m/sec) curve
K_P = 1706.4343
b = 1.470863
# Initial Turning PWM output
PWM_TURN = 5
# min and max PWM output
PWM_MAX = 4000 # 1100 For velocity of 0.49 m/sec for safety (for small tire)
PWM_MIN = 5 

rotation_radius = 0.2962 # The distance from the center to the wheel in meters
# wheel_diameter: 0.153932

# Correction multiplier for drift. Chosen through experimentation TODO
DRIFT_MULTIPLIER = 120
# Set linear velocity and PWM variable values for each wheel TODO
velLeftWheel = 0
velRightWheel = 0

cmd_motor_pub = rospy.Publisher("cmd_motor", String, queue_size = 1)
cmd_vel_pub = rospy.Publisher("cmd_vel_gui", Twist, queue_size = 1)
battery_string_pub = rospy.Publisher("battery_percentage", String, queue_size = 1)
info_pub = rospy.Publisher("info", String, queue_size = 1)

def calc_pwm(msg):
    
    # Calculate the PWM value given the desired velocity 

    pwmLeft = K_P * abs(msg.linear.x) + b
    pwmRight = K_P * abs(msg.linear.x) + b

    # Take care of the correct direction of rotation
    if(msg.linear.x < 0):
        pwmLeft = -1 * pwmLeft
        pwmRight = -1 * pwmRight

    # Calculate the PWM value given the angular velocity

    wheelVel = rotation_radius * abs(msg.angular.z)
    PWM_TURN = K_P * wheelVel + b
    #print('[CMD_to_TICKS] pwmAngular: ', PWM_TURN)

    # Check if we need to turn 
    if (abs(msg.angular.z) > abs(msg.linear.x)):  # Turn, if rotation > linear: published from ROS Mobile GUI

        # Turn left if z is positive
        if (msg.angular.z > 0.0):
            rospy.logdebug('[CMD_to_TICKS] TURNING LEFT..')
            pwmLeft = -PWM_TURN
            pwmRight = PWM_TURN
        # Turn right if z is negative   
        else:
            rospy.logdebug('[CMD_to_TICKS] TURNING RIGHT..')
            pwmLeft = PWM_TURN
            pwmRight = -PWM_TURN
            
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
        #pwmLeft -= (int)(avgDifference * DRIFT_MULTIPLIER)
        #pwmRight += (int)(avgDifference * DRIFT_MULTIPLIER)

    # Handle low and Hight PWM values
    if (abs(pwmLeft) < PWM_MIN or abs(pwmRight) < PWM_MIN):
        pwmLeft = 0
        pwmRight = 0
        rospy.loginfo("[CMD_to_TICKS] PWM data is less than minimum limit ! ")

    if (abs(pwmLeft) > PWM_MAX or abs(pwmRight) > PWM_MAX):
        pwmRight = 0
        pwmLeft = 0
        rospy.loginfo("[CMD_to_TICKS] PWM data exceeds the maximum limit ! ")

    pwmRight = int(pwmRight)
    pwmLeft = int(pwmLeft)
    #print('[CMD_to_TICKS] pwmRight: ', pwmRight)
    #print('[CMD_to_TICKS] pwmLeft: ', pwmLeft)    
    motor_msg = String()
    motor_msg.data = 'GO;' + str(pwmLeft) + ';' + str(pwmRight)
    cmd_motor_pub.publish(motor_msg)
    #rospy.loginfo('CMD_to_TICKS] Message calculated and published')
    
    # publish the same cmd_vel for ROS Mobile GUI for visualisation
    cmd_vel_gui = Twist()
    cmd_vel_gui = msg
    cmd_vel_pub.publish(cmd_vel_gui)

def battery_state_to_string(state_msg):

    string_msg = String()
    info_msg = String()

    percentage = state_msg.percentage*100
    percentage = math.floor(percentage)

    if(percentage < 20):
        info_msg.data = 'Batterie ist leer, bitte aufladen.'
        info_pub.publish(info_msg)
    elif(percentage >= 20):
        info_msg.data = 'Batterie ist gut, bitte fahren.'
        info_pub.publish(info_msg)

    # assign BatteryState msgs = String msgs to display on GUI
    string_msg.data = str(percentage) + ' %'
    battery_string_pub.publish(string_msg)


def listener():

    rospy.init_node('cmd_to_ticks', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, calc_pwm)
    rospy.Subscriber("/battery", BatteryStateMelodic, battery_state_to_string)

    rospy.spin()

if __name__ == '__main__':
    listener()