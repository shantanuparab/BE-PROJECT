#!/usr/bin/python

"""
Class for low level control of our car. It assumes ros-12cpwmboard has been
installed
"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time


class DkLowLevelCtrl():
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        rospy.init_node('dk_llc')

        self._servo_msg       = ServoArray()
        for i in range(16): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")
        #--- Create the Subscriber to Twist commands
        self.ros_sub_value          = rospy.Subscriber("/cmd_vel", Twist, self.set_value)
        rospy.loginfo("> Subscriber corrrectly initialized")


        rospy.loginfo("Initialization complete")



    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    def send_servo_msg(self,val):
        for i in range(16):
            self._servo_msg.servos[i].servo = i+1
            self._servo_msg.servos[i].value = val
            rospy.loginfo("Sending to %s command %d"%(i+1,val))

        self.ros_pub_servo_array.publish(self._servo_msg)



    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(10)
        value=0
        while not rospy.is_shutdown():
            self.send_servo_msg(value)
            value+=1
            rate.sleep()

if __name__ == "__main__":
    dk_llc     = DkLowLevelCtrl()
    dk_llc.run()
