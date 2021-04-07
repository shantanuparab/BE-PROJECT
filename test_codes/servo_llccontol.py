#!/usr/bin/python

"""
Class for Robot Control. It assumes ros-12cpwmboard has been
installed
"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time

class ServoConvert():
    def __init__(self, id=1, max_val=450,min_val=125):
        self.value      = 0.0
        self.value_out  = min_val
        self._center    = 0.5*(max_val-min_val)
        self._range     = max_val-min_val
        self._half_range= 0.5*range
        self.id         = id

    def get_value_out(self, value_in):
        #--- value is in [-1, 1]
        self.value      = value_in
        # Processing on Input values
        self.value_out  = int(value_in)
        print self.id, self.value_out
        return(self.value_out)


class RobotLowLevelCtrl():
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")

        rospy.init_node('robot_llc')

        self.actuators = {}
        self.actuators['shoulder_1']  = ServoConvert(id=1)
        self.actuators['shoulder_2']  = ServoConvert(id=2)
        self.actuators['elbow_1']  = ServoConvert(id=3)
        self.actuators['elbow_2']  = ServoConvert(id=4)
        self.actuators['wrist_1']  = ServoConvert(id=5)
        self.actuators['wrist_2']  = ServoConvert(id=6)
        rospy.loginfo("> Actuators corrrectly initialized")

        self._servo_msg       = ServoArray()
        for i in range(16): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")
        #--- Create the Subscriber to Twist commands
        self.ros_sub_value          = rospy.Subscriber("/cmd_vel", Twist, self.set_value)
        rospy.loginfo("> Subscriber corrrectly initialized")

        self.ros_sub_motions=rospy.Subscriber("/execute_motion", String ,self.set_motion)


        rospy.loginfo("Initialization complete")



    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.actuators['shoulder_1'].get_value_out(125)
        self.actuators['shoulder_2'].get_value_out(125)
        self.actuators['elbow_1'].get_value_out(125)
        self.actuators['elbow_2'].get_value_out(125)
        self.actuators['wrist_1'].get_value_out(125)
        self.actuators['wrist_2'].get_value_out(125)

        rospy.loginfo("Setting actutors to idle")
        self.send_servo_msg()

    def set_motion(self,msg):
        if msg=="Hi":
            self.motion_hi()
        else if msg=="pick"
            self.motion_pick()

    def motion_hi(self):
        rate = rospy.Rate(100)
        i=0
        while not i>1000:
            self.actutors['shoulder_1'].get_value_out(450)
            self.actutors['elbow_1'].get_value_out(160)
            self.send_servo_msg()
            rate.sleep()
            self.actutors['elbow_1'].get_value_out(125)
            self.send_servo_msg()
            rate.sleep()
            i++
        rospy.loginfo("Command Executed")

    def motion_pick():
        self.actutors['shoulder_1'].get_value_out(160)
        self.actutors['shoulder_2'].get_value_out(160)
        self.actutors['elbow_1'].get_value_out(160)
        self.actutors['elbow_2'].get_value_out(160)
        self.actutors['wrist_1'].get_value_out(160)
        self.actutors['wrist_2'].get_value_out(160)
        self.send_servo_msg()
        rospy.loginfo("Command Executed")




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
    robot_llc     = RobotLowLevelCtrl()
    robot_llc.run()
