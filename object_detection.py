#!/usr/bin/env python


 #Import Statements for neccessary packages
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import *
import cv2
import rospy

class image_proc():

	# Initialise everything
	def __init__(self):

		rospy.init_node('robot_perception') #Initialise rosnode


		self.image_sub = rospy.Subscriber("raspicam_node/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()


	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Image window",self.img)
			cv2.waitKey(3)
        except CvBridgeError as e:
			print(e)
			return

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
