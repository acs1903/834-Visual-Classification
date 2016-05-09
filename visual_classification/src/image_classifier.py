#! /usr/bin/env python

import rospy
import actionlib
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from activity_template.msg import ActivityExampleAction, ActivityExampleGoal, ActivityExampleResult

class ImageClassifierNode:
    def __init__(self):

        #initialize subscribers and publisher
        rospy.Subscriber("/axis1", CompressedImage,self.classification_callback, queue_size = 1)
        rospy.Subscriber("/do_classification", Bool, self.callback, queue_size=1)
        self.classification_pub = rospy.Publisher("/image_classification",String)
        self.do_classification = False

    def callback(self, action):
        self.do_classification = action.data

    def classification_callback(self, image):
        if self.do_classification:
            image_data = image.data
            np_arr = np.fromstring(image_data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            client = actionlib.SimpleActionClient("Visual_Classification"), ActivityExampleAction)
            client.wait_for_server()
            goal = ActivityExampleGoal()
            goal.input = "Here's a problem for the server to solve." #figure this out
            # Sends the goal to the action server.
            client.send_goal(goal)
            # Waits for the server to finish performing the action.
            client.wait_for_result()
            result = client.get_result()
            msg = String()
            msg.data = result

            self.classification_pub.publish(msg)

if __name__ == "__main__":

    # Initialize the ROS client API:
    rospy.init_node("visual_classification_node")
    node = ImageClassifierNode()

    # Enter the ROS main loop:
    rospy.spin()
