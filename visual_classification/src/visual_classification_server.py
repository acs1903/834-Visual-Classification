#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import cv2

from classifier import cnn
from sensor_msgs.msg import CompressedImage
from visual_classification.msg import VisualClassificationAction, VisualClassificationGoal, VisualClassificationResult, VisualClassificationFeedback

class VisualClassificationActivity(object):
    def __init__(self, activity_name='visual_classification'):

        rospy.Subscriber("/axis1/compressed", CompressedImage,self.image_callback, queue_size = 1)

        self.action_server = actionlib.SimpleActionServer(activity_name, VisualClassificationAction, self.execute_action, False)
        self.action_server.start()

    def image_callback(self,image):
        self.image_data = image.data

    def execute_action(self, goal):
        # YOUR CODE HERE! Replace this method.
        rospy.loginfo("Action called with input: {}".format(goal.input))

        # This is how you can send feedback to the client
        feedback = VisualClassificationFeedback()
        feedback.current_step = "starting to convert from CompressedImage to Numpy Array"
        self.action_server.publish_feedback(feedback)

        image = self.image_data
        np_arr = np.fromstring(image, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        print image_np.shape #(576, 704, 3)

        feedback = VisualClassificationFeedback()
        feedback.current_step = "finished converting from CompressedImage to Numpy Array"
        self.action_server.publish_feedback(feedback)

        feedback = VisualClassificationFeedback()
        feedback.current_step = "classifying image"
        self.action_server.publish_feedback(feedback)

        # Before you exit, be sure to succeed or fail.
        classification = cnn(image_np) ##TO DO: FIGURE THIS OUT
        result = VisualClassificationResult()
        result.output = classification #"I'm done!" # Put your output here
        self.action_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('visual_classification_server')
    activity = VisualClassificationActivity()
    rospy.loginfo('Visual classifier running')
    rospy.spin()
