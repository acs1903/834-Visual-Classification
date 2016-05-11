#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import scipy.misc
from PIL import Image
import cv2
import tensorflow as tf
from inception import main
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
        rospy.loginfo("Action called with input: {}".format(goal.input))

        # This is how you can send feedback to the client
        feedback = VisualClassificationFeedback()
        feedback.current_step = "starting to convert from CompressedImage to Numpy Array"
        self.action_server.publish_feedback(feedback)

        image = self.image_data
        np_arr = np.fromstring(image, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image = Image.fromarray(image_np)
        image.save('/home/cognitiverobotics/catkin_ws/src/cognitiverobotics/student_code/visual_classification/scratch/test_image.jpg')
        #image.save('/home/student/Desktop/test_image.jpg')

        feedback = VisualClassificationFeedback()
        feedback.current_step = "finished converting from CompressedImage to Numpy Array"
        self.action_server.publish_feedback(feedback)

        feedback = VisualClassificationFeedback()
        feedback.current_step = "classifying image"
        self.action_server.publish_feedback(feedback)

        # Before you exit, be sure to succeed or fail.
        classification_list = main('why is this an argument?')
        print classification_list
        top_result = classification_list[0]

        dog_list = [ 'dalmatian' , 'coach dog', 'carriage dog', 'Newfoundland', 'Newfoundland dog', 'Eskimo dog', 'husky',
                    'African hunting dog', 'hyena dog', 'Cape hunting dog', 'Lycaon pictus',
                    'German shepherd', 'German shepherd dog', 'German police dog', 'alsatian',
                    'dogsled', 'dog sled', 'dog sleigh', 'Old English sheepdog', 'bobtail', 'French bulldog',
                    'Bernese mountain dog', 'Maltese dog', 'Maltese terrier', 'Maltese', 'Greater Swiss Mountain dog',
                    'affenpinscher', 'monkey pinscher', 'monkey dog', 'pug', 'pug-dog', 'Tibetan terrier', 'chrysanthemum dog',
                    'Shetland sheepdog', 'Shetland sheep dog', 'Shetland', 'terrier' , 'dog' , 'Chihuahua','kelpie', 'Great Dane','whippet', 'malinois']

        truck_list = ['tank', 'army tank', 'armored combat vehicle', 'armoured combat vehicle', 'fire engine', 'fire truck', 'garbage truck', 'dustcart', 'pickup', 'pickup truck', 'tow truck', 'tow car', 'wrecker', 'trailer truck', 'tractor trailer', 'trucking rig', 'rig', 'articulated lorry', 'recreational vehicle', 'RV', 'R.V.', 'tractor']

        car_list = ["jeep, landrover", "limousine, limo", "minivan","Model T","racer, race car, racing car" , "sports car, sport car", "convertible", "cab, hack, taxi, taxicab","ambulance","beach wagon, station wagon, wagon, estate car, beach waggon, station waggon, waggon"]

        airplane_list = ["airliner", "warplane, military plane", "airship, dirigible", "balloon", "space shuttle"]


        if 'dog' in top_result:
            classification = 'dog'
        elif 'terrier' in top_result:
            classification = 'dog'
        elif top_result in dog_list:
            classification = 'dog'
        elif top_result in car_list:
            classification = 'car'
        elif top_result in airplane_list:
            classification = 'airplane'
        elif top_result in truck_list:
            classification = 'truck'
        result = VisualClassificationResult()
        result.output = classification #"I'm done!" # Put your output here
        self.action_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('visual_classification_server')
    activity = VisualClassificationActivity()
    rospy.loginfo('Visual classifier running')
    rospy.spin()
