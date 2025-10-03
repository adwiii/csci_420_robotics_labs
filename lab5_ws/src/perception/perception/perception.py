#!/usr/bin/env python
from ament_index_python.packages import get_package_share_directory
import os
import cv2
import pickle

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3, PoseStamped, Point

from cv_bridge import CvBridge

from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image

from sklearn.neural_network import MLPClassifier
from marine_life_detection import MarineLifeDetection
import numpy as np

# Create a class which we will use to take keyboard commands and convert them to a position
class Perception(Node):
    # On node initialization
    def __init__(self):
        super().__init__('Perception')

        # Allows us to read images from rostopics
        self.bridge = CvBridge()

        # Marine Life Detection node
        self.detector = MarineLifeDetection()
        self.detected = False
        self.classification = None
        self.image = None

        # Load the classes
        self.classes = ['dolphin', 'seal', 'penguin']

        # # Load the model
        share_directory = get_package_share_directory('perception')
        model_path = os.path.join(share_directory, 'model.pkl')
        self.model = None
        # The file that is included with the lab is a 0 byte placeholder for what you will build in Checkpoint 4
        if os.path.exists(model_path) and os.path.getsize(model_path) > 0:
            with open(model_path, 'rb') as f:
                self.model = pickle.load(f)
            self.get_logger().info(f'model.pkl successfully loaded')
        else:
            self.get_logger().info(f'model.pkl not found, not loading classifier (TODO Checkpoint 4)')

        # TODO Checkpoints 3 & 4
        # Create the publishers and subscriber
        self.image_sub = self.create_subscription(Image, '/uav/sensors/camera', self.downfacing_camera_callback, 1)

        # Set the timer to call the mainloop of our class
        self.rate = 0.25
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def downfacing_camera_callback(self, msg):
        # TODO Checkpoint 3
        # Convert to opencv format using self.bridge
        # Use functions implemented in `marine_life_detection.py` to determine if animal is present
        # Save to self.detected

        # TODO Checkpoint 4
        # Save the opencv image to self.image so it can be processed further in the mainloop
        pass

    # Can be used to get image into the right structure to be passed into the model for CHECKPOINT 4
    # This function replaces `load_image` in the `classify_single_image` example
    def preprocess_image(self, img):
        # Reshape and convert to gray
        img = cv2.resize(img, (48, 48))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        x = img.reshape((1, -1))
        x = x.astype('float32')
        x = x / 255.0
        return x

    def mainloop(self):
        pass
        # TODO Checkpoint 3
        # Publish whether an animal was detected on /marine_life_detected

        # if self.model is not None:
        #   if self.detected:
        #     # TODO Checkpoint 4
        #     # If the animal was detected
        #     # Load the image (using self.preprocess_image)
        #     # Predict what class it is
        #     # Get the class index using argmax
        #     # Save the classification to self.classification

        #   if self.classification is not None:
        #     # TODO Checkpoint 4
        #     # Publish the class on /marine_life_classification



def main():
    rclpy.init()
    try:
        rclpy.spin(Perception())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()