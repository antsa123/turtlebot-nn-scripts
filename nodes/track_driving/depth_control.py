#! /usr/bin/env python

# This is a ROS node for controlling the robot with a neural network based on depth images from Kinect.

import rospy
import cv_bridge
import cv2
import os
import numpy as np
from keras.models import load_model
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from time import sleep


class depth_listener:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.model = load_model(os.getenv('HOME') + '/catkin_ws/src/ohjaus/src/depth_toimiva_NoNormalization.hdf5')
        self.control_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
        self.depth_sub_black = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback_black)
        self.depth_sub_green = rospy.Subscriber("/kinect2/sd/image_depth", Image, self.callback_green)
        self.velo = 0.0
        self.angular = 0.0
        self.control = np.array([[0, 0, 0]])
        sleep(1)
        np.set_printoptions(precision=3)


    # Callback for the black turtlebot (Kinect 1)
    def callback_black(self, data):
        try:
            # Read image
            cv_img = self.bridge.imgmsg_to_cv2(data, "16UC1")

            try:
                # Crop
                cv_img = cv_img[160:, :]
                # Resize
                img = cv2.resize(cv_img, dsize=(0, 0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
                # Clip to 3m
                img = np.clip(img, 0, 3000)
                img = img.astype('float32')
                # Reshape as required by the network
                img = img.reshape((80, 160, 1))
                # Feed the image to the neural network
                self.control = self.model.predict(np.array([img]))
                print self.control
            except Exception as e:
                print(e)
                
        except cv_bridge.CvBridgeError as e:
            print(e)


    # Callback for the green turtlebot (Kinect 2)
    def callback_green(self, data):
        try:
            # Read image
            cv_img = self.bridge.imgmsg_to_cv2(data, "16UC1")
            try:
                # Crop (original image size 252x304)
                cv_img = cv_img[100:, :]
                # Resize
                img = cv2.resize(cv_img, dsize=(160, 80), interpolation=cv2.INTER_AREA)
                # Clip to 3m
                img = np.clip(img, 0, 3000)
                img = img.astype('float32')
                # Reshape as required by the network
                img = img.reshape((80, 160, 1))
                # Feed the image to the neural network
                self.control = self.model.predict(np.array([img]))
                print self.control
            except Exception as e:
                print(e)
            
        except cv_bridge.CvBridgeError as e:
            print(e)


    def run_controller(self):
        # Produce control signal 6 times a second
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.controller(self.control)
            r.sleep()
        

    def controller(self, control_signal):
        # Control_signal is a numpy array of three items [x, y, z]
        # If x is greatest -> rotate clockwise
        # If y is greatest -> driwe straight
        # If y is greatest -> rotate clockwise
        
        # If the network is over 80% sure about the direction
        if np.amax(control_signal) > 0.8:
            # if 'x' is greatest
            if control_signal[0, 0] == max(control_signal[0]):
                signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.8))  #1.0
            # if 'y' is greatest
            elif control_signal[0, 1] == max(control_signal[0]):
                signal = Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            # is 'z' is greatest
            else:
                signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.8)) # 1.0

        # If the network is unsure about the direction, simultaneously drive and rotate.
        # The networks output is weighted
        else:
            if control_signal[0,1] < 0.15:
                kerroin = 0.0
            else:
                kerroin = 0.3 #0.2
            suoraan = control_signal[0, 1] * kerroin
            if control_signal[0, 0] > control_signal[0, 2]:
                kaanny = control_signal[0,0] * 0.8 #0.6
                signal = Twist(Vector3(suoraan, 0, 0), Vector3(0, 0, kaanny))
            else:
                kaanny = control_signal[0, 2] * 0.8 #0.6
                signal = Twist(Vector3(suoraan, 0, 0), Vector3(0, 0, -kaanny))
                
        # Send the final motion control signal to the turtlebot
        self.control_pub.publish(signal)


def main():
    ic = depth_listener()
    rospy.init_node("depth_listener", anonymous=True)
    ic.run_controller()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


