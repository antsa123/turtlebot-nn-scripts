#! /usr/bin/env python

# This is a ROS node for controlling the robot with a neural network based on laser range finder data

import rospy
import cv2
import os
import numpy as np
from keras.models import load_model
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from time import sleep


class laser_listener:
    def __init__(self):
        self.model = load_model(os.getenv('HOME') + '/catkin_ws/src/ohjaus/src/laser_1_toimiva.hdf5')
        self.control_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.velo = 0
        self.angular = 0
        self.control = np.array([[0, 0, 0]])
        sleep(1)
        np.set_printoptions(precision=3)


    def callback(self, data):
        try:
            data = np.asarray(data.ranges).astype('float32')
            # Convert nan-values to zeros
            data = np.nan_to_num(data)
            # Reshape data and feed it to the neural network
            self.control = self.model.predict(np.array([data.reshape(1081, 1)]))

        except Exception as e:
            print(e)


    def run_controller(self):
        # Produce control signal 20 time a second
        r = rospy.Rate(20)
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
                signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.8))  # kisavire 1.5
            # if 'y' is greatest
            elif control_signal[0, 1] == max(control_signal[0]):
                signal = Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))  # kisavire 0.8
            # is 'z' is greatest
            else:
                signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.8))
                
        # If the network is unsure about the direction, simultaneously drive and rotate.
        # The networks output is weighted
        else:
            suoraan = control_signal[0, 1] * 0.4 #0.6
            if control_signal[0, 0] > control_signal[0,2]:
                kaanny = control_signal[0, 0] * 1.2 #1.6
                signal = Twist(Vector3(suoraan, 0, 0), Vector3(0, 0, kaanny))
            else:
                kaanny = control_signal[0, 2] * 1.2 #1.6
                signal = Twist(Vector3(suoraan, 0, 0), Vector3(0, 0, -kaanny))
                
        # Send the final motion control signal to the turtlebot
        self.control_pub.publish(signal)


def main():
    ic = laser_listener()
    rospy.init_node("laser_listener", anonymous=True)
    ic.run_controller()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


