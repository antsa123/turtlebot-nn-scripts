#! /usr/bin/env python

# This is a ROS node that publishes the DNN prediction of a hallway follower neural network.

import rospy, os
import numpy as np
from keras.models import load_model
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from sklearn.preprocessing import normalize
from time import sleep


class LaserHallwayFollower:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.laser_model = load_model(os.getenv('HOME') + '/catkin_ws/src/ohjaus/src/noConv_laser_hallwayFollower3.hdf5')
        self.hallway_control_pub = rospy.Publisher('/hallway_control', Vector3, queue_size=5)
        self.latest_scan = None
        #self.control = np.array([[0, 0, 0]])
        np.set_printoptions(precision=2, suppress=True)


    def callback_laser(self, data):
        # Convert laser scan to a numpy array
        data = np.asarray(data.ranges).astype('float32')
        # Convert nan-values to zeros
        data = np.nan_to_num(data)
        # Clip all values to a max distance of 10m
        data = np.clip(data, 0, 10)
        # Normalize by converting data to a unit vector
        data = normalize(data.reshape(1, -1))
        # Reshape data to the form the DNN understands
        self.latest_scan = np.array([data.reshape(1081,)])


    def run(self):
        r = rospy.Rate(20)
        if self.latest_scan is not None:
            try:
                while not rospy.is_shutdown():
                    control_signal = self.laser_model.predict(self.latest_scan)
                    # Publish the prediction as a Vector 3, with the forward-variable being at least 0.8
                    self.hallway_control_pub.publish(Vector3(control_signal[0, 0], np.amax(np.array([control_signal[0, 1], 0.8])), control_signal[0, 2]))
                    r.sleep()
            except KeyboardInterrupt:
                rospy.on_shutdown(h=0)
        else:
            print "none"


def main():
    # Create the object
    lhf = LaserHallwayFollower()
    rospy.init_node("laser_hallway_follower", anonymous=True)
    # Wait for the object to init and begin receiving laser scans
    sleep(0.5)
    lhf.run()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()

