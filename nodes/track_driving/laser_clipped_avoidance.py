#! /usr/bin/env python

# This is a ROS node that publishes the DNN prediction of an obstacle avoidance neural network.
# Place into /home/user/catkin_ws/src/PACKAGE/src of Turtlebot with the keras network


import rospy, os
import numpy as np
from keras.models import load_model
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from sklearn.preprocessing import normalize
from time import sleep


class LaserObstacleAvoidance:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.laser_model = load_model(os.getenv('HOME') + '/catkin_ws/src/ohjaus/src/laser-klipattu1m-conv1-flipped-data-inputshape.hdf5')
        self.obstacle_avoidance_pub = rospy.Publisher('/laser2_control', Vector3, queue_size=5)
        self.latest_scan = []
        np.set_printoptions(precision=2, suppress=True)


    def callback_laser(self, data):
        # Convert laser scan to a numpy array
        data = np.asarray(data.ranges).astype('float32')
        # Convert values of less than 15cm to 1m
        for i in range(len(data)):
            if data[i] < 0.15:
                data[i] = 1.0
        # Convert nan-values to zeros
        data = np.nan_to_num(data)
        # Clip all values to a max distance of 1m
        data = np.clip(data, 0, 1)
        #data /= 2
        # Reshape data to the form the DNN understands
        self.latest_scan = np.array([data.reshape(1081, 1)])


    def run(self):
        r = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                # Feed latest scan to neural network
                control_signal = self.laser_model.predict(self.latest_scan)
                if np.argmax(control_signal) == 0:
                    print("Vasen")
                elif np.argmax(control_signal) == 1:
                    print(50* " " + "Suoraan")
                else:
                    print(100*" " + "Oikea")
                # Check if network is 70% sure of evading
                if True: #control_signal[0, 0] > 0.7 or control_signal[0, 2] > 0.7:
                    # Publish the control signal
                    self.obstacle_avoidance_pub.publish(Vector3(control_signal[0, 0], control_signal[0, 1], control_signal[0, 2]))
                else:
                    # Publish empty control signal
                    self.obstacle_avoidance_pub.publish(Vector3(0.0, 0.0, 0.0))
                r.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    ic = LaserObstacleAvoidance()
    rospy.init_node("laser_clipped_obstacle_avoidance", anonymous=True)
    sleep(1)
    ic.run()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
