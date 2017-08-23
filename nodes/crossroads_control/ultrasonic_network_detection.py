#! /usr/bin/env python

# This is a ROS node for detecting different types of crossroads with the green Turtlebot
# using HC-SR04 -ultrasonic range finders. The detection is neural network -based and depends of the
# width of the hallway published by the hallway_width_publisher-node.

import rospy, os
import numpy as np
from keras.models import load_model
from std_msgs.msg import Int8, Float32
from ohjaus.msg import IntList
from time import sleep


class UsonicDetection:
    def __init__(self):
        self.usonic_sub = rospy.Subscriber('/usonic', IntList, self.callback_usonic)
        self.wallway_width_sub = rospy.Subscriber('/hallway_width', Float32, self.callback_hallway_width)
        self.hallway_width = 1.5
        self.no_wall_dist = self.hallway_width / 0.85  # 1.2
        self.wall_dist = self.hallway_width  # 1.0
        self.usonic_model = load_model(os.getenv('HOME') + '/catkin_ws/src/ohjaus/src/usonic_model_3_kaikki_99%.hdf5')
        self.prediction_pub = rospy.Publisher('/usonic_detection', Int8, queue_size=5)
        self.dists = np.array([])
        self.prediction = 0
        np.set_printoptions(precision=2, suppress=True)


    def callback_usonic(self, data):
        dist_list = []
        for dist in data.data:
            # Convert distances to meters and normalize them with the hallway width
            dist = float(dist)
            dist /= 100.0
            dist /= self.hallway_width
            dist_list.append(dist)
        self.dists = np.array([dist_list])


    def callback_hallway_width(self, data):
        self.hallway_width = data.data
        self.no_wall_dist = self.hallway_width / 0.85  # 1.2
        self.wall_dist = self.hallway_width  # 1.0


    def predict(self):
        # Use the neural network to predict if the robot is currently at a crossroads
        network_prediction = self.usonic_model.predict([self.dists])
        print network_prediction

        max_prediction = max(network_prediction[0])
        # Publish the prediction only if the network is over 70% sure of it
        if max_prediction > 0.7:
            for i in range(8):
                if max_prediction == network_prediction[0, i]:
                    self.prediction = i
        else:
            self.prediction = 0

#
        self.prediction_pub.publish(self.prediction)


    def run(self):
        r = rospy.Rate(5)
        try:
            while not rospy.is_shutdown():
                self.predict()
                r.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    ic = UsonicDetection()
    rospy.init_node("usonic_detection", anonymous=True)
    sleep(1)
    ic.run()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()

