#! /usr/bin/env python

# This is a ROS node for detecting crossroads of laser range finder data with a neural network, and
# publishind this prediction to be used by other nodes.

import rospy, os
import numpy as np
from keras.models import load_model
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
from sklearn.preprocessing import normalize


class LaserCrossroadsPrediction:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.laser_model = load_model(os.getenv('HOME') + '/catkin_ws/src/ohjaus/src/nadam_eiConv_simpler_50test_risteys.hdf5')
        self.crossroads_detection_pub = rospy.Publisher('/laser_detection', Int8, queue_size=5)
        self.latest_scan = None
        np.set_printoptions(precision=2, suppress=True)


    def callback_laser(self, data):
        data = np.asarray(data.ranges).astype('float32')
        data = np.nan_to_num(data)
        data = np.clip(data, 0, 10)
        data = normalize(data.reshape(1, -1))
        #self.latest_scan = data.reshape((1, 1081, 1))
        self.latest_scan = data.reshape((1, 1081))


    def run(self):
        r = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                if self.latest_scan is not None:
                    network_prediction = self.laser_model.predict(self.latest_scan)
                    # Check if network is 70% sure of one crossroads type
                    if np.amax(network_prediction) > 0.7:
                        # Publish its index
                        self.crossroads_detection_pub.publish(np.argmax(network_prediction))
                    else:
                        # Publish 0
                        self.crossroads_detection_pub.publish(0)

                r.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    ic = LaserCrossroadsPrediction()
    rospy.init_node("laser_crossroads_detection", anonymous=True)
    ic.run()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()

