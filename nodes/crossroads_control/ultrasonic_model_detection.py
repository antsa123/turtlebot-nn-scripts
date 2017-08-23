#! /usr/bin/env python

# This is a ROS node for detecting different types of crossroads with the green Turtlebot
# using HC-SR04 -ultrasonic range finders. The detection is model-based and depends of the
# width of the hallway published by the hallway_width_publisher-node.


import rospy
from std_msgs.msg import Int8, Float32
from ohjaus.msg import IntList


class UsonicDetection:
    def __init__(self):
        self.usonic_sub = rospy.Subscriber('/usonic', IntList, self.callback_usonic)
        self.wallway_width_sub = rospy.Subscriber('/hallway_width', Float32, self.callback_hallway_width)
        self.hallway_width = 1.5
        self.no_wall_dist = self.hallway_width / 0.85  # 1.2
        self.wall_dist = self.hallway_width  # 1.0
        self.prediction_pub = rospy.Publisher('/usonic_detection', Int8, queue_size=5)
        self.dists = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.prediction_mask = [False, False, False, False, False, False, False, False]
        self.prediction = 0


    def callback_usonic(self, data):
        self.dists = []
        for dist in data.data:
            # Convert input data to meters
            self.dists.append(float(dist) / 100)
        self.prediction_mask = []
        for dist in self.dists:
            # Check if there is a wall or obstacle in the direction of each sensor
            if dist > self.no_wall_dist or dist < 0:
                self.prediction_mask.append(True)
            else:
                self.prediction_mask.append(False)


    def callback_hallway_width(self, data):
        self.hallway_width = data.data
        self.no_wall_dist = self.hallway_width / 0.85  # 1.2
        self.wall_dist = self.hallway_width  # 1.0


    def predict(self):
        # Only 4 sensors are currently installed, hence the 'x':s

        # Corner left [1, x, 0, x, 0, x, 1, x]
        if self.prediction_mask[0] and not self.prediction_mask[2] and \
                                       not self.prediction_mask[4] and self.prediction_mask[6]:
            self.prediction = 1

        # Corner right [1, x, 1, x, 0, x, 0, x]
        elif self.prediction_mask[0] and self.prediction_mask[2] and \
                                     not self.prediction_mask[4] and not self.prediction_mask[6]:
            self.prediction = 2

        # T-crossroads left [1, x, 0, x, 1, x, 1, x]
        elif self.prediction_mask[0] and not self.prediction_mask[2] and \
                                             self.prediction_mask[4] and self.prediction_mask[6]:
            self.prediction = 3

        # T-crossroadr right [1, x, 1, x, 1, x, 0, x]
        elif self.prediction_mask[0] and self.prediction_mask[2] and \
                                         self.prediction_mask[4] and not self.prediction_mask[6]:
            self.prediction = 4

        # T-crossroads [1, x, 1, x, 0, x, 1, x]
        elif self.prediction_mask[0] and self.prediction_mask[2] and \
                                     not self.prediction_mask[4] and self.prediction_mask[6]:
            self.prediction = 5

        # X-crossroads [1, x, 1, x, 1, x, 1, x]
        elif self.prediction_mask[0] and self.prediction_mask[2] and \
                                         self.prediction_mask[4] and self.prediction_mask[6]:
            self.prediction = 6

        # Dead end [1, x, 0, x, 0, x, 0, x]
        elif self.prediction_mask[0] and not self.prediction_mask[2] and \
                                         not self.prediction_mask[4] and not self.prediction_mask[6]:
            self.prediction = 7

        # Nothing
        else:
            self.prediction = 0

        # Publish the prediction
        self.prediction_pub.publish(self.prediction)


    def run(self):
        r = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                self.predict()
                r.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    ic = UsonicDetection()
    rospy.init_node("usonic_detection", anonymous=True)
    ic.run()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()

