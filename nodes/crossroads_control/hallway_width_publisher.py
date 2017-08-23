#! /usr/bin/env python

# This is a ROS node for publishing the width of the hallway to a topic, to be used by other nodes

import rospy
from std_msgs.msg import Float32


class HallwayWidthPublisher:
    def __init__(self):
        self.hw_pub = rospy.Publisher('/hallway_width', Float32, queue_size=5)
        self.hallway_width = 1.0


    def run(self):
        r = rospy.Rate(1)
        try:
            while not rospy.is_shutdown():
                self.hw_pub.publish(self.hallway_width)
                r.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    ic = HallwayWidthPublisher()
    rospy.init_node("hallway_width_publisher", anonymous=True)
    ic.run()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
