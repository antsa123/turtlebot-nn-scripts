#!/usr/bin/env python

# This is a ROS node that publishes movement controls to the Turtlebot based on both laser_obstacle_avoidance and
# laser_hallway_follower nodes.

import rospy, sys
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from time import sleep


class RataAjo:
    def __init__(self):
        try:
            # Neural network node subscribers and variables
            self.obstacle_avoidance_sub = rospy.Subscriber('/avoidance_control', Vector3, self.callback_avoidance)
            self.hallway_follower_sub = rospy.Subscriber('/hallway_control', Vector3, self.callback_hallway)
            self.obstacle_avoidance_control = [0.0, 0.0, 0.0]
            self.hallway_follower_control = [0.0, 0.0, 0.0]

            # Motion control and path following initialization
            self.control_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)

            print 'initialization done'

        except Exception as e:
            print e
    # ---------------------------------------------------------------------------------------------

    def callback_avoidance(self, data):
        self.obstacle_avoidance_control = np.array([data.x, data.y, data.z])
    # ---------------------------------------------------------------------------------------------

    def callback_hallway(self, data):
        self.hallway_follower_control = np.array([data.x, data.y, data.z])
    # ---------------------------------------------------------------------------------------------

    def run_controller(self, event):
        try:
            # Evade an obstacle
            if np.amax(self.obstacle_avoidance_control) != 0:
                print " evading \r",
                sys.stdout.flush()
                control_signal = np.asarray(self.obstacle_avoidance_control)

            # Follow the hallway
            else:
                print "following \r",
                sys.stdout.flush()
                control_signal = np.asarray(self.hallway_follower_control)


            # Control signal is a 3 item numpy array [x, y, z]
            # x = greatest -> turn left, y = greatest -> go straght, Z = greatest -> turn right

            largest = np.amax(control_signal)
            # If 80% sure of the correct direction              0.3 0.6
            if largest > 0.8:
                # If direction is to left
                if control_signal[0] == largest:
                    signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.8))
                # If direction is to straight
                elif control_signal[1] == largest:
                    signal = Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
                # If direction is to right
                else:
                    signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.8))

            # If unsure of the correct direction, simultaneously go straight and turn.
            # Neural network output is weighted.
            else:
                k = 0.5
                s = control_signal[1] * k
                if control_signal[0] > control_signal[2]:
                    r = control_signal[0] * 0.6
                    signal = Twist(Vector3(s, 0, 0), Vector3(0, 0, r))
                else:
                    r = control_signal[2] * 0.6
                    signal = Twist(Vector3(s, 0, 0), Vector3(0, 0, -r))

            # Final control signal is sent to the turtlebot
            self.control_pub.publish(signal)

        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    rao = RataAjo()
    rospy.init_node("rataAjoOhjaus", anonymous=True)
    sleep(1)
    rao.run_timer = rospy.Timer(rospy.Duration(0.1), rao.run_controller)

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()