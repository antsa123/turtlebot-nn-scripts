#!/usr/bin/env python

# This is a ROS node for combining two neural networks' outputs and creating a motion control signal of them

import rospy
from geometry_msgs.msg import Twist, Vector3


class Voting:
    def __init__(self):
        self.net1_sub = rospy.Subscriber('/laser_control', Vector3, self.net1_callback)
        self.net2_sub = rospy.Subscriber('/depth_obstacle_avoidance', Vector3, self.net2_callback)
        self.control_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)

        self.net1_control = Vector3(0.0, 0.0, 0.0)
        self.net2_control = Vector3(0.0, 0.0, 0.0)

        self.linear = 0.3
        self.angular = 0.8
        self.limit = 0.45
        self.direction = 1 # -1 = clockwise, 1 = counterclockwise


    def net1_callback(self, data):
        self.net1_control = data

    def net2_callback(self, data):
        self.net2_control = data

    def run_controller(self, event):
        # Possible controls: right, left, straight, left-straight, right-straight, left-right (=no decision)
        r1 = self.net1_control.z
        r2 = self.net2_control.z

        s1 = self.net1_control.y
        s2 = self.net2_control.y

        l1 = self.net1_control.x
        l2 = self.net2_control.x

        controls = [r1*r2, l1*l2, s1*s2, l1*s2 + s1*l2, r1*s2 + r2*s1, l1*r2 + l2*r1]

        action = controls.index(max(controls))

        # if max is under limit or left-right
        if max(controls) < self.limit or action == 5:
            print 'No decision', max(controls)
            lin = 0.05
            ang = l1+l2-r1-r2 # self.direction * self.angular

        elif action == 0: # go right
            print 'Right', max(controls)
            lin = 0.0
            ang = -self.angular
            self.direction = -1

        elif action == 1: # go left
            print 'Left', max(controls)
            lin = 0.0
            ang = self.angular
            self.direction = 1

        elif action == 2: #go straight
            print 'Straight', max(controls)
            lin = self.linear
            ang = 0.0

        elif action == 3: # left-straight
            print 'Left-straight', max(controls)
            lin = 0.2 * (s1+s2)
            ang = 0.6 * (l1+l2)
            self.direction = 1

        else: # action = 4, right-straight
            print 'Right-straight', max(controls)
            lin = 0.2 * (s1+s2)
            ang = -0.6 * (r1+r2)
            self.direction = -1

        msg = Twist(Vector3(lin, 0.0, 0.0), Vector3(0.0, 0.0, ang))
        self.control_pub.publish(msg)

    def run_controller2(self, event):
        try:
            r1 = self.net1_control.z
            r2 = self.net2_control.z

            s1 = self.net1_control.y
            s2 = self.net2_control.y

            l1 = self.net1_control.x
            l2 = self.net2_control.x

            control_signal = [0.5 * (l1 + l2), 0.5 * (s1 + s2), 0.5 * (r1 + r2)]

            # If 80% sure of the correct direction
            if max(control_signal) > 0.8:
                # If direction is to left
                if control_signal[0] == max(control_signal):
                    print 'left'
                    signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, self.angular))
                # If direction is to straight
                elif control_signal[1] == max(control_signal):
                    print 'straight'
                    signal = Twist(Vector3(self.linear, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
                # If direction is to right
                else:
                    print 'right'
                    signal = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -self.angular))

            # If unsure of the correct direction, simultaneously go straight and turn
            # Neural network output is weighted
            else:
                # If less than 15% sure about going straight, do not
                if control_signal[1] < 0.15:
                    k = 0.0
                else:
                    k = 0.2
                s = control_signal[1] * k
                if control_signal[0] > control_signal[2]:
                    print 'left-straight'
                    r = control_signal[0] * 0.6
                    signal = Twist(Vector3(s, 0, 0), Vector3(0, 0, r))
                else:
                    print 'right-straight'
                    r = control_signal[2] * 0.6
                    signal = Twist(Vector3(s, 0, 0), Vector3(0, 0, -r))

            # Final control signal is sent to the turtlebot
            self.control_pub.publish(signal)

        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    ic = Voting()
    rospy.init_node("voting", anonymous=True)
    ic.run_timer = rospy.Timer(rospy.Duration(0.1), ic.run_controller)

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()