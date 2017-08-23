#!/usr/bin/env python

# This is a ROS node for controlling the robot based on crossroads detection and hallway follower.
# The robot moves according to either a pre-given list of instructions, or by instructions given
# by the planner-node. The instructions are actions to take at "nodes" (crossroads and dead ends),
# e.g. "turn left". Upon copleting it's list of instructions (path), the robot stops and waits for
# next instructions.

# Depends on the following nodes:
#   depth_obstacle_avoidance
#   laser_hallway_follower
#   laser_crossroads_detection
#   ultrasonic_model_detection OR ultrasonic_network_detection

# Place into /home/user/catkin_ws/src/PACKAGE/src of the green Turtlebot.

import rospy, sys, os
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Bool, Int8, Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sqrt


class TurtlebotController:
    def __init__(self):
        try:

            # Neural network node subscribers and variables
            self.obstacle_avoidance_sub = rospy.Subscriber('/depth_obstacle_avoidance', Vector3, self.callback_avoidance)
            self.hallway_follower_sub = rospy.Subscriber('/hallway_control', Vector3, self.callback_hallway)
            self.laser_crossroads_sub = rospy.Subscriber('/laser_detection', Int8, self.callback_laser_detection)
            self.usonic_crossroads_sub = rospy.Subscriber('/usonic_detection', Int8, self.callback_usonic_detection)
            self.obstacle_avoidance_control = [0.0, 0.0, 0.0]
            self.hallway_follower_control = [0.0, 0.0, 0.0]
            self.laser_crossroads_type = 0
            self.usonic_crossroads_type = 0

            # Motion control and path following initialization
            self.control_pub = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)

            # Enviromental parameters
            self.wallway_width_sub = rospy.Subscriber('/hallway_width', Float32, self.callback_hallway_width)
            self.hallway_width = 1.0
            self.no_wall_dist = self.hallway_width / 0.85  # 1.2
            self.wall_dist = self.hallway_width  # 1.0

            # Rotation angle and location detection initialization
            self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
            self.angle = 0.0
            self.target_angle = 0.0
            self.crossroads_turn_angle = (2.0 * pi / 7.0)
            self.turning_accuracy = 0.05
            self.new_crossroads = False
            self.previous_node_x = 0.0
            self.previous_node_y = 0.0
            self.distance_from_last_node = 0.0

            # Subscribtion to planners topic and path initialization
            self.path_sub = rospy.Subscriber('/path', String, self.callback_path)
            self.plan_pub = rospy.Publisher('/plan_done', Bool, queue_size=5)
            self.plan_done = False
            self.path = ['U', 'R', 'L', 'R']
            print "path = ",
            print self.path

            self.permission_to_dock = False

            np.set_printoptions(precision=3)

        except Exception as e:
            print(e)
    # ---------------------------------------------------------------------------------------------

    def callback_avoidance(self, data):
        self.obstacle_avoidance_control = np.array([data.x, data.y, data.z])

    # ---------------------------------------------------------------------------------------------

    def callback_hallway(self, data):
        self.hallway_follower_control = np.array([data.x, data.y, data.z])
    # ---------------------------------------------------------------------------------------------

    def callback_laser_detection(self, data):
        self.laser_crossroads_type = data.data
    # ---------------------------------------------------------------------------------------------

    def callback_usonic_detection(self, data):
        self.usonic_crossroads_type = data.data
    # ---------------------------------------------------------------------------------------------

    def callback_hallway_width(self, data):
        self.hallway_width = data.data
        self.no_wall_dist = self.hallway_width / 0.85  # 1.2
        self.wall_dist = self.hallway_width  # 1.0
    # ---------------------------------------------------------------------------------------------

    def callback_path(self, data):
        # Save new instructions to path
        for i in range(0, len(data.data)):
            self.path.append(data.data[i])
        # Turn to the direction of the path and escape using hallway follower and/or obstacle avoidance
        self.rotate_at_crossroads()
        # Remove previous instruction
        self.path.pop(0)
        # Escape from crossroads
        self.escape(self.hallway_width)
        self.plan_done = False
        print
    # ---------------------------------------------------------------------------------------------

    def callback_odom(self, data):
        # Read robots angle and convert it to +-pi radians from quaternions
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([0, 0, z, w])
        self.angle = yaw

        if self.new_crossroads:
            # Save the coordinates of the latest node for escaping
            self.previous_node_x = data.pose.pose.position.x
            self.previous_node_y = data.pose.pose.position.y
            self.new_crossroads = False

        # Update current coordinates
        current_x = data.pose.pose.position.x
        current_y = data.pose.pose.position.y
        self.distance_from_last_node = sqrt((self.previous_node_x - current_x) ** 2 + (self.previous_node_y - current_y) ** 2)
    # ---------------------------------------------------------------------------------------------

    def rotate_at_crossroads(self):
        r = rospy.Rate(20)
        direction = self.path[0]
        try:

            # Count target angle
            if direction == 'L':
                # Rotation 90 degrees counter clockwise
                if ((pi - self.crossroads_turn_angle) <= self.angle) and (self.angle <= pi):
                    self.target_angle = self.angle - (2 * pi - self.crossroads_turn_angle)
                else:
                    self.target_angle = self.angle + self.crossroads_turn_angle
            elif direction == 'R':
                # Rotation 90 degrees clockwise
                if (-pi <= self.angle) and (self.angle <= (-pi + self.crossroads_turn_angle)):
                    self.target_angle = self.angle + (2 * pi - self.crossroads_turn_angle)
                else:
                    self.target_angle = self.angle - self.crossroads_turn_angle
            elif direction == 'U':
                # Rotation 180 degrees counter clockwise
                if (pi / 5.0) < self.angle <= pi:
                    self.target_angle = self.angle - (6 * pi / 5)
                else:
                    self.target_angle = self.angle + (4 * pi / 5)
            elif direction == 'S':
                return

            # Rotate until close enough of target
            current_to_target_angle = abs(max(self.target_angle, self.angle) - min(self.target_angle, self.angle))
            while not (current_to_target_angle < self.turning_accuracy) and not rospy.is_shutdown():
                current_to_target_angle = abs(max(self.target_angle, self.angle) - min(self.target_angle, self.angle))
                print '\r' + str(self.angle) + ' ' + str(self.target_angle),
                if direction == 'L' or direction == 'U':
                    self.movement_controller([1, 0, 0])
                elif direction == 'R':
                    self.movement_controller([0, 0, 1])
                print " turning  ",
                sys.stdout.flush()
                r.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)
        print
    # ---------------------------------------------------------------------------------------------

    def escape(self, dist):
        try:
            r = rospy.Rate(20)
            #print
            # Depth image control until 'dist' away from node
            while (self.distance_from_last_node < dist) and (not rospy.is_shutdown()):
                print '\r' + 'escape',
                sys.stdout.flush()
                self.movement_controller()
                r.sleep()
            print  # Used for print formatting
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)
    # ---------------------------------------------------------------------------------------------
    # ---------------------------------------------------------------------------------------------

    def run_controller(self, event):
        try:
            if not self.plan_done:

                print '\r' + str(self.laser_crossroads_type) + " " + str(self.usonic_crossroads_type) + " ",
                # Check if usonic thinks robot is at crossroads or dead end (NOT corner)
                if self.usonic_crossroads_type > 2:
                    # Check if laser agrees
                    if self.laser_crossroads_type == (self.usonic_crossroads_type - 2):
                        print ' ' + str(self.usonic_crossroads_type) + "          "
                        # Enable the saving of crossroads coordinates
                        self.new_crossroads = True

                        # Paths is not empty
                        if self.path:
                            # Turn to the direction of the path and escape using hallway follower and/or obstacle avoidance
                            self.rotate_at_crossroads()
                            # Remove previous instruction
                            self.path.pop(0)
                            # Escape from crossroads
                            print
                            self.escape(self.hallway_width)

                        # Path is empty
                        else:
                            if not self.plan_done:
                                self.plan_pub.publish(True)
                                self.plan_done = True
                                print "Plan done"
                                if self.permission_to_dock:
                                    os.system('roslaunch ohjaus activate.launch')
                                    self.permission_to_dock = False

                    else:
                        self.movement_controller()

                    ## Check if usonic thinks robot is at a corner
                    #elif self.usonic_crossroads_type == 1:
                    #    self.new_crossroads = True
                    #    print "L  "
                    #    # Rotate
                    #    self.rotate_at_crossroads('L')
                    #    # Escape
                    #    self.escape(1.0)
                    #elif self.usonic_crossroads_type == 2:
                    #    self.new_crossroads = True
                    #    print "R  "
                    #    # Rotate
                    #    self.rotate_at_crossroads('R')
                    #    # Escape
                    #    self.escape(1.0)

                # Not at node
                else:
                    self.movement_controller()

        except KeyboardInterrupt:
            pass
    # ---------------------------------------------------------------------------------------------


    # ---------------------------------------------------------------------------------------------

    def movement_controller(self, control_signal=None):
        try:
            if control_signal is None:
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
            # If 80% sure of the correct direction
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
                    r = control_signal[0] * 0.6
                    signal = Twist(Vector3(s, 0, 0), Vector3(0, 0, r))
                else:
                    r = control_signal[2] * 0.6
                    signal = Twist(Vector3(s, 0, 0), Vector3(0, 0, -r))

            # Final control signal is sent to the turtlebot
            self.control_pub.publish(signal)

        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)
    # ---------------------------------------------------------------------------------------------

    def execute_first_instruction(self):
        if self.path:
            # Turn to the direction of the path and escape using hallway follower and/or obstacle avoidance
            self.rotate_at_crossroads()
            # Remove previous instruction
            self.path.pop(0)
            # Escape from crossroads
            self.escape(self.hallway_width)
            self.plan_done = False
            print
    # ---------------------------------------------------------------------------------------------


def main():
    ic = TurtlebotController()
    rospy.init_node("turtlebot_controller", anonymous=True)
    ic.execute_first_instruction()
    ic.run_timer = rospy.Timer(rospy.Duration(0.1), ic.run_controller)

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
