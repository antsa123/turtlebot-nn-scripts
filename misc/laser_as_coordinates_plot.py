#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ohjaus.msg import IntList

import os

from tf.transformations import euler_from_quaternion

from time import sleep
import matplotlib.pyplot as plt

""" This is a custom ROS node that listens to Hokuyo laser scan topics and plots the 1081 distance measurements in real time
    as x and y coordinates. There is also support for plotting ultrasonic measurements. Helps visualize laser scans."""

## The class itself that listens to laser topics and does plotting
class laser_listener:

    def __init__(self):
        homedir = os.getenv('HOME')
        user = os.getenv('LOGNAME')
        self.corridor_width = 10.0

        # Subscription to rostopics
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.ultrasound_sub = rospy.Subscriber("/usonic", IntList, self.callback_usonic)
        self.velo = 0
        self.angular = 0
        self.control = np.array([[0, 0, 0]])

        # Parameters for calculating coordinates from scan data
        deg_step = 270 / 1081.0
        rad_step = deg_step * np.pi / 180.0
        start_angle = -45 * np.pi / 180.0
        stop_angle = 225 * np.pi / 180.0

        rads = np.arange(start_angle, stop_angle, rad_step)
        self.xmul = np.cos(rads)
        self.ymul = np.sin(rads)

        self.current_angle = 0
        self.past_angle = 0

        self.current_x = 0
        self.current_y = 0
        self.past_x = 0
        self.past_y = 0


        # Parameters for "short term memory"
        # History is saved as a circular buffer array and history_length tells how many of the latest scans are plotted.
        # Scans are plotted so that the robot is always in the origo and all other coordinates are moved according to
        # odometry information
        self.history_length = 1
        self.xhistory = np.zeros((self.history_length * 1081,))
        self.yhistory = np.zeros((self.history_length * 1081,))
        # Indices for the buffer array
        self.startc = 0
        self.endc = 1081

        # Params for ultrasonic sensor measurements
        self.ultrasoundx = np.zeros((8,))
        self.ultrasoundy = np.zeros((8,))
        degs = np.array([-90, -30, 0, 30, 90, 150, 180, 210])
        rads = degs * np.pi / 180.0
        self.uxmul = np.cos(rads)
        self.uymul = np.sin(rads)
        self.uoffset = np.array([0, 12, 15, 12, 5, 12, 15, 12])  # in cm


        # How many scans to ignore before updating "memory". This or odometry movements can be used to make history more
        # useful and easily visualizable.
        # self.ign = 5
        # self.n = 1

        # Initialize plotting and the canvas
        self.figure = plt.figure(figsize=(15,15))
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.ion()


        self.ax = self.figure.add_subplot(111)
        # The line objects have the real x and y parameters for drawing and they are saved as object's variables.
        self.line1, self.line2, = self.ax.plot([], [], 'o', [], [], 'ro')

        # Subscription to odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        sleep(1)
        np.set_printoptions(precision=3)

    ## Done after recieving laser scan.
    def callback(self, data):
        try:
            data = np.asarray(data.ranges).astype('float32')
            # NAN to 0
            data = np.nan_to_num(data)
            if self.corridor_width is not None:
                data = data.clip(0, self.corridor_width)

            # Transform scan data to coordinates
            if self.past_x is not None:
                xdiff = self.current_x - self.past_x
                ydiff = self.current_y - self.past_y
                adiff = self.current_angle - self.past_angle
                cos, sin = np.cos(adiff), np.sin(adiff)
                (x, y) = (self.xmul * data, self.ymul * data)
                ## Uncomment this when "ignore" params are used
                # if self.n % self.ign == 0:
                #     self.n = 0
                ## Uncomment this if history is updated only after the robot has moved or turned.
                # if (abs(xdiff) > 0.05 or abs(ydiff) > 0.05) or abs(adiff) > np.pi/18 or np.mean(self.xhistory) == 0:
                ## Uncomment this if history_lenght == 1 and real time plotting is the target.
                if True:
                    ## Write to buffer array and set the next indices to be used.
                    self.xhistory[self.startc:self.endc] = x
                    self.yhistory[self.startc:self.endc] = y
                    self.startc += 1081
                    self.endc += 1081
                    ## If array length is used. Start from the beginning and overwrite past history.
                    if self.endc >= (self.history_length) * 1081:
                        self.startc = 0
                        self.endc = 1081
                    # Rotation and translation to each history
                    # Translation
                    self.xhistory -= xdiff
                    self.yhistory -= ydiff
                    xhist = np.copy(self.xhistory)
                    # Rotation
                    self.xhistory = xhist * cos + self.yhistory * sin
                    self.yhistory = - xhist * sin + self.yhistory * cos

                    ## Make current set to the past
                    self.past_x = self.current_x
                    self.past_y = self.current_y
                    self.past_angle = self.current_angle
                # self.n += 1

        except Exception as e:
            print(e)

    ## Called each time odom message is recieved
    def callback_odom(self, data):
        ## Get the coordinates from odom and convert to euler from quaternion
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([0, 0, z, w])
        # Euler angle in radians
        self.current_angle = yaw
        (self.current_x, self.current_y) = (data.pose.pose.position.x, data.pose.pose.position.y)
        ## If the first time --> past is current
        if self.past_x is None:
            (self.past_x, self.past_y, self.past_angle) = (self.current_x, self.current_y, self.current_angle)

    ## Called each time ultrasonic message is recieved
    def callback_usonic(self, data):
        ## Convert custom message to numpy arrays and then convert data from cm to m
        data = np.array([float(data.data[0]), float(data.data[1]), float(data.data[2]), float(data.data[3]),
                         float(data.data[4]), float(data.data[5]), float(data.data[6]), float(data.data[7])])
        adiff = self.current_angle - self.past_angle
        for i in range(len(data)):
            if data[i] == -1:
                self.ultrasoundx[i], self.ultrasoundy[i] = 0, 0

                continue
            else:
                data[i] += self.uoffset[i]
                self.ultrasoundx[i] = data[i] * self.uxmul[i]
                self.ultrasoundy[i] = data[i] * self.uymul[i]
                self.ultrasoundx[i] *= 0.01
                self.ultrasoundy[i] *= 0.01

    def run_controller(self):
        # Update scan image 3 times per second
        r = rospy.Rate(3)
        while (not rospy.is_shutdown()):
            ## Plot laser scan as coordinates to line1 (blue dots)
            self.line1.set_xdata(self.xhistory)
            self.line1.set_ydata(self.yhistory)
            ## Plot usonic as coordinates to line2 (red dots)
            self.line2.set_xdata(self.ultrasoundx)
            self.line2.set_ydata(self.ultrasoundy)
            self.figure.canvas.draw()
            plt.show()
            sleep(0.1)
            r.sleep()

def main():
    ic = laser_listener()
    rospy.init_node("laser_listener", anonymous=True)
    ic.run_controller()

    try:
        # Needed for the subscription
        rospy.spin()


    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()


