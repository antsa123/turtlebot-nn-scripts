#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from time import sleep
import matplotlib.pyplot as plt

""" Node for plotting angular velocities for left and right and linear velocity as a bar chart in real time."""

## Container class
class BarPlotter:

    def __init__(self):
        # Initial values vor velocities. 0 -> rotate left, 1 -> linear velocity straight, 2 -> rotate right
        self.cmd0 = 0
        self.cmd1 = 0
        self.cmd2 = 0

        # Subscriber
        self.twist_sub = rospy.Subscriber("/navigation_velocity_smoother/raw_cmd_vel", Twist, self.callback)

        # Plotting initialization and settings
        plt.ion()
        self.fig = plt.figure(figsize=(15,15))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_ylim(0,1)
        # Take the bar objects to a list while creating them.
        self.bars = self.ax.bar([1, 3, 5], [0, 0, 0], tick_label=['Turn left', 'Go straight', 'Turn right'], align='center')
        for tick in self.ax.xaxis.get_major_ticks():
            tick.label.set_fontsize(16)


    # Gets called after each velocity message
    def callback(self, data):
        if data.angular.z <= 0:
            self.cmd2 = abs(data.angular.z)
            self.cmd0 = 0
        else:
            self.cmd0 = data.angular.z
            self.cmd2 = 0
        self.cmd1 = data.linear.x / 0.3 #  Linear velocity is normalized to [0, 1] for visualization

    # Actual plotting loop
    def plot(self):
        r = rospy.Rate(3)
        while (not rospy.is_shutdown()):
            # Instead of redrawing we adjust the height of the bar objects in the list self.bars
            self.bars[0].set_height(self.cmd0)
            self.bars[1].set_height(self.cmd1)
            self.bars[2].set_height(self.cmd2)
            self.fig.canvas.draw()
            plt.show()
            # sleep(0.1)
            r.sleep()

if __name__ == '__main__':
    bars = BarPlotter()
    rospy.init_node("vel_command_listener", anonymous=True)
    bars.plot()

    try:
        # Needed for the subscription
        rospy.spin()


    except KeyboardInterrupt:
        print("Shutting down")
