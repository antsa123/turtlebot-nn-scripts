#! /usr/bin/env python

# This is a ROS node that publishes the distances measured by the ultrasonic range finders of the green turtlebot.
# Place into /home/user/catkin_ws/src/PACKAGE/src of Turtlebot.
# Building the IntList-message into the used ROS package is required.


import serial, rospy, serial.tools.list_ports
from time import sleep
from ohjaus.msg import IntList
import os


class UltrasonicArray:
    def __init__(self):
        try:
            # Find the serial port Arduino is connected to
            ports = list(serial.tools.list_ports.comports())
            arduino_port = ""
            for p in ports:
                if p[2] == "USB VID:PID=1a86:7523":
                    arduino_port = p[0]
                    print arduino_port
            if arduino_port == "":
                raise serial.SerialException

            self.usonic_pub = rospy.Publisher('/usonic', IntList, queue_size=10)
            self.ser = serial.Serial()
            self.ser.baudrate = 115200
            self.ser.port = arduino_port
            self.ser.timeout = 0.1      # Timeout to wait for response
            self.ser.xonxoff = False    # Disable software flow control
            self.ser.dsrdtr = False     # Disable hardware (DSR/DTR) flow control
            self.ser.rtscts = False     # Disable hardware (RTS/CTS) flow control
            self.ser.open()
            while not self.ser.isOpen():
                pass
        except serial.SerialException:
            print ("USB-error")


    def read_dists(self):
        r = rospy.Rate(20)
        unfinished_line = ""

        while not rospy.is_shutdown():
            dists = []
            # Ask distances from Arduino
            self.ser.write('r')
            # Wait a bit for the serial line
            sleep(0.3)
            try:
                # The responce is a string of 8 integers with spaces between, normally ending in \n
                # Green turtlebot sometimes reads the line incorrectly
                line = (self.ser.readline())
                print line
                # Check if new line has \n
                if '\n' in line:
                    buf = line.split('\n')
                    # Add the first part (remains of last line) to unfinished_line
                    unfinished_line += buf[0]
                    # Extract distances from the now finished line
                    foo = unfinished_line.split(' ')
                    for dist in foo:
                        dists.append(int(dist))
                    # Publish distances
                    self.usonic_pub.publish(dists)
                    # Save the rest of the line
                    unfinished_line = buf[1]

                # Line does not have \n and thus is unfinished, but starts from the right part
                else:
                    unfinished_line += line

                r.sleep()

            except ValueError:
                # Should not get here anymore
                print "Ultrasound-error"
                self.ser.flushOutput()
                self.ser.flushInput()
                self.read_dists()

            except serial.SerialException:
                pass

        # After KeyboardInterrupt
        self.ser.flushOutput()
        self.ser.flushInput()
        self.ser.close()


    def close(self):
        self.ser.close()


def main():
    try:
        ser = UltrasonicArray()
        rospy.init_node("ultrasonic_array", anonymous=True)
    except serial.SerialException:
        return
    except KeyboardInterrupt:
        return

    try:
        ser.read_dists()
    except KeyboardInterrupt:
        ser.close()


main()