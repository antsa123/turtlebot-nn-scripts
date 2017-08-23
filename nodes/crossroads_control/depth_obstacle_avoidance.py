#! /usr/bin/env python

# This is a ROS node for publishing a prediction of a neural network about the need to avoid oncoming
# obstacles.


import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
import numpy as np
from keras.models import load_model
import rospy, cv2, os, cv_bridge


class Depth_obstacle_avoidance():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.control_pub = rospy.Publisher('/depth_obstacle_avoidance', Vector3, queue_size=5)
        self.depth_sub = rospy.Subscriber('/kinect2/sd/image_depth', Image, self.callback_img)
        self.depth_model = load_model(os.getenv('HOME') + '/catkin_ws/src/vector-map/navigointi/DNN_Project/vaisto_verkot/depth_toimiva.hdf5')
        self.latest_cv_img = None

    def callback_img(self, data):
        self.latest_cv_img = self.bridge.imgmsg_to_cv2(data, "16UC1")


    def run(self):
        r = rospy.Rate(10)
        while self.latest_cv_img is None:
            print "no image"
        try:
            while not rospy.is_shutdown():
                # Crop (image size 252x304)
                cv_img = self.latest_cv_img[100:, :]

                # Shrink to 160x80
                cv_img = cv2.resize(cv_img, dsize=(160, 80), interpolation=cv2.INTER_AREA)

                ## Katkaise ja normalisoi
                cv_img = np.clip(cv_img, 0, 5000)
                cv_img = cv_img.astype('float32')

                # kuva = np.expand_dims(kuva, 2)
                cv_img = cv_img.reshape((80, 160, 1))

                control_signal = self.depth_model.predict(np.array([cv_img]))
                # If the prediction to turn either left or right is greater than 40%, publish that prediction
                if control_signal[0, 0] > 0.4 or control_signal[0, 2] > 0.4:
                    self.control_pub.publish(Vector3(control_signal[0, 0], control_signal[0, 1], control_signal[0, 2]))
                # Else, publish an "empty" prediction
                else:
                    self.control_pub.publish(Vector3(0.0, 0.0, 0.0))
                r.sleep()
        except KeyboardInterrupt:
            rospy.on_shutdown(h=0)


def main():
    ic = Depth_obstacle_avoidance()
    rospy.init_node("depth_obstacle_avoidance", anonymous=True)
    ic.run()

    try:
        # Needed for the subscriber
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()