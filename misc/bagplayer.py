import rosbag
import cv_bridge
import cv2
import numpy as np
import time
import os

## Params for clipping data and normalization
max = 2000.0
min = 0

np.set_printoptions(threshold=np.nan)

""" Script for playing rosbags and saving the outputs of camera or laser scan topics as numpy arrays."""


## Plays the bagfile in bagfilepath and listens to image topics. Saves images as 32 bit numpy arrays
def read_img_from_bag(bagfilepath, savepath):
    global framenum
    bag = rosbag.Bag(bagfilepath)
    bridge = cv_bridge.CvBridge()
    for topic, msg, t in bag.read_messages(topics=['/camera/depth/image_raw']):
        picname = "frame%05d.np" % (framenum)

        # Read image
        pic = bridge.imgmsg_to_cv2(msg, '16UC1')

        # Crop image
        pic = pic[160:, :]

        #        # Show image
        #        cv2.imshow('kuva', pic)
        #        cv2.waitKey(10000)

        # Resize picture
        pic = cv2.resize(pic, dsize=(0, 0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)

        # Clip values
        pic = np.clip(pic, 0, 2000)

        # Convert to 32 bit floating point numbers
        # Normalize values to range [0,1] by dividing with max value
        pic = pic.astype('float32')
        pic = pic / max

        # Save to file as a numpy array
        pic.tofile(savepath + os.sep + picname)
        framenum += 1
    bag.close()


## Plays the bagfile in bagfilepath and listens to /scan topic. Writes messages as numpy arrays to savepath.
def laser_from_bag(bagfilepath, savepath):
    global framenum
    bag = rosbag.Bag(bagfilepath)
    for topic, msg, t in bag.read_messages(topics=['/scan']):
        scan = np.asarray(msg.ranges).astype('float32')
        scan = np.nan_to_num(scan)
        scan = np.clip(scan, 0, 5000)
        scanname = 'scan_%05d.np' % framenum
        scan.tofile(savepath + os.sep + scanname)
        framenum += 1


## Go through subdirectories in path and look for bagfiles and subdirectories recursively. Works in a depth first search
#  kind of way
def walk_subdirectories(path):
    # Init subdirectory list for current directory
    new_subdir = []
    # Go through all files in the parameter path
    for filu in os.listdir(path):
        # If directory is found --> append to the global subdirectories and current directory's subdirs.
        if os.path.isdir(path + os.sep + filu):
            subdir.append(path + os.sep + filu)
            new_subdir.append(path + os.sep + filu)
        # If bagfile is found --> append to global bagfiles list
        elif '.bag' in filu:
            bagfiles.append(path + os.sep + filu)
        else:
            # print('    jokin meni vikaan')
            pass
    # Call this function recursively to each subdirectory in current directory.
    for folder in new_subdir:
        walk_subdirectories(folder)


if __name__ == '__main__':
    bagfiles = []
    subdir = []
    framenum = 1
    walk_subdirectories(os.getenv("HOME") + '/syvyyskuvat/syvyyskuvat2')
    sp = os.getenv("HOME") + '/syvyyskuvat/syvyyskuvat2'
    for bagfile in bagfiles:
        if 'suo' in bagfile:
            savepath = sp + os.sep + 'suoraan_np'
        elif 'oik' in bagfile:
            savepath = sp + os.sep + 'oikea_np'
        elif 'vas' in bagfile:
            savepath = sp + os.sep + 'vasen_np'
        else:
            savepath = ""
        read_img_from_bag(bagfile, savepath)

