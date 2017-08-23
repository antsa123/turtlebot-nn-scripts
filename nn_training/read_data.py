import numpy as np
import cv2
import os
from sklearn.preprocessing import normalize
#import matplotlib.pyplot as plt


""" Contains several functions used in reading data to lists from labeled folders.
    All of the following functions follow the same idea as the first one. You should get a pretty good idea what's
    happening by reading the first functiond comments"""

## Folder param is the path to a folder containing specific labeled data.
#  Lists X* are training data lists that are dynamically appended from the folder.
#  Y is a list that contains the known training output.
#  X* and Y have to have the same order
def read_synchronized_npy(folder, X1, X2, X3, X4, Y):
    ## Init the target classes to zero
    y = np.array([0,0,0])
    ## Label according to the folder name
    if 'oik' in folder:
        y = np.array([0,0,1])
        y1 = np.array([0,0,1])
        y2 = np.array([1, 0, 0])
    elif 'suo' in folder:
        y = np.array([0, 1, 0])
        y1 = np.array([0, 1, 0])
        y2 = np.array([0, 1, 0])
    elif 'vas' in folder:
        y = np.array([1, 0, 0])
        y1 = np.array([1, 0, 0])
        y2 = np.array([0, 0, 1])
    else:
        assert()
    ## Go through every file in the folder and read the npy file to an array, reshape it and append to the correct
    # training data list
    for filu in sorted(os.listdir(folder)):
        if 'frame' in filu:
            x = np.load(folder + os.sep + filu)
            x = np.clip(x, 0, 5000)
            # x *= 0.001
            x = x.reshape(80, 160, 1)
            X1.append(x)
        elif 'scan' in filu:
            x = np.load(folder + os.sep + filu)
            x1 = np.clip(x, 0, 10)
            x2 = np.clip(x, 0, 1)
            x3 = x2[::-1]
            # x1 = normalize(x1.reshape(1,-1))
            # x2 = normalize(x2.reshape(1, -1))
            X2.append(x1.reshape((1081,1)).astype('float32'))
            X3.append(x2.reshape((1081,1)).astype('float32'))
            # Flipattu setti
            X3.append(x3.reshape((1081,1)).astype('float32'))
            Y.append(y1)
        elif 'sonic' in filu:
            x = np.load(folder + os.sep + filu)
            x *= 0.01
            X4.append(x.astype('float32'))



def read_synchronized(folder, X1, X2, Y):
    y = np.array([0,0,0])
    if 'oik' in folder:
        y = np.array([0,0,1])
    elif 'suo' in folder:
        y = np.array([0, 1, 0])
    elif 'vas' in folder:
        y = np.array([1, 0, 0])
    else:
        assert()
    for filu in sorted(os.listdir(folder)):
        if 'frame' in filu:
            x = np.fromfile(folder + os.sep + filu, dtype='float32', count=-1, sep="")
            x = np.clip(x, 0, 5000)
            x /= 5000
            x = x.reshape(80,160,1)
            X1.append(x)
        if 'scan' in filu:
            x = np.fromfile(folder + os.sep + filu, dtype='float32', count=-1, sep="")
            x = np.clip(x, 0, 10.0)
            x = x[180:901]
            x = normalize(x.reshape(1, -1)).reshape((721,1))
            X2.append(x)
            Y.append(y)

## Function for reading image data from saved .jpg files
def read_imgs(folder, X, Y):
    if 'oikealle' in folder:
        y = np.array([0,0,1])
    elif 'vasemmalle' in folder:
        y = np.array([1,0,0])
    elif 'suoraan' in folder:
        y = np.array([0,1,0])
    else:
        assert()
    subdirs = os.listdir(folder)
    for subdir in subdirs:
        polku = (folder + os.sep + subdir)
        if os.path.isdir(polku):
            for kuva in os.listdir(polku):
                if kuva[-4:] == '.jpg':
                    kuva = cv2.imread(polku + os.sep + kuva,0)
                    kuva = cv2.resize(kuva, None, fx=0.25, fy=0.25, interpolation = cv2.INTER_AREA)
                    kuva = np.expand_dims(kuva, 2)
                    kuva = kuva.reshape((120,160,1)) / 255
                    X.append(kuva)
                    Y.append(y)
        else:
            pass

## Function for reading image data from saved np arrays
def read_imgs2(folder, X, Y):
    y = np.array([0,0,0])
    if 'oik' in folder:
        y = np.array([0,0,1])
    elif 'vas' in folder:
        y = np.array([1,0,0])
    elif 'suo' in folder:
        y = np.array([0,1,0])
    else:
        assert()
    for img in os.listdir(folder):
        x = np.fromfile(folder + os.sep + img, dtype='float32', count=-1, sep="")
        x = x.reshape(80,160,1)
        X.append(x)
        Y.append(y)


def read_lasers(folder, X, Y):
    y = np.array([0, 0, 0])
    if 'oik' in folder:
        y = np.array([0, 0, 1])
    elif 'vas' in folder:
        y = np.array([1, 0, 0])
    elif 'ete' in folder:
        y = np.array([0, 1, 0])
    else:
        assert ()

    for scan in os.listdir(folder):
        x = np.fromfile(folder + os.sep + scan, dtype='float32', count=-1, sep="")
        # x = x.reshape((1081, 1))
        # Clip and normalize
        x = np.clip(x, 0, 10)
        #x /= 5
        x = normalize(x.reshape(1,-1))
        x = x.reshape((1081,1))

        X.append(x)
        Y.append(y)

def read_lasers_crossroads(folder, X, Y, train='v'):

    #y = np.array([0, 0, 0, 0, 0, 0, 0])
    y = np.array([0,0,0,0,0])

    # output = [ei, TL, TR, T, plus, (umpi)]
    if 'umpi' in folder:
        #y = np.array([0, 0, 0, 0, 0, 1])
        y = np.array([1, 0, 0, 0, 0])
    elif 'ei_rist' in folder:
        #y = np.array([1, 0, 0, 0, 0, 0])
        y = np.array([1, 0, 0, 0, 0])
    elif 'T_rist' in folder:
        #y = np.array([0, 0, 0, 1, 0, 0])
        y = np.array([0, 0, 0, 1, 0])
    elif 'TL_rist' in folder:
        #y = np.array([0, 1, 0, 0, 0, 0])
        y = np.array([0, 1, 0, 0, 0])
    elif 'TR_rist' in folder:
        #y = np.array([0, 0, 1, 0, 0, 0])
        y = np.array([0, 0, 1, 0, 0])
    elif 'plus' in folder:
        #y = np.array([0, 0, 0, 0, 1, 0])
        y = np.array([0, 0, 0, 0, 1])
    else:
        assert()

    for scan in os.listdir(folder):
        x = np.fromfile(folder + os.sep + scan, dtype='float32', count=-1, sep="")
        x = np.clip(x, 0, 10)
        #x /= 3.0
        #coords = laserit_koordinaateiksi(x)
        #X.append(coords)
        #x = normalize(x.reshape(1, -1))
        x = x.reshape((1081,))
        X.append(x)
        Y.append(y)


def read_simu_crossroads(X,Y, net):

    output1_folders = []
    output0_folders = []
    path = '/home/neuron/opetuslaserit/risteykset/risteys_scan_simu/'

    if net == "o":
        output0_folders = ['left', 'deadend', 'straight', 'T-left']
        output1_folders = ['right', 'left_right', 'T-right']

    elif net == "v":
        output0_folders = ['deadend', 'right', 'straight', 'T-right']
        output1_folders = ['left', 'left_right', 'T-left']

    elif net == "s":
        output0_folders = ['deadend', 'left', 'right', 'left_right']
        output1_folders = ['T-left', 'straight', 'T-right']

    else:
        assert()

    for folder in output1_folders:
        y = 1

        folderpath = path + folder

        for scan in os.listdir(folderpath):
            x = np.fromfile(folderpath + os.sep + scan, dtype='float32', count=-1, sep="")
            x = x.reshape((1081,1))
            X.append(x)
            Y.append(y)

    for folder in output0_folders:
        y = 0

        folderpath = path + folder
        for scan in os.listdir(folderpath):
            x = np.fromfile(folderpath + os.sep + scan, dtype='float32', count=-1, sep="")
            x = x.reshape((1081, 1))
            X.append(x)
            Y.append(y)


## Convert laser scans to x and y coordinates in a numpy array
def laser2coordinates(scan):
    deg_step = 270 / 1081.0
    rad_step = deg_step * (np.pi / 180.0)
    rads = np.arange((-45 * np.pi / 180), (225 * np.pi / 180), rad_step)
    xmul = np.cos(rads)
    ymul = np.sin(rads)
    (x, y) = (xmul * scan, ymul * scan)
    return np.asarray((x, y)).T
