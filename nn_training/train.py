import numpy as np

# Import network definitions
from network_definitions import *
# Import functions for reading data
from read_data import *
# Import keras nn libraries
from keras.callbacks import ModelCheckpoint, Callback
from keras.optimizers import SGD
# Sklearn function for easy randomized data splitting
from sklearn.model_selection import train_test_split

""" The pipeline for training a network in a single script. Use GPU in training by running in terminal and adding
    THEANO_FLAGS=device=gpu,floatX=float32 in front of the bash command.
    For example:
    THEANO_FLAGS=device=gpu,floatX=float32 python /home/neuron/PycharmProjects/nn/train.py"""

if __name__ == '__main__':

    X = []
    Y = []

    ## Read data to lists here
    #
    #
    #
    #
    #


    ## Convert lists to numpy arrays for the networks

    X = np.asarray(X)
    Y = np.asarray(Y)

    ## Create the model object using some definition
    model = model_laser_definition()

    ## Create a checkpoint for saving the network when validation loss decreases
    cp = ModelCheckpoint('/home/neuron/all_in_one/lase-clip1m-conv1-flipped-data.hdf5', save_best_only=True)

    ## Give optimizer parameters
    sgd = SGD(lr=0.0005, decay=1e-6, momentum=0.9, nesterov=True)

    ## Split data to training and validation data using sklearn
    X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.3, random_state=12)

    ## Compile model to C --> must be done with theano and tensorflow backends
    model.compile(loss='mean_squared_error', optimizer=sgd, metrics=['accuracy'])

    ## Start training with given hyperparameters
    model.fit(X_train, Y_train, batch_size=200, epochs=30000, verbose=1, validation_data=(X_test, Y_test), callbacks=[cp], shuffle=True)