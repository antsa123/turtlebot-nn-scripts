import keras.backend as K
import matplotlib.pyplot as plt
import os
import numpy as np
import time

from keras.models import load_model
from math import ceil

## Set learning / training to False
K.set_learning_phase(0)


## Change filepath to match the desired model
homedir = os.getenv('HOME')
filepath = homedir + os.sep + 'laserclipped.hdf5'
# filepath = homedir + os.sep + 'depth.hdf5'

## Load the actual model from hdf5
model = load_model(filepath)


"""Example calls to the following classes and functions are set as comments in below the class definitions."""


## Function to convert scans to x and y coordinates
def laser2coordinates(scan):
    deg_step = 270 / 1081.0
    rad_step = deg_step * (np.pi / 180.0)
    rads = np.arange((-45 * np.pi / 180), (225 * np.pi / 180), rad_step)
    xmul = np.cos(rads)
    ymul = np.sin(rads)
    scan = scan.reshape((1081,))
    x, y = xmul * scan, ymul * scan
    return x, y

## Class to visualize laser networks
class LaserVisualization():

    def __init__(self, model):
        print(model.summary())
        # Model definition and trained weights
        self.model = model
        # Place holder for self.iterate inputs (None)
        self.input_img = model.input
        # All layers by name
        self.layer_dict = dict([(layer.name, layer) for layer in model.layers])
        # All max activations for filters by layer name
        self.filter_dict = dict()
        self.filters = 25

        # Laser model parameters (not used)
        self.kernel = 25
        self.conv1_output_size = (1057, 1)
        self.conv2_output_size = (504, 1)
        self.conv1 = model.layers[0]
        self.conv2 = model.layers[3]
        self.pool1 = model.layers[1]
        self.pool2 = model.layers[4]

    ## Normalization function
    def normalize(self, x):
        return x / K.sqrt(K.mean(K.square(x)) + 1e-5)


    ## Plot feature maps generated by a certain input from each layer.
    def plot_feature_maps(self, size, range_o, layer_index=1):
        layer_output = self.get_layer_output(layer_index)
        for j in range_o:
            fig, axes = plt.subplots(self.filters, sharex=True, sharey=True)
            for i in range(self.filters):
                axes[i].imshow(layer_output[j,:,i].reshape(size).T, interpolation='nearest', aspect='auto', vmin=0, vmax=5)
                axes[i].yaxis.set_visible(False)
            fig.suptitle('Input %d generated these feature maps in layer %d' %(j, layer_index))

    ## Plot laser scans as a colour gradient
    def plot_input_data(self, range_o, title=""):
        global X
        global Y
        fig, axes = plt.subplots(len(range_o), sharex=True, sharey=True)
        for i, j in zip(range_o, range(len(axes))):
            im = axes[j].imshow(X[i].T, interpolation='nearest', aspect='auto', vmin=0.2, vmax=1.5)
            axes[j].yaxis.set_visible(False)
        fig.subplots_adjust(right=0.8)
        cbar_ax = fig.add_axes([0.85, 0.15, 0.05, 0.7])
        fig.colorbar(im, cax=cbar_ax)
        fig.suptitle(title)

    ## Get output from a certain layer
    def get_layer_output(self, layer_index):
        output = K.function([model.layers[0].input, K.learning_phase()], [model.layers[layer_index].output])
        return output([X, 0])[0]

    ## Function that creates inputs that maximize desired neurons' activation from random initialization
    def get_inputs_with_max_activation(self, range_o, layer_name, input_shape = (1081,1), l2_coeff = 0.65, step_size = 1):
        kept_filters = []
        for filter_index in range_o:
            print('Processing filter %d' %(filter_index + 1))
            start_time = time.time()

            layer_output = self.layer_dict[layer_name].output
            if 'dense' in layer_name:
                loss = K.mean(layer_output[:, filter_index])
            else:
                loss = K.mean(layer_output[:, :, filter_index])


            grads = K.gradients(loss, self.input_img)

            grads = self.normalize(grads)

            self.iterate = K.function([self.input_img], [loss, grads])

            ## Start from random or specific input initialization
            # input_img_data = np.random.random((1, input_shape[0], input_shape[1]))
            input_img_data = np.ones((1, 1081, 1)) * 3

            # Gradient ascent to maximize desired neuron's / filter's activation
            # Change the input according to the gradient on each iteration.
            for i in range(20):
                loss_v, grads_v = self.iterate([input_img_data])
                input_img_data = l2_coeff * (input_img_data + grads_v * step_size)
                print(4 * ' ' + 'Current loss: %.3f' %loss_v)

                # Stop gradient ascent if loss goes to 0 or negative (Failed optimization)
                if loss_v <= 0.:
                    break
            # Save inputs that were successfully optimized
            if loss_v > 0:
                img =input_img_data[0]
                kept_filters.append((img, loss_v))

            end_time = time.time()

            print(8 * ' ' + 'Filter %d processed in %ds' % (filter_index + 1, end_time - start_time))

            ## Match inputs to layer name in the object's variable dict.
            self.filter_dict[layer_name] = (kept_filters, input_shape)

    ## Plot the inputs that were created using the method above.
    ## If save parameter is set to True images are saved one by one to the current working directory
    def visualize_filters(self, layer_name, save=False):
        kept_filters, input_shape = self.filter_dict[layer_name]
        kept_filters.sort(key=lambda x: x[1], reverse=True)
        try:
            # nrows = int(ceil(len(kept_filters) / 4))
            ncols = int(ceil(len(kept_filters) / 4))
            nrows = int(ceil(len(kept_filters) / ncols))
            # ncols = 3
            # nrows = 1
        except ZeroDivisionError:
            ncols = 1
            nrows = len(kept_filters)

        fig, axes = plt.subplots(nrows, ncols)
        k = 0

        for col in range(ncols):
            for row in range(nrows):
                try:
                    img, loss = kept_filters[k]
                    k += 1
                except IndexError:
                    break
                img = np.reshape(img[:, :], (input_shape[0], input_shape[1]))
                x, y = laser2coordinates(img)
                if save:
                    plt.figure()
                    plt.plot(x, y, '.')
                    plt.xlim(-5, 5)
                    plt.ylim(-5, 5)
                    plt.savefig('%d + %d.pdf' %(col, row))
                    plt.close()
                elif axes.shape == (ncols,):
                    axes[col].plot(x, y, 'g.')
                elif axes.shape == (nrows,):
                    axes[row].plot(x, y, 'g.')
                else:
                    axes[row, col].plot(x, y, 'g.')

### Laser data visualization

# lv = LaserVisualization(model)

# range_a = range(0,25)

## Clipped params
# lv.get_inputs_with_max_activation(range_a,'conv1d_1', l2_coeff=0.85, step_size=3); lv.visualize_filters('conv1d_1')
# lv.get_inputs_with_max_activation(range_a,'conv1d_2', l2_coeff=0.8, step_size=1); lv.visualize_filters('conv1d_2')
# lv.get_inputs_with_max_activation(range(0,150),'dense_1'); lv.visualize_filters('dense_1', save=True)
# lv.get_inputs_with_max_activation([0,1,2],'dense_2', l2_coeff=0.65, step_size=1); lv.visualize_filters('dense_2', save=True)

## 1_toimiva params
# lv.get_inputs_with_max_activation(range_a,'conv1d_1', l2_coeff=0.90, step_size=20); lv.visualize_filters('conv1d_1')
# lv.get_inputs_with_max_activation(range_a,'conv1d_2', l2_coeff=0.85, step_size=2); lv.visualize_filters('conv1d_2')
# lv.get_inputs_with_max_activation(range(0,100),'dense_1', l2_coeff=0.65, step_size=1); lv.visualize_filters('dense_1', save=True)
# lv.get_inputs_with_max_activation([0,1,2],'dense_2', l2_coeff=0.9, step_size=1); lv.visualize_filters('dense_2', save=True)

# range_a = range(100,105)
# range_b = range(2300, 2310)
# range_c = range(3100,3110)
#
# lv.plot_input_data(range_a, 'Kaanny oikealle input')
# lv.plot_input_data(range_b, 'Aja suoraan input')
# lv.plot_input_data(range_c, 'Kaanny vasemmalle input')
#
# lv.plot_feature_maps(lv.conv1_output_size, range_a, 1)
# lv.plot_feature_maps(lv.conv2_output_size, range_a, 3)
#
# plt.show()

## Visualization for depth image networks. Works the same way as laser visualization
class DepthImageFeatureVisualization():

    def __init__(self, model):
        print(model.summary())
        # Model definition and trained weights
        self.model = model
        # Place holder for self.iterate inputs (None)
        self.input_img = model.input
        # All layers by name
        self.layer_dict = dict([(layer.name, layer) for layer in model.layers])
        # All max activations for filters by layer name
        self.filter_dict = dict()

    def normalize(self, x):
        return x / K.sqrt(K.mean(K.square(x)) + 1e-5)

    ## Input images have to be normalized. Otherwise they can't be shown as images.
    def deprocess_image(self, x):

        # Normalize between 0 and 1
        x -= x.mean()
        x /= (x.std() + 1e-5)
        x *= 0.1

        x += 0.5
        x = np.clip(x, 0, 1)

        # Multiply by 255 to make it an RGB image to show
        x *= 255

        # Clip again to ensure no errors and convert to integers
        x = np.clip(x, 0 ,255).astype('uint8')

        return x

    def get_inputs_with_max_activation(self, range_o, layer_name, input_shape = (80,160,1), l2_coeff = 0.6, step_size = 2):
        kept_filters = []
        for filter_index in range_o:
            print('Processing filter %d' %(filter_index + 1))
            start_time = time.time()

            layer_output = self.layer_dict[layer_name].output

            if 'dense' in layer_name:
                loss = K.mean(layer_output[:,filter_index])

            else:
                loss = K.mean(layer_output[:, :, :, filter_index])


            grads = K.gradients(loss, self.input_img)

            grads = self.normalize(grads)

            self.iterate = K.function([self.input_img], [loss, grads])

            input_img_data = np.random.random((1, input_shape[0], input_shape[1], input_shape[2]))
            # input_img_data = np.ones((1, input_shape[0], input_shape[1], input_shape[2]))

            input_img_data = (input_img_data - 0.5) * 20 + 123

            #Gradient ascent
            for i in range(100):
                loss_v, grads_v = self.iterate([input_img_data])
                input_img_data = l2_coeff * (input_img_data + grads_v * step_size)
                print(4 * ' ' + 'Current loss: %.3f' %loss_v)

                if loss_v <= 0.:
                    break
            if loss_v > 0:
                img = self.deprocess_image(input_img_data[0])
                kept_filters.append((img, loss_v))

            end_time = time.time()

            print(8 * ' ' + 'Filter %d processed in %ds' % (filter_index + 1, end_time - start_time))

            self.filter_dict[layer_name] = (kept_filters, input_shape)

    ## If save parameter is set to True images are saved one by one to the current working directory
    def visualize_filters(self, layer_name, save=False):
        kept_filters, input_shape = self.filter_dict[layer_name]
        kept_filters.sort(key=lambda x: x[1], reverse=True)
        try:
            ncols = int(ceil(len(kept_filters) / 4))
            nrows = int(ceil(len(kept_filters) / ncols))
        except ZeroDivisionError:
            ncols = 1
            nrows = len(kept_filters)

        fig, axes = plt.subplots(nrows, ncols)
        k = 0

        for col in range(ncols):
            for row in range(nrows):
                try:
                    img, loss = kept_filters[k]
                    k += 1
                except IndexError:
                    break
                img = np.reshape(img[:, :], (input_shape[0], input_shape[1]))
                if save:
                    plt.figure()
                    plt.imshow(img, cmap='gray')
                    # plt.xlim(-5, 5)
                    # plt.ylim(-5, 5)
                    plt.savefig('%d + %d.pdf' %(col, row))
                    plt.close()
                elif axes.shape == (ncols,):
                    axes[col].imshow(img, 'gray')
                elif axes.shape == (nrows,):
                    axes[row].imshow(img, 'gray')
                else:
                    axes[row, col].imshow(img, cmap='gray')


### Depth data visualization
# dv = DepthImageFeatureVisualization(model)
# dv.get_inputs_with_max_activation(range(0,32), 'conv2d_1', (80, 160, 1)); dv.visualize_filters('conv2d_1', save=True)
# dv.get_inputs_with_max_activation(range(0,32), 'conv2d_2', l2_coeff=0.8); dv.visualize_filters('conv2d_2', save=True)
# dv.get_inputs_with_max_activation(range(0,64), 'conv2d_3', l2_coeff=0.5, step_size=2); dv.visualize_filters('conv2d_3', save=True)
# dv.get_inputs_with_max_activation([0,1,2], 'dense_1', l2_coeff=0.55, step_size=10); dv.visualize_filters('dense_1', save=True)
# dv.get_inputs_with_max_activation([0,1,2], 'dense_1', l2_coeff=0.90, step_size=0.5); dv.visualize_filters('dense_1', save=True)


# plt.show()

# model, model_inputs, print_shape_only=False, layer_name=None
# get_activations(model, X, layer_name='conv2d_1')