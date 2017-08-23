import keras
from keras.models import Sequential, Model
from keras.layers import Conv2D, Conv1D, Dense, Dropout, Flatten, MaxPooling2D, MaxPooling1D, Merge, concatenate, Input, add
from keras.regularizers import l1, l2

""" Several network definitions for classification. For further use --> please use functional API instead of sequential
    API as in the definitions after line 85. Sequential API is being depreceated slowly and makes it hard to create
    complex MIMO networks or fusion networks. Use unique names for layers to make network fusion and reusing easier."""

classes = 3

## SEQUENTIAL DEFINITION
def model_img_definition(only_conv=False):
    ## Create sequential base
    model = Sequential()


    ## input dimensions and parameters for the network
    imgw = 80
    imgh = 160
    channels = 1

    N = 412

    input_shape = (imgw, imgh, channels)
    filters = 25
    filters2 = 35
    kernel_size = (5,5)

    ## Add layer by layer
    model.add(Conv2D(filters, kernel_size, activation='relu', input_shape = input_shape, kernel_regularizer=l2(0.005))) #32 filtteria
    model.add(MaxPooling2D((2,2), strides = 2))
    model.add(Conv2D(filters, kernel_size, activation='relu', kernel_regularizer=l2(0.005))) #32 filtteria
    model.add(MaxPooling2D((2,2), strides = 2))
    model.add(Conv2D(filters2, kernel_size, activation='relu', kernel_regularizer=l2(0.005))) #64 filtteria
    model.add(MaxPooling2D((2,2), strides = 2))
    model.add(Flatten())
    if only_conv:
        return model
    model.add(Dense(classes, activation='softmax', kernel_regularizer = l2(0.05)))


    return model

## SEQUENTIAL DEFINITION
def model_laser_definition(only_conv=False, array_size=None):
    model = Sequential()

    if array_size is None:
        array_size = (1081, 1)
    else:
        array_size = array_size

    filters = 25
    filters2 = 25
    kernel_size = 25

    model.add(Conv1D(filters, kernel_size, activation='elu', input_shape = array_size, kernel_regularizer=l2(0.01)))
    model.add(MaxPooling1D())
    model.add(Dropout(0.5))
    model.add(Conv1D(filters2, kernel_size, activation='elu', kernel_regularizer=l2(0.01)))
    model.add(MaxPooling1D())
    model.add(Flatten())
    if only_conv:
        return model
    model.add(Dense(150, activation='elu'))
    model.add(Dense(classes, activation='softmax'))
    # model.add(Dense(100, input_shape=array_size, activation='elu', kernel_regularizer=l2(0.01)))
    # model.add(Dense(100, activation='elu', kernel_regularizer=l2(0.01)))
    # model.add(Dense(classes, activation='softmax'))
    return model

## SEQUENTIAL DEFINITION
def model_merged_laser_depth_definition():
    lasermodel = model_laser_definition(only_conv=True, array_size=(721,1))
    depthmodel = model_img_definition(only_conv=True)
    merged = Merge([depthmodel,lasermodel], mode='concat')

    model = Sequential()
    model.add(merged)
    model.add(Dense(150, activation='elu'))
    model.add(Dense(classes, activation='softmax'))

    return model

## FUNCTIONAL API DEFINITION
def model_all_in_definition():
    classes = 3

    ## INPUTS and their shapes
    image_input = Input(shape=(40, 132, 1), dtype='float32', name='image_input')
    laser_input1 = Input(shape=(1081,), dtype='float32', name='laser_input_non_clipped')
    laser_input2 = Input(shape=(1081,), dtype='float32', name='laser_input_clipped')
    usonic_input = Input(shape=(8,), dtype='float32', name='usonic_input')

    ## Image model and params

    img_filters = 25
    img_filters2 = 35
    img_kernel_size = (5, 5)

    ## Create layer objects and use them as callables
    #  Example: layer2 is first hidden layer and its output is an input to layer 3.
    #  layer2 = layertype(layerparameters)(layer's input layer)
    #  layer3 = layertype(layerparameters)(layer2)
    img_convolution_1 = Conv2D(img_filters, img_kernel_size, activation='relu', kernel_regularizer=l2(0.005), name="img_convolution_1")(image_input)
    img_pooling_1 = MaxPooling2D((2,2), strides = 2, name="img_pooling_1")(img_convolution_1)
    img_convolution_2 = Conv2D(img_filters, img_kernel_size, activation='relu', kernel_regularizer=l2(0.005), name="img_convolution_2")(img_pooling_1)
    img_pooling_2 = MaxPooling2D((2,2), strides = 2, name="img_pooling_2")(img_convolution_2)
    img_convolution_3 = Conv2D(img_filters2, img_kernel_size, activation='relu', kernel_regularizer=l2(0.005), name="img_convolution_3")(img_pooling_2)
    img_pooling_3 = MaxPooling2D((2,2), strides = 2, name="img_pooling_3")(img_convolution_3)
    img_flatten = Flatten(name="img_flatten")(img_pooling_3)
    dense_img = Dense(100, activation='elu')(img_flatten)

    ## Laser model not clipped

    dense_laser1 = Dense(30, activation='elu', input_shape=(1081,), name='laser_input_layer1', kernel_regularizer=l2(0.01))(laser_input1)

    ## Laser model clipped

    dense_laser2 = Dense(30, activation='elu', input_shape=(1081,), name='laser_input_layer2', kernel_regularizer=l2(0.01))(laser_input2)

    ## Usonic model

    dense_usonic1 = Dense(10, activation='elu', input_shape=(8,), name='usonic_input_layer', kernel_regularizer=l2(0.01))(usonic_input)
    dense_usonic2 = Dense(20, activation='elu', name='usonic_hidden_layer', kernel_regularizer=l2(0.01))(dense_usonic1)

    merged = concatenate([dense_img, dense_laser1, dense_laser2])

    main_output = Dense(classes, activation='softmax', name='turning_prediction')(merged)

    model = Model(inputs=[image_input, laser_input1, laser_input2], outputs=[main_output])

    return model

def model_merge_pretrained(laser_model, depth_model):
    image_input = Input(shape=(80, 160, 1), dtype='float32', name='image_input')
    laser_input = Input(shape=(1081, 1), dtype='float32', name='laser_input')

    ## Transforming sequential layers to functional api (No names inserted)

    laser_layer_names = [layer.name for layer in laser_model.layers]
    depth_layer_names = [layer.name for layer in depth_model.layers]

    i = 0
    k = 0
    for name in laser_layer_names:
        laser_model.get_layer(name=name).trainable = False
        laser_model.get_layer(name=name).name='laser_layer%d' %i
        i+=1
    for name in depth_layer_names:
        depth_model.get_layer(name=name).trainable = False
        depth_model.get_layer(name=name).name = 'depth_layer%d' %k
        k+=1

    # Laser
    laser_model_layers = laser_model.layers
    x = laser_model_layers[0](laser_input)

    for k in range(1,len(laser_model_layers)):
        x = laser_model_layers[k](x)

    laser_prediction_layer = x


    # Depth
    depth_model_layers = depth_model.layers

    x = depth_model_layers[0](image_input)

    for i in range(1, len(depth_model_layers)):
        x = depth_model_layers[i](x)

    depth_prediction_layer = x

    merged = add(inputs=[depth_prediction_layer, laser_prediction_layer])
    x = Dense(10, kernel_regularizer=l2(0.01))(merged)
    main_output = Dense(classes, activation='softmax', name='prediction_layer')(x)

    model = Model(inputs=[image_input, laser_input], outputs=[main_output])

    print(model.summary())

    return model


def model_merged_laser_depth_definition_functional():
    classes = 3

    image_input = Input(shape=(80, 160, 1), dtype='float32', name='image_input')
    laser_input = Input(shape=(721, 1), dtype='float32', name='laser_input')

    ## Defining laser model with functional api

    laser_filters = 25
    laser_filters2 = 25
    laser_kernel_size = 25

    laser_convolution_1 = Conv1D(laser_filters, laser_kernel_size, activation='elu', input_shape = (721, 1), kernel_regularizer=l2(0.01), name="laser_convolution_1")(laser_input)
    laser_pool_1 = MaxPooling1D(name="laser_pool_1")(laser_convolution_1)
    laser_dropout = Dropout(0.5, name='laser_dropout')(laser_pool_1)
    laser_convolution_2 = Conv1D(laser_filters2, laser_kernel_size, activation='elu', kernel_regularizer=l2(0.01), name="laser_convolution_2")(laser_dropout)
    laser_pool_2 = MaxPooling1D(name="laser_pool_2")(laser_convolution_2)
    laser_flatten = Flatten(name="laser_flatten")(laser_pool_2)

    ## Defining image model with functional api

    img_filters = 25
    img_filters2 = 35
    img_kernel_size = (5, 5)

    img_convolution_1 = Conv2D(img_filters, img_kernel_size, activation='relu', input_shape = (80, 160, 1), kernel_regularizer=l2(0.005), name="img_convolution_1")(image_input) #32 filtteria
    img_pooling_1 = MaxPooling2D((2,2), strides = 2, name="img_pooling_1")(img_convolution_1)
    img_convolution_2 = Conv2D(img_filters, img_kernel_size, activation='relu', kernel_regularizer=l2(0.005), name="img_convolution_2")(img_pooling_1) #32 filtteria
    img_pooling_2 = MaxPooling2D((2,2), strides = 2, name="img_pooling_2")(img_convolution_2)
    img_convolution_3 = Conv2D(img_filters2, img_kernel_size, activation='relu', kernel_regularizer=l2(0.005), name="img_convolution_3")(img_pooling_2) #64 filtteria
    img_pooling_3 = MaxPooling2D((2,2), strides = 2, name="img_pooling_3")(img_convolution_3)
    img_flatten = Flatten(name="img_flatten")(img_pooling_3)

    merged = concatenate([img_flatten, laser_flatten])

    x = Dense(150, activation='elu', name='dense_1')(merged)
    main_output = Dense(classes, activation='softmax', name='dodging_prediction')(x)

    model = Model(inputs=[image_input, laser_input], outputs=[main_output])

    return model

def model_laser_noConvolution_definition():
    laser_input = Input(shape=(1081,), dtype='float32', name='laser_input')
    dense1 = Dense(200, activation='elu', name='dense1', kernel_regularizer=l2(0.01))(laser_input)
    dense1 = Dense(200, activation='elu', name='hidden1', kernel_regularizer=l2(0.01))(dense1)
    dense1 = Dense(200, activation='elu', name='hidden2', kernel_regularizer=l2(0.01))(dense1)
    #flatten1 = Flatten(name='flatten1')(dense1)
    output = Dense(classes, activation='softmax', name='output_layer')(dense1)

    model = Model(inputs=[laser_input], outputs=[output])

    return model

def model_usonic_definition():
    usonic_input = Input(shape=(8,), dtype='float32', name='usonic_input')
    dense1 = Dense(10, activation='relu', name='dense1')(usonic_input)
    dense2 = Dense(20, activation='relu', kernel_regularizer=l2(0.001), name='dense2')(dense1)
    output = Dense(classes, activation='softmax', name='output_layer')(dense2)

    model = Model(inputs=[usonic_input], outputs=[output])
    return  model
