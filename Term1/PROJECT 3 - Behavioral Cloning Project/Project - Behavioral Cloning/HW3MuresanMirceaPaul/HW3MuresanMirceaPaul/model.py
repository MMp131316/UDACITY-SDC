#import necessary libs
import pandas as pd
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
#%matplotlib inline

from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.layers import Convolution2D, MaxPooling2D
from keras.optimizers import SGD, Adam, RMSprop
from keras.utils import np_utils

from sklearn.model_selection import train_test_split
from keras.callbacks import ModelCheckpoint, EarlyStopping
from keras.layers.normalization import BatchNormalization

print("loaded")

#import the data
driving_csv = pd.read_csv("driving_log.csv")

# Examine data
print("Number of datapoints: %d" % len(driving_csv))

# Extract centre image and steering angle from table
X_path = [driving_csv.loc[i][0] for i in range(len(driving_csv))]
y = [driving_csv.loc[i][3] for i in range(len(driving_csv))]

# import the images
X_images = [mpimg.imread(image_path) for image_path in X_path]

# View random image
print("Images: %d" % len(X_images))
r = np.random.randint(len(X_images)) 
#plt.title("Steering angle: {0}".format(driving_csv.loc[r][3]))
#plt.imshow(X_images[r])
#save an image for the report
#plt.imsave("C:\\center.bmp",X_images[r])
print("Image shape: ", X_images[0].shape)

#Function for pre-processing an image
def preprocess_image(image_array, output_shape=(33, 100), colorspace='yuv'):

    # convert image to another colorspace
    if colorspace == 'yuv':
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2YUV)
    elif colorspace == 'hsv':
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2HSV)
    elif colorspace == 'hls':
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2HLS)
    elif colorspace == 'rgb':
        image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
    
    # crops top and bottom portion
    # The entire width of the image is preserved
    image_array = image_array[51:150, 10:310]
    #Blur the image to filter out artefacts
    kernel_size = 5  # Must be an odd number (3, 5, 7...)
    image_array = cv2.GaussianBlur(image_array, (kernel_size, kernel_size), 0)
    # resize image to output_shape
    image_array = cv2.resize(image_array, (output_shape[1], output_shape[0]), interpolation=cv2.INTER_AREA)
    return image_array

#store in X_images2 the pre-processed images
X_images2 = [preprocess_image(img, colorspace='yuv') for img in X_images] 
#generate a random number of the length of X_images2
r = np.random.randint(len(X_images2)) 
#plt.title("Steering angle: {0}".format(driving_csv.loc[r][3]))
#show the pre-processed image to see if function works correctly
#plt.imshow(X_images2[r]) 
print("Images 2 shape: ", X_images2[0].shape)
# steering angles histogram
#plt.hist(y)
#plt.title("Steering angles")
#plt.xlabel("Value")
#plt.ylabel("Frequency")
#split data-set into train and test set, where test set is 10% of the original set
X_train, X_test, y_train, y_test = train_test_split(X_images2, y, test_size=0.1, random_state=1)

X_train = np.array(X_train)
X_test = np.array(X_test)

y_train = np.array(y_train)
y_test = np.array(y_test)

mirror = True
# Create mirror images
mirror = np.ndarray(shape=(X_train.shape))
count = 0
#iterate through all images and flip them
for i in range(len(X_train)):
	mirror[count] = np.fliplr(X_train[i])
	count += 1
mirror.shape

# Create mirror image labels
mirror_angles = y_train * -1
r = np.random.randint(len(mirror)) 
#plt.imshow(mirror[r]) 
#we save an image and its mirror 
#plt.imsave("C:\\mirror_center.bmp",mirror[r])
#plt.imsave("C:\\orig_center.bmp",X_train[r])
# concatenate original training sets with mirror set
X_train = np.concatenate((X_train, mirror), axis=0)
y_train = np.concatenate((y_train, mirror_angles),axis=0)

#create validation set, this set is 10% of the training dataset
X_train, X_valid, y_train, y_valid = train_test_split(X_train, y_train, test_size=0.1, random_state=0)

X_train = np.array(X_train)
X_test = np.array(X_test)

y_train = np.array(y_train)
y_test = np.array(y_test)

X_valid = np.array(X_valid)
y_valid = np.array(y_valid)

#show sets dimensions
print("X_train shape: ", X_train.shape)
print("X_test shape: ", X_test.shape)
print("y_train shape: ", y_train.shape)
print("y_test shape: ", y_test.shape)
print("X_valid shape: ", X_valid.shape)
print("y_valid shape: ", y_valid.shape)

width = X_train[0].shape[1]
height = X_train[0].shape[0]
channels = X_train[0].shape[2]
print("height: ", height)
print("width: ", width)

#Test proposed model. The model was inspired by nVidia paper and coma.ai models
#The first layer is batch normalization, then five layers(as in nvidia paper) then 3 fully connected layers(of dimensions similar to coma.ai) with dropout
model = Sequential()
model.add(BatchNormalization(axis=1, input_shape=(height,width,channels)))
model.add(Convolution2D(16, 3, 3, border_mode='valid', subsample=(2,2), activation='relu'))
model.add(Convolution2D(24, 3, 3, border_mode='valid', subsample=(1,2), activation='relu'))
model.add(Convolution2D(36, 3, 3, border_mode='valid', activation='relu'))
model.add(Convolution2D(48, 2, 2, border_mode='valid', activation='relu'))
model.add(Convolution2D(48, 2, 2, border_mode='valid', activation='relu'))
model.add(Flatten())
model.add(Dense(512))
model.add(Dropout(.5))
model.add(Activation('relu'))
model.add(Dense(10))
model.add(Activation('relu'))
model.add(Dense(1))
#this line prints the model structure
model.summary()

#The model is compiled using the adam optimizer
#the learning rate set is of 0.0001
#the loss function is Mean Square Error
adam = Adam(lr=0.0001)
model.compile(loss='mse',
              optimizer=adam)

# Model will save the weights whenever validation loss improves
checkpoint = ModelCheckpoint(filepath = 'model.h5', verbose = 1, save_best_only=True, monitor='val_loss')

# Discontinue training when validation loss fails to decrease
callback = EarlyStopping(monitor='val_loss', patience=2, verbose=1)

# Train model for 10 epochs and a batch size of 128
model.fit(X_train,
        y_train,
        nb_epoch=10,
        verbose=1,
        batch_size=128,
        shuffle=True,
        validation_data=(X_valid, y_valid),
        callbacks=[checkpoint, callback]
        )
# We can also save the model architecture using the commands shown bellow
json_string = model.to_json()
with open("model.json", "w") as f:
    f.write(json_string)  
print("Model Saved")
