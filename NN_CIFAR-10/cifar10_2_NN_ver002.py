#!/usr/bin/env python

# More info here:
# https://github.com/ARM-software/CMSIS_5/issues/325
#
import numpy as np
import os

def unpickle(file):
    import pickle
    with open(file, 'rb') as fo:
        dict = pickle.load(fo)
    return dict

# Download and expand CIFAR-10 data if needed
if (os.path.isfile('cifar-10-python.tar.gz') == False):
  os.system("wget https://www.cs.toronto.edu/~kriz/cifar-10-python.tar.gz")
  os.system("tar -xf cifar-10-python.tar.gz")

# First walk-through the data to create mean value
mean_array = np.zeros((32*32*3))

for batch_id in range(1,6) :
  dict = unpickle("cifar-10-batches-py/data_batch_"+str(batch_id))
  mean_array = mean_array +  0.2 * np.mean(dict['data'], axis=0) 

# convert it to uint8
mean_array = np.rint(mean_array)
mean_array = np.clip(mean_array, 0, 255)

# how many images we want to generate
num_of_images = 10

image_count = 0

#now we can output the image data from test batch
dict = unpickle("cifar-10-batches-py/test_batch")
for image_id in range(len(dict['labels'])):
  if (image_count >= num_of_images):
    break
  raw_image = dict['data'][image_id]
  label = dict['labels'][image_id]
  # subtract the image mean
  delta_image = raw_image - mean_array
  # first reshape into CHW format
  reshape_image = np.reshape(delta_image, (3, 32, 32), order='C')

  #
  # (CHW: RR...R, GG..G, BB..B; HWC: RGB, RGB, ... RGB).
  #
  # change shape from CHW to HWC
  reshape_image = np.swapaxes(reshape_image, 0, 1)
  reshape_image = np.swapaxes(reshape_image, 1, 2)

  # saturate and make sure it is of type int8
  reshape_image = np.clip(reshape_image, -128, 127)
  input_image = reshape_image.astype('int8')

  # generate header file
  out_f = open("image_"+str(image_count)+".h", 'w')
  out_f.write("#define IMG_DATA {")
  input_image.tofile(out_f, sep=",", format="%d")
  out_f.write("}\n\n")

  out_f.write("#define TEST_LABEL "+str(label)+"\n\n");
  out_f.close()

  # generate bin file
  out_f = open(str(label)+"_"+str(image_id)+".bin", 'wb')
  bin_input_image = bytearray(input_image)
  out_f.write(bin_input_image)
  out_f.close()

  image_count = image_count + 1
