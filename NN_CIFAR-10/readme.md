# STM32F769I-DISCO

This program implement the Neural Network example, present on the last CMSIS library.
You need to put the "test_batch.bin" [1] file on the SDCard.

The program read randomly 375 images from the first 1000 images on the "test_batch.bin" file, visualize the image and call the cifar program using the content of the binary file extracted from the "cifar10_bin_file.tar" to evaluate the image.

The script "cifar10_2_NN_dump_meandata.py" is modifyed to save the a binary and headr file of the calculated mean. The original script is here [2] with very helpful info about the image format.

The "mean_array_uint8.h" file is the dump of the mean value calculated by the python script. I use it to prepare the image file before to feed the "cifar" function.

"The network is trained using the setup in Caffe where input data is raw RGB image minus data set mean image" [2]

After the last image on screen, the program save a file on SDCard, "test_batch.txt" with the result. If you touch the screem  will appear a simple statistic about images.

The saved file is like that:
``` 
Check:BAD,Image#:5045,Label:horse,Label#:07,Result#:02,ResultVal:117,Time:0.0927;
Check:OK,Image#:1422,Label:ship,Label#:08,Result#:08,ResultVal:127,Time:0.0927;
Check:BAD,Image#:5601,Label:horse,Label#:07,Result#:09,ResultVal:127,Time:0.0927;
```

[1] https://www.cs.toronto.edu/~kriz/cifar.html

[2] https://github.com/ARM-software/CMSIS_5/issues/325
