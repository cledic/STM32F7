# STM32F769I-DISCO

This program use the example of use of Neural Network present on the last CMSIS library.
You need to put the "test_batch.bin" [1] file on the SDCard.

The program read randomly 375 images from the file, visualize the image and call the cifar program to evaluate the image.
After the last image on screen, the program save a file on SDCard, "test_batch.txt" with the result. If you touch the screem  will appear a simple statistic about images.

UPDATE:
I have uploaded the BMP image equivalent to the array data used by the example. The colors are wrong.

[1] https://www.cs.toronto.edu/~kriz/cifar.html
