# STM32F769I-DISCO

This program implement the Neural Network example, present on the last CMSIS library.
You need to put the "test_batch.bin" [1] file on the SDCard.

The program read randomly 375 images from the file, visualize the image and call the cifar program to evaluate the image.
After the last image on screen, the program save a file on SDCard, "test_batch.txt" with the result. If you touch the screem  will appear a simple statistic about images.

The saved file is like that:
``` 
Check:BAD,Image#:5045,Label:horse,Label#:07,Result#:02,ResultVal:117,Time:0.0927;
Check:OK,Image#:1422,Label:ship,Label#:08,Result#:08,ResultVal:127,Time:0.0927;
Check:BAD,Image#:5601,Label:horse,Label#:07,Result#:09,ResultVal:127,Time:0.0927;
```

UPDATE:
I have uploaded the BMP image equivalent to the array data used by the example. The colors are wrong.

[1] https://www.cs.toronto.edu/~kriz/cifar.html
