# Background subtraction

Background subtraction, using approximate median algorithm, with STM32F7 Disco. 

Thanks to Seth Benton. 

More on EETimes article: 

http://www.eetimes.com/document.asp?doc_id=1275604&page_number=1&piddl_msgpage=4#msgs

Decompress the sanf32.7z.001 and sanf32.7z.002 files. 
This file is the entire movie converted as singles RGB file, each one appended to the other, forming a 260MByte unique file.

Put the file on a SDCard. The program will read it.

Script to generate the sanf32.bin file:
mplayer.exe -nosound -vf scale=320:240 -vo png:z=0 -fps 10 "san_fran_traffic_30sec_QVGA_Cinepak.avi"

mogrify -format rgb *.png

cat *.rgb > sanf32.bin

del *.rgb

del *.png
