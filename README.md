# AGLA: SUMP logic digital analyzer for Arduino compatible with PulseView #


This Arduino sketch implements the SUMP protocol compatible with [PulseView](https://sigrok.org/wiki/PulseView). It is based on [Andrew Gillham's implementation](https://github.com/gillham/logic_analyzer), but the acquisition part has been completely redesigned. It uses inline assembler and relies on Timer1 for sampling frequencies between 10 Hz and 1 MHz. For 2 MHz and 5 MHz, special purpose acquisition procedures are used that wait for the trigger condition, but that are not able to store the required amount of data before the trigger point. The 2 MHz acquisition procedure uses an inline assembly function. The 5 MHz acquisition procedure is a rolled out loop, which samples at a rate of 5.3333 MHz -- and corrects this error by inserting a NOP after every fifth sample. In any case, in both procedures (2 and 5 MHz), the sample matching the trigger condition (if any) is stored as the first sample. 

The sketch sends the acquired samples in a backwards manner to the client, which seems to be preferred way for PulseView. If you want to interact with older SUMP clients, you probably have to undefine the compile-time constant REVERSED or you have to edit the CFG files. 

On the UNO compatible boards, the six Arduino pins 8-13 are the input channels and the memory depth is 1.5k of samples. On the Arduino Mega board 8 channels are supported and 7k of samples. Pins 22-29 (Port A) are used by default on the Mega boards. It should be easy to adapt the sketch to other boards with AVR MCUs. 

Recently, I wrote a [blog post](https://hinterm-ziel.de/index.php/2021/10/07/doing-it-the-logical-way/) that explains how to use AGLA (the Arduino Generic Logic Analyzer) and logic analyzers in general.

#### Problem ####
Currently, the sketch seems to work only with PulseView 0.4.2, and you have to press the `scan` button two or three times. The reason behind that is that initially, when connecting, the Arduino is reset and then waits 2 seconds before the sketch is started. So the second or third time, when you press the `scan` button and a connect sequence is sent to the Arduino, it is received and the Arduino replies. With the more recent nightly-built, the initial connect sequence is only sent once even if you press the scan button multiple times, so no connection can be established. A way around it is to disable the auto-reset feature of the UNO. One easy way to do that is to put a 10 µF (or higher) cap between `RESET` and `GND` (plus on `RESET`, minus on `GND`!). Then it also works with the most recent version.

