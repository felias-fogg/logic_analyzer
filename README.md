## AGLA: SUMP logic digital analyzer for Arduino compatible with PulseView
=====

This Arduino sketch implements the SUMP protocol compatible with [PulseView](https://sigrok.org/wiki/PulseView). It is based on [Andrew Gillham's implementation](https://github.com/gillham/logic_analyzer), but the acquisition part has been completely redesigned. It uses inline assembler and relies on Timer1 for sampling frequencies between 10 Hz and 1 MHz. For 2 MHz and 5 MHz, special purpose acquisition procedures are used that wait for the trigger condition, but that are not able to store the required amount of data before the trigger point. The 2 MHz acquisition procedure uses an inline assembly function. The 5 MHz acquisition procedure is a rolled out loop, which samples at a rate of 5.3333 MHz -- and corrects this error by inserting a NOP after every fifth sample. In any case, in both procedures (2 and 5 MHz), the sample matching the trigger condition (if any) is stored as the first sample (which was not the case in Andrew's implementation). 

The sketch sends the acquired samples in a backwards manner to the client, which seems to be preferred way for PulseView. If you want to interact with older SUMP clients, you probably have to undefine the compile-time constant REVERSED or you have to edit the CFG files. 

On the UNO compatible boards, the six Arduino pins 8-13 are the input channels. On the Arduino Mega board 8 channels are supported and 7k of samples. Pins 22-29 (Port A) are used by default on the Mega boards. It should be trivial to adapt the sketch to other boards with AVR MCUs. 


