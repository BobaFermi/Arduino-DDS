# Arduino-DDS

This code was written with Arduino Due in mind, for Analog Devices AD9958 or AD9959 DDS chips. The Due is connected to an Ethernet shield, LCD screen, the DDS breakout board, and several BNC connectors are used as digital inputs and outputs.

There are many variations on breakout boards and controller boards for the Analog Devices AD9958 and AD9959 Direct Digital Synthesiser (DDS) chips. This Arduino code was written for consistency in controllers, to allow the control of the DDS chip with a series of TTL signals and to save money on controller boards (breakout boards with controller boards seem to be more expensive than buying breakout boards and Arduinos. The DDS digital pins work on 3.3 V logic, which can either be used with an Arduino Due, or basically any other Arduino if the 5 V regulator is switched out for a 3.3 V one. This code was written for an Arduino Due.

The code has three modes of operation - listening mode, interpreter mode and execution mode. Because the Ethernet shield and DDS both communicate via SPI, they share clock and data pins on the Due so can not be used simultaneously. Their Chip Select pins differ, so we can select which device listens at what times. 
The idea is that all the commands expected to be used during an experimental sequence are sent before the sequence begins, and a series of digital signals are used during the sequence to execute each of the commands.

Listening mode - Activated using a digital pin. The command queue is cleared if it was populated before. Ethernet port is active, and the Arduino listens for UDP packets sent to its IP address. The packets are only accepted if they are of the minimum length.

Interpreter mode - Ethernet port is deactivated, DDS CS pin is activated. The UDP strings are interpreted. The expected format is "a#b#c.x#d.y#e.z@", where:

a - mode of operation (single frequency or linear frequency sweep?)

b - output channel of DDS (CH1 or CH2 in AD9958, CH1, CH2, CH3 or CH4 in AD9959)

c.x - start frequency of linear sweep, or just the desired constant frequency 

d.y - end frequency for linear sweep (required to be in UDP string, but ignored if signle frequency mode selected)

e.z - duration for linear sweep (required to be in UDP string, but ignored if signle frequency mode selected)

The # symbols separate the parameters of a command and the @ symbols separate different commands, so the Arduino can tell the difference between each of them. For each parameter the Due extracts, it converts it from string to int (a, b) or float (c.x, d.y, e.z). The frequencies and duration need to be converted to values related to the clock rate and phase accumulator range of the DDS chip before being transmitted. An array is populated with all the final values ready to be sent to the DDS.

Execution mode - Two digital pins are used to either execute the first command on the array before deleting it, or just to delete it and move on to the next command (in rare cases). 
