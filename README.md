# LIN2LINrepeaterMEGA

like LIN2LIN, but uses a MEGA for MO UARTzzz


takes openpilot panda LIN data, buffers it, repackages it, and sends it on honda accord serial steering K-LIN

**below taking from comments in the code

 all input pins are INPUT_PULLUP (pulled up internallY) and active low (except rotary pot input)

7: input- Force LKAS ON.<br>
10: output- softwareserial Tx (inverted) driving transistor to pull down Klin .<br>
12: input- set Rotary POT current pos as center.<br>
13: ouput- used for LED when LKAS is on.<br>
A0: input- Force Rotary as INPUT for apply_steer.<br>
A3: input- enable input of raw MCU LIN stream on RX (pin 1) buffered, and sent on pin 10 (this allows for rotary to override signal) (note: when this is set, lkas_off data will not be sent unless it comes across the LIN in)
A4: input- send sendDebug on Serial.<br>
A5: rotary pot input.<br><br>

OP to LIN2LIN data structure (it still sends 4 bytes, but the last 2 are 0x00).

i will prolly change that later to only 2 bytes, but this code will not need to be changed<br>
byte1: 01A0####    #### = is big_steer   A = Active   first 2 bits is the byte counter<br>
byte2: 10A#####    ##### = is little steer
