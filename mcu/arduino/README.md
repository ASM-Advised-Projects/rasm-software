This directory holds all Arduino related programs and libraries for running the Arduino board within the 'plan B' electrical system. There is also the SerialPort.py file which runs a minimal program for sending and recieving character data from the arduino when it's running RasmCommand.ino.

The RasmCommand.ino file defines the program to fulfill the required serial interface needed by the UC_Board class of the periphery subsystem. See the RasmCommand.ino file for more comments.

Note that both the Filter and Rasm libraries have an external dependency of the StandardCplusplus Arduino library.