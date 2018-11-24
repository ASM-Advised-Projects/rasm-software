'''
This program runs two concurrent character pipes (one for input and one for 
output) to the specified serial device at a specified baudrate. The serial
device file path is given as the first command line argument while the baudrate
is given as the second command line argument.

Example usage:
$ python SerialPort.py /dev/cu.usbmodem1421 115200
'''

import sys
import serial
import threading

serial_port = None
closing = False

def run_outgoing_pipe():
    try:
        while True:
            data = sys.stdin.readline()
            data = data[:len(data)-1]
            if data == 'q':  # exit program
                break
            
            serial_port.write(data)
            serial_port.flush()
    except:
        pass

def run_incoming_pipe():
    try:
        while True:
            data = serial_port.read(size=1)
            if closing:
                break
            if len(data) < 1:  # read timeout
                continue
            
            sys.stdout.write(data + '\n')
            sys.stdout.flush()
    except:
        pass

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Exactly two command line arguments are required.")

    serial_port = serial.Serial(sys.argv[1], int(sys.argv[2]))
    serial_port.timeout = 0.1;

    outpipe_thread = threading.Thread(target=run_outgoing_pipe)
    inpipe_thread = threading.Thread(target=run_incoming_pipe)

    outpipe_thread.start()
    inpipe_thread.start()

    outpipe_thread.join()
    closing = True
    inpipe_thread.join()
    serial_port.close()
