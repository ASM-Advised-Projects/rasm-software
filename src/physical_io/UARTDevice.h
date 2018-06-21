/**
 * UARTDevice.h
 */

#ifndef UARTDEVICE_INCLUDED
#define UARTDEVICE_INCLUDED

/**
 * UART device class that wraps a UART device file in a minimal interface.
 */
class UARTDevice {
private:
    unsigned int dev;
    int fd;
    int baudrate;
public:
    UARTDevice(unsigned int device, unsigned int baudrate);
    virtual int open();
    virtual int write(char *tx, int length);
    virtual int read(char *rx, int length);
    virtual void close();
    ~UARTDevice();
};

#endif