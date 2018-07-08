/**
 * serial_interface.h
 * 
 * This file holds structures that implement the state-machines needed to
 * communicate via spi, i2c, and uart with external devices using well defined
 * protocols. These protocols guarantee serial communication robustness.
 * 
 * The functionality of the  serial interface devices hosted by the embedded architecture + OS are
 * wrapped in a high-level buffered io interface. This buffered io layer is
 * used by the protocols and is provided as a public interface so it can also 
 * be used outside of this file. Note that this sort of outside usage will
 * negate any guarantees of robustness.
 * 
 * The serial devices, their serial interface, and their purpose that the 
 * structures in this file act as agents for are:
 *   - AVR uController; SPI slave; motor driving / encoder reading / power 
 *     managment board signal reading.
 *   - gen4 HMI Display; UART; duplex gui-SBC data transfer.
 *   - 
 */

#ifndef SERIAL_COMMS_INCLUDED
#define SERIAL_COMMS_INCLUDED

#include "physical_io/serial/I2CDevice.h"
#include "physical_io/serial/SPIDevice.h"
#include "physical_io/serial/UARTDevice.h"



#endif
