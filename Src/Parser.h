/***********************************************************************************
* Parser.h
*
* Implementation the BMP280
* See: http://www.bosch.com
*
* Date			Author          Notes
* 30/01/2017	Stefan Meyre    Initial release
 *
***********************************************************************************/

#ifndef Parser_H
#define Parser_H

//    I2C ADDRESS/BITS/SETTINGS
//    -----------------------------------------------------------------------

#define BMP280_ADDRESS                (0x76 << 1)
#define BMP280_CHIPID                 0x58

//    REGISTERS
//    -----------------------------------------------------------------------

#define     BMP280_REGISTER_STARTDATA           0xF7


#endif



volatile uint8_t rx_bytes_counter;
volatile uint8_t command_ready;
volatile uint8_t data_ready;
uint8_t rx_bytes[10];


void parseFlightMonitorData(void);
