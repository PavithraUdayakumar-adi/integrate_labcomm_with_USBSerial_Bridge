/*
 Linear Technology DC2343A Demonstration Board.

 @verbatim
 This code implements a simple LT_printf function over the Arduino serial port.
 @endverbatim

 REVISION HISTORY
 $Revision: 4639 $
 $Date: 2016-01-29 16:42:40 -0500 (Fri, 29 Jan 2016) $

 Copyright (c) 2014, Linear Technology Corp.(LTC)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those
 of the authors and should not be interpreted as representing official policies,
 either expressed or implied, of Linear Technology Corp.

*/

#ifndef __COMM_SMBUS_H__
#define __COMM_SMBUS_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <Arduino.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
typedef struct
{
  void *descriptor; //!< not needed for Arduino
} port_configuration_t;

#define COMM_SMBUS_BAUD 400000
#define SMBUS_ALERT_RESPONSE_ADDRESS_7BITS 0b0001100

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//extern LTC4162_LION chip;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int8_t Comm_Smbus_Init(void);

//
// Performs a SMBus Write Word PEC transaction.
//
int Comm_Smbus_Reg16_Write_PEC(uint8_t addr7,                           //!< 7-bit target i2c address
                               uint8_t command_code,                    //!< SMBus command code to write to at target i2c address.
                               uint16_t data,                           //!< The data word to write.
                               port_configuration_t *port_configuration //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                              );
// Version of the above that accepts a PEC code as an argument and transmits it -- useful for testing the IC's PEC checker.
int Comm_Smbus_Reg16_Write_PEC(uint8_t addr7,                           //!< 7-bit target i2c address.
                               uint8_t command_code,                    //!< SMBus command code to write to at target i2c address.
                               uint16_t data,                           //!< The data word to write.
                               uint8_t pec,                             //!< Allows specifying the PEC to use.
                               port_configuration_t *port_configuration //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                              );

//
// Performs a SMBus Read Word PEC transaction.
//
int Comm_Smbus_Reg16_Read_PEC(uint8_t addr7,                            //!< 7-bit target i2c address
                              uint8_t command_code,                     //!< SMBus command code to read from at i2c address.
                              uint16_t *data_ptr,                       //!< Memory location to store the data read.
                              port_configuration_t *port_configuration  //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                             );

//
// Performs a SMBus Read Byte transaction without using PEC.
//
int Comm_Smbus_Reg8_Read(uint8_t addr7,                            //!< 7-bit target i2c address
                         uint8_t command_code,                     //!< SMBus command code to read from at i2c address.
                         uint8_t *data_ptr,                        //!< Memory location to store whole register data read from SMBus/i2c port.
                         port_configuration_t *port_configuration  //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                        );

//
// Performs a SMBus Write Byte transaction without using PEC.
//
int Comm_Smbus_Reg8_Write(uint8_t addr7,                            //!< 7-bit target i2c address
                          uint8_t command_code,                     //!< SMBus command code to write to at i2c address.
                          uint8_t data,                             //!< The data byte to write.
                          port_configuration_t *port_configuration  //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                         );

/*  Computes and returns the SMBus PEC on an array of bytes. */
uint8_t comm_smbus_crc8_calc(uint8_t *data_ptr, int size);
#endif
