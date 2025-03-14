
/*!
LT_I2C: Routines to communicate with ATmega328P's hardware I2C port.

REVISION HISTORY
$Revision: 6144 $
$Date: 2016-12-05 19:00:27 -0500 (Mon, 05 Dec 2016) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
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

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

/*! @file
    @ingroup LT_I2C
    Library Header File for LT_I2C: Routines to communicate with ATmega328P's hardware I2C port.
*/

#ifndef LT_I2C_H
#define LT_I2C_H

//! i2c_enable or quikeval_I2C_init must be called before using any of the other I2C routines.
void i2c_enable(uint32_t clockFrequency);

//! Write a block of data, starting at register specified by "command" and ending at (command + length - 1)
//! @return 0 on success, 1 on failure
int8_t i2c_write_block_data(uint8_t address,    //!< 7-bit I2C address
                            uint8_t command,    //!< Command byte
                            uint8_t length,     //!< Length of array
                            uint8_t *values     //!< Byte array to be written
                           );

//! Read a block of data, starting at register specified by "command" and ending at (command + length - 1)
//! @return 0 on success, 1 on failure
int8_t i2c_read_block_data(uint8_t address,     //!< 7-bit I2C address
                           uint8_t command,     //!< Command byte
                           uint8_t length,      //!< Length of array
                           uint8_t *values      //!< Byte array to be read
                          );

//! Read a block of data, no command byte, reads length number of bytes and stores it in values.
//! @return 0 on success, 1 on failure
int8_t i2c_read_block_data(uint8_t address,     //!< 7-bit I2C address
                           uint8_t length,      //!< Length of array
                           uint8_t *values      //!< Byte array to be read
                          );

#endif  // LT_I2C_H
