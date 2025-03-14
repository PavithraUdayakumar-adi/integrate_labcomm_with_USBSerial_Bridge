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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <Arduino.h>

#include "Linduino.h"
#include "demoboard.h"
#include "LT_I2C_Wire.h"
#include "Comm_SMBus.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define COMM_SMBUS_CRC_ERROR  -1

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//LTC4162_LION chip;
const byte PROGMEM crc8_ccitt_table[] = {
  0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
  0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
  0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
  0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
  0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
  0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
  0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
  0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
  0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
  0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
  0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
  0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
  0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
  0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
  0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
  0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
uint8_t comm_smbus_crc8_calc(uint8_t *data_ptr, int size);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int8_t Comm_Smbus_Init(void)
{
  int8_t result = 0;
  i2c_enable(COMM_SMBUS_BAUD);

  return result;
}

// SMBus Read Word with PEC
// TODO: Create a version that returns the PEC that was received, to aid with diagnostics from the PC side.
int Comm_Smbus_Reg16_Read_PEC(uint8_t addr7,                            //!< IC's SMBus/i2c 7-bit address.
                              uint8_t command_code,                     //!< SMBus command code (aka subcommand)
                              uint16_t *data_ptr,                       //!< Memory location to store whole register data read from SMBus/i2c port.
                              port_configuration_t *port_configuration  //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                             )
{
  UNUSED(port_configuration);
  uint8_t value[3];
  byte i2c_fail;
  
  digitalWrite(LED_AMBER, HIGH);
  i2c_enable(COMM_SMBUS_BAUD);
  i2c_fail = i2c_read_block_data(addr7, command_code, sizeof(value), value);
  if(i2c_fail == 0)
  {
    uint8_t bytes_to_crc[] = {(uint8_t)(addr7 << 1), command_code,
                              (uint8_t)((addr7 << 1) | 1), value[0], value[1]};
    uint8_t crc_calc = comm_smbus_crc8_calc(bytes_to_crc, sizeof(bytes_to_crc));
    if(crc_calc == value[2])
    {
      LOWER_BYTE(*data_ptr) = value[0];
      UPPER_BYTE(*data_ptr) = value[1];
    }
    else
    {
      i2c_fail = -1;
    }
  }
  digitalWrite(LED_AMBER, LOW);
  return i2c_fail;
}

// SMBus Write Word with PEC. Returns 0 on success and a non-0 error code on failure.
int Comm_Smbus_Reg16_Write_PEC(uint8_t addr7,                           //!< 7-bit destination i2c address to write to.
                               uint8_t command_code,                    //!< SMBus command code to write to.
                               uint16_t data,                           //!< Whole register data to be written to SMBus/i2c port, little endian byte order.
                               port_configuration_t *port_configuration //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                              )
{
  UNUSED(port_configuration);
  byte i2c_fail;

  i2c_enable(COMM_SMBUS_BAUD);
  uint8_t bytes_to_crc[] = {(uint8_t)(addr7 << 1), command_code, LOWER_BYTE(data), UPPER_BYTE(data), 0x00};
  bytes_to_crc[sizeof(bytes_to_crc)-1] = comm_smbus_crc8_calc(bytes_to_crc, sizeof(bytes_to_crc) - 1);
  i2c_fail = i2c_write_block_data(addr7, command_code, sizeof(bytes_to_crc) - sizeof(addr7) - sizeof(command_code), &bytes_to_crc[sizeof(addr7) + sizeof(command_code)]);
  return i2c_fail;
}
// Version of the above that accepts a PEC code as an argument and transmits it -- useful for testing the IC's PEC checker.
int Comm_Smbus_Reg16_Write_PEC(uint8_t addr7,                           //!< 7-bit destination i2c address to write to.
                               uint8_t command_code,                    //!< SMBus command code to write to.
                               uint16_t data,                           //!< Whole register data to be written to SMBus/i2c port, little-endian byte order.
                               uint8_t pec,                             //!< Allows specifying the PEC to use.
                               port_configuration_t *port_configuration //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
                              )
{
  UNUSED(port_configuration);
  byte i2c_fail;

  i2c_enable(COMM_SMBUS_BAUD);
  uint8_t payload_bytes[] = {LOWER_BYTE(data), UPPER_BYTE(data), pec};
  i2c_fail = i2c_write_block_data(addr7, command_code, sizeof(payload_bytes) - sizeof(pec), payload_bytes);
  return i2c_fail;
}

// SMBus Read Byte, no PEC:
int Comm_Smbus_Reg8_Read(uint8_t addr7,                            //!< IC's SMBus/i2c address.
                         uint8_t command_code,                     //!< SMBus command code (aka subaddress)
                         uint8_t *data_ptr,                        //!< Memory location to store whole register data read from SMBus/i2c port.
                         port_configuration_t *port_configuration  //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
)
{
  UNUSED(port_configuration);
  i2c_enable(COMM_SMBUS_BAUD);
  byte i2c_fail = i2c_read_block_data(addr7, command_code, sizeof(*data_ptr), data_ptr);
  return i2c_fail;
}

// SMBus Write Byte, no PEC:
int Comm_Smbus_Reg8_Write(uint8_t addr7,                            //!< IC's SMBus/i2c address.
                          uint8_t command_code,                     //!< SMBus command code (aka subaddress)
                          uint8_t data,                             //!< The data byte to write.
                          port_configuration_t *port_configuration  //!< Any necessary SMBus/i2c port configuration information, such as a file descriptor or control register location.
)
{
  UNUSED(port_configuration);
  byte i2c_fail;
  i2c_enable(COMM_SMBUS_BAUD);
  i2c_fail = i2c_write_block_data(addr7, command_code, sizeof(data), &data);
  return i2c_fail;
}

uint8_t comm_smbus_crc8_calc(uint8_t *data_ptr, int size)
{
  uint8_t crc = 0;
  uint8_t byte_num;

  for(byte_num = 0; byte_num < size; byte_num++)
  {
    crc = crc8_ccitt_table[crc ^ *data_ptr++];;
  }

  return crc;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=