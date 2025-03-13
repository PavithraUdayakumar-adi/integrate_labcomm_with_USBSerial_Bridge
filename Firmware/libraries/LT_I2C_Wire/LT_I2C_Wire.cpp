/*!
LT_I2C_Wire: Routines to communicate with M0's hardware I2C port via Arduino Wire interface.

@verbatim
https://www.arduino.cc/en/Reference/Wire
@endverbatim

REVISION HISTORY
$Revision: 6144 $
$Date: 2016-12-05 19:00:27 -0500 (Mon, 05 Dec 2016) $
*/

#include "ADI_Wire.h"
#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_I2C_Wire.h"

// Setup the hardware I2C interface.
void i2c_enable(uint32_t clockFrequency)
{
  Wire.begin();
  Wire.setClock(clockFrequency);
}

// Write a block of data at register specified by "command."
int8_t i2c_write_block_data(uint8_t address, uint8_t command, uint8_t length, uint8_t *values)
{
  byte retval = 0;

  Wire.beginTransmission(address);  // transmit to device
  Wire.write(command);              // sends instruction byte
  Wire.write(values, length);       // send values
  Wire.endTransmission();           // stop transmitting
  return retval;
}

// Read a block of data at register specified by "command."
int8_t i2c_read_block_data(uint8_t address, uint8_t command, uint8_t length, uint8_t *values)
{
  byte retval = 0;

  Wire.beginTransmission(address);
  Wire.write(command);
  retval = Wire.endTransmission(false);
  if(0 == retval) retval = i2c_read_block_data(address, length, values);
  return retval;
}

// Read a block of data.
int8_t i2c_read_block_data(uint8_t address, uint8_t length, uint8_t *values)
{
  byte retval = 0;

  Wire.beginTransmission(address);
  retval = (length != Wire.requestFrom(address, length, true));
  if(0 == retval)
  {
    uint8_t i = 0;
    while (Wire.available() && (i < length))
    {
      values[i++] = Wire.read();
    }
  }
  return retval;
}
