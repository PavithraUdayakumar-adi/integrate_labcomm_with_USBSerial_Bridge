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

#ifndef __DEMOBOARD_H__
#define __DEMOBOARD_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <Arduino.h>
#include "Linduino.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define DEMOBOARD_ID_STRING_SIZE          50
#define DEMOBOARD_ID_STRING               "LTC4162,Cls,D4162,01,01,DC,DC2038A,-------------\n" // bobbytemp "LTC4021,Cls,D4021,01,01,DC,DEMOBOARD,-------------\n"
#define DEMOBOARD_LTC_STRING              "USBSPI,ArduinoZero,Labcomm,---------------------\n" // appended programmatically to include board variant (or 'None') and FW revision
#define DEMOBOARD_FW_REV_MAJOR            0
#define DEMOBOARD_FW_REV_MINOR            0
#define DEMOBOARD_FW_REV_SUB_MINOR        0
#define DEMOBOARD_FW_REV_BETA             1

// Pinouts related to names defined in "C:\Users\<username>\AppData\Local\Arduino15\packages\arduino\hardware\samd\1.6.15\variants\arduino_zero\variant.cpp"
#define LED_GREEN   19  // PB2
#define LED_AMBER  27  // PA28

// Debug pins on J3
#define DEBUG1      31  // PB23
#define DEBUG2      30  // PB22
#define DEBUG3      39  // PA21
#define DEBUG4      6   // PA20
#define DEBUG5      12  // PA19
#define DEBUG6      10  // PA18
#define DEBUG7      37  // PA17
#define DEBUG8      35  // PA16
#define DEBUG9      5   // PA15
#define DEBUG10     2   // PA14
#define DEBUG11     38  // PA13
#define DEBUG12     22  // PA12
#define DEBUG13     24  // PB11
#define DEBUG14     23  // PB10
#define DEBUG15     0   // PA11
#define DEBUG16     1   // PA10
#define DEBUG17     3   // PA9
#define DEBUG18     4   // PA8
#define DEBUG19     9   // PA7
#define DEBUG20     8   // PA6
#define DEBUG21     18  // PA5
#define DEBUG22     17  // PA4
#define DEBUG23     16  // PB9
#define DEBUG24     15  // PB8

#define SDA_HOST    32  // PA23
#define SCL_HOST    33  // PA22
#define SMBALERT    43  // PA2

#define IS_SMBALERT_ACTIVE() ((digitalRead(SMBALERT) == LOW ? 1 : 0))

typedef struct
{
  char  id_string[DEMOBOARD_ID_STRING_SIZE];
  uint8_t version[4];
  char  ltc_string[DEMOBOARD_ID_STRING_SIZE];
} ID_STRUCT_TYPE;

// General purpose macros
#define UNUSED(x) (void)(x)
#define FWD_DECLARE extern
#define LENGTH(array)     (sizeof(array)/sizeof(*array))
#define MASK(size, shift)   (((1LL << (size)) - 1) << (shift))
#define USUB_WSAT(a,b) ((a>b) ? a-b : 0)
#define BITS_TO_BYTES(x) ((x + (BITS_PER_BYTE - 1)) >> (NBITS(BITS_PER_BYTE) - 1))
#define MIN(x, y)           (x < y ? x : y)
#define MAX(x, y)           (x > y ? x : y)

// Macros to return the number of bits needed to store a number
#define NBITS2(n)           ((n&2)?1:0)
#define NBITS4(n)           ((n&(0xC))?(2+NBITS2(n>>2)):(NBITS2(n)))
#define NBITS8(n)           ((n&0xF0)?(4+NBITS4(n>>4)):(NBITS4(n)))
#define NBITS16(n)          ((n&0xFF00)?(8+NBITS8(n>>8)):(NBITS8(n)))
#define NBITS32(n)          ((n&0xFFFF0000)?(16+NBITS16(n>>16)):(NBITS16(n)))
#define NBITS(n)            (n==0?0:NBITS32(1L*n)+1)

// Note, these don't work on constants so that I can use them for assignments too.
#define UPPER_BYTE(x) (((LT_union_int16_2bytes*)(&(x)))->LT_byte[MSB])
#define LOWER_BYTE(x) (((LT_union_int16_2bytes*)(&(x)))->LT_byte[LSB])
#define UPPER_WORD(x) (((LT_union_uint32_2uint16s*)(&(x)))->LT_uint16[MSW])
#define LOWER_WORD(x) (((LT_union_uint32_2uint16s*)(&(x)))->LT_uint16[LSW])

// Constants to make the code more readable.
#define US_PER_SEC               1000000
#define MS_PER_SEC               1000
#define SEC_PER_HR               3600
#define BITS_PER_BYTE            8
// #define UINT8_MAX                255

// todo - This needs to be in a more appropriate place, but for now just get it out of the Comm files.
//        The Comm files are a pretty nice place for it to be though, since the idea is that the FW and SW need
//        to have a common set of error codes so that you can get error information from one side to the other.
//        It is also useful for error codes to not change from demo board to demo board, so that new boards can get
//        (at least some) error information to software that existed before the demo board was built.
typedef enum
{
  // Error IDS reserved for all demo boards.
  DEMOBOARD_ERROR_ID_NONE = 0,
  DEMOBOARD_ERROR_ID_EXCEPTION = 1,
  DEMOBOARD_ERROR_ID_COMMAND_NOT_SUPPORTED_SW = 2,
  DEMOBOARD_ERROR_ID_COMMAND_NOT_SUPPORTED_FW = 3,
  DEMOBOARD_ERROR_ID_BAD_DATA_FORMAT = 4,
  DEMOBOARD_ERROR_ID_TIMEOUT = 5,
  DEMOBOARD_ERROR_ID_USB_COMM = 6,
  DEMOBOARD_ERROR_ID_I2C_COMM = 7,
  DEMOBOARD_ERROR_ID_SMBUS_PEC = 8,
  DEMOBOARD_ERROR_ID_STREAM_OVERRUN = 9,

  // Error IDs above this are specific to the demo board.
  DEMOBOARD_ERROR_ID_SPECIFIC = 256,

  DEMOBOARD_ERROR_TODO = 65535 // figure out a more appropriate code later.

} DEMOBOARD_ERROR_ID_TYPE;

#define RECEIVE_BFR_LEN       1024
#define TRANSMIT_BFR_LEN      3072

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
extern const __attribute__((section(".id_data"))) ID_STRUCT_TYPE id_struct;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif

