/*

 LabComm M0+

REVISION HISTORY
$Revision: 2682 $
$Date: 2017-04-27 11:02:17 -0400 (Thu, 27 Apr 2017) $

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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <Arduino.h>
#include <stdint.h>
#include <delay.h>

#include "Linduino.h"
#include "demoboard.h"
#include "Labcomm_Packet_Parser.h"
#include "Labcomm_dispatcher.h"
#include "Comm_SMBus.h"
#include "fifo_buffer.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// This places a copy of the ID information at a specific place in memory, if the .id_data section is defined with load flags.
// This is most easily done by modifying \hardware\arduino\avr\platform.txt in the Arduino directory, but isn't necessary for the sketch
// to operate.  It just makes it easy for the demoboard GUI to determine what version hex file it's packaged with.
const __attribute__((section(".id_data"))) ID_STRUCT_TYPE id_struct =
{
    DEMOBOARD_ID_STRING,
    {DEMOBOARD_FW_REV_MAJOR, DEMOBOARD_FW_REV_MINOR, DEMOBOARD_FW_REV_SUB_MINOR, DEMOBOARD_FW_REV_BETA},
    DEMOBOARD_LTC_STRING
};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static const uint8_t pins[] = {LED_AMBER, LED_GREEN, DEBUG1, DEBUG2, DEBUG3, DEBUG4, DEBUG5, DEBUG6, DEBUG7, DEBUG8, DEBUG9, DEBUG10, DEBUG11, DEBUG12,
                               DEBUG13, DEBUG14, DEBUG15, DEBUG16, DEBUG17, DEBUG18, DEBUG19, DEBUG20, DEBUG21, DEBUG22, DEBUG23, DEBUG24};

static uint16_t ltc_update_time_next;

static t_fifo_buffer receive_fifo;
static t_fifo_buffer transmit_fifo;
static uint8_t receive_fifo_buffer[RECEIVE_BFR_LEN];
static uint8_t transmit_fifo_buffer[TRANSMIT_BFR_LEN];

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void demoboard_flash_status_led(void);
static void demoboard_handle_comms(void);
static void amber_led_indicates_smbalert(void);
void debugFifoContents();
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
// Setup the program
{
  for(uint8_t pin_num = 0; pin_num < sizeof(pins); pin_num++)
  {
    digitalWrite(pins[pin_num], LOW);
    pinMode(pins[pin_num], OUTPUT);
  }

  SerialUSB.begin(115200);          // enable the serial port for 115200 baud
  SerialUSB.flush();

  fifoBuf_init(&receive_fifo, receive_fifo_buffer, RECEIVE_BFR_LEN);
  fifoBuf_init(&transmit_fifo, transmit_fifo_buffer, TRANSMIT_BFR_LEN);
  Comm_GUI_Init();
  Labcomm_dispatcher_init();
  Comm_Smbus_Init();
}

void loop()
{
  // Get time since last loop execution.
  int16_t ltc_update_time_diff = ltc_update_time_next - (uint16_t) millis();

  // Flash led to indicate state.
  demoboard_flash_status_led();
  amber_led_indicates_smbalert();

  // Handle serial communication. All Labcomm packet processing and dispatch is here.
  demoboard_handle_comms();
  //debugFifoContents();
  return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void debugFifoContents() {
    SerialUSB.println("---- Debug FIFO Contents ----");

    // Debug receive_fifo
    SerialUSB.print("Receive FIFO: ");
    uint16_t receive_fifo_used = (receive_fifo.wr + receive_fifo.buf_size - receive_fifo.rd) % receive_fifo.buf_size;
    for (uint16_t i = 0; i < receive_fifo_used; i++) {
        uint8_t data = receive_fifo.buf_ptr[(receive_fifo.rd + i) % receive_fifo.buf_size];
        SerialUSB.print(data, HEX); // Print data in HEX
        SerialUSB.print(" ");
    }
    SerialUSB.println();

    // Debug transmit_fifo
    SerialUSB.print("Transmit FIFO: ");
    uint16_t transmit_fifo_used = (transmit_fifo.wr + transmit_fifo.buf_size - transmit_fifo.rd) % transmit_fifo.buf_size;
    for (uint16_t i = 0; i < transmit_fifo_used; i++) {
        uint8_t data = transmit_fifo.buf_ptr[(transmit_fifo.rd + i) % transmit_fifo.buf_size];
        SerialUSB.print(data, HEX); // Print data in HEX
        SerialUSB.print(" ");
    }
    SerialUSB.println();

    SerialUSB.println("------------------------------");
}


static void amber_led_indicates_smbalert(void)
{
  // Light the AMBER LED if and only if SMBALERT# is asserted.
  SerialUSB.println("SMBUS ALERT------------AMBER LED is ON");
  const unsigned int AMBER_LED_PIN_NUMBER = 43;
  static int led_state = -1;
  int next_led_state;
  if (!digitalRead(AMBER_LED_PIN_NUMBER)) {
    // SMBALERT# asserted (active low)
    next_led_state = 1;
  } else {
    next_led_state = 0;
  }
  if (next_led_state != led_state) {
    digitalWrite(AMBER_LED_PIN_NUMBER, next_led_state);
    led_state = next_led_state;
  }
}

static void demoboard_flash_status_led(void)
{
  static unsigned long led_last_time = 0;
  static boolean led_toggle = false;
  static int     phase = 0;
  const uint16_t delay_ms[] = { 50, 150, 50, 150, 50, 150, 50, 150,
                                100, 300, 100, 300, 100, 300, 100, 300,
                                200, 600, 200, 600, 200, 600,
                                400, 800, 400, 800,
                                200, 600, 200, 600, 200, 600,
                                100, 300, 100, 300, 100, 300, 100, 300,
                                50, 150, 50, 150, 50, 150, 50, 150 };

  // Time to flip LED state?
  if(millis() - led_last_time > delay_ms[phase])
  {
    // toggle the led.
    digitalWrite(LED_GREEN, led_toggle);
    led_last_time = millis();
    phase = (phase + 1) % (sizeof(delay_ms) / sizeof(delay_ms[0]));
    led_toggle = !led_toggle;
  }

  return;
}
// static constexpr int RX_MESSAGE_BUFFER_SIZE = 32;
// char *rx_buffer[RX_MESSAGE_BUFFER_SIZE];
// static constexpr int RX_MESSAGE_MAXIMUM_LENGTH = 512;
// 	int rx_buffer_index =0;
// 	unsigned int rx_message_index=0;
// 	int rx_message_start_index=0;

static void demoboard_handle_comms(void)
{
  // while (SerialUSB.available() > 0 && fifoBuf_getFree(&receive_fifo) > 0) {
  //   // todo - old comment
  //   // yes this is slow, but the arduino cant do anything faster anyway
  //   // for better hw it would be better to copy a group at once...
  //   fifoBuf_putByte(&receive_fifo, SerialUSB.read());
  // }
  SerialUSB.println("-----------------------------------------------Start -------------------------------------------------");
                                //len=14                                                                      //dev_addr
  //uint8_t simulated_data[] = {0x0E, 0x4E, 0x4C, 0x54, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x01, 0x00, 0x18, 0x3B, 0x31, 0xFB};
  uint8_t simulated_data[] = {0x10, 0x4E, 0x4C, 0x54, 0x00, 0x00, 0x00, 0x20, 0x00, 0x06, 0x01, 0x01, 0x18, 0x0D, 0x00, 0x07, 0x28, 0x3D};
  //uint8_t simulated_data[] = {0x0E, 0x4E, 0x4C, 0x54, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x01, 0x00, 0x18, 0x01, 0x22, 0x7B};
  uint8_t frame_size = sizeof(simulated_data);

    for (uint8_t i = 0; i < frame_size; i++) {
      if (fifoBuf_getFree(&receive_fifo) > 0) {
        fifoBuf_putByte(&receive_fifo, simulated_data[i]);
      }
    }
    // //Populate CommMessage
    // if (fifoBuf_getUsed(&receive_fifo) > 0) {
    //   uint8_t message_length = fifoBuf_getByte(&receive_fifo);  
    //   SerialUSB.println("mess length=");SerialUSB.print(message_length,HEX);
        
    //   if (fifoBuf_getUsed(&receive_fifo) >= message_length) {  
    //         uint8_t message[message_length];
    //         fifoBuf_getData(&receive_fifo, message, message_length);
            
    //          CommMessage *newMessage = new CommMessage();
    //         // newMessage->message = (char*)malloc(message_length);
    //         // memcpy(newMessage->message, message, message_length);
    //         // newMessage->message_length = message_length;
    //         // newMessage->mode = COMM_MODE_BINARY;
    //         // SerialUSB.print("New Message: ");
    //         for (int i = 0; i < message_length; i++) {
    //           SerialUSB.print(message[i], HEX);
    //         }
    //         SerialUSB.println();
    //     }
    // }
  // Process received bytes
  //SerialUSB.print("---- fifo Used ----");
  SerialUSB.println("Length of receive buffer: ");SerialUSB.print(fifoBuf_getUsed(&receive_fifo), HEX);
  if(fifoBuf_getUsed(&receive_fifo) > 0)
  {
    if(Labcomm_Handle_Comms(&receive_fifo, &transmit_fifo) == true)
    {
      // Received input was used by the GUI
      SerialUSB.println("Received used by GUI");
      //SerialUSB.println(transmit_fifo.buf_size);
    }
    else
    {
      // Neither the GUI nor the Terminal Comm wanted this character. Check if QuikEval wants it, otherwise throw it away.
      uint8_t input = fifoBuf_getByte(&receive_fifo);
      if(input == 'I')
      {
        fifoBuf_putData(&transmit_fifo, id_struct.id_string, sizeof(id_struct.id_string));
      }
      else if(input == 'i')
      {
        fifoBuf_putData(&transmit_fifo, id_struct.ltc_string, sizeof(id_struct.ltc_string));
      }
    }
  }

  // Feed bytes out into Arduino Serial object from the larger ring buffer used by the communication interfaces.
  uint8_t temp_buffer[TRANSMIT_BFR_LEN];
  uint16_t temp_bytes_to_send = MIN(SerialUSB.availableForWrite(), fifoBuf_getUsed(&transmit_fifo));
  SerialUSB.print("temp_bytes_to_send = ");SerialUSB.println(temp_bytes_to_send);
  if(temp_bytes_to_send != 0)
  {
    fifoBuf_getData(&transmit_fifo, temp_buffer, temp_bytes_to_send);
    SerialUSB.write(temp_buffer, temp_bytes_to_send);
  }
  SerialUSB.println("DONE");
  return;
}

