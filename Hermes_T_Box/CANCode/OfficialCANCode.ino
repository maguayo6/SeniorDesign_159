/*
  Authors:  Eric Desgranges
  Licensed under the GNU General Public License v3.0
*/

#include <FlexCAN.h>

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;
CAN_message_t rxMsg, msg;

// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t hex[17] = "0123456789abcdef";
  uint8_t working;
  while (dumpLen--) {
    working = *bytePtr++;
    Serial.write(hex[working >> 4]);
    Serial.write(hex[working & 15]);
  }
  Serial.write('\r');
  Serial.write('\n');
}

// -------------------------------------------------------------
void setup(void)
{
  pinMode(ledPin, OUTPUT); digitalWrite(ledPin, HIGH); delay(200); digitalWrite(ledPin, LOW);    // Blink once!
  Can0.begin();
  Serial.println(F("Hello Teensy E85!"));
  msg.len = 8;
  msg.id = 0x222;
  for ( int idx = 0; idx < 8; ++idx ) {
    msg.buf[idx] = idx;
  }
}

// -------------------------------------------------------------
void loop(void)
{
  Serial.print("CAN BUS Send: ");
  Can0.write(msg);
  digitalWrite(ledPin, HIGH); delay(500); digitalWrite(ledPin, LOW);
  delay(500);
}
