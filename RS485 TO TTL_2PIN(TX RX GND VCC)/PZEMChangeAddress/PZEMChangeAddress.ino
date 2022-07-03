/*
  Copyright (c) 2021 Jakub Mandula

  Example of setting a custom ModBUS address for the PZEM modules.
  ================================================================

  Each PZEM modules has two addresses. A default general address (0xF8),
  which every module will listen on by default. And a custom address.

  The custom address can be used if multiple PZEM modules are to be used
  on a single ModBUS.

  This script will use the general address in order to set the custom
  address of the connected PZEM module.

  Therefore make sure only one PZEM module is connected at a time!
  Otherwise all connected modules will receive the same custom address.

*/

#include "PZEM004Tv30.h"

#define USE_SOFTWARE_SERIAL //add
#define PZEM004_SOFTSERIAL //add 07Jun2022

#define SSerialTxControl 7   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define Pin13LED         13
unsigned long startMillis1; 

#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 9
#define PZEM_TX_PIN 8
#endif

#if !defined(PZEM_SERIAL)
#define PZEM_SERIAL Serial
#endif

/* Hardware Serial2 is only available on certain boards.
   For example the Arduino MEGA 2560
*/
#if defined(USE_SOFTWARE_SERIAL)
#include <SoftwareSerial.h>
/*************************
    Use SoftwareSerial for communication
   ---------------------

   The ESP32 platform does not support the SoftwareSerial as of now
   Here we initialize the PZEM on SoftwareSerial with RX/TX pins PZEM_RX_PIN and PZEM_TX_PIN
*/
SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);

#elif defined(ESP32)
/*************************
    ESP32 initialization
   ---------------------

   The ESP32 HW Serial interface can be routed to any GPIO pin
   Here we initialize the PZEM on PZEM_SERIAL with RX/TX pins PZEM_RX_PIN and PZEM_TX_PIN
*/
PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);

#else
/*************************
    Arduino/ESP8266 initialization
   ---------------------

   Not all Arduino boards come with multiple HW Serial ports.
   Serial2 is for example available on the Arduino MEGA 2560 but not Arduino Uno!
   The ESP32 HW Serial interface can be routed to any GPIO pin
   Here we initialize the PZEM on PZEM_SERIAL with default pins
*/
PZEM004Tv30 pzem(PZEM_SERIAL);

#endif

/*******************************************
   Set your address over here..
   The address can be between 0x01 and 0xF7
 *******************************************/
#if !defined(SET_ADDRESS)
#define SET_ADDRESS 0x61
#endif

// Set to true in order to increment the address every iteration
#define INCREMENT false

void preTransmissionArduino();
void postTransmissionArduino();

void setup() {
	startMillis1 = millis();
  Serial.begin(115200);
  pinMode(Pin13LED, OUTPUT);
  pinMode(SSerialTxControl, OUTPUT);

  // Callbacks allow us to configure the RS485 transceiver correctly
  pzem.preTransmission(preTransmissionArduino);
  pzem.postTransmission(postTransmissionArduino);
  
  //digitalWrite(SSerialTxControl, RS485Receive);
}

void loop() {
  static uint8_t addr = SET_ADDRESS;

  // Print out current custom address
  Serial.print("Previous address:   0x");
  Serial.println(pzem.readAddress(), HEX);

   // Set the custom address
  Serial.print("Setting address to: 0x");
  Serial.println(addr, HEX);
 if (!pzem.setAddress(addr))
  {
    // Setting custom address failed. Probably no PZEM connected
    Serial.println("Error setting address.");
  } else {
    // Print out the new custom address
    Serial.print("Current address:    0x");
    Serial.println(pzem.readAddress(), HEX);
    Serial.println();
  }

  // Increment the address every loop if desired
  if (INCREMENT) {
    addr++;
    if (addr >= PZEM_DEFAULT_ADDR)
      addr = 0x01;
  }

  delay(200);
}

void preTransmissionArduino()
{

  digitalWrite(Pin13LED, HIGH);
  delay(1);
  digitalWrite(SSerialTxControl, RS485Transmit);
  delay(1);

}

void postTransmissionArduino()
{
 /* 1- PZEM-017 DC Energy Meter */

  delay(3); 
  digitalWrite(Pin13LED, LOW);
  delay(1);
  digitalWrite(SSerialTxControl, RS485Receive);

}
