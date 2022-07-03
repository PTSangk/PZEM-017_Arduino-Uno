/*
Copyright (c) 2021 Jakub Mandula

Example of using one PZEM module with Software Serial interface.
================================================================

If only RX and TX pins are passed to the constructor, software 
serial interface will be used for communication with the module.

*/

#include "PZEM004Tv30.h"
//#include <SoftwareSerial.h>

#define USE_SOFTWARE_SERIAL //add
#define PZEM004_SOFTSERIAL //add 07Jun2022

#define SSerialTxControl 7   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define Pin13LED         13



// #if defined(ESP32)
    // #error "Software Serial is not supported on the ESP32"
// #endif

// /* Use software serial for the PZEM
 // * Pin 12 Rx (Connects to the Tx pin on the PZEM)
 // * Pin 13 Tx (Connects to the Rx pin on the PZEM)
// */
// #if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
// #define PZEM_RX_PIN 9
// #define PZEM_TX_PIN 8
// #endif


// SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
// PZEM004Tv30 pzem(pzemSWSerial);


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

void preTransmissionArduino();
void postTransmissionArduino();

void setup() {
    /* Debugging serial */
    Serial.begin(115200);
	
	pinMode(Pin13LED, OUTPUT);
  pinMode(SSerialTxControl, OUTPUT);
  
    // Callbacks allow us to configure the RS485 transceiver correctly
  pzem.preTransmission(preTransmissionArduino);
  pzem.postTransmission(postTransmissionArduino);
  
  //digitalWrite(SSerialTxControl, RS485Receive);
  
  
  
  
/*    Serial.println("Change Shunt Value");
 if (!pzem.setShuntType(0x01))
  {
    // Setting custom address failed. Probably no PZEM connected
    Serial.println("Error To Change Shunt Value");
  } else {
    // Print out the new custom address
    Serial.print("Shunt Value:    0x");
    Serial.println(pzem.getShunttype(), HEX);
    Serial.println();
  } */
}

void loop() {
         
    Serial.print("Custom Address:");
    Serial.println(pzem.readAddress(), HEX);
	
    // Read the data from the sensor
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
	uint16_t Shunttype = pzem.getShunttype();
    //float frequency = pzem.frequency();
    //float pf = pzem.pf();

    // Check if the data is valid
    if(isnan(voltage)){
        Serial.println("Error reading voltage");
    } else if (isnan(current)) {
        Serial.println("Error reading current");
    } else if (isnan(power)) {
        Serial.println("Error reading power");
    } else if (isnan(energy)) {
        Serial.println("Error reading energy");
    } //else if (isnan(frequency)) {
      //  Serial.println("Error reading frequency");
    //} else if (isnan(pf)) {
       // Serial.println("Error reading power factor");
    //} 
	else {

        // Print the values to the Serial console
        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
        Serial.print("Shunttype: 0x");    Serial.print(Shunttype,HEX); Serial.println("");
		//Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        //Serial.print("PF: ");           Serial.println(pf);
    }

    Serial.println();
    delay(2000);
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