#include <Arduino.h>
#include <Modbus.h>

/**
  @file TempSensor.ino
  Modbus-Arduino Example - TempSensor (Modbus Serial)
    Copyright (C) 2023 Pascal JEAN aka epsilonrt
    Copyright (C) 2014 André Sarmento Barbosa
    https://github.com/epsilonrt/modbus-serial
*/

// to test read =>  mbpoll -m rtu -b38400 -a14 -t 3:hex -r 2 -c 1  /dev/ttyAMA0
// to test write => mbpoll -m rtu -b38400 -a14 -t 4:hex -r 3 /dev/ttyAMA0 0x03

#include <ModbusSerial.h>

#define DATA_SIZE 26    // 26 bytes is a lower than RX FIFO size (127 bytes)
#define BAUD      38400UL  // Any baudrate from 300 to 115200
#define TEST_UART 1     // Serial1 will be used for the loopback testing with different RX FIFO FULL values
#define RXPIN     4     // GPIO 4 => RX for Serial1
#define TXPIN     5     // GPIO 5 => TX for Serial1

// Used Pins
const int SensorPin = A0;
const int TxenPin = -1; // -1 disables the feature, change that if you are using an RS485 driver, this pin would be connected to the DE and /RE pins of the driver.

const byte SlaveId = 14;
// Modbus Registers Offsets (0-9999)
const int SensorIreg = 0;
const int CounterIreg = 1;
const int InjectorHreg = 2;

uint16_t counter = 0x1000;
uint16_t inc = 1;

#define MySerial Serial1 // define serial port used, Serial most of the time, or Serial1, Serial2 ... if available

// ModbusSerial object
ModbusSerial mb (MySerial, SlaveId, TxenPin);

long ts;

void setup() {
  Serial.begin(115200);
  Serial.printf("Start ");
  Serial.println();

  MySerial.begin(BAUD, SERIAL_8E1, RXPIN, TXPIN); 
 

  mb.config (BAUD);
  mb.setAdditionalServerData ("BEST_TEST"); // for Report Server ID function (0x11)


  // Add SensorIreg register - Use addIreg() for analog Inputs
  mb.addIreg (SensorIreg);
  mb.addIreg(CounterIreg, counter);
  mb.addHreg(InjectorHreg, 1);
  mb.setHregBounds (InjectorHreg, 1, 10);

  ts = millis();
}

void loop() {

  mb.Ireg(CounterIreg, counter);
  
  // Call once inside loop() - all magic here
  mb.task();

  if(mb.hreg(InjectorHreg) != inc){
    inc = mb.hreg(InjectorHreg);
  }

  // Read each two seconds
  if (millis() > ts + 2000) {
    
    ts = millis();
    counter += inc;
    // Setting raw value (0-1024)
    mb.Ireg (SensorIreg, analogRead (SensorPin));
  }
}
