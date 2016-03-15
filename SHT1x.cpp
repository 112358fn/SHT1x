/**
 * SHT1x Library
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "SHT1x.h"

SHT1x::SHT1x(int dataPin, int clockPin)
{
  _dataPin = dataPin;
  _clockPin = clockPin;
}
SHT1x::SHT1x(int dataPin, int clockPin, int powerPin)
{
  _dataPin = dataPin;
  _clockPin = clockPin;
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH);
  state=0;
}
SHT1x::SHT1x(int dataPin, int clockPin, int powerPin, int gndPin)
{
  _dataPin = dataPin;
  _clockPin = clockPin;
  pinMode(powerPin, OUTPUT);
  pinMode(gndPin, OUTPUT);
  digitalWrite(powerPin, HIGH);
  digitalWrite(gndPin, LOW);
  state=0;
}


/* ================  Public methods ================ */
/**
 * Reads current temperature & temperature-corrected relative humidity
 */
void SHT1x::update()
{
  const int _gHumidCmd = 0b00000101;// Command to send to the SHT1x to request humidity
  const int _gTempCmd  = 0b00000011;// Command to send to the SHT1x to request Temperature
  switch (state){
    case 0:
      sendCommandSHT(_gTempCmd, _dataPin, _clockPin);
      state=1;
      break;
    case 1:
      // Wait Conversion
      if(waitForResultSHT(_dataPin))state=2;
      else state=1;
      break;
    case 2:
      // Get temperature value
      rawTemp = getData16SHT(_dataPin, _clockPin);
      skipCrcSHT(_dataPin, _clockPin);
      state=3;
      break;
    case 3:
      sendCommandSHT(_gHumidCmd, _dataPin, _clockPin);
      state=4;
      break;
    case 4:
      // Wait Conversion
      if(waitForResultSHT(_dataPin))state=5;
      else state = 4;
      break;
    case 5:
      // Get relative humidity value
      rawHum = getData16SHT(_dataPin, _clockPin);
      skipCrcSHT(_dataPin, _clockPin);
      state=0;
      break;
  }

  return ;
}
/**
 * Reads the current temperature in degrees Celsius
 */
float SHT1x::readTemperatureC()
{
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;  // for 14 Bit @ 5V
  const float D2 =   0.01; // for 14 Bit DEGC

  // Convert raw value to degrees Celsius
  _temperature = (rawTemp * D2) + D1;

  return (_temperature);
}

/**
 * Reads the current temperature in degrees Fahrenheit
 */
float SHT1x::readTemperatureF()
{
  float _temperature;       // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;   // for 14 Bit @ 5V
  const float D2 =   0.018; // for 14 Bit DEGF

  // Convert raw value to degrees Fahrenheit
  _temperature = (rawTemp * D2) + D1;

  return (_temperature);
}

/**
 * Reads current temperature-corrected relative humidity
 */
float SHT1x::readHumidity()
{
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value

  // Conversion coefficients from SHT15 datasheet
  const float C1 = -4.0;       // for 12 Bit
  const float C2 =  0.0405;    // for 12 Bit
  const float C3 = -0.0000028; // for 12 Bit
  const float T1 =  0.01;      // for 14 Bit @ 5V
  const float T2 =  0.00008;   // for 14 Bit @ 5V

  // Apply linear conversion to raw value
  _linearHumidity = C1 + C2 * rawHum + C3 * rawHum * rawHum;

  // Get current temperature for humidity correction
  _temperature = readTemperatureC();

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * rawHum) + _linearHumidity;

  return (_correctedHumidity);
}


/* ================  Private methods ================ */

/**
 */
void SHT1x::sendCommandSHT(int _command, int _dataPin, int _clockPin)
{
  int ack;

  // Transmission Start
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  shiftOut(_dataPin, _clockPin, MSBFIRST, _command);

  // Verify we get the correct ack
  digitalWrite(_clockPin, HIGH);
  pinMode(_dataPin, INPUT);
  ack = digitalRead(_dataPin);
  if (ack != LOW) {
    //Serial.println("Ack Error 0");
  }
  digitalWrite(_clockPin, LOW);
  ack = digitalRead(_dataPin);
  if (ack != HIGH) {
    //Serial.println("Ack Error 1");
  }
}


/**
 * waitForResultSHT(int _dataPin)
 * Check if the conversion has ended in which case returns true
 */
bool SHT1x::waitForResultSHT(int _dataPin)
{
  pinMode(_dataPin, INPUT);
  if (digitalRead(_dataPin) == LOW) return true;
  else return false;
}

/**
 */
int SHT1x::getData16SHT(int _dataPin, int _clockPin)
{
  int val;

  // Get the most significant bits
  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, OUTPUT);
  val = shiftIn(_dataPin, _clockPin, MSBFIRST);
  val *= 256;

  // Send the required ack
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // Get the least significant bits
  pinMode(_dataPin, INPUT);
  val |= shiftIn(_dataPin, _clockPin, MSBFIRST);

  return val;
}

/**
 */
void SHT1x::skipCrcSHT(int _dataPin, int _clockPin)
{
  // Skip acknowledge to end trans (no CRC)
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);

  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);
}
