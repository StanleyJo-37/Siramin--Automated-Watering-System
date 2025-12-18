#include "SoilMoistureSensor.hpp"
#include "Arduino.h"

void SoilMoistureSensor::begin()
{
  pinMode(pin, INPUT);
}

float SoilMoistureSensor::read()
{
  int rawValue = analogRead(pin);
      
  int constrainedValue = constrain(rawValue, min(wetValue, dryValue), max(wetValue, dryValue));
  
  float percentage = map(constrainedValue, dryValue, wetValue, 0, 100);
  
  return percentage;
}

void SoilMoistureSensor::calibrate()
{
}
