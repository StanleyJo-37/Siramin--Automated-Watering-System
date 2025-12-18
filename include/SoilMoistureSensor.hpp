#pragma once

#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

class SoilMoistureSensor {
  public:
    SoilMoistureSensor(int pin = 27)
      : pin(pin) {}

    void begin();
    float read();

    void calibrate();

  private:
    int pin;
    float dryValue, wetValue;
};
