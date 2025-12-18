#pragma once

#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

struct HallEffectReading {
  float flowRate;
  float volume;

  HallEffectReading(float flowRate, float volume)
    : flowRate(flowRate), volume(volume) {}
};

class YSF401 {
  public:
    YSF401(int pin = 27, float K = 7.5)
      : pulsePin(pin), K(K), totalPulse(0), lastTime(0) {}

    void begin();
    uint64_t getAndResetPulse();
    float getFlowRate(uint64_t pulses, unsigned long elapsedTime);
    float getCurrentVolume(uint64_t pulses, unsigned long elapsedTime);
    HallEffectReading measure();

    void calibrate();

    static YSF401* instance;

  private:
    int pulsePin;
    float K;

    volatile uint64_t totalPulse;
    unsigned long lastTime;
    
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    
    static void IRAM_ATTR pulseISR();
};
