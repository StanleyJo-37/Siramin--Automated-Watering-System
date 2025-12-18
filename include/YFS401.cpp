#include "YFS401.hpp"
#include "Arduino.h"

YSF401* YSF401::instance = nullptr;

void YSF401::begin()
{
  instance = this;
  pinMode(pulsePin, INPUT_PULLUP);
  attachInterrupt(pulsePin, YSF401::pulseISR, FALLING);
}

uint64_t YSF401::getAndResetPulse()
{
  uint64_t pulses;

  portENTER_CRITICAL(&mux);
  pulses = totalPulse;
  totalPulse = 0;
  lastTime = 0;
  portEXIT_CRITICAL(&mux);

  return pulses;
}

float YSF401::getFlowRate(uint64_t pulses, unsigned long elapsedTime)
{
  float frequency = (pulses * 1000.0) / elapsedTime;
  float flowRate = (frequency * 60) / K;

  return flowRate;
}

float YSF401::getCurrentVolume(uint64_t pulses, unsigned long elapsedTime)
{
  float frequency = (pulses * 1000.0) / elapsedTime;
  float flowRate = (frequency * 60) / K;

  return pulses / (K * 60);
}

HallEffectReading YSF401::measure()
{
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  instance->lastTime = currentTime;

  if (elapsedTime == 0) return HallEffectReading(0.0f, 0.0f);

  uint64_t currentPulses = getAndResetPulse();

  float flowRate = getFlowRate(currentPulses, elapsedTime);
  float volume = getCurrentVolume(currentPulses, elapsedTime);

  return HallEffectReading(flowRate, volume);
}

void YSF401::calibrate()
{
}

void IRAM_ATTR YSF401::pulseISR()
{
  if (instance) {
    portENTER_CRITICAL(&instance->mux);
    ++instance->totalPulse;
    portEXIT_CRITICAL(&instance->mux);
  }
}
