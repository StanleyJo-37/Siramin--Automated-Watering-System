#include "YFS201C.hpp"
#include "Arduino.h"

YFS201C* YFS201C::instance = nullptr;

void YFS201C::begin()
{
  instance = this;
  pinMode(this->pulsePin, INPUT);
  attachInterrupt(this->pulsePin, YFS201C::pulseISR, FALLING);
}

uint64_t YFS201C::getAndResetPulse()
{
  uint64_t pulses;

  portENTER_CRITICAL(&mux);
  pulses = totalPulse;
  totalPulse = 0;
  lastTime = 0;
  portEXIT_CRITICAL(&mux);

  return pulses;
}

float YFS201C::getFlowRate(uint64_t pulses, unsigned long elapsedTime)
{
  float frequency = (pulses * 1000.0) / elapsedTime;
  float flowRate = (frequency * 60) / this->K;

  return flowRate;
}

float YFS201C::getCurrentVolume(uint64_t pulses, unsigned long elapsedTime)
{
  float frequency = (pulses * 1000.0) / elapsedTime;
  float flowRate = (frequency * 60) / this->K;

  return pulses / (this->K * 60);
}

HallEffectReading YFS201C::measure()
{
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  lastTime = currentTime;

  if (elapsedTime == 0) return HallEffectReading(0.0f, 0.0f);

  uint64_t currentPulses = this->getAndResetPulse();

  float flowRate = this->getFlowRate(currentPulses, elapsedTime);
  float volume = this->getCurrentVolume(currentPulses, elapsedTime);

  return HallEffectReading(flowRate, volume);
}

void YFS201C::calibrate()
{
}

void IRAM_ATTR YFS201C::pulseISR()
{
  if (instance) {
    portENTER_CRITICAL(&instance->mux);
    ++instance->totalPulse;
    portEXIT_CRITICAL(&instance->mux);
  }
}
