#pragma once

#include <SensorReading.hpp>

class FAO56ET
{
private:
  SensorReading readings;
  float P = 101.3;
  float gamma = 0.0665;
  float G = 0.0;
public:
  FAO56ET(SensorReading readings, float P=101.3, float gamma=0.0665, float G=0.0)
    : readings(readings), P(P), gamma(gamma), G(G) {};
  float getEt();
};