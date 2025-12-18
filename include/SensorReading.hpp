#pragma once

#include <iostream>

struct SensorReading {
  float temperature, humidity, solarRadiation, windSpeed, soilMoisture;

  SensorReading(float temperature, float humidity, float solarRadiation, float windSpeed, float soilMoisture)
    : temperature(temperature), humidity(humidity), solarRadiation(solarRadiation), windSpeed(windSpeed), soilMoisture(soilMoisture) {};
};