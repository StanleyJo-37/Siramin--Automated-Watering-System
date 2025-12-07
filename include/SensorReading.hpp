#include <iostream>

struct SensorReading {
  float temperature, humidity, lightIntensity, windSpeed, soilMoisture, soilTemperature;

  SensorReading(float temperature, float humidity, float lightIntensity, float windSpeed, float soilMoisture, float soilTemperature)
    : temperature(temperature), humidity(humidity), lightIntensity(lightIntensity), windSpeed(windSpeed), soilMoisture(soilMoisture), soilTemperature(soilTemperature) {};
};