#include "FAO56ET.hpp"
#include <cmath>

float FAO56ET::getEt()
{
  // 1. Extract variables for cleaner math
  float T = readings.temperature;
  float RH = readings.humidity;
  float Rs = readings.solarRadiation; // MJ/m2/day
  float u2 = readings.windSpeed;      // m/s

  // 3. Saturation Vapor Pressure (es)
  float es = 0.6108 * exp((17.27 * T) / (T + 237.3));

  // 4. Actual Vapor Pressure (ea)
  float ea = es * (RH / 100.0);

  // 5. Slope of Vapor Pressure Curve (delta)
  float delta = (4098 * es) / pow((T + 237.3), 2);

  // 6. Net Radiation (Rn) - Simplified
  // We assume Net Radiation is roughly 77% of Solar Radiation (albedo effect)
  float Rn = 0.77 * Rs; 

  // 7. FAO-56 Penman-Monteith Equation
  float numerator = (0.408 * delta * (Rn - G)) + (gamma * (900 / (T + 273)) * u2 * (es - ea));
  float denominator = delta + (gamma * (1 + 0.34 * u2));

  float et0 = numerator / denominator;

  // Sanity check: ET cannot be negative
  if (et0 < 0) return 0.0f;
  
  return et0;
}