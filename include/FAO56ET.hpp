#include <SensorReading.hpp>

class FAO56ET
{
private:
  SensorReading readings;
public:
  FAO56ET(SensorReading readings) : readings(readings) {};
  ~FAO56ET();
  float getEt();
};