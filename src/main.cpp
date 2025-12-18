#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <YFS401.hpp>
#include <SensorReading.hpp>
#include <FAO56ET.hpp>
#include <BlynkSimpleEsp32.h>
#include <SoilMoistureSensor.hpp>

// Sensors
#define BMP280 Adafruit_BMP280

// States
volatile int displayState = 0;

// PWM Setups
constexpr unsigned int PWM_FREQ = 5000;
constexpr unsigned int PWM_RES = 8;

// GPIO PINS
constexpr unsigned int LCD_SDA_PIN = 8;
constexpr unsigned int LCD_SCL_PIN = 9;

constexpr unsigned int LED_R_PIN = 5;
constexpr unsigned int LED_G_PIN = 7;
constexpr unsigned int LED_B_PIN = 6;

constexpr unsigned int RELAY_PIN = 1;
constexpr unsigned int YSF401_PIN = 2;
constexpr unsigned int DHT_PIN = 0;

constexpr unsigned int SOIL_MOISTURE_PIN = 4;
constexpr unsigned int PHOTO_RESISTOR_PIN = 3;

// LED Channels
constexpr unsigned int CH_R = 0;
constexpr unsigned int CH_G = 1;
constexpr unsigned int CH_B = 2;

// Misc Values
float YSF401_K = 7.5f;
float kc = 1.0f;
float soil_area = 2.0f;
float WIND_SPEED = 2.0f;

unsigned long previousMillis = 0;
unsigned long interval = 30UL * 60UL * 1000UL;
volatile float cumETc = 0.0f;

// Sensors and Outputs
// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht11 = DHT(DHT_PIN, DHT11);
YSF401 ysf401 = YSF401(YSF401_PIN, YSF401_K);
SoilMoistureSensor soilMoistureSensor = SoilMoistureSensor(SOIL_MOISTURE_PIN);

// Thresholds
float cumEtcThreshold = 3.0f;
float soilMoistureThreshold = 30.0f;

// Blynk and WiFi Auth
constexpr char BLYNK_TEMPLATE_ID[] = "";
constexpr char BLYNK_TEMPLATE_NAME[] = "";
constexpr char BLYNK_AUTH_TOKEN[] = "";
BlynkTimer timer;

char ssid[] = "";
char pass[] = "";

float getSolarRadiation() {
  int rawLdr = analogRead(PHOTO_RESISTOR_PIN);
  // Map 0-4095 (Dark-Bright) to 0-1000 W/m2
  float watts = map(rawLdr, 0, 4095, 0, 1000); 
  // Convert W/m2 to MJ/m^2/day (0.0864 factor)
  return watts * 0.0864;
}

void water(float targetVolume, const uint32_t measure_interval = 1000, const uint32_t timeout = 60000) {
  float accVolume = 0.0f;

  unsigned long lastMeasure = millis();
  unsigned long startTime   = millis();

  Serial.printf(">> PUMP ON. Target: %.2f L\n", targetVolume);
  // Water and begin measurement
  digitalWrite(RELAY_PIN, LOW);

  // Measurement loop until accVolume satisifes the amount of water needed
  while (accVolume < targetVolume) {
    Blynk.run();
    unsigned long now = millis();

    if (now - startTime > timeout) {
      Serial.println("ERROR: Watering timeout! Possible dry source or blocked flow.");
      break;
    }

    // Calculate once per second
    if (now - lastMeasure >= measure_interval) {
      // Get reading and add volume to accumulated volume
      HallEffectReading wateringReading = ysf401.measure();
      accVolume += wateringReading.volume;

      if (wateringReading.flowRate <= 0.1f) {  
        Serial.println("WARNING: No flow detected! Shutting off.");
        break;
      }
      Serial.printf("Flow: %.1f L/m | Total: %.2f L\n", wateringReading.flowRate, accVolume);
      lastMeasure = now;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  // Turn relay off
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println(">> PUMP OFF.");

  // Reset and flush variables
  ysf401.getAndResetPulse();
}

SensorReading readAndSendReadings() {
  float temperature = dht11.readTemperature();
  float humidity = dht11.readHumidity();
  float soilMoisture = soilMoistureSensor.read();
  float solarRadiation = getSolarRadiation();
  float windSpeed = WIND_SPEED;

  if (isnan(temperature) || isnan(humidity)) {
    temperature = 25.0;
    humidity = 50.0;
    Serial.println("DHT Error: Using defaults.");
  }

  SensorReading r(temperature, humidity, soilMoisture, solarRadiation, windSpeed);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.printf("T:%.0f H:%.0f S:%.0f%%", r.temperature, r.humidity, r.soilMoisture);
  lcd.setCursor(0,1);
  lcd.printf("Rad:%.1f ET:%.1f", r.solarRadiation, cumETc);

  Blynk.virtualWrite(V0, r.temperature);
  Blynk.virtualWrite(V1, r.humidity);
  Blynk.virtualWrite(V2, r.soilMoisture);
  Blynk.virtualWrite(V3, cumETc);

  return r;
}

float getVolumeNeeded(float et) {
  return et * soil_area;
}

void setup() {
  // Serial Monitor
  Serial.begin(115200);
  delay(3000);
  Serial.println("Setup initation");
  
  // LED Setup
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(CH_G, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);

  ledcAttachPin(LED_R_PIN, CH_R);
  ledcAttachPin(LED_G_PIN, CH_G);
  ledcAttachPin(LED_B_PIN, CH_B);

  // Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud");

  // LCD Setup
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  pinMode(PHOTO_RESISTOR_PIN, INPUT);
  // HTU21D
  dht11.begin();

  soilMoistureSensor.begin();

  // YSF401
  ysf401.begin();

  Serial.println("Setup finished, all sensor initiated");

}

void loop() {
  Blynk.run();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    SensorReading readings = readAndSendReadings();

    FAO56ET fao56et = FAO56ET(readings);
    float et = fao56et.getEt();
    float etInterval = et * (interval / 86400000.0);
    
    cumETc += etInterval;
    float volume = getVolumeNeeded(etInterval);

    if (readings.soilMoisture < soilMoistureThreshold && cumETc > cumEtcThreshold) {
      float volume = getVolumeNeeded(cumETc);
      water(volume);

      cumETc = 0;
    } else {
      Serial.println("No watering needed.");
    }
  }
}
