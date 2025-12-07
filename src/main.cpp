#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_HTU21DF.h>
#include <YFS201C.hpp>
#include <SensorReading.hpp>
#include <FAO56ET.hpp>
#include <BlynkSimpleEsp32.h>

// LCD
#define LCD LiquidCrystal_I2C

// Sensors
#define HTU21D Adafruit_HTU21DF
#define BMP280 Adafruit_BMP280

// States
volatile int displayState = 0;

// PWM Setups
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RES = 8;

// GPIO PINS
constexpr int LCD_SDA_PIN = 21;
constexpr int LCD_SCL_PIN = 22;

constexpr int LED_R_PIN = 25;
constexpr int LED_G_PIN = 26;
constexpr int LED_B_PIN = 27;

constexpr int RELAY_PIN = 12;
constexpr int YSF201C_PIN = 13;

// LED Channels
constexpr int CH_R = 0;
constexpr int CH_G = 1;
constexpr int CH_B = 2;

// Misc Values
float YSF201C_K = 7.5f;
float kc = 1.0f;
float soil_area = 2.0f;

// LCD
LCD lcd = LCD(0x27, 16, 2);

volatile float cumETc = 0.0f;
unsigned long previousMillis = 0;
unsigned long interval = 30UL * 60UL * 1000UL;

// Sensors
HTU21D htu21d = HTU21D();
YFS201C yfs201c = YFS201C(YSF201C_PIN, YSF201C_K);

// Thresholds
float cumEtcThreshold = 3.0f;
float soilMoistureThreshold = 0.3f;

// Blynk and WiFi Auth
constexpr char BLYNK_TEMPLATE_ID[] = "";
constexpr char BLYNK_TEMPLATE_NAME[] = "";
constexpr char BLYNK_AUTH_TOKEN[] = "";
BlynkTimer timer;

char ssid[] = "";
char pass[] = "";

void water(float targetVolume, const uint32_t measure_interval = 1000, const uint32_t timeout = 60000) {
  float accVolume = 0.0f;

  unsigned long lastMeasure = millis();
  unsigned long startTime   = millis();

  // Water and begin measurement
  digitalWrite(RELAY_PIN, HIGH);

  // Measurement loop until accVolume satisifes the amount of water needed
  while (accVolume < targetVolume) {
    unsigned long now = millis();

    if (now - startTime > timeout) {
      Serial.println("ERROR: Watering timeout! Possible dry source or blocked flow.");
      break;
    }

    // Calculate once per second
    if (now - lastMeasure >= measure_interval) {
      // Get reading and add volume to accumulated volume
      HallEffectReading wateringReading = yfs201c.measure();
      accVolume += wateringReading.volume;
      
      // Update last measure
      lastMeasure = now;

      if (wateringReading.flowRate <= 0.1f) {  
        Serial.println("WARNING: No flow detected! Shutting off.");
        break;
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  // Turn relay off
  digitalWrite(RELAY_PIN, LOW);

  // Reset and flush variables
  yfs201c.getAndResetPulse();
}

SensorReading readAndSendReadings() {
  float temperature = htu21d.readTemperature();
  float humidity = htu21d.readHumidity();


}

float getVolumeNeeded(float et) {
  return et * soil_area;
}

void setup() {
  // Serial Monitor
  Serial.begin(9600);

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

  // HTU21D
  htu21d.begin();

  // YFS201C
  yfs201c.begin();

}

void loop() {
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
    }
  }
}
