#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <YFS401.hpp>
#include <SensorReading.hpp>
#include <FAO56ET.hpp>
#include <BlynkSimpleEsp32.h>
#include <SoilMoistureSensor.hpp>
#include "esp_netif.h"

// Sensors
#define BMP280 Adafruit_BMP280

// GPIO PINS (User Defined)
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

// PWM Setups
constexpr unsigned int PWM_FREQ = 5000;
constexpr unsigned int PWM_RES = 8;
constexpr unsigned int CH_R = 0;
constexpr unsigned int CH_G = 1;
constexpr unsigned int CH_B = 2;

// Misc Values
float YSF401_K = 7.5f;
float kc = 1.0f;
float SOIL_AREA = 0.05f;
float WIND_SPEED = 2.0f; 

// Timers
unsigned long previousMillis = 0;
unsigned long previousMillisSensor = 0;
unsigned long interval = 60000UL; // Start with 1 min, then 10 min
unsigned long sensorInterval = 2000UL;
volatile float cumETc = 0.0f;
bool isWatering = false; // Safety flag

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht11 = DHT(DHT_PIN, DHT11);
YSF401 ysf401 = YSF401(YSF401_PIN, YSF401_K);
SoilMoistureSensor soilMoistureSensor = SoilMoistureSensor(SOIL_MOISTURE_PIN);

// Thresholds
float cumEtcThreshold = 3.0f;
float soilMoistureThreshold = 30.0f;

// Blynk Auth
constexpr char BLYNK_TEMPLATE_ID[] = "TMPL6XWzaUZzt";
constexpr char BLYNK_TEMPLATE_NAME[] = "Smart Watering System";
constexpr char BLYNK_AUTH_TOKEN[] = "Ubj-r5QACAgdQwkuSRvPdrXAu3mVF5TA";
char ssid[] = "iPhone (Jason)";
char pass[] = "123123haha";

// --- HELPERS ---
void setLedColor(int r, int g, int b) {
  ledcWrite(CH_R, r);
  ledcWrite(CH_G, g);
  ledcWrite(CH_B, b);
}

float getVolumeNeeded(float et); // Forward declaration

// --- WATERING FUNCTION ---
void water(float targetVolume, const uint32_t measure_interval = 1000, const uint32_t timeout = 60000) {
  isWatering = true; // Set flag
  float accVolume = 0.0f;
  unsigned long lastMeasure = millis();
  unsigned long startTime   = millis();

  Serial.printf(">> PUMP ON. Target: %.2f L\n", targetVolume);
  
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Watering...");
  lcd.setCursor(0, 1); lcd.print("Target: "); lcd.print(targetVolume); lcd.print("L");

  digitalWrite(RELAY_PIN, LOW); // Active LOW

  while (accVolume < targetVolume) {
    Blynk.run();
    unsigned long now = millis();

    if (now - startTime > timeout) {
      Serial.println("ERROR: Timeout!");
      lcd.clear(); lcd.print("Error: Timeout");
      setLedColor(255, 0, 0); delay(1000);
      break;
    }

    if (now - lastMeasure >= measure_interval) {
      HallEffectReading wateringReading = ysf401.measure();
      accVolume += wateringReading.volume;
      
      Blynk.virtualWrite(V6, accVolume);

      if (wateringReading.flowRate <= 0.1f) {  
        Serial.println("WARNING: No flow!");
        lcd.clear(); lcd.print("Err: No Flow");
        setLedColor(255, 0, 0); delay(1000);
        break;
      }
      Serial.printf("Flow: %.1f L/m | Total: %.2f L\n", wateringReading.flowRate, accVolume);
      lastMeasure = now;
    }
    delay(10);
  }
  
  digitalWrite(RELAY_PIN, HIGH); // OFF
  Serial.println(">> PUMP OFF.");
  ysf401.getAndResetPulse();
  Blynk.virtualWrite(V6, 0.00f);
  isWatering = false; // Release flag
}

// --- BLYNK MANUAL INPUT (V10) ---
// Button set to PUSH mode (Sends 1 on press, 0 on release)
BLYNK_WRITE(V10) {
  int buttonState = param.asInt(); 

  // Only trigger if button is PRESSED (1) and pump is not already running
  if (buttonState == 1 && !isWatering) {
    Serial.println(">> Manual Button V10 Pressed!");

    // 1. Define Standard Manual Dose
    // Since input is just ON/OFF, we assume a "Standard Full Water"
    // equal to the threshold (3.0mm)
    float manualET = 3.0f; 

    // 2. Perform Same Calculation
    float volume = getVolumeNeeded(manualET);

    // 3. LED Logic
    setLedColor(0, 0, 255); // Blue for Manual

    // 4. Update Blynk & LCD
    Blynk.virtualWrite(V4, "Manual: 3mm");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Manual Button");
    lcd.setCursor(0, 1); lcd.print("Dose: 3.0mm");
    delay(1000);

    // 5. Water
    water(volume);

    // 6. Reset System
    cumETc = 0; 
    setLedColor(0, 0, 0); 
    
    // Reset Button in App (if Switch mode)
    Blynk.virtualWrite(V10, 0);
  }
}

float getSolarRadiation() {
  int rawLdr = analogRead(PHOTO_RESISTOR_PIN);
  float watts = map(rawLdr, 0, 4095, 0, 1000); 
  return watts * 0.0864;
}

SensorReading readAndSendReadings() {
  float temperature = dht11.readTemperature();
  float humidity = dht11.readHumidity();
  float soilMoisture = soilMoistureSensor.read();
  float solarRadiation = getSolarRadiation();
  float windSpeed = WIND_SPEED;

  if (isnan(temperature) || isnan(humidity)) {
    temperature = 25.0; humidity = 50.0;
  }

  SensorReading r(temperature, humidity, soilMoisture, solarRadiation, windSpeed);

  // Only update LCD if NOT watering to prevent flickering override
  if (!isWatering) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.printf("T:%.0f H:%.0f S:%.0f%%", r.temperature, r.humidity, r.soilMoisture);
    lcd.setCursor(0,1);
    lcd.printf("Rad:%.1f ET:%.1f", r.solarRadiation, cumETc);
  }

  Blynk.virtualWrite(V0, r.temperature);
  Blynk.virtualWrite(V1, r.humidity);
  Blynk.virtualWrite(V8, r.soilMoisture);
  Blynk.virtualWrite(V9, cumETc);

  return r;
}

float getVolumeNeeded(float et) {
  return et * SOIL_AREA;
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  // LED
  pinMode(LED_R_PIN, OUTPUT); pinMode(LED_G_PIN, OUTPUT); pinMode(LED_B_PIN, OUTPUT);
  ledcSetup(CH_R, PWM_FREQ, PWM_RES); ledcSetup(CH_G, PWM_FREQ, PWM_RES); ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(LED_R_PIN, CH_R); ledcAttachPin(LED_G_PIN, CH_G); ledcAttachPin(LED_B_PIN, CH_B);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected");

  // DNS Fix
  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif) {
    esp_netif_dns_info_t dns;
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = esp_ip4addr_aton("8.8.8.8");
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
  }

  // Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);

  // Hardware Init
  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("System Starting");

  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, HIGH); // Relay OFF
  pinMode(PHOTO_RESISTOR_PIN, INPUT);
  
  dht11.begin();
  soilMoistureSensor.begin();
  ysf401.begin();

  Blynk.run();
  Blynk.virtualWrite(V2, WIND_SPEED);
  Blynk.virtualWrite(V7, SOIL_AREA);
  
  // Init Flash
  setLedColor(255, 255, 255); delay(500); setLedColor(0, 0, 0);
}

void loop() {
  Blynk.run();
  unsigned long currentMillis = millis();

  // --- FAST TIMER (2s) ---
  if (currentMillis - previousMillisSensor >= sensorInterval) {
    previousMillisSensor = currentMillis;
    readAndSendReadings(); 
  }
  
  // --- SLOW TIMER (1m then 10m) ---
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    interval = 10UL * 60UL * 1000UL; // Lock to 10 mins

    if (!isWatering) {
        lcd.clear(); lcd.print("Checking ET...");
    }
    
    SensorReading readings = readAndSendReadings();
    FAO56ET fao56et = FAO56ET(readings);
    
    float et = fao56et.getEt();
    float etInterval = et * (interval / 86400000.0f);
    cumETc += etInterval;
    
    float volume = getVolumeNeeded(cumETc); 

    if (readings.soilMoisture < soilMoistureThreshold && cumETc > cumEtcThreshold) {
      
      String waterStatus;
      if (volume < 3.00f) {
        waterStatus = "Sufficient"; setLedColor(0, 255, 0); 
      } 
      else if (volume >= 3.00f && volume < 6.00f) {
        waterStatus = "Needs Water"; setLedColor(255, 255, 0);
      }
      else {
        waterStatus = "Too Low"; setLedColor(255, 0, 0);
      }

      Blynk.virtualWrite(V4, waterStatus);
      Blynk.virtualWrite(V6, volume);
      water(volume);
      cumETc = 0;
      setLedColor(0, 0, 0);
    } else {
      if (!isWatering) {
          lcd.clear();
          lcd.setCursor(0, 0); lcd.print("Status: OK");
          lcd.setCursor(0, 1); lcd.print("Next Check 10m");
          setLedColor(0, 255, 0); delay(2000); setLedColor(0, 0, 0);
      }
    }
  }
}