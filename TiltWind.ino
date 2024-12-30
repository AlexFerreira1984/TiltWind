// Firmware for XIAO BLE Sense - Wind Monitoring System
// Includes BME680 Sensor, IMU, Solar Panel, and Bluefruit BLE Communication

#include <Arduino.h>
#include <Adafruit_BME680.h>
#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"

// Pin Definitions
#define SOLAR_PIN A0
#define BATTERY_PIN A1

LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A

// BLE Services
BLEService batteryService = BLEService("0000180f-0000-1000-8000-00805f9b34fb"); // Battery Service
BLEService deviceInfoService = BLEService("0000180a-0000-1000-8000-00805f9b34fb"); // Device Information Service

// BLE Characteristics
BLECharacteristic batteryVoltageChar = BLECharacteristic("00002a19-0000-1000-8000-00805f9b34fb");
BLECharacteristic solarVoltageChar = BLECharacteristic("00002a6e-0000-1000-8000-00805f9b34fb");
BLECharacteristic windDirectionChar = BLECharacteristic("00002a6d-0000-1000-8000-00805f9b34fb");
BLECharacteristic windSpeedChar = BLECharacteristic("00002a70-0000-1000-8000-00805f9b34fb");
BLECharacteristic gustSpeedChar = BLECharacteristic("00002a71-0000-1000-8000-00805f9b34fb");
BLECharacteristic temperatureChar = BLECharacteristic("00002a1f-0000-1000-8000-00805f9b34fb");
BLECharacteristic humidityChar = BLECharacteristic("00002a6f-0000-1000-8000-00805f9b34fb");
BLECharacteristic pressureChar = BLECharacteristic("00002a6d-0000-1000-8000-00805f9b34fb");

// Sensors
Adafruit_BME680 bme;

unsigned long lastBLETransmission = 0;
#define BLE_INTERVAL 10000

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize IMU
  if (myIMU.begin() != 0) {
    Serial.println("IMU Device error");
  } else {
    Serial.println("IMU Device OK!");
  }

  // Initialize BME680
  if (!bme.begin(0x76)) {
    Serial.println("BME Device error");
  } else {
    Serial.println("BME Device OK!");
  }

  // Initialize Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Max transmit power
  Bluefruit.setName("TiltWind");

  // BLE Services
  batteryService.begin();
  deviceInfoService.begin();

  // BLE Characteristics Configuration
  windDirectionChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  windDirectionChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  windDirectionChar.setFixedLen(4);
  windDirectionChar.begin();

  windSpeedChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  windSpeedChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  windSpeedChar.setFixedLen(4);
  windSpeedChar.begin();

  gustSpeedChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  gustSpeedChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  gustSpeedChar.setFixedLen(4);
  gustSpeedChar.begin();

  batteryVoltageChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  batteryVoltageChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryVoltageChar.setFixedLen(4);
  batteryVoltageChar.begin();

  solarVoltageChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  solarVoltageChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  solarVoltageChar.setFixedLen(4);
  solarVoltageChar.begin();

  temperatureChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  temperatureChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  temperatureChar.setFixedLen(4);
  temperatureChar.begin();

  humidityChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  humidityChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  humidityChar.setFixedLen(4);
  humidityChar.begin();

  pressureChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  pressureChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pressureChar.setFixedLen(4);
  pressureChar.begin();

  // Start advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.addService(deviceInfoService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start();

  Serial.println("BLE TiltWind Initialized");
}

void loop() {
  static float windDirection = 0.0;
  static float windSpeed = 0.0;
  static float gustSpeed = 0.0;

  // Read IMU for wind measurements
  float x = myIMU.readFloatAccelX();
  float y = myIMU.readFloatAccelY();
  float z = myIMU.readFloatAccelZ();
  windDirection = atan2(y, x) * (180.0 / PI);
  windSpeed = sqrt(x * x + y * y);
  gustSpeed = max(gustSpeed, windSpeed);

  Serial.print("Wind Direction: "); Serial.println(windDirection);
  Serial.print("Wind Speed: "); Serial.println(windSpeed);
  Serial.print("Gust Speed: "); Serial.println(gustSpeed);

  // Read BME680 data
  float temperature = 0.0, humidity = 0.0, pressure = 0.0;
  if (bme.performReading()) {
    temperature = bme.temperature;
    humidity = bme.humidity;
    pressure = bme.pressure / 100.0;

    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
    Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
  }

  // Read battery and solar voltages
  float batteryVoltage = analogRead(BATTERY_PIN) * 0.0066; // Convert ADC value
  float solarVoltage = analogRead(SOLAR_PIN) * 0.0066; // Convert ADC value
  Serial.print("Battery Voltage: "); Serial.println(batteryVoltage);
  Serial.print("Solar Voltage: "); Serial.println(solarVoltage);

  // BLE transmission every 10 seconds
  if (millis() - lastBLETransmission >= BLE_INTERVAL) {
    lastBLETransmission = millis();

    windDirectionChar.write((uint8_t*)&windDirection, sizeof(windDirection));
    windSpeedChar.write((uint8_t*)&windSpeed, sizeof(windSpeed));
    gustSpeedChar.write((uint8_t*)&gustSpeed, sizeof(gustSpeed));
    batteryVoltageChar.write((uint8_t*)&batteryVoltage, sizeof(batteryVoltage));
    solarVoltageChar.write((uint8_t*)&solarVoltage, sizeof(solarVoltage));
    temperatureChar.write((uint8_t*)&temperature, sizeof(temperature));
    humidityChar.write((uint8_t*)&humidity, sizeof(humidity));
    pressureChar.write((uint8_t*)&pressure, sizeof(pressure));

    Serial.println("BLE Data Transmitted");
  }

  delay(100);
}
