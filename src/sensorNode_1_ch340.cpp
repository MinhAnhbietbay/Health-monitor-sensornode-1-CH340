#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// ---- Cấu hình phần cứng ----
#define ONE_WIRE_PIN 18
#define NODE_ID 1
#define BUFFER_SIZE 100  // MAX30102

// DS18B20
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// MPU6050
MPU6050 mpu(Wire);

// MAX30102
MAX30105 particleSensor;
uint32_t redBuffer[BUFFER_SIZE];
uint32_t irBuffer[BUFFER_SIZE];
int32_t hr = 0, spo2 = 0;
int8_t validHR = 0, validSpO2 = 0;

// ---- WiFi + UDP ----
const char* ssid = "DESKTOP-FTLL525 2479";
const char* password = "9z7-6B47";
const int udpPort = 8888;
WiFiUDP udp;

// ---- WiFi ----
void connectWiFi() {
  Serial.println("Đang kết nối WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: "); Serial.println(WiFi.localIP());
}

// ---- Init cảm biến ----
void initDS18B20() { sensors.begin(); }

void initMPU6050() {
  Wire.begin(21, 22);
  mpu.begin();
  mpu.calcGyroOffsets(true);
}

void initMAX30102() {
  Wire.begin(21, 22);
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Khong tim thay MAX30102!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

// ---- Đọc cảm biến ----
float readTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

void readMPU6050(float &accX, float &accY, float &accZ,
                 float &gyroX, float &gyroY, float &gyroZ,
                 float &angleX, float &angleY, float &angleZ) {
  mpu.update();
  accX = mpu.getAccX(); accY = mpu.getAccY(); accZ = mpu.getAccZ();
  gyroX = mpu.getGyroX(); gyroY = mpu.getGyroY(); gyroZ = mpu.getGyroZ();
  angleX = mpu.getAngleX(); angleY = mpu.getAngleY(); angleZ = mpu.getAngleZ();
}

void readMAX30102() {
  // Đọc 100 mẫu MAX30102
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.check());
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE,
                                         redBuffer, &spo2, &validSpO2,
                                         &hr, &validHR);
}

// ---- Gửi UDP ----
IPAddress getBroadcastIP() {
  IPAddress broadcastIP;
  for (int i = 0; i < 4; i++)
    broadcastIP[i] = WiFi.localIP()[i] | (~WiFi.subnetMask()[i]);
  return broadcastIP;
}

void sendUDPData(IPAddress broadcastIP, float tempC,
                 float accX, float accY, float accZ,
                 float gyroX, float gyroY, float gyroZ,
                 float angleX, float angleY, float angleZ,
                 int hr, int spo2) {
  String data = String(NODE_ID) + "," +
                String(tempC) + "," +
                String(accX) + "," + String(accY) + "," + String(accZ) + "," +
                String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + "," +
                String(angleX) + "," + String(angleY) + "," + String(angleZ) + "," +
                String(hr) + "," + String(spo2);

  udp.beginPacket(broadcastIP, udpPort);
  udp.print(data);
  udp.endPacket();

  Serial.println("Đã gửi: " + data);
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  connectWiFi();
  udp.begin(udpPort);
  initDS18B20();
  initMPU6050();
  initMAX30102();
}

// ---- Loop ----
void loop() {
  float tempC = readTemperature();
  float accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ;
  readMPU6050(accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ);

  readMAX30102();

  IPAddress broadcastIP = getBroadcastIP();
  sendUDPData(broadcastIP, tempC,
              accX, accY, accZ,
              gyroX, gyroY, gyroZ,
              angleX, angleY, angleZ,
              hr, spo2);

  delay(3000);
}
