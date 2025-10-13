#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// ----
#define ONE_WIRE_PIN 18     // Chân cảm biến DS18B20
#define NODE_ID 1           // ID của node cảm biến

// DS18B20
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// MPU6050
MPU6050 mpu(Wire);

// ----
const char* ssid = "DESKTOP-FTLL525 2479";
const char* password = "9z7-6B47";
const int udpPort = 8888;

WiFiUDP udp;

// ----
// ----

// Kết nối Wi-Fi
void connectWiFi() {
  Serial.println("Đang kết nối WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
}

// Khởi tạo cảm biến nhiệt độ
void initDS18B20() {
  sensors.begin();
  Serial.println("DS18B20 đã sẵn sàng!");
}

// Khởi tạo cảm biến MPU6050
void initMPU6050() {
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  Serial.println("Đang khởi tạo MPU6050...");
  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("MPU6050 đã sẵn sàng!");
}

// ----
// ----

// Đọc nhiệt độ từ DS18B20
float readTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

// Đọc dữ liệu từ MPU6050
void readMPU6050(float &accX, float &accY, float &accZ,
                 float &gyroX, float &gyroY, float &gyroZ,
                 float &angleX, float &angleY, float &angleZ) {
  mpu.update();
  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
  angleX = mpu.getAngleX();
  angleY = mpu.getAngleY();
  angleZ = mpu.getAngleZ();
}

// ----
// ----

// Tính địa chỉ broadcast dựa trên IP hiện tại
IPAddress getBroadcastIP() {
  IPAddress broadcastIP;
  for (int i = 0; i < 4; i++) {
    broadcastIP[i] = WiFi.localIP()[i] | (~WiFi.subnetMask()[i]);
  }
  return broadcastIP;
}

// Gửi dữ liệu qua UDP
void sendUDPData(IPAddress broadcastIP, float tempC,
                 float accX, float accY, float accZ,
                 float gyroX, float gyroY, float gyroZ,
                 float angleX, float angleY, float angleZ) {
  String data = String(NODE_ID) + "," +
                String(tempC) + "," +
                String(accX) + "," + String(accY) + "," + String(accZ) + "," +
                String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + "," +
                String(angleX) + "," + String(angleY) + "," + String(angleZ);

  udp.beginPacket(broadcastIP, udpPort);
  udp.print(data);
  udp.endPacket();

  Serial.print("Đã gửi tới ");
  Serial.print(broadcastIP);
  Serial.println(":");
  Serial.println("--------------------------------");
  Serial.println(udpPort);
}

// ----
// ----
void setup() {
  Serial.begin(115200);
  connectWiFi();
  udp.begin(udpPort);
  initDS18B20();
  initMPU6050();
}

void loop() {
  // Đọc cảm biếnggitgggg
  float tempC = readTemperature();
  float accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ;
  readMPU6050(accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ);

  // In ra Serial
  Serial.print("Node ID: "); Serial.println(NODE_ID);
  Serial.print("Nhiệt độ: "); Serial.println(tempC);
  Serial.print("Gia tốc: "); Serial.print(accX); Serial.print(", "); Serial.print(accY); Serial.print(", "); Serial.println(accZ);
  Serial.print("Gyro: "); Serial.print(gyroX); Serial.print(", "); Serial.print(gyroY); Serial.print(", "); Serial.println(gyroZ);
  Serial.print("Góc: "); Serial.print(angleX); Serial.print(", "); Serial.print(angleY); Serial.print(", "); Serial.println(angleZ);


  // Gửi dữ liệu
  IPAddress broadcastIP = getBroadcastIP();
  sendUDPData(broadcastIP, tempC, accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ);

  delay(3000);
}
