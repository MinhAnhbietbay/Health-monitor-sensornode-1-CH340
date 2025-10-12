#include <WiFi.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_PIN 18    // Chân cảm biến DS18B20
#define NODE_ID 1         // ID của node cảm biến

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
 
const char* ssid = "DESKTOP-FTLL525 2479";
const char* password = "9z7-6B47";

const int udpPort = 8888;                 // Cổng UDP

WiFiUDP udp;

void setup() {
  Serial.begin(115200);

  // Kết nối Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nSensor Node Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());

  udp.begin(udpPort);
  sensors.begin(); // Khởi tạo DS18B20
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.println(tempC);

  // Tính broadcast IP tự động
  IPAddress broadcastIP;
  for (int i = 0; i < 4; i++) {
    broadcastIP[i] = WiFi.localIP()[i] | (~WiFi.subnetMask()[i]);
  }

  // Gửi dữ liệu qua UDP (format: NODE_ID,temp)
  String data = String(NODE_ID) + "," + String(tempC);
  udp.beginPacket(broadcastIP, udpPort);
  udp.print(data);
  udp.endPacket();

  Serial.print("Sent to ");
  Serial.print(broadcastIP);
  Serial.print(":");
  Serial.println(udpPort);
  delay(3000); // Gửi mỗi giây
}