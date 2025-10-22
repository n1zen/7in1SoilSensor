#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial mySerial(2, 3);  // RX, TX
#define RE 7
#define DE 8

void setup() {
  Serial.begin(9600);
  mySerial.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  delay(500);
}

void loop() {
  byte queryData[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  byte receivedData[19];
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  mySerial.write(queryData, sizeof(queryData));  // Send the query data to the NPK sensor
  mySerial.flush();
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  delay(1000);  // Wait for 1 second

  if (mySerial.available() >= sizeof(receivedData)) {  // Check if there are enough bytes available to read
    mySerial.readBytes(receivedData, sizeof(receivedData));  // Read the received data into the receivedData array

    // Parse and print the received data in decimal format
    unsigned int soilHumidity = (receivedData[3] << 8) | receivedData[4];
    unsigned int soilTemperature = (receivedData[5] << 8) | receivedData[6];
    unsigned int soilConductivity = (receivedData[7] << 8) | receivedData[8];
    unsigned int soilPH = (receivedData[9] << 8) | receivedData[10];
    unsigned int nitrogen = (receivedData[11] << 8) | receivedData[12];
    unsigned int phosphorus = (receivedData[13] << 8) | receivedData[14];
    unsigned int potassium = (receivedData[15] << 8) | receivedData[16];

    JsonDocument doc;

    float humidity = (float)soilHumidity / 10.0;
    float temperature = (float)soilTemperature / 10.0;
    float ph = (float)soilPH / 10.0;

    doc["Moist"] = humidity;
    doc["Temp"] = temperature;
    doc["EC"] = soilConductivity;
    doc["pH"] = ph;
    doc["nitrogen"] = nitrogen;
    doc["phosphorus"] = phosphorus;
    doc["potassium"] = potassium;

    serializeJson(doc, Serial);
    Serial.println();
  }
}

