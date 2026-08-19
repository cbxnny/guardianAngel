#include <ArduinoBLE.h>
#include "Arduino_LSM6DS3.h"
#include "DHT.h"

#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// BLE UUIDs
#define SERVICE_UUID "7be9de62-591e-41d8-8d58-c88651f54e36"
#define TEMP_CHAR_UUID "bf6b74a8-062b-4737-831b-9a9b84023b3b"
#define HUMIDITY_CHAR_UUID "c49f637b-8f32-4c6d-8163-1ad46b3b2f60"
#define COMMAND_CHAR_UUID "6ca8f9c2-369e-442a-b455-d1248f3367f5"
#define ACCEL_CHAR_UUID "6af5cfd5-b6f0-4f57-9285-4fbb57146706"

BLEService sensorService(SERVICE_UUID);
BLEFloatCharacteristic tempChar(TEMP_CHAR_UUID, BLERead);
BLEByteCharacteristic humidityChar(HUMIDITY_CHAR_UUID, BLERead);
BLEByteCharacteristic commandChar(COMMAND_CHAR_UUID, BLERead | BLEWrite);
BLECharacteristic accelChar(ACCEL_CHAR_UUID, BLERead | BLENotify, 20);

bool imuActive = false;
unsigned long imuStartTime = 0;
const int IMU_DURATION = 3000;  //3 sec

void setup() {
  Serial.begin(9600);
  while (!Serial)

  dht.begin();

  if (!IMU.begin()) {
    Serial.println("IMU initialization failed!");
  }

  // BLE Setup
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1)
      ;
  }

  BLE.setLocalName("Ben_Sensor");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(tempChar);
  sensorService.addCharacteristic(humidityChar);
  sensorService.addCharacteristic(commandChar);
  sensorService.addCharacteristic(accelChar);
  BLE.addService(sensorService);
  commandChar.writeValue(0);
  BLE.advertise();

  Serial.println("BLE Active. Waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Read DHT11 every 60s and print to Serial
      static unsigned long lastDHTRead = 0;
      if (millis() - lastDHTRead >= 60000) {
        float temp = dht.readTemperature();
        float humidity = dht.readHumidity();

        if (!isnan(temp)) {
          tempChar.writeValue(temp);
          Serial.print("Temperature: ");
          Serial.print(temp);
          Serial.println(" °C");
        }

        if (!isnan(humidity)) {
          uint8_t humidityInt = (uint8_t)humidity;  // convert to int
          humidityChar.writeValue(humidityInt);
          Serial.print("Humidity: ");
          Serial.print(humidity);
          Serial.println(" %");
        }

        lastDHTRead = millis();
      }

      // IMU when command is received
      if (commandChar.written()) {
        byte commandValue = commandChar.value();
        Serial.print("Command received: ");
        Serial.println(commandValue);
        
        if (commandValue == 1 && !imuActive) {
          imuActive = true;
          imuStartTime = millis();
          Serial.println("IMU activated for 3 seconds!");
        }
      }

      // IMU when active
      if (imuActive) {
        if (millis() - imuStartTime <= IMU_DURATION) {
          static unsigned long lastIMURead = 0;
          if (millis() - lastIMURead >= 100) { // 10Hz
            float x, y, z;
            if (IMU.accelerationAvailable()) {
              IMU.readAcceleration(x, y, z);
              int16_t packedData[3] = {
                (int16_t)(x * 1000),
                (int16_t)(y * 1000),
                (int16_t)(z * 1000)
              };
              accelChar.writeValue((byte*)packedData, sizeof(packedData));
              
              
              Serial.print("Accel: X=");
              Serial.print(x);
              Serial.print(" Y=");
              Serial.print(y);
              Serial.print(" Z=");
              Serial.println(z);
            }
            lastIMURead = millis();
          }
        } else {
          imuActive = false;
          commandChar.writeValue(0);  
          Serial.println("IMU session ended.");
        }
      }
    }
    
    Serial.println("Disconnected.");
  }
}