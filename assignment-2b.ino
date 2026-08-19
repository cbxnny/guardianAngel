#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

// BLE Configuration
#define SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define ACCEL_CHAR_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"
#define CONFIG_CHAR_UUID    "19B10002-E8F2-537E-4F6C-D104768A1214"
#define STATUS_CHAR_UUID    "19B10003-E8F2-537E-4F6C-D104768A1214"

// Sampling Configuration
#define SAMPLING_RATE_HZ    50
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLING_RATE_HZ)
#define ACCEL_RANGE_G       8
#define RING_BUFFER_SIZE    250  // 5 seconds at 50Hz

// Threshold for burst mode trigger (in milli-g)
#define IMPACT_THRESHOLD_MG 3000  // 3g impact

// Data structure for accelerometer sample (8 bytes)
struct AccelSample {
  uint16_t sequence;
  int16_t ax;  // milli-g
  int16_t ay;  // milli-g
  int16_t az;  // milli-g
} __attribute__((packed));

// BLE Service and Characteristics
BLEService guardianService(SERVICE_UUID);
BLECharacteristic accelChar(ACCEL_CHAR_UUID, BLERead | BLENotify, sizeof(AccelSample) * 3);
BLECharacteristic configChar(CONFIG_CHAR_UUID, BLERead | BLEWrite, 20);
BLECharacteristic statusChar(STATUS_CHAR_UUID, BLERead | BLENotify, 20);

// Ring buffer for storing recent samples
AccelSample ringBuffer[RING_BUFFER_SIZE];
int ringBufferIndex = 0;
uint16_t sequenceNumber = 0;

// Timing variables
unsigned long lastSampleTime = 0;
bool isStreaming = false;
bool burstMode = false;
unsigned long burstStartTime = 0;
#define BURST_DURATION_MS 3000  // 3 seconds of burst after threshold

// Battery monitoring
float batteryVoltage = 3.7;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for Serial
  
  Serial.println("GuardianAngel Fall Detection System");
  Serial.println("Initializing...");
  
  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize IMU!");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }
  
  Serial.println("IMU initialized successfully");
  Serial.print("Accelerometer sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("ERROR: Failed to initialize BLE!");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }
  
  // Set BLE parameters
  BLE.setLocalName("GuardianAngel");
  BLE.setDeviceName("GuardianAngel-Sensor");
  BLE.setAdvertisedService(guardianService);
  
  // Add characteristics to service
  guardianService.addCharacteristic(accelChar);
  guardianService.addCharacteristic(configChar);
  guardianService.addCharacteristic(statusChar);
  
  // Add service
  BLE.addService(guardianService);
  
  // Set initial values
  updateStatusChar();
  
  // Start advertising
  BLE.advertise();
  
  Serial.println("BLE advertising started");
  Serial.println("Device Name: GuardianAngel");
  Serial.println("Waiting for connection...");
  
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Handle BLE connections
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    
    isStreaming = true;
    
    while (central.connected()) {
      unsigned long currentTime = millis();
      
      // Sample at specified rate
      if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = currentTime;
        
        if (IMU.accelerationAvailable()) {
          float ax, ay, az;
          IMU.readAcceleration(ax, ay, az);
          
          // Convert to milli-g
          AccelSample sample;
          sample.sequence = sequenceNumber++;
          sample.ax = (int16_t)(ax * 1000);
          sample.ay = (int16_t)(ay * 1000);
          sample.az = (int16_t)(az * 1000);
          
          // Store in ring buffer
          ringBuffer[ringBufferIndex] = sample;
          ringBufferIndex = (ringBufferIndex + 1) % RING_BUFFER_SIZE;
          
          // Calculate magnitude for threshold detection
          float magnitude = sqrt(ax*ax + ay*ay + az*az) * 1000;
          
          // Check for impact threshold
          if (magnitude > IMPACT_THRESHOLD_MG && !burstMode) {
            burstMode = true;
            burstStartTime = currentTime;
            Serial.println("IMPACT DETECTED! Entering burst mode");
          }
          
          // Exit burst mode after duration
          if (burstMode && (currentTime - burstStartTime > BURST_DURATION_MS)) {
            burstMode = false;
            Serial.println("Exiting burst mode");
          }
          
          // Send notification
          accelChar.writeValue((uint8_t*)&sample, sizeof(AccelSample));
          
          // Debug output (reduced frequency)
          if (sequenceNumber % 50 == 0) {
            Serial.print("Seq: ");
            Serial.print(sample.sequence);
            Serial.print(" | Accel (mg): X=");
            Serial.print(sample.ax);
            Serial.print(" Y=");
            Serial.print(sample.ay);
            Serial.print(" Z=");
            Serial.print(sample.az);
            Serial.print(" | Mag=");
            Serial.println(magnitude);
          }
        }
      }
      
      // Handle configuration writes
      if (configChar.written()) {
        handleConfigWrite();
      }
      
      // Update status periodically
      if (currentTime % 5000 < 50) {
        updateStatusChar();
      }
    }
    
    Serial.println("Disconnected from central");
    digitalWrite(LED_BUILTIN, LOW);
    isStreaming = false;
  }
  
  // Blink LED when advertising
  if (!isStreaming) {
    digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
  }
}

void handleConfigWrite() {
  uint8_t config[20];
  int len = configChar.readValue(config, 20);
  
  if (len > 0) {
    Serial.print("Config received: ");
    for (int i = 0; i < len; i++) {
      Serial.print(config[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    // Simple command protocol
    // 0x01: Start streaming
    // 0x02: Stop streaming
    // 0x03: Reset sequence
    
    if (config[0] == 0x01) {
      isStreaming = true;
      Serial.println("Streaming started");
    } else if (config[0] == 0x02) {
      isStreaming = false;
      Serial.println("Streaming stopped");
    } else if (config[0] == 0x03) {
      sequenceNumber = 0;
      Serial.println("Sequence reset");
    }
  }
}

void updateStatusChar() {
  // Status format: [battery_level(1byte), firmware_version(3bytes), error_code(1byte)]
  uint8_t status[20] = {0};
  
  // Battery level (0-100%)
  status[0] = (uint8_t)(batteryVoltage / 4.2 * 100);
  
  // Firmware version (e.g., 1.0.0)
  status[1] = 1;  // Major
  status[2] = 0;  // Minor
  status[3] = 0;  // Patch
  
  // Error code (0 = no error)
  status[4] = 0;
  
  // Current sample rate
  status[5] = SAMPLING_RATE_HZ;
  
  // Burst mode flag
  status[6] = burstMode ? 1 : 0;
  
  statusChar.writeValue(status, 7);
}