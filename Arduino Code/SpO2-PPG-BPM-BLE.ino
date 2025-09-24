#include <Wire.h>
#include <Arduino.h>
#include <bluefruit.h>        // Adafruit Bluefruit nRF52 library
#include "spo2_algorithm.h"   // SparkFun repo

// MAX30102 register map
#define MAX30102_ADDR 0x57
#define REG_INT_STATUS_1   0x00
#define REG_INT_ENABLE_1   0x02
#define REG_MODE_CONFIG    0x09
#define REG_SPO2_CONFIG    0x0A
#define REG_LED1_PA        0x0C
#define REG_LED2_PA        0x0D
#define REG_FIFO_DATA      0x07

#define PPG_RDY_BIT        (1 << 6)
#define INT_PIN A3

volatile bool newDataReady = false;

// Buffers for SpO2 algorithm
//#define BUFFER_SIZE 100   // about 5 sec @ ~20Hz
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// Algorithm results
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// -------- Filtering support --------
#define RAW_AVG_SIZE 4
uint32_t irAvgBuf[RAW_AVG_SIZE] = {0};
uint32_t redAvgBuf[RAW_AVG_SIZE] = {0};
int irAvgIndex = 0, redAvgIndex = 0;
uint64_t irSum = 0, redSum = 0;

uint32_t irHist[3] = {0};
uint32_t redHist[3] = {0};

float smoothedHR = 0;
float smoothedSpO2 = 0;

// -------- BLE Service & Characteristics --------
BLEService bioService("180D");   // Custom Bio Service, can use 128-bit UUID

BLECharacteristic bpmChar("2A37");   // BPM
BLECharacteristic spo2Char("2A5F");  // SpO2
BLECharacteristic irChar("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");   // Custom UUID for IR
BLECharacteristic redChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");  // Custom UUID for RED

// -------- Helper functions --------
uint32_t movingAverage(uint32_t newVal, uint32_t *buffer, int &index, int size, uint64_t &sum) {
  sum -= buffer[index];
  buffer[index] = newVal;
  sum += newVal;
  index = (index + 1) % size;
  return (uint32_t)(sum / size);
}

uint32_t median3(uint32_t a, uint32_t b, uint32_t c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  else if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  else return c;
}

float emaFilter(float prev, float newVal, float alpha) {
  return alpha * newVal + (1.0f - alpha) * prev;
}

void sensorISR() {
  newDataReady = true;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX30102_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

// -------- BLE Setup --------
void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);   // Max TX power
  Bluefruit.setName("SpO2_Sensor");

  // Create service + characteristics
  bioService.begin();

  bpmChar.setProperties(CHR_PROPS_NOTIFY);
  bpmChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bpmChar.setFixedLen(2);  // uint16_t
  bpmChar.begin();

  spo2Char.setProperties(CHR_PROPS_NOTIFY);
  spo2Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  spo2Char.setFixedLen(1); // uint8_t
  spo2Char.begin();

  irChar.setProperties(CHR_PROPS_NOTIFY);
  irChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  irChar.setFixedLen(4);   // uint32_t
  irChar.begin();

  redChar.setProperties(CHR_PROPS_NOTIFY);
  redChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  redChar.setFixedLen(4);  // uint32_t
  redChar.begin();

  // Start advertising
  Bluefruit.Advertising.addService(bioService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start();
}

// -------- Arduino Setup --------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("MAX30102 + BLE GATT demo");

  Wire.begin();

  // Reset device
  writeRegister(REG_MODE_CONFIG, 0x40);
  delay(100);

  // Enable Data Ready interrupt
  writeRegister(REG_INT_ENABLE_1, PPG_RDY_BIT);

  // Set SpO2 mode
  writeRegister(REG_MODE_CONFIG, 0x03);

  // SpO2 config: sample rate
  writeRegister(REG_SPO2_CONFIG, 0x25);

  // LED amplitudes
  writeRegister(REG_LED1_PA, 0x20);  // Red
  writeRegister(REG_LED2_PA, 0x20);  // IR

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), sensorISR, FALLING);

  // Init BLE
  setupBLE();

  Serial.println("Setup complete, advertising...");
}

// -------- Arduino Loop --------
void loop() {
  if (newDataReady) {
    newDataReady = false;
    readRegister(REG_INT_STATUS_1); // clear interrupt

    // Read FIFO
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(REG_FIFO_DATA);
    Wire.endTransmission(false);

    Wire.requestFrom(MAX30102_ADDR, (uint8_t)6);
    if (Wire.available() == 6) {
      uint32_t ir = ((uint32_t)Wire.read() << 16) |
                    ((uint32_t)Wire.read() << 8) |
                    (Wire.read());
      ir &= 0x3FFFF;

      uint32_t red = ((uint32_t)Wire.read() << 16) |
                     ((uint32_t)Wire.read() << 8) |
                     (Wire.read());
      red &= 0x3FFFF;

      // Filtering
      irHist[0] = irHist[1]; irHist[1] = irHist[2]; irHist[2] = ir;
      redHist[0] = redHist[1]; redHist[1] = redHist[2]; redHist[2] = red;

      uint32_t irMed = median3(irHist[0], irHist[1], irHist[2]);
      uint32_t redMed = median3(redHist[0], redHist[1], redHist[2]);

      uint32_t irFilt = movingAverage(irMed, irAvgBuf, irAvgIndex, RAW_AVG_SIZE, irSum);
      uint32_t redFilt = movingAverage(redMed, redAvgBuf, redAvgIndex, RAW_AVG_SIZE, redSum);

      irBuffer[bufferIndex] = irFilt;
      redBuffer[bufferIndex] = redFilt;
      bufferIndex++;

      if (bufferIndex >= BUFFER_SIZE) {
        bufferIndex = 0;
        bufferFilled = true;
      }

      if (bufferFilled) {
        maxim_heart_rate_and_oxygen_saturation(
          irBuffer, BUFFER_SIZE,
          redBuffer,
          &spo2, &validSPO2,
          &heartRate, &validHeartRate
        );

        if (validHeartRate) smoothedHR = emaFilter(smoothedHR, (float)heartRate, 0.3f);
        if (validSPO2) smoothedSpO2 = emaFilter(smoothedSpO2, (float)spo2, 0.3f);

        // --- Send via BLE ---
        uint16_t bpm_val = (validHeartRate) ? (uint16_t)smoothedHR : 0;
        uint8_t spo2_val = (validSPO2) ? (uint8_t)smoothedSpO2 : 0;

        // Data
        //BPM
        bpmChar.notify(&bpm_val, sizeof(bpm_val));

        // SpO2
        spo2Char.notify(&spo2_val, sizeof(spo2_val));
        //IR

        irChar.notify(&irFilt, sizeof(irFilt));

        //Red
        redChar.notify(&redFilt, sizeof(redFilt));

        // Debug print
        Serial.print("IR="); Serial.print(irFilt);
        Serial.print(" | RED="); Serial.print(redFilt);
        Serial.print(" | BPM="); Serial.print(bpm_val);
        Serial.print(" | SpO2="); Serial.print(spo2_val);
        Serial.println("%");
      }
    }
  }
  delay(100);
}
