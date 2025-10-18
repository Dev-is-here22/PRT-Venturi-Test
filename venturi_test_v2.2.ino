/*
 ** SDO - pin 11
 ** SDI - pin 12
 ** CLK - pin 13
 ** CS  - pin 4
*/

#include <SPI.h>
#include <SD.h>

// ======================== Pins & Constants ========================
#define pressure1_pin 76
#define pressure2_pin 77
#define pressure3_pin 78
#define pressure4_pin 79
#define flowMeter_pin 7
#define PIN_1WIRE 8
#define chipSelect 4

#define psi1500 103.4214
#define psi1000 68.9476
#define ADC_resolution 16

// ======================== Globals: Debug Control ========================
unsigned long lastSerialPrint = 0;
const unsigned long SERIAL_PRINT_INTERVAL = 125; // ms

// ======================== Globals: Analog / Pressure ========================
double maxAnalogRead;

float pressure1, pressure2, pressure3, pressure4;
float baseline1 = 0, baseline2 = 0, baseline3 = 0, baseline4 = 0;
float total1 = 0, total2 = 0, total3 = 0, total4 = 0;
float p4_p3 = 0;

unsigned int sample = 0;
const unsigned int samples = 1;  
const int loops = 100000;        // baseline averaging loops

// ======================== Globals: Flow / Timing ========================
volatile double waterFlow = 0;          // cumulative liters (Hall)
volatile unsigned long pulseCount = 0;  // pulses since last interval

float flowRate = 0.0f;   // instantaneous Hall flow (L/s)

double absolute_time = 0;             // cumulative ms as double
unsigned long startingTime = 0;       // last time mark (ms)
static const unsigned READ_PERIOD_MS = 125; // 8 Hz polling window
unsigned long elapsed_readFlow;
unsigned long elapsed_print;

// ======================== Globals: UFM-01 (1-Wire) ========================
float infl_lps = 0.0f;  // instantaneous flow (L/s) from UFM
float temp_C   = 0.0f;  // temperature from UFM
float cufl_L   = 0.0f;  // cumulative liters from UFM

uint32_t infl_raw;
uint32_t temp_raw;
uint32_t cufl_raw;

// ======================== Globals: SD & Buffering ========================
File dataFile;
String filename;

const uint16_t BUFFER_SIZE = 1028;
char writeBuffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;

// ======================================================================
//                          1-Wire Low-Level
// ======================================================================

inline void ow_driveLow() { pinMode(PIN_1WIRE, OUTPUT); digitalWrite(PIN_1WIRE, LOW); }
inline void ow_release()  { pinMode(PIN_1WIRE, INPUT_PULLUP); } // assumes external 4.7k to 5V
inline int  ow_read()     { return digitalRead(PIN_1WIRE); }

// Reset + presence detect
// Master low ~500 us, release; sample presence ~70 us after release.
bool ow_reset() {
  ow_driveLow();
  delayMicroseconds(500);            // reset low
  ow_release();
  delayMicroseconds(70);             // wait for presence window
  bool presence = (ow_read() == LOW); // slave pulls low for 60–240 us
  delayMicroseconds(430);            // finish the reset slot
  return presence;
}

// Write a single bit
// '1': pull low ~3 us then release (slot ~60 us)
// '0': pull low ~60 us then release.
void ow_writeBit(bool v) {
  if (v) {
    ow_driveLow();
    delayMicroseconds(3);
    ow_release();
    delayMicroseconds(57);
  } else {
    ow_driveLow();
    delayMicroseconds(60);
    ow_release();
    delayMicroseconds(10);
  }
}

// Read a single bit
// Pull low ~4 us, release, sample ~11–15 us, finish to 60 us slot.
uint8_t ow_readBit() {
  uint8_t bit;
  ow_driveLow();
  delayMicroseconds(4);
  ow_release();
  delayMicroseconds(11);
  bit = (ow_read() == HIGH) ? 1 : 0;
  delayMicroseconds(45);
  return bit;
}

void ow_writeByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    ow_writeBit(b & 0x01);
    b >>= 1; // LSB first
  }
}

uint8_t ow_readByte() {
  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t bit = ow_readBit();
    b |= (bit << i); // LSB first
  }
  return b;
}

// ======================== CRC-8 ========================
uint8_t crc8_ds(const uint8_t* val, uint8_t length) {
  uint8_t crc_val = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc_val ^= val[i];
    for (uint8_t t = 8; t > 0; --t) {
      if (crc_val & 0x80) { crc_val = (uint8_t)((crc_val << 1) ^ 0x31u); }
      else { crc_val <<= 1; }
    }
  }
  return crc_val;
}

// ======================================================================
//                       UFM-01 High-Level Read
// ======================================================================
// Command 0x5B, start register 0x30, then read 12 bytes:
// [InFl_L, InFl_M, InFl_H, InFl_CRC,
//  T_L,    T_M,    T_H,    T_CRC,
//  CuFl_L, CuFl_M, CuFl_H, CuFl_CRC]
bool ufm_read_block(uint8_t* buf12) {
  if (!ow_reset()) return false;
  ow_writeByte(0x5B);
  ow_writeByte(0x30);
  for (uint8_t i = 0; i < 12; i++) {
    buf12[i] = ow_readByte();
  }
  ow_reset(); // end session cleanly (optional in original code)
  return true;
}

// ======================================================================
//                      SD: Unique File + Flush
// ======================================================================
File createUniqueFile() {
  int fileIndex = 1;
  const String baseName = "Test";
  const String extension = ".csv";
  String fileName;

  // Find the first free filename
  do {
    fileName = baseName + String(fileIndex) + extension;
    fileIndex++;
  } while (SD.exists(fileName));

  // Create and write header once
  File newFile = SD.open(fileName, FILE_WRITE);
  if (!newFile) {
    Serial.println("Error creating file!");
    while (true);
  }
  newFile.println("Pressure1(bar),Pressure2(bar),Pressure3(bar),Pressure4(bar),P4/P3,HallLiters(L),HallFlowRate(L/s),USLiters(L),USFlowRate(L/s),Temperature(°C),Time(ms)");
  newFile.close();

  // Remember the name and reopen for appends
  filename = fileName;
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Failed to open file for logging.");
    while (true);
  }

  Serial.print("Logging to: ");
  Serial.println(filename);
  return dataFile;
}

void tryFlush(bool force = false) {
  static unsigned long lastFlush = 0;
  if (force || (millis() - lastFlush) >= 1000 || bufferIndex > BUFFER_SIZE - 128) {
    if (bufferIndex > 0) {
      dataFile.write((uint8_t*)writeBuffer, bufferIndex);
      bufferIndex = 0;
    }
    dataFile.flush();
    lastFlush = millis();
  }
}

// ======================================================================
//                           Calibration
// ======================================================================
void pressureCalibration() {
  // Baseline calibration
  for (int i = 0; i < loops; i++) {
    total1 += analogRead(pressure1_pin);
    total2 += analogRead(pressure2_pin);
    total3 += analogRead(pressure3_pin);
    total4 += analogRead(pressure4_pin);
  }

  baseline1 = total1 / loops;
  baseline2 = total2 / loops;
  baseline3 = total3 / loops;
  baseline4 = total4 / loops;

  Serial.print("Baselines: ");
  Serial.print(baseline1); Serial.print(", ");
  Serial.print(baseline2); Serial.print(", ");
  Serial.print(baseline3); Serial.print(", ");
  Serial.println(baseline4);

  total1 = total2 = total3 = total4 = 0;
}

// ======================================================================
//                           Interrupts
// ======================================================================
void pulse() {
  pulseCount++;
  waterFlow += 1.0 / 63.0; // 63 pulses = 1 L
}

// ======================================================================
//                               Setup
// ======================================================================
void setup() {
  pinMode(pressure1_pin, INPUT);
  pinMode(pressure2_pin, INPUT);
  pinMode(pressure3_pin, INPUT);
  pinMode(pressure4_pin, INPUT);
  pinMode(flowMeter_pin, INPUT_PULLUP);

  // Analog read resolution
  analogReadResolution(ADC_resolution);
  maxAnalogRead = (pow(2, ADC_resolution)) - 1;

  ow_release(); // idle 1-Wire bus

  Serial.begin(2000000);
  while (!Serial) { /* wait for USB serial */ }
  Serial.println(F("USB connected"));

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed.");
    while (true);
  }
  Serial.println("initialization done.");

  attachInterrupt(digitalPinToInterrupt(flowMeter_pin), pulse, RISING);

  pressureCalibration();
  dataFile = createUniqueFile();

  startingTime = millis();
}

// ======================================================================
//                               Loop
// ======================================================================
void loop() {
  // Accumulate raw analog reads
  total1 += analogRead(pressure1_pin);
  total2 += analogRead(pressure2_pin);
  total3 += analogRead(pressure3_pin);
  total4 += analogRead(pressure4_pin);
  sample++;

  if (sample >= samples) {
    // Averages
    float avg1 = total1 / sample;
    float avg2 = total2 / sample;
    float avg3 = total3 / sample;
    float avg4 = total4 / sample;

    // Convert to pressure (bar) using same formula and baselines
    pressure1 = (avg1 - baseline1) * psi1500 / (maxAnalogRead - baseline1) + 1;
    pressure2 = (avg2 - baseline2) * psi1000 / (maxAnalogRead - baseline2) + 1;
    pressure3 = (avg3 - baseline3) * psi1500 / (maxAnalogRead - baseline3) + 1;
    pressure4 = (avg4 - baseline4) * psi1500 / (maxAnalogRead - baseline4) + 1;
    if (pressure3 > 0){
      p4_p3 = pressure4/pressure3;
    }

    // Timing
    unsigned long now = millis();
    unsigned long elapsed = now - startingTime;
    elapsed_readFlow += elapsed;
    elapsed_print += elapsed;
    absolute_time += elapsed;

    // UFM read every > READ_PERIOD_MS (unchanged)
    if (elapsed_readFlow > READ_PERIOD_MS) {
      elapsed_readFlow = 0;
      uint8_t b[12] = {0};
      bool ok = ufm_read_block(b);
      if (!ok) {
        Serial.println(F("No presence / 1-Wire reset failed."));
        return;
      }

      // Parse raw values (unchanged endianness & fields)
      infl_raw = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
      temp_raw = (uint32_t)b[4] | ((uint32_t)b[5] << 8) | ((uint32_t)b[6] << 16);
      cufl_raw = (uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16);

      // Scaling (unchanged)
      // Instantaneous flow: 0.01 l/h per count
      // Temperature:        0.01 °C per count
      // Cumulative:         0.1 l per count
      infl_lps = (infl_raw * 0.01f) / 3600; // l/s
      temp_C   = temp_raw * 0.01f;          // °C
      cufl_L   = (cufl_raw * 0.1f) - 42;          // L
    }
    startingTime = now;

    // Instantaneous Hall flow (L/s)
    if (elapsed > 0) {
      flowRate = (pulseCount / 63.0) / (elapsed / 1000.0);
    } else {
      flowRate = 0;
    }
    pulseCount = 0;

    // CSV line (unchanged format and order)
    int n = snprintf(
      writeBuffer + bufferIndex,
      BUFFER_SIZE - bufferIndex,
      "%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.0f\n",
      pressure1, pressure2, pressure3, pressure4, p4_p3,
      waterFlow, flowRate, cufl_L, infl_lps, temp_C, absolute_time
    );
    if (n > 0) bufferIndex += n;

    // Flush if needed
    tryFlush();

    // Debug prints (only every SERIAL_PRINT_INTERVAL ms)
    if (elapsed_print > SERIAL_PRINT_INTERVAL) {
      elapsed_print = 0;

      Serial.print("P1: ");         Serial.println(pressure1);
      Serial.print("P2: ");         Serial.println(pressure2);
      Serial.print("P3: ");         Serial.println(pressure3);
      Serial.print("P4: ");         Serial.println(pressure4);
      Serial.print("P4/P3: ");      Serial.println(p4_p3);
      Serial.print("Total Hall: "); Serial.println(waterFlow);
      Serial.print("Flow Hall: ");  Serial.println(flowRate);
      Serial.print("Total US: ");   Serial.println(cufl_L);
      Serial.print("Flow US: ");    Serial.println(infl_lps);
      Serial.print("Temp: ");       Serial.println(temp_C);
      Serial.print("T: ");          Serial.println(absolute_time);
      Serial.println();
    }

    // Reset accumulators
    total1 = total2 = total3 = total4 = 0;
    sample = 0;
  }
}