#include <SPI.h>
#include <SD.h>

// ======================= DEBUG & CONFIG =======================
#define DEBUG_PRINTS

// ======================= PIN DEFINITIONS =======================
#define SD_CHIP_SELECT              4
#define ULTRASONIC_FLOW_METER_PIN   8
#define HALL_FLOW_METER_PIN         53
#define PRESSURE_SENSOR_1_PIN       77    // A1
#define PRESSURE_SENSOR_2_PIN       78    // A2
#define PRESSURE_SENSOR_3_PIN       79    // A3
#define PRESSURE_SENSOR_4_PIN       80    // A4

// ======================= CONSTANTS =============================
#define PSI1500_TO_BAR          103.4214  // bar
#define PSI1000_TO_BAR          68.9476   // bar
#define MIN_CURRENT             0.004     // mA
#define MAX_CURRENT             0.02      // mA
#define ADC_BIT_RESOLUTION      16
#define BAUD_RATE               2000000   // bit/s
#define CALIBRATION_LOOPS       1
#define FLOW_READ_INTERVAL      20000     // µs
#define SERIAL_PRINT_INTERVAL   125000    // µs
#define PULSES_TO_LITER         63.0      // x pulses = 1 L
#define SD_BUFFER_SIZE          4096
#define SD_WRITE_SIZE           512

// ======================= GLOBAL VARIABLES ======================
// Flags
bool isTestRunning = false;         

// File
File dataFile;
String filename;
char writeBuffer[SD_BUFFER_SIZE];
uint32_t bufferIndex = 0;
uint32_t printSize;
bool sdReady = false;

// Pressure calibration
double calibrationZero1;
double calibrationZero2;
double calibrationZero3;
double calibrationZero4;
const double currentRange = MAX_CURRENT - MIN_CURRENT;

// Pressure sensors data
double poliToLinePressure1;
double poliToLinePressure2;
double upstreamPressure;
double downstreamPressure;
double downstreamToUpstreamRatio;

//UMF-01 data
uint32_t instantaneousFlowRaw;
uint32_t temperatureRaw;
uint32_t cumulativeFlowRaw;

double instantaneousFlow  = 0;
double temperature        = 0;
double cumulativeFlow     = 0;

// Hall flow meter data
uint16_t pulseCount = 0;
double cumulativeFlowHall;  

// Timing variables
uint32_t now;
uint32_t startingTime;
uint32_t loopDuration;
uint32_t passedFromStart;
uint32_t passedFromFlowRead;
uint32_t passedFromPrint;

static inline void driveLow() { pinMode(ULTRASONIC_FLOW_METER_PIN, OUTPUT); digitalWrite(ULTRASONIC_FLOW_METER_PIN, LOW); }
static inline void driveHigh()  { pinMode(ULTRASONIC_FLOW_METER_PIN, INPUT_PULLUP); }



/*
===============================================================
============================ FILE =============================
===============================================================
*/

// Create a new SD file with unique name: Test1.csv, Test2.csv, ...
File createUniqueFile() {
  int fileIndex = 1;
  const String baseName = "Test";
  const String extension = ".csv";
  String fileName;

  do {
    fileName = baseName + String(fileIndex) + extension;
    fileIndex++;
  } while (SD.exists(fileName));

  File newFile = SD.open(fileName, FILE_WRITE);
  if (!newFile) {
    Serial.println("Error creating file!");
    while (true);
  }
  newFile.println("Pressure1(bar),Pressure2(bar),Pressure3(bar),Pressure4(bar),P4/P3,HallLiters(L),USLiters(L),USFlowRate(L/s),Temperature(°C),Time(µs)");
  newFile.close();

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

void trySdWrite() {
  while (bufferIndex >= SD_WRITE_SIZE) {
    if (bufferIndex > 0) {
      if (dataFile.write((uint8_t*)writeBuffer, SD_WRITE_SIZE) != SD_WRITE_SIZE) { 
        Serial.println("ERROR: failed to write data to the file");
        break;
      }
      bufferIndex -= SD_WRITE_SIZE;
      if (bufferIndex > 0) {
        memmove(writeBuffer, writeBuffer + SD_WRITE_SIZE, bufferIndex);
      }
    }
  }
}

void flushAll() {
  trySdWrite();
  if (bufferIndex > 0) {
    size_t written = dataFile.write((uint8_t*)writeBuffer, bufferIndex);
    if (written != bufferIndex) {
      Serial.println("ERROR: failed to write tail bytes to the file");
    }
    bufferIndex = 0;
  }
  dataFile.flush();
}



/*
===============================================================
=================== PRESSURE CALIBRATION ======================
===============================================================
*/

void pressureCalibration() {
  // Baseline calibration

  Serial.println("Calibrating pressure sensors...");

  double totalCalibrationSum1 = 0;
  double totalCalibrationSum2 = 0;
  double totalCalibrationSum3 = 0;
  double totalCalibrationSum4 = 0;

  for (uint32_t i = 0; i < CALIBRATION_LOOPS; i++) {

    totalCalibrationSum1 += analogRead(PRESSURE_SENSOR_1_PIN);
    totalCalibrationSum2 += analogRead(PRESSURE_SENSOR_2_PIN);
    totalCalibrationSum3 += analogRead(PRESSURE_SENSOR_3_PIN);
    totalCalibrationSum4 += analogRead(PRESSURE_SENSOR_4_PIN);
  }

  calibrationZero1 = totalCalibrationSum1 / CALIBRATION_LOOPS;
  calibrationZero2 = totalCalibrationSum2 / CALIBRATION_LOOPS;
  calibrationZero3 = totalCalibrationSum3 / CALIBRATION_LOOPS;
  calibrationZero4 = totalCalibrationSum4 / CALIBRATION_LOOPS;

  Serial.print("Pressure offsets: ");
  Serial.print(calibrationZero1); Serial.print(", ");
  Serial.print(calibrationZero2); Serial.print(", ");
  Serial.print(calibrationZero3); Serial.print(", ");
  Serial.println(calibrationZero4);
}



/*
===============================================================
========================= FLOW METERS =========================
===============================================================
*/

// Hall effect flow meter
inline void pulse() { pulseCount++; }


// UMF-01
bool resetUmf01() {         
  driveLow();
  delayMicroseconds(550);             
  driveHigh();
  delayMicroseconds(70);             
  bool presence = (digitalRead(ULTRASONIC_FLOW_METER_PIN) == LOW); 
  delayMicroseconds(430);            
  return presence;
}

void writeByte(uint8_t dataByte) {
  for (uint8_t i = 0; i < 8; i++) {
    if (dataByte & 0x01) {
      driveLow();
      driveHigh();
      delayMicroseconds(57);
    } else {
      driveLow();
      delayMicroseconds(60);
      driveHigh();
    }
    dataByte >>= 1; 
  }
}

uint8_t readByte() {
  uint8_t dataByte = 0;
  for (uint8_t i = 0; i < 8; i++) {
    driveLow();
    delayMicroseconds(4);
    driveHigh();
    delayMicroseconds(11);
    uint8_t pinState = (digitalRead(ULTRASONIC_FLOW_METER_PIN) == HIGH) ? 1 : 0;
    delayMicroseconds(45);
    dataByte |= (pinState << i); 
  }
  return dataByte;
}

bool readBlock(uint8_t* buf12) {
  if (!resetUmf01()) return false;
  writeByte(0x5B);
  writeByte(0x30);
  for (uint8_t i = 0; i < 12; i++) {
    buf12[i] = readByte();
  }
  resetUmf01();
  return true;
}



/*
===============================================================
======================= TEST CONTROL ==========================
===============================================================
*/

void startTest() {
  if (!isTestRunning) {
    Serial.println("Starting test...");

    if (!isSdReady()) {
      Serial.println("ERROR: Insert SD card, then press '1' to start again.");
      isTestRunning = false;
      return;
    }

    bufferIndex = 0;

    while (!resetUmf01()) { 
      Serial.println("ERROR: flow meter doesn't respond");
      delay(1000);
    }
    writeByte(0x5A); // Reset cumulative flow
    Serial.println("Test started!");
    isTestRunning = true;

    startingTime = micros();
    passedFromPrint = startingTime;
    passedFromFlowRead = startingTime;
  }
}

void stopTest() {
  if (isTestRunning) {
    isTestRunning = false;

    if (dataFile) {
      flushAll();       
      dataFile.close();
    }
  
    Serial.println("Test stopped");
    Serial.print("Data logged in: ");
    Serial.println(filename);
  }
}

// Handle serial commands: '1'=start, '2'=stop, '3'=calibrate
void caseList(int myCase) {
  if (Serial.available() > 0) { myCase = Serial.read(); }
  switch (myCase) {
    case '1': startTest(); break;
    case '2': stopTest(); break;
    case '3': if (!isTestRunning) { pressureCalibration(); } break;
  }
}

bool isSdReady() {
  Serial.println("Checking SD card... ");
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("ERROR: SD card not found");
    sdReady = false;
    return false;
  }
  if (!dataFile) { dataFile = createUniqueFile(); }
  sdReady = true;
  return true;
}



/*
===============================================================
===================== UFM-01 DATA PARSING =====================
===============================================================
*/

void readFlowMeter() {
  uint8_t byteArray[12] = {0};
  readBlock(byteArray);

  instantaneousFlowRaw  = (uint32_t)byteArray[0] | ((uint32_t)byteArray[1] << 8) | ((uint32_t)byteArray[2] << 16);
  temperatureRaw        = (uint32_t)byteArray[4] | ((uint32_t)byteArray[5] << 8) | ((uint32_t)byteArray[6] << 16);
  cumulativeFlowRaw     = (uint32_t)byteArray[8] | ((uint32_t)byteArray[9] << 8) | ((uint32_t)byteArray[10] << 16);

  cumulativeFlow    = (cumulativeFlowRaw * 0.1f);             // L
  instantaneousFlow = (instantaneousFlowRaw * 0.01f) / 3600;  // l/s
  temperature       = temperatureRaw * 0.01f;                 // °C
}



/*
===============================================================
======================= SERIAL PRINT ==========================
===============================================================
*/

inline void serialPrint() {
  #ifdef DEBUG_PRINTS
  Serial.print("P1:         "); Serial.print(poliToLinePressure1);                  Serial.println(" bar");
  Serial.print("P2:         "); Serial.print(poliToLinePressure2);                  Serial.println(" bar");
  Serial.print("P3:         "); Serial.print(upstreamPressure);                     Serial.println(" bar");
  Serial.print("P4:         "); Serial.print(downstreamPressure);                   Serial.println(" bar");
  Serial.print("P4/P3:      "); Serial.print(downstreamToUpstreamRatio, 3);         Serial.println();
  Serial.print("Total Hall: "); Serial.print(cumulativeFlowHall);                   Serial.println(" L");
  Serial.print("Total:      "); Serial.print(cumulativeFlow);                       Serial.println(" L");
  Serial.print("Flow US:    "); Serial.print(instantaneousFlow, 3);                 Serial.println(" L/s");
  Serial.print("Temp:       "); Serial.print(temperature);                          Serial.println(" °C");
  Serial.print("Time:       "); Serial.print(double(passedFromStart)/1000000, 2);   Serial.println(" s");
  Serial.println();
  #else
  Serial.print(poliToLinePressure1);          Serial.print(",");            
  Serial.print(poliToLinePressure2);          Serial.print(",");     
  Serial.print(upstreamPressure);             Serial.print(",");     
  Serial.print(downstreamPressure);           Serial.print(",");     
  Serial.print(downstreamToUpstreamRatio, 3); Serial.print(",");     
  Serial.print(cumulativeFlowHall);           Serial.print(",");     
  Serial.print(cumulativeFlow);               Serial.print(",");     
  Serial.print(instantaneousFlow, 3);         Serial.print(",");     
  Serial.print(temperature);                  Serial.print(",");     
  Serial.print(passedFromStart);              Serial.print(",");
  Serial.println();
  #endif
}



/*
===============================================================
========================== SETUP ==============================
===============================================================
*/

void setup() {
  pinMode(ULTRASONIC_FLOW_METER_PIN, OUTPUT);
  pinMode(PRESSURE_SENSOR_1_PIN, INPUT);
  pinMode(PRESSURE_SENSOR_2_PIN, INPUT);
  pinMode(PRESSURE_SENSOR_3_PIN, INPUT);
  pinMode(PRESSURE_SENSOR_4_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_FLOW_METER_PIN), pulse, RISING);

  analogReadResolution(ADC_BIT_RESOLUTION);

  Serial.begin(BAUD_RATE);
  while (!Serial); 
  Serial.println();
  Serial.println("USB Connected");

  isSdReady();

  pressureCalibration();

  Serial.println("Ready!");

}



/*
===============================================================
======================= MAIN LOOP =============================
===============================================================
*/

void loop() {

  while (!isTestRunning) { caseList(0); }

  now = micros();

  poliToLinePressure1 = (((analogRead(PRESSURE_SENSOR_1_PIN) - calibrationZero1) * MIN_CURRENT * PSI1500_TO_BAR) / (calibrationZero1 * currentRange)) + 1;
  poliToLinePressure2 = (((analogRead(PRESSURE_SENSOR_2_PIN) - calibrationZero2) * MIN_CURRENT * PSI1000_TO_BAR) / (calibrationZero2 * currentRange)) + 1;
  upstreamPressure    = (((analogRead(PRESSURE_SENSOR_3_PIN) - calibrationZero3) * MIN_CURRENT * PSI1500_TO_BAR) / (calibrationZero3 * currentRange)) + 1;
  downstreamPressure  = (((analogRead(PRESSURE_SENSOR_4_PIN) - calibrationZero4) * MIN_CURRENT * PSI1500_TO_BAR) / (calibrationZero4 * currentRange)) + 1;

  if (upstreamPressure > 0) { downstreamToUpstreamRatio = downstreamPressure / upstreamPressure; }

  cumulativeFlowHall = pulseCount / PULSES_TO_LITER;

  passedFromStart = now - startingTime;

  // Format CSV row into write buffer
  printSize = snprintf(
    writeBuffer + bufferIndex,
    SD_BUFFER_SIZE - bufferIndex,
    "%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%lu\n",
    poliToLinePressure1, poliToLinePressure2, upstreamPressure, downstreamPressure, downstreamToUpstreamRatio,
    cumulativeFlowHall, cumulativeFlow, instantaneousFlow, temperature, passedFromStart
  );

  if (printSize > 0) { bufferIndex += printSize; }

  trySdWrite();

  if (now - passedFromFlowRead >= FLOW_READ_INTERVAL) {
    passedFromFlowRead = now;
    readFlowMeter();
  }

  if (now - passedFromPrint >= SERIAL_PRINT_INTERVAL) {
    passedFromPrint = now;
    serialPrint();
  }

  caseList(0);

}