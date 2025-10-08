/*

 ** SDO - pin 11

 ** SDI - pin 12

 ** CLK - pin 13

 ** CS - pin 4

*/

#include <SPI.h>
#include <SD.h>

#define pressure1_pin 76
#define pressure2_pin 77
#define pressure3_pin 78
#define flowMeter_pin 13
#define chipSelect 4

// Flow and timing
volatile double waterFlow = 0;         // cumulative liters
volatile unsigned long pulseCount = 0; // pulses since last interval
double absolute_time = 0;
unsigned long startingTime = 0;

// Pressure data
float pressure1, pressure2, pressure3;
float baseline1 = 0, baseline2 = 0, baseline3 = 0;
float total1 = 0, total2 = 0, total3 = 0;
unsigned int sample = 0;
const unsigned int samples = 1;
const unsigned int loops = 100000;

// Flow rate
float flowRate = 0.0; // instantaneous flow (L/s)

// SD file
File dataFile;
String filename;

// Buffering for SD writes
const uint16_t BUFFER_SIZE = 256;
char writeBuffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;

// --------- Create unique file -----------
String createUniqueFile() {
  int fileIndex = 1; 
  String baseName = "Test";
  String extension = ".csv";
  String fileName;

  // Find free filename
  do {
    fileName = baseName + String(fileIndex) + extension;
    fileIndex++;
  } while (SD.exists(fileName));

  // Create new file
  File newFile = SD.open(fileName, FILE_WRITE);
  if (newFile) {
    newFile.println("Pressure1(bar),Pressure2(bar),Pressure3(bar),WaterFlow(L),FlowRate(L/s),Time(ms)");
    newFile.close();
    Serial.print("Created file: ");
    Serial.println(fileName);
  } else {
    Serial.println("Error creating file!");
    return "";
  }
  return fileName;
}

// --------- Setup ----------
void setup() {
  pinMode(pressure1_pin, INPUT);
  pinMode(pressure2_pin, INPUT);
  pinMode(pressure3_pin, INPUT);

  Serial.begin(115200);
  while (!Serial);

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed.");
    while (true);
  }
  Serial.println("initialization done.");

  attachInterrupt(digitalPinToInterrupt(flowMeter_pin), pulse, RISING);

  // Baseline calibration
  for (int i = 0; i < loops; i++) {
    total1 += analogRead(pressure1_pin);
    total2 += analogRead(pressure2_pin);
    total3 += analogRead(pressure3_pin);
  }
  baseline1 = total1 / loops;
  baseline2 = total2 / loops;
  baseline3 = total3 / loops;

  Serial.print("Baselines: ");
  Serial.print(baseline1); Serial.print(", ");
  Serial.print(baseline2); Serial.print(", ");
  Serial.println(baseline3);

  total1 = total2 = total3 = 0;

  filename = createUniqueFile();
  if (filename != "") {
    dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile) {
      Serial.println("Failed to open file for logging.");
      while (true);
    }
  }

  startingTime = millis();
}

// --------- Loop ----------
void loop() {
  // Sensor reads
  total1 += analogRead(pressure1_pin);
  total2 += analogRead(pressure2_pin);
  total3 += analogRead(pressure3_pin);
  sample++;

  if (sample >= samples) {
    float avg1 = total1 / sample;
    float avg2 = total2 / sample;
    float avg3 = total3 / sample;

    pressure1 = (avg1 - baseline1) * 103.4214 / (1023 - baseline1) + 1;
    pressure2 = (avg2 - baseline2) * 103.4214 / (1023 - baseline2) + 1;
    pressure3 = (avg3 - baseline3) * 103.4214 / (1023 - baseline3) + 1;

    unsigned long now = millis();
    unsigned long elapsed = now - startingTime;
    absolute_time += elapsed;
    startingTime = now;

    // Compute flow rate (L/s)
    flowRate = (pulseCount / 63.0) / (elapsed / 1000.0);
    pulseCount = 0;

    // Build CSV line
    int n = snprintf(writeBuffer + bufferIndex, BUFFER_SIZE - bufferIndex,
                     "%.2f,%.2f,%.2f,%.3f,%.3f,%.0f\n",
                     pressure1, pressure2, pressure3, waterFlow, flowRate, absolute_time);

    if (n > 0) bufferIndex += n;

    // Flush if buffer is near full
    if (bufferIndex > BUFFER_SIZE - 64) {
      dataFile.write((uint8_t*)writeBuffer, bufferIndex);
      bufferIndex = 0;
    }

    // Debug (optional, comment out for max speed)
    Serial.print("P1: "); Serial.print(pressure1);
    Serial.print(" | P2: "); Serial.print(pressure2);
    Serial.print(" | P3: "); Serial.print(pressure3);
    Serial.print(" | Total L: "); Serial.print(waterFlow);
    Serial.print(" | Flow L/s: "); Serial.print(flowRate);
    Serial.print(" | T: "); Serial.println(absolute_time);

    // Reset counters
    total1 = total2 = total3 = 0;
    sample = 0;
  }
}

// --------- Interrupt handler ----------
void pulse() {
  pulseCount++;
  waterFlow += 1.0 / 63.0;  // 63 pulses = 1 L
}