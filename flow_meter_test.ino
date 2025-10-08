
// -------- Configuration --------
static const uint8_t PIN_1WIRE = 8;     // Change if you wire a different pin
static const unsigned READ_PERIOD_MS = 250; // 4 Hz polling (module produces 1–8 Hz data)

// -------- 1-Wire low-level helpers --------
inline void ow_driveLow() { pinMode(PIN_1WIRE, OUTPUT); digitalWrite(PIN_1WIRE, LOW); }
inline void ow_release()  { pinMode(PIN_1WIRE, INPUT_PULLUP); } // relies on external 4.7k to 5V
inline int  ow_read()     { return digitalRead(PIN_1WIRE); }

// 1-Wire RESET + presence detect
// Master low ~500 us, release; sample presence ~70 us after release.
bool ow_reset() {
  ow_driveLow();
  delayMicroseconds(500);           // reset low
  ow_release();
  delayMicroseconds(70);            // wait for presence window
  bool presence = (ow_read() == LOW); // slave pulls low for 60–240 us
  // Finish the reset slot
  delayMicroseconds(430);
  return presence;
}

// Write a single bit 
// '1' bit: pull low ~3 us then release for rest of 60 us slot
// '0' bit: pull low ~60 us then release.
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
// Pull low ~4 us, release, sample around 11–15 us, finish 60 us slot.
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

// -------- CRC-8 --------
uint8_t crc8_ds(const uint8_t* val, uint8_t length) {
  uint8_t crc_val = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc_val ^= val[i];
    for (uint8_t t = 8; t > 0; --t) {
      if (crc_val & 0x80) { crc_val = (uint8_t)((crc_val << 1) ^ 0x31u); }
      else            { crc_val <<= 1; }
    }
  }
  return crc_val;
}

// -------- UFM-01 high-level read --------
// Command 0x5B, then start register 0x30, then read 12 bytes: 
// [InFl_L, InFl_M, InFl_H, InFl_CRC,
//  T_L,   T_M,   T_H,   T_CRC,
//  CuFl_L,CuFl_M,CuFl_H,CuFl_CRC]
bool ufm_read_block(uint8_t* buf12) {
  // Reset & presence
  if (!ow_reset()) return false;

  // Send "continuous read starting at REG" command
  // Per datasheet: 0x5B, REG (0x30). Checksum is cumulative of parameters
  ow_writeByte(0x5B);
  ow_writeByte(0x30);

  // Now read 12 bytes sequentially 
  for (uint8_t i = 0; i < 12; i++) {
    buf12[i] = ow_readByte();
  }

  // Optional: another reset to end the session cleanly
  ow_reset();
  return true;
}

void setup() {
  // Prepare pin and serial
  ow_release(); // idle bus
  Serial.begin(115200);
  while (!Serial) { /* wait for USB serial */ }

  Serial.println(F("USB connected"));
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < READ_PERIOD_MS) return;
  last = now;

  uint8_t b[12] = {0};
  bool ok = ufm_read_block(b);

  if (!ok) {
    Serial.println(F("No presence / 1-Wire reset failed."));
    return;
  }

  // Parse values
  uint32_t infl_raw = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
  uint8_t  infl_crc = b[3];
  uint8_t  infl_calc = crc8_ds(b, 3);

  uint32_t temp_raw = (uint32_t)b[4] | ((uint32_t)b[5] << 8) | ((uint32_t)b[6] << 16);
  uint8_t  temp_crc = b[7];
  uint8_t  temp_calc = crc8_ds(&b[4], 3);

  uint32_t cufl_raw = (uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16);
  uint8_t  cufl_crc = b[11];
  uint8_t  cufl_calc = crc8_ds(&b[8], 3);

  // Instantaneous flow: 0.01 l/h per count
  // Temperature: 0.01 °C per count
  // Cumulative: 0.01 l per count
  float infl_lph = infl_raw * 0.01f;   // Instantaneous flow in l/h
  float temp_C   = temp_raw * 0.01f;   // Temperature in °C
  float cufl_L   = cufl_raw * 0.01f;   // Cumulative flow in liters

  // Print
  Serial.print(F("Instantaneous flow: (raw 24b=0x"));
  Serial.print(infl_raw, HEX);
  Serial.print(F("): "));
  if (infl_calc == infl_crc) {
    Serial.print(infl_lph, 2);
    Serial.print(F(" l/h"));
  } else {
    Serial.print(F("CRC ERROR (got 0x"));
    Serial.print(infl_crc, HEX);
    Serial.print(F(", calc 0x"));
    Serial.print(infl_calc, HEX);
    Serial.print(F("), raw=0x"));
    Serial.print(infl_raw, HEX);
  }

  Serial.print(F(" | Temp: (raw 24b=0x"));
  Serial.print(temp_raw, HEX);
  Serial.print(F("): "));
  if (temp_calc == temp_crc) {
    Serial.print(temp_C, 2);
    Serial.print(F(" °C"));
  } else {
    Serial.print(F("CRC ERROR (got 0x"));
    Serial.print(temp_crc, HEX);
    Serial.print(F(", calc 0x"));
    Serial.print(temp_calc, HEX);
    Serial.print(F("), raw=0x"));
    Serial.print(temp_raw, HEX);
  }

  Serial.print(F(" | Cumulative (raw 24b=0x"));
  Serial.print(cufl_raw, HEX);
  Serial.print(F("): "));
  if (cufl_calc == cufl_crc) {
    Serial.print(cufl_L, 2);
    Serial.print(F(" L"));
  } else {
    Serial.print(F("CRC ERR got 0x"));
    Serial.print(cufl_crc, HEX);
    Serial.print(F(", calc 0x"));
    Serial.print(cufl_calc, HEX);
    Serial.print(F("]"));
  }
  Serial.println();
}