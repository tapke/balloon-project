#include <Wire.h>
#include <JTEncode.h>
#include <math.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define WSPR_TONE_SPACING       1.4648          // 1.4648 Hz
#define WSPR_CTC                10672           // CTC value for WSPR
#define WSPR_DEFAULT_FREQ       10140000      // 10.1400 Mhz
#define PLL_FREQ                90000000000ULL  // 900 Mhz

int lcdPin = 12;

JTEncode jtencode;
TinyGPS gps_data;

SoftwareSerial gps(4, 2); // RX Pin, TX Pin

char message[] = "NOCALL AA00";
char call_sign[] = "KD8UPW";
char loc[] = "AA00";
uint8_t dbm = 10;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t ctc;
float tone_spacing;

// Global variables used in ISRs
volatile bool proceed = false;

// Timer interrupt vector.  This toggles the variable we use to gate
// each column of output to ensure accurate timing.  Called whenever
// Timer1 hits the count set below in setup().
ISR(TIMER1_COMPA_vect)
{
  proceed = true;
}

void setup() {
  gps.begin(9600);
  Serial.begin(115200);
  
  ctc = WSPR_CTC;
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;  

  Wire.begin();                        // Initialize I2C-communication as master
                                       //    SDA on pin ADC04
                                       //    SCL on pin ADC05
  SetFrequency (WSPR_DEFAULT_FREQ);             // Set TX-Frequency [10,14 MHz]
  SetParkMode ();                      // Intialize park mode

  setup_timer_for_wspr_txmit();
}

void loop() {
  if(can_transmit()) {
    encode();
  }
  delay(90000);
}

bool can_transmit() {
  float lat, lon;
  unsigned long fix_age, millis_to_wait, sec_mult;
  int year, retries;
  byte month, day, hour, minute, second, hundredths;
 
  fix_age = TinyGPS::GPS_INVALID_AGE; // reset fix_age variable to force fresh read from gps every time
  populate_gps_data();
  while (retries++ < 10) {
    gps_data.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);  
    if (fix_age != TinyGPS::GPS_INVALID_AGE) {
      sec_mult = minute % 2;
      sec_mult *= 60;
      sec_mult += second;
      sec_mult = 120 - sec_mult;
      Serial.print("HH: "); Serial.print(hour); Serial.print(" mm: "); Serial.print(minute);
      Serial.print(" ss: "); Serial.print(second);
      Serial.println();
      millis_to_wait = sec_mult * 1000;
      Serial.print("Millis to wait: "); Serial.println(millis_to_wait);
      break;
    }
    delay(1000);
    populate_gps_data();
  }
  if (retries >= 10) {
    return false;
  }
  gps_data.f_get_position(&lat, &lon, &fix_age);
  if (!valid_coords(lat, lon)) {
    return false;
  }
  if(millis_to_wait != 0) {
    delay(millis_to_wait);
  }
  to_maidenhead(lat, lon);

  return true;
}

void populate_gps_data() {
  Serial.println("In populate_gps_data function");
  while (true) {
    while (gps.available()) {
      if (gps_data.encode(gps.read())) {
        Serial.println("Found gps data!");
        return;
      }
    }
  }
}

bool valid_coords(double lat, double lon) {
  if(lat <= 0) // equator or South
    return false;
  if(lon < -169 || lon > -19) // Alaska and Africa
    return false;
  return true;
}

void to_maidenhead(double lat, double lon) {
  lat += 90;
  lon += 180;
  loc[0] = 'A' + (int)floor(lon / 20);
  loc[1] = 'A' + (int)floor(lat / 10);
  loc[2] = (int)floor(fmod((lon / 2), 10.0)) + '0';
  loc[3] = (int)floor(fmod(lat, 10.0)) + '0';
  loc[4] = '\0';
}

// Loop through the string, transmitting one character at a time.
void encode()
{
  Serial.println("Inside encode");
  uint8_t i;
  unsigned long tx_freq;
  char print_byte[3];

  Serial.print("call_sign: "); Serial.println(call_sign);
  Serial.print("loc: "); Serial.println(loc);
  Serial.print("dbm: "); Serial.println(dbm);
  Serial.print("symbol_count: "); Serial.println(symbol_count);
  Serial.print("WSPR_DEF_FREQ: "); Serial.println(WSPR_DEFAULT_FREQ);
  
  // Clear out the old transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  jtencode.wspr_encode(call_sign, loc, dbm, tx_buffer);

  SetPower(3);
  
  for(i = 0; i < symbol_count; i++)
  {
    tx_freq = WSPR_DEFAULT_FREQ + (tx_buffer[i] * tone_spacing);
    SetFrequency(tx_freq);
    TX_ON();
    proceed = false;
    while(!proceed);
    TX_OFF();
  }

  // Turn off the output
  TX_OFF();
  
  memset(loc, 0, 5);
}

void TX_ON () {                        // Enables output on CLK0 and disables Park Mode on CLK1
  Si5351a_Write_Reg (17, 128);         // Disable output CLK1
  Si5351a_Write_Reg (16, 79);          // Enable output CLK0, set crystal as source and Integer Mode on PLLA
}

void TX_OFF () {                       // Disables output on CLK0 and enters Park Mode on CLK1
  Si5351a_Write_Reg (16, 128);         // Disable output CLK0
  //Si5351a_Write_Reg (17, 111);         // Enable output CLK1, set crystal as source and Integer Mode on PLLB
}

void SetFrequency (unsigned long frequency) { // Frequency in Hz; must be within [7810 Hz to 200 Mhz]
  #define F_XTAL 27003720;             // Frequency of Quartz-Oscillator
  #define c 1048574;                   // "c" part of Feedback-Multiplier from XTAL to PLL
  unsigned long fvco;                  // VCO frequency (600-900 MHz) of PLL
  unsigned long outdivider;            // Output divider in range [4,6,8-900], even numbers preferred
  byte R = 1;                          // Additional Output Divider in range [1,2,4,...128]
  byte a;                              // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  unsigned long b;                     // "b" part of Feedback-Multiplier from XTAL to PLL
  float f;                             // floating variable, needed in calculation
  unsigned long MS0_P1;                // Si5351a Output Divider register MS0_P1, P2 and P3 are hardcoded below
  unsigned long MSNA_P1;               // Si5351a Feedback Multisynth register MSNA_P1
  unsigned long MSNA_P2;               // Si5351a Feedback Multisynth register MSNA_P2
  unsigned long MSNA_P3;               // Si5351a Feedback Multisynth register MSNA_P3

  outdivider = 900000000 / frequency;  // With 900 MHz beeing the maximum internal PLL-Frequency
  
  while (outdivider > 900){            // If output divider out of range (>900) use additional Output divider
    R = R * 2;
    outdivider = outdivider / 2;
  }
  if (outdivider % 2) outdivider--;    // finds the even divider which delivers the intended Frequency

  fvco = outdivider * R * frequency;   // Calculate the PLL-Frequency (given the even divider)

  switch (R){                          // Convert the Output Divider to the bit-setting required in register 44
    case 1: R = 0; break;              // Bits [6:4] = 000
    case 2: R = 16; break;             // Bits [6:4] = 001
    case 4: R = 32; break;             // Bits [6:4] = 010
    case 8: R = 48; break;             // Bits [6:4] = 011
    case 16: R = 64; break;            // Bits [6:4] = 100
    case 32: R = 80; break;            // Bits [6:4] = 101
    case 64: R = 96; break;            // Bits [6:4] = 110
    case 128: R = 112; break;          // Bits [6:4] = 111
  }

  a = fvco / F_XTAL;                   // Multiplier to get from Quartz-Oscillator Freq. to PLL-Freq.
  f = fvco - a * F_XTAL;               // Multiplier = a+b/c
  f = f * c;                           // this is just "int" and "float" mathematics
  f = f / F_XTAL;
  b = f;

  MS0_P1 = 128 * outdivider - 512;     // Calculation of Output Divider registers MS0_P1 to MS0_P3
                                       // MS0_P2 = 0 and MS0_P3 = 1; these values are hardcoded, see below

  f = 128 * b / c;                     // Calculation of Feedback Multisynth registers MSNA_P1 to MSNA_P3
  MSNA_P1 = 128 * a + f - 512;
  MSNA_P2 = f;
  MSNA_P2 = 128 * b - MSNA_P2 * c; 
  MSNA_P3 = c;

  Si5351a_Write_Reg (16, 128);                      // Disable output during the following register settings 
  Si5351a_Write_Reg (26, (MSNA_P3 & 65280) >> 8);   // Bits [15:8] of MSNA_P3 in register 26
  Si5351a_Write_Reg (27, MSNA_P3 & 255);            // Bits [7:0]  of MSNA_P3 in register 27
  Si5351a_Write_Reg (28, (MSNA_P1 & 196608) >> 16); // Bits [17:16] of MSNA_P1 in bits [1:0] of register 28
  Si5351a_Write_Reg (29, (MSNA_P1 & 65280) >> 8);   // Bits [15:8]  of MSNA_P1 in register 29
  Si5351a_Write_Reg (30, MSNA_P1 & 255);            // Bits [7:0]  of MSNA_P1 in register 30
  Si5351a_Write_Reg (31, ((MSNA_P3 & 983040) >> 12) | ((MSNA_P2 & 983040) >> 16)); // Parts of MSNA_P3 und MSNA_P1
  Si5351a_Write_Reg (32, (MSNA_P2 & 65280) >> 8);   // Bits [15:8]  of MSNA_P2 in register 32
  Si5351a_Write_Reg (33, MSNA_P2 & 255);            // Bits [7:0]  of MSNA_P2 in register 33
  Si5351a_Write_Reg (42, 0);                        // Bits [15:8] of MS0_P3 (always 0) in register 42
  Si5351a_Write_Reg (43, 1);                        // Bits [7:0]  of MS0_P3 (always 1) in register 43
  Si5351a_Write_Reg (44, ((MS0_P1 & 196608) >> 16) | R);  // Bits [17:16] of MS0_P1 in bits [1:0] and R in [7:4]
  Si5351a_Write_Reg (45, (MS0_P1 & 65280) >> 8);    // Bits [15:8]  of MS0_P1 in register 45
  Si5351a_Write_Reg (46, MS0_P1 & 255);             // Bits [7:0]  of MS0_P1 in register 46
  Si5351a_Write_Reg (47, 0);                        // Bits [19:16] of MS0_P2 and MS0_P3 are always 0
  Si5351a_Write_Reg (48, 0);                        // Bits [15:8]  of MS0_P2 are always 0
  Si5351a_Write_Reg (49, 0);                        // Bits [7:0]   of MS0_P2 are always 0
  if (outdivider == 4){
    Si5351a_Write_Reg (44, 12 | R);                 // Special settings for R = 4 (see datasheet)
    Si5351a_Write_Reg (45, 0);                      // Bits [15:8]  of MS0_P1 must be 0
    Si5351a_Write_Reg (46, 0);                      // Bits [7:0]  of MS0_P1 must be 0
  } 
  Si5351a_Write_Reg (177, 32);                      // This resets PLL A
}

void SetParkMode () {                               // Sets CLK1 to the Park Mode frequency of 150 MHz to keep the Si5351a warm during key-up
  Si5351a_Write_Reg (17, 128);                      // Disable output during the following register settings
  Si5351a_Write_Reg (34, 255);                      // Bits [15:8] of MSNB_P3
  Si5351a_Write_Reg (35, 254);                      // Bits [7:0]  of MSNB_P3
  Si5351a_Write_Reg (36, 0);                        // Bits [17:16] of MSNB_P1 in bits [1:0]
  Si5351a_Write_Reg (37, 14);                       // Bits [15:8]  of MSNB_P1
  Si5351a_Write_Reg (38, 169);                      // Bits [7:0]  of MSNB_P1
  Si5351a_Write_Reg (39, 252);                      // Parts of MSNB_P3 und MSNB_P1
  Si5351a_Write_Reg (40, 130);                      // Bits [15:8]  of MSNB_P2
  Si5351a_Write_Reg (41, 82);                       // Bits [7:0]  of MSNB_P2
  Si5351a_Write_Reg (50, 0);                        // Bits [15:8] of MS1_P3
  Si5351a_Write_Reg (51, 1);                        // Bits [7:0]  of MS1_P3
  Si5351a_Write_Reg (52, 0);                        // Bits [17:16] of MS1_P1 in bits [1:0] and R in [7:4]
  Si5351a_Write_Reg (53, 1);                        // Bits [15:8]  of MS1_P1
  Si5351a_Write_Reg (54, 0);                        // Bits [7:0]  of MS1_P1
  Si5351a_Write_Reg (55, 0);                        // Bits [19:16] of MS1_P2 and MS1_P3
  Si5351a_Write_Reg (56, 0);                        // Bits [15:8]  of MS1_P2
  Si5351a_Write_Reg (57, 0);                        // Bits [7:0]   of MS1_P2
  Si5351a_Write_Reg (177, 128);                     // This resets PLL B
}

void SetPower (byte power){                         // Sets the output power level
  if (power == 0 || power > 4){power = 0;}          // valid power values are 0 (25%), 1 (50%), 2 (75%) or 3 (100%)
  switch (power){
    case 1:
      Si5351a_Write_Reg (16, 76);                   // CLK0 drive strength = 2mA; power level ~ -8dB
      break;
    case 2:
      Si5351a_Write_Reg (16, 77);                   // CLK0 drive strength = 4mA; power level ~ -3dB
      break;
    case 3:
      Si5351a_Write_Reg (16, 78);                   // CLK0 drive strength = 6mA; power level ~ -1dB
      break;
    case 4:
      Si5351a_Write_Reg (16, 79);                   // CLK0 drive strength = 8mA; power level := 0dB
      break;
  }
}

void Si5351a_Write_Reg (byte regist, byte value){   // Writes "byte" into "regist" of Si5351a via I2C
  Wire.beginTransmission(96);                       // Starts transmission as master to slave 96, which is the
                                                    // I2C address of the Si5351a (see Si5351a datasheet)
  Wire.write(regist);                               // Writes a byte containing the number of the register
  Wire.write(value);                                // Writes a byte containing the value to be written in the register
  Wire.endTransmission();                           // Sends the data and ends the transmission
}

void setup_timer_for_wspr_txmit() {
  // Set up Timer1 for interrupts every symbol period.
  noInterrupts();          // Turn off interrupts.
  TCCR1A = 0;              // Set entire TCCR1A register to 0; disconnects
                           //   interrupt output pins, sets normal waveform
                           //   mode.  We're just using Timer1 as a counter.
  TCNT1  = 0;              // Initialize counter value to 0.
  TCCR1B = (1 << CS12) |   // Set CS12 and CS10 bit to set prescale
    (1 << CS10) |          //   to /1024
    (1 << WGM12);          //   turn on CTC
                           //   which gives, 64 us ticks
  TIMSK1 = (1 << OCIE1A);  // Enable timer compare interrupt.
  OCR1A = ctc;             // Set up interrupt trigger count;
  interrupts();            // Re-enable interrupts.
}


