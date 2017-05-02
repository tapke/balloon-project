#include <Arduino.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <math.h>
#include <string.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

extern "C" {
  #include "i2c.h"
  #include "si5351a.h"
}

#define WSPR_TONE_SPACING       1.4648          // 1.4648 Hz
#define WSPR_CTC                10672           // CTC value for WSPR
#define WSPR_DEFAULT_FREQ       10140000UL      // 10.1400 Mhz
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

  i2cInit();
  //si5351aSetFrequency(WSPR_DEFAULT_FREQ);

  delay(500);

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
      Serial.print("sec_mult: "); Serial.println(sec_mult);
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
  
  // Clear out the old transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  jtencode.wspr_encode(call_sign, loc, dbm, tx_buffer);

//  for(i = 0; i < symbol_count; i++)
//  {
//    if(i % 54 == 0) {
//      Serial.println();
//    }
//    sprintf(print_byte, "%02x ", tx_buffer[i]);
//    Serial.print(print_byte);
//  }
//  Serial.println(); Serial.println();
  si5351aSetClk1Frequency(150000000UL);
  
  for(i = 0; i < symbol_count; i++)
  {
    tx_freq = WSPR_DEFAULT_FREQ + (tx_buffer[i] * tone_spacing);
    si5351aSetFrequency(tx_freq);
    proceed = false;
    while(!proceed);
  }

  // Turn off the output
  si5351aOutputOff(SI_CLK0_CONTROL);
  si5351aOutputOff(SI_CLK1_CONTROL);

  memset(loc, 0, 5);
}


