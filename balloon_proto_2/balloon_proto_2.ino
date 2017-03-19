#include <Arduino.h>
#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <math.h>
#include <string.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "LowPower.h"


#define WSPR_TONE_SPACING       146           // ~1.46 Hz
#define WSPR_CTC                10672         // CTC value for WSPR
#define WSPR_DEFAULT_FREQ       14150000UL    // 10140200UL

Si5351 si5351;
JTEncode jtencode;
TinyGPS gps;
SoftwareSerial ss(4, 2);

unsigned long freq;
char message[] = "KD8UPW EM79";
char call[] = "KD8UPW";
char loc[] = "EM79";
uint8_t dbm = 27;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t ctc, tone_spacing;

// Global variables used in ISRs
volatile bool proceed = false;
volatile bool processing = false;

// Timer interrupt vector.  This toggles the variable we use to gate
// each column of output to ensure accurate timing.  Called whenever
// Timer1 hits the count set below in setup().
ISR(TIMER1_COMPA_vect)
{
  proceed = true;
}

void setup() {
  ss.begin(9600);
  
  freq = WSPR_DEFAULT_FREQ;
  ctc = WSPR_CTC;
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;  

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 27000000UL, 0);

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

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
  delay(60000);
}

bool can_transmit() {
  float lat, lon;
  unsigned long fix_age;
  int year, retries;
  byte month, day, hour, minute, second, hundredths;
 
  populate_gps_data();
  while (retries++ < 10) {
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);  
    if (fix_age != TinyGPS::GPS_INVALID_AGE) {
      break;
    }
    delay(1000);
    populate_gps_data();
  }
  if (retries >= 10) {
    return false;
  }
  gps.f_get_position(&lat, &lon, &fix_age);
  if (!valid_coords(lat, lon)) {
    return false;
  }
  if(minute % 2 != 0) {
    delay((60 - second) * 1000);
  }
  to_maidenhead(lat, lon);
  return true;
}

void populate_gps_data() {
  while (true) {
    while (ss.available()) {
      if (gps.encode(ss.read())) {
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
  loc[5] = '\0';
}

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  // Clear out the old transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);
//  digitalWrite(LED_PIN, HIGH);

  for(i = 0; i < symbol_count; i++)
  {
      si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
      proceed = false;
      while(!proceed);
  }

  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
  //digitalWrite(LED_PIN, LOW);
}


