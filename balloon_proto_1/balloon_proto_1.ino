#include "Arduino.h"
#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
extern "C"{
  #include "i2c.h"
  #include "si5351a.h"
}

int lcdPin = 12;

TinyGPS gps;
SoftwareSerial ss(4, 2);
SoftwareSerial lcd(255, lcdPin);

char loc[5];

static void smartdelay(unsigned long ms);

void setup() {
  i2cInit();
  
  Serial.begin(115200);

  ss.begin(9600);
  pinMode(lcdPin, OUTPUT);
  digitalWrite(lcdPin, HIGH);
  lcd.begin(9600);
  delay(100);
  lcd.write(12);
  lcd.write(17);
}

void loop() {
  float lat, lon;
  unsigned long age;
  
  Serial.println("FROM GPS:");
  int sats = gps.satellites();
  gps.f_get_position(&lat, &lon, &age);  
  if(sats == TinyGPS::GPS_INVALID_SATELLITES) {
    Serial.println("Bad Position Data");
    lcd.print("Sats: "); lcd.print(sats); lcd.write(13);
    smartdelay(5);
  }
  else {
    toMaidenhead(lat, lon, loc);
    Serial.println(loc);
    lcd.write(12);
    lcd.print("Lat:"); lcd.print((int)floor(lat)); lcd.print(" Lon:"); lcd.print((int)floor(lon)); lcd.write(13);
    lcd.print("MDNHD: "); lcd.print(loc);
    smartdelay(5);
  }

  smartdelay(2000);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void toMaidenhead(double lat, double lon, char loc[]) {
  lat += 90;
  lon += 180;
  loc[0] = 'A' + (int)floor(lon / 20);
  loc[1] = 'A' + (int)floor(lat / 10);
  loc[2] = (int)floor(fmod((lon / 2), 10.0)) + '0';
  loc[3] = (int)floor(fmod(lat, 10.0)) + '0';
//  double rLat = (lat - floor(lat)) * 60;
//  double rLon = (lon - 2 * floor(lon / 2)) * 60;
//  loc[4] = 'a' + (int)floor(rLon / 5);
//  loc[5] = 'a' + (int)floor(rLat / 2.5);
  loc[5] = '\0';
}

