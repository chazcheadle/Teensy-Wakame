#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define SATX 63
#define SATY 89
#define SATR 36

#define sclk 13  // SCLK can also use pin 14
#define miso 12  // MISO can also use pin XXX
#define mosi 11  // MOSI can also use pin 7
#define cs   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define dc   9   //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define rst  8   // RST can use any pin
#define sdcs 4   // CS for SD card, can use any pin

Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, mosi, sclk, rst);
#define ST7735_DARKGREY 0xC618

// The TinyGPS++ object
TinyGPSPlus gps;
// Setup UART (Hardware Serial #1 pins 0, 1)
HardwareSerial GPS_Ser = HardwareSerial();

static const int MAX_SATELLITES = 40;

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
TinyGPSCustom satNumber[4]; // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];

long COMP_previousMillis = 0;        // will store last time LED was updated
long COMP_interval = 100;
long GPS_previousMillis = 0;        // will store last time LED was updated
long GPS_interval = 1000;


// Setup mag sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

bool magPresent = true;
float headingDegrees, prevHeadingDegrees;
const char *headingCardinal;

void setup() {
  // Debug data on hardware serial
//  Serial.begin(115200);

  // GPS data from software serial
  GPS_Ser.begin(9600);

  // Initialize all the uninitialized TinyGPSCustom objects for tracking satellites
  for (int i=0; i<4; ++i)
  {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snr[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }
  
  // Start i2c
  Wire.begin();
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  // If your TFT's plastic wrap has a Green Tab, use the following:
  tft.initR(INITR_GREENTAB); // initialize a ST7735R chip, green tab
  
  
//   Use this initializer (uncomment) if you're using a 1.44" TFT
  tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab
//  Serial.println("Initialized");

  // large block of text
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);

  tft.setCursor(1, 10);
  tft.print("Lat: ");
  tft.setCursor(1, 18);
  tft.print("Lng: ");
  tft.setCursor(1, 34);
  tft.print("Alt: ");
  tft.setCursor(1, 26);
  tft.print("Sat: ");


  // Start mag sensoring
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
//    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    magPresent = false;
    //while(1);
  }
  
  // Draw Satellite map
  drawSatelliteMap();

}


static char printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}



void loop() {
  unsigned long currentMillis = millis();
 
  if (currentMillis - COMP_previousMillis > COMP_interval) {
    COMP_previousMillis = currentMillis;
    getHeadingDegrees();
    // Draw compass needle.
    displayCompassNeedle();
  }
  if (millis() > 10000 && gps.charsProcessed() < 10) // uh oh
  {
    tft.setCursor(1,1);
    tft.print("ERROR: not getting any GPS data!");
    // dump the stream to Serial
//    Serial.println("GPS stream dump:");
    while (true) // infinite loop
      if (GPS_Ser.available() > 0) // any data coming in?
        tft.println(GPS_Ser.read());
  }

  if (currentMillis - GPS_previousMillis > GPS_interval) {
    GPS_previousMillis = currentMillis;
    // Print GPS data.
    displayGPSData();
  
    displaySatellite(60.0, 45);
    displaySatellite(0.0, 110);
    displaySatellite(45.0, 315);

  }
}

void testdrawtext(const char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color, ST7735_BLACK);
  tft.setTextWrap(true);
  tft.print(text);
}

void getHeadingDegrees() {
  sensors_event_t event; 
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI; 
  
//  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  }


void displayGPSData() {

  getGPSData();

  tft.setTextWrap(false);
  tft.setCursor(1,1);
  // Draw title bar background
  tft.fillRect(0, 0, 128, 9, ST7735_WHITE);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
  getTime(gps.time);
  getDate(gps.date);

  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setCursor(31, 10);
  tft.println(gps.location.lat(), 6);
  tft.setCursor(31, 18);
  tft.println(gps.location.lng(), 6);
  tft.setCursor(31, 34);
  tft.println(gps.altitude.meters(), 1);
  tft.setCursor(31, 26);
  tft.println(gps.satellites.value());

  if (magPresent) {
    headingCardinal = TinyGPSPlus::cardinal(headingDegrees);
    printStr(gps.location.isValid() ? headingCardinal : "*** ",1);
    tft.setCursor(1, 42);
    tft.print("Hdg: ");
    int len  = strlen(headingCardinal);
    tft.print(headingCardinal);
    for (int i=len; i<4; ++i)
      tft.print(' ');
  }

}

void getGPSData() {
  while (GPS_Ser.available())
      gps.encode(GPS_Ser.read());
      if (totalGPGSVMessages.isUpdated())
      {
        for (int i=0; i<4; ++i)
        {
          int no = atoi(satNumber[i].value());
          // Serial.print(F("SatNumber is ")); Serial.println(no);
          if (no >= 1 && no <= MAX_SATELLITES)
          {
            sats[no-1].elevation = atoi(elevation[i].value());
            sats[no-1].azimuth = atoi(azimuth[i].value());
            sats[no-1].snr = atoi(snr[i].value());
            sats[no-1].active = true;
          }
        }
        
        int totalMessages = atoi(totalGPGSVMessages.value());
        int currentMessage = atoi(messageNumber.value());
        if (totalMessages == currentMessage)
        {
          Serial.print(F("Sats=")); Serial.print(gps.satellites.value());
          Serial.print(F(" Nums="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(i+1);
              Serial.print(F(" "));
            }
          Serial.print(F(" Elevation="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(sats[i].elevation);
              Serial.print(F(" "));
            }
          Serial.print(F(" Azimuth="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(sats[i].azimuth);
              Serial.print(F(" "));
            }
          
          Serial.print(F(" SNR="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(sats[i].snr);
              Serial.print(F(" "));
            }
          Serial.println();
  
          for (int i=0; i<MAX_SATELLITES; ++i)
            sats[i].active = false;
        }
      } 
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_Ser.available())
      gps.encode(GPS_Ser.read());
      if (totalGPGSVMessages.isUpdated())
      {
        for (int i=0; i<4; ++i)
        {
          int no = atoi(satNumber[i].value());
          // Serial.print(F("SatNumber is ")); Serial.println(no);
          if (no >= 1 && no <= MAX_SATELLITES)
          {
            sats[no-1].elevation = atoi(elevation[i].value());
            sats[no-1].azimuth = atoi(azimuth[i].value());
            sats[no-1].snr = atoi(snr[i].value());
            sats[no-1].active = true;
          }
        }
        
        int totalMessages = atoi(totalGPGSVMessages.value());
        int currentMessage = atoi(messageNumber.value());
        if (totalMessages == currentMessage)
        {
          Serial.print(F("Sats=")); Serial.print(gps.satellites.value());
          Serial.print(F(" Nums="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(i+1);
              Serial.print(F(" "));
            }
          Serial.print(F(" Elevation="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(sats[i].elevation);
              Serial.print(F(" "));
            }
          Serial.print(F(" Azimuth="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(sats[i].azimuth);
              Serial.print(F(" "));
            }
          
          Serial.print(F(" SNR="));
          for (int i=0; i<MAX_SATELLITES; ++i)
            if (sats[i].active)
            {
              Serial.print(sats[i].snr);
              Serial.print(F(" "));
            }
          Serial.println();
  
          for (int i=0; i<MAX_SATELLITES; ++i)
            sats[i].active = false;
        }
      } 
    
  } while (millis() - start < ms);
}

static void getTime(TinyGPSTime &t)
{
  if (!t.isValid())
  {
    tft.print("Waiting for signal...");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d  ", t.hour(), t.minute(), t.second());
    tft.print(sz);
  }
  smartDelay(0);
}

static void getDate(TinyGPSDate &d)
{
  if (d.isValid())
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    tft.print(sz);
  }
  smartDelay(0);
}

//  tft.drawCircle(63, 89, 36, ST7735_WHITE);
//  tft.drawCircle(63, 89, 18, ST7735_WHITE);
//  tft.drawFastVLine(63, 53, 72, ST7735_WHITE);
//  tft.drawFastHLine(27, 89, 72, ST7735_WHITE);

/**
 * Draw a circular map that represents the plane upon which satellites can
 * be projected in a top-down 2D view.
 */
void drawSatelliteMap() {
  // clear the map
  tft.fillRect(SATX - SATR, SATY - SATR, SATX + SATR, SATY + SATR, ST7735_BLACK); 
  // Draw outer ring
  tft.drawCircle(SATX, SATY, SATR, ST7735_WHITE);
  // Draw inner ring
  tft.drawCircle(SATX, SATY, round(SATR/2), ST7735_DARKGREY);
  // Draw crosshairs
  tft.drawFastVLine(SATX, SATY - SATR, SATR*2, ST7735_DARKGREY);
  tft.drawFastHLine(SATX - SATR, SATY, SATR*2, ST7735_DARKGREY);
}

/**
 * Display compass needle.
 */
void displayCompassNeedle() {
  int nx, ny;

  if (prevHeadingDegrees) {
    // The X and Y coordinates of the tip of the compass needle.
    nx = round(sin((double)prevHeadingDegrees * PI / 180) * (SATR - 1));
    ny = round(cos((double)prevHeadingDegrees * PI / 180) * (SATR - 1));
    // Draw inner ring
    tft.drawCircle(SATX, SATY, round(SATR/2), ST7735_DARKGREY);
    // Draw crosshairs
    tft.drawFastVLine(SATX, SATY - SATR, SATR*2, ST7735_DARKGREY);
    tft.drawFastHLine(SATX - SATR, SATY, SATR*2, ST7735_DARKGREY);
    tft.drawLine(SATX, SATY, SATX - nx, SATY - ny, ST7735_BLACK);
  }
  // The X and Y coordinates of the tip of the compass needle.
  nx = round(sin((double)headingDegrees * PI / 180) * (SATR - 1));
  ny = round(cos((double)headingDegrees * PI / 180) * (SATR - 1));
  prevHeadingDegrees = headingDegrees;
  tft.drawLine(SATX, SATY, SATX - nx, SATY - ny, ST7735_RED);
}

/**
 * Display satellites on the satellite map according to where they appear on a 
 * 2D plan.
 * The current heading is taken into account.
 */
void displaySatellite(const double& elevation, const double& azimuth) {
  int x, ex, ey;
  // The distance from the center to the satellite.
  x = round(cos(elevation * PI / 180) * (SATR - 2));  

  // The X and Y coordinates of the satellite on the map.
  ex = round(sin((azimuth - (double)headingDegrees) * PI / 180) * x);
  ey = round(cos((azimuth - (double)headingDegrees) * PI / 180) * x);

   // Draw demo satellites
  tft.fillCircle(SATX + ex, SATY - ey, 2, ST7735_GREEN);

}

void receiveData(int byteCount) {
}

void sendData() {
}

