


/***************************************************
 Electronical weighbridge



 ****************************************************/

#define PRG_NAME_SHORT  "EWB"
#define PRG_VERSION     "0.2.0"

//#define RELEASE          // switch on release mode.. (no debug mode, small foortprint)
#define DEBUG            // debug mode on/off

#ifdef RELEASE           // RELEASE deactivates the DEBUG mode!
#undef DEBUG
#endif


#include <Arduino.h>
#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/
#include <ArduinoJson.h>    // https://github.com/bblanchon/ArduinoJson
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

//#include <EEPROM.h>

//#include <Time.h>
#include <ctime>          // std::tm
#include <locale>         // std::locale, std::time_get, std::use_facet

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h" // https://github.com/adafruit/Adafruit_ILI9341
#include "time_ntp.h"

#include <OneWire.h>
#include <Q2HX711.h>

#define TTY_SPEED 115200  // Serial speed

// === new by Zentris ===
// Order NodeMCO Pins to GPIO numbers
#define NodeMCU_D0   16    // ILI9341: RST
#define NodeMCU_D1    5    // ILI9341: SPI-DC
#define NodeMCU_D2    4    // ILI9341: SPI-CS
#define NodeMCU_D3    0    // HX711: data pin
#define NodeMCU_D4    2    // 1Wire: sensor DS18B20
#define NodeMCU_D5   14    // ILI9341: SPI-CLK
#define NodeMCU_D6   12    // ILI9341: SPI-MISO
#define NodeMCU_D7   13    // ILI9341: SPI-MOSI
#define NodeMCU_D8   15    // HX711: clock pin
#define NodeMCU_D9    3
#define NodeMCU_D10   1

// Order ILI9341 to NodeMCU Pins
#define TFT_RST         NodeMCU_D0
#define TFT_DC          NodeMCU_D1
#define TFT_CS          NodeMCU_D2
#define PIN_BACKLIGHT   NodeMCU_D4   // fixed on Vcc
#define TFT_CLK         NodeMCU_D5
#define TFT_MISO        NodeMCU_D6
#define TFT_MOSI        NodeMCU_D7

#define SIGNAL_LED      LED_BUILTIN   // internal LED ! (blue)

#define DS18B20_GPIO    NodeMCU_D4

/* ---------------------------------------------
   --- NTP server ip ---
   ------------------------------------------ */
struct { int b1 = 192;  int b2 = 168;  int b3 = 178;  int b4 =  1; } ntpIP; // Fritzbox

unsigned long ntpdRefreshTime = 0;    // time if the get the ntp time re-read

/* ---------------------------------------------
   --- Wifi- access data ---
   ---------------------------------------------
   :: (?) It can be stored more than one WiFi connection - the device check
          for availability and connect to the first was found
   :: (!) This structure should be defined into the privats.h file!
   SSID      = the WiFi SSID name to connect
   PASSWORD  = the WiFi identification password
*/
struct accessPoint {const char* SSID; const char* PASSWORD;};

/* ---------------------------------------------
   --- Thingspeak API connection data ---
   ---------------------------------------------
   :: (?) Connection data for a Thingspeak account to sent the measurement
          data to one or more data representation fields
   :: (!) This structure should be defined into the privats.h file!
   tsServer   = Thingspeak domain name
   tsServerIP = Thingspeak server ip
   tsAPIKey   = individual API key for our channel
   tsFieldNo  = number of field in our channel where the data will be shown
                The field number refereced with the sensorId number from
                the Sensor definition data structure (!)
                It means, the number sensorId number is the field number
                off Thingspeak fields.
*/
#define numberOfTSFields 5  // (field # "0" for comments!)
struct tsData {String tsServer; String tsServerIP; String tsAPIKey; String tsDataSet[numberOfTSFields];};

#include "privates.h"


uint8_t MACArray[6];                  // return type of mac request
String macID;                         // readable mac address

tm * tblock;

time_t ulSecs2000_timer = 0;          // normalised timestamp in sec from 1.1.2000 00:00:00
date_time_t *date = new date_time_t;  // date structure for output calculation
time_t rawtime;
time_t tsSendTime;                    // last sending time to thingspeak server

int looptime = 0;                     // loop time counter in main loop

// waage
#define MEASURING_COUNT 6

static const byte hx711_data_pin = NodeMCU_D3;
static const byte hx711_clock_pin = NodeMCU_D8;
static const float divisor = 100.0;
static const float factorCalibrate2g = 199.0/428.0;

static unsigned int measurementLoop = 0;
static float correkture2null = 82244.8 - 33.5;
static float weight    = 0;
static float weightCurrent = 0;
static float dataArray[MEASURING_COUNT];

static float temperature;
static bool  temperature_valid = false;
static unsigned long tempLoopCnt = 0;


#define NTP_REFRESH_AFTER          3600*1000  // after what time the ntp
                                              // timestamp will be refreshed
                                              // (only continous mode)
IPAddress ipAddrNTPServer(ntpIP.b1, ntpIP.b2, ntpIP.b3, ntpIP.b4);    // local fritz box

ESP8266WiFiMulti WiFiMulti;

ESP8266WebServer server(80);

Q2HX711 hx711(hx711_data_pin, hx711_clock_pin);  // initialise HX711

OneWire  DS18B20(DS18B20_GPIO);

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);


#define FIXED_FONT_CHAR_WIDTH   6
#define FIXED_FONT_CHAR_HEIGHT  8
//
#define ROTATION_NO             0
#define ROTATION_90             1
#define ROTATION_180            2
#define ROTATION_270            3
//
#define SMALLEST_TEXT_SIZE      1
#define SMALL_TEXT_SIZE         2
#define MEDIUM_TEXT_SIZE        3
#define BIG_TEXT_SIZE           4
#define BIGGEST_TEXT_SIZE       5
//
#define FIXED_HORIZONTAL_SPACING  1
#define FIXED_VERTICAL_SPACING    0

//
int tft_rotation;
int tft_char_width;
int tft_char_height;
int tft_lines;
int tft_columns;
int tft_text_size;
int tft_hor_spacing;
int tft_vert_spacing;


// Pos day: x = 10 y = 8
#define POS_DATE_DAY_X      10
#define POS_DATE_DAY_Y       8
//
// Pos . x = 34 y = 8
#define POS_DATE_DOT1_X      34
#define POS_DATE_DOT1_Y       8
//
// Pos month x = 46 y = 8
#define POS_DATE_MONTH_X      46
#define POS_DATE_MONTH_Y       8
//
// Pos . x = 70 y = 8
#define POS_DATE_DOT2_X      70
#define POS_DATE_DOT2_Y       8
//
// Pos year x = 82 y = 8
#define POS_DATE_YEAR_X      82
#define POS_DATE_YEAR_Y       8
//
// Pos WDay: x = 149 y = 8
#define POS_WDAY_X     149
#define POS_WDAY_Y       8
//
// /////////////////////////////////////
//
// Pos hour: x = 201 y = 8
#define POS_TIME_HOUR_X     201
#define POS_TIME_HOUR_Y       8
//
// Pos : : x = 225 y = 8
#define POS_TIME_QUOTE_X     225
#define POS_TIME_QUOTE_Y       8
//
// Pos minute: x = 237 y = 8
#define POS_TIME_MINUTE_X     237
#define POS_TIME_MINUTE_Y       8
//
// Pos dot: x = 261 y = 8
#define POS_TIME_DOT_X     261
#define POS_TIME_DOT_Y       8
//
// Pos second: x = 273 y = 8
#define POS_TIME_SECOND_X     273
#define POS_TIME_SECOND_Y       8
//
// /////////////////////////////////////
//
// Pos Temp:
// Pos degree: x = 38 y = 112
// Pos Dot: x = 74 y = 112
// Pos Tenth: x = 92 y = 112
#define POS_TEMP_TEMP_X      38
#define POS_TEMP_TEMP_Y     112
#define POS_TEMP_DOT_X       74
#define POS_TEMP_DOT_Y      112
#define POS_TEMP_TENTH_X     92
#define POS_TEMP_TENTH_Y    112


//
// Pos Load:
// Pos Load Full: x = 199 y = 112
// Pos Load Dot: x = 217 y = 112
// Pos Load Tenth: x = 235 y = 112
#define POS_LOAD_LOAD_X     180
#define POS_LOAD_LOAD_Y      63

#define POS_LOAD_DOT_X      217
#define POS_LOAD_DOT_Y      112

#define POS_LOAD_TENTH_X    235
#define POS_LOAD_TENTH_Y    112
//
// /////////////////////////////////////
//
// Pos AByte: x = 68 y = 184
#define POS_IP_ABYTE_X  68
#define POS_IP_ABYTE_Y  184
//
// Pos Dot1: x = 104 y = 184
#define POS_IP_DOT1_X  104
#define POS_IP_DOT1_Y  184
//
// Pos BByte: x = 116 y = 184
#define POS_IP_BBYTE_X  116
#define POS_IP_BBYTE_Y  184
//
// Pos Dot3: x = 152 y = 184
#define POS_IP_DOT2_X  152
#define POS_IP_DOT2_Y  184
//
// Pos CByte: x = 164 y = 184
#define POS_IP_CBYTE_X  164
#define POS_IP_CBYTE_Y  184
//
// Pos Dot3: x = 200 y = 184
#define POS_IP_DOT3_X  200
#define POS_IP_DOT3_Y  184
//
// Pos DByte: x = 212 y = 184
#define POS_IP_DBYTE_X  212
#define POS_IP_DBYTE_Y  184
//
// /////////////////////////////////////
//
// Pos Free mem: x = 73 y = 215
#define POS_FREE_MEM_X  73
#define POS_FREE_MEM_Y 215
//
// Pos Free SD: x = 220 y = 215
#define POS_FREE_SD_X  232
#define POS_FREE_SD_Y  215


#define DISPLAY_REFRESH_INTERVALL 900

void tft_SetTextCursor( int column, int line ) {
  tft.setCursor( ((tft_text_size * tft_char_width) + tft_hor_spacing) * (column-1),
                ((tft_text_size * tft_char_height) + tft_vert_spacing ) * (line-1) );
}

void tft_MapTFTParams( void ) {
  tft_columns = tft.width()/((tft_char_width * tft_text_size) + tft_hor_spacing);
  tft_lines = tft.height()/((tft_char_height * tft_text_size) + tft_vert_spacing);
}

void tft_drawFrameLabels(void)
{
  int textSizeSave;

  tft_SetTextCursor( 2, 2 );

  tft.setTextColor(ILI9341_WHITE);
  textSizeSave = tft_text_size;
  tft_text_size = MEDIUM_TEXT_SIZE;
  tft.setTextSize(tft_text_size);

  tft_SetTextCursor( 2, 3 );

/*
  tft.setCursor( tft.getCursorX() + tft_char_width, tft.getCursorY() + 2*tft_char_height);
  tft_text_size = textSizeSave;
  tft.setTextSize(tft_text_size);
  tft.print("O");

  textSizeSave = tft_text_size;
  tft_text_size = MEDIUM_TEXT_SIZE;
  tft.setTextSize(tft_text_size);
  tft.print("C");
*/
//  tft.setCursor( tft.getCursorX()+(6 * tft_text_size*tft_char_width), tft.getCursorY() );

  int xSave = tft.getCursorX()+ tft_char_width;

  tft.setCursor( tft.getCursorX()+ tft_char_width, tft.getCursorY() + 2*tft_char_height );
  tft.print("GEWICHT");

  tft.setCursor( xSave, 34 + tft.getCursorY() + 2*tft_char_height );
  tft.print("TEMPgrdC");

  tft_text_size = textSizeSave;
  tft.setTextSize(tft_text_size);

  tft_SetTextCursor( 2, tft_lines );

  tft.setCursor( tft.getCursorX(), tft.getCursorY()-tft_char_height-1 );
  tft.print("mem: ");
  tft.print("     B");

  tft_SetTextCursor( 14, tft_lines );
  tft.setCursor( tft.getCursorX()+tft_char_width/2, tft.getCursorY()-tft_char_height-1 );
  tft.print("LT:  ");
  tft.print("     ms");
}


void tft_drawFrames( int background, int textforeground,  int foreground )
{
  int vy_top, vy_bottom;

  tft.fillScreen(background);
  tft.setTextColor(textforeground);

//
// 1st Frame
  tft_SetTextCursor( 1, 1 );
  vy_top = tft.getCursorY();
  tft_SetTextCursor( 1, 3 );
  vy_bottom = tft.getCursorY();
  tft.drawRoundRect(tft.getCursorX(), vy_top, tft.width()-1, vy_bottom - vy_top, 10, foreground);
  tft.drawFastVLine( (tft.width()/2)-2*2*tft_char_width, vy_top, vy_bottom - vy_top, foreground);
  tft.drawFastVLine( (tft.width()/2)+2*2*tft_char_width, vy_top, vy_bottom - vy_top, foreground);

//
//2nd frame
  tft_SetTextCursor( 1, 4 );
  vy_top = tft.getCursorY();
  tft_SetTextCursor( 1, tft_lines-3 );
  vy_bottom = tft.getCursorY();
  tft_SetTextCursor( 1, 4 );
  tft.drawRoundRect(tft.getCursorX(), vy_top, tft.width()-1, vy_bottom - vy_top, 10, foreground);

//
//new 3rd frame
  tft_SetTextCursor( 1, tft_lines-1 );
  vy_top = tft.getCursorY();
  tft_SetTextCursor( 1, tft_lines+1 );
  vy_bottom = tft.getCursorY();
  tft_SetTextCursor( 1, tft_lines-1 );
  tft.drawRoundRect(tft.getCursorX(), vy_top, tft.width()-1, vy_bottom - vy_top, 10, foreground);
  tft.drawFastVLine( (tft.width()/2) - 1, vy_top, vy_bottom - vy_top, foreground);
}

//
// ************
//
void tft_DateDot1Refresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  int savX, savY;
  static unsigned long mSecs;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor(POS_DATE_DOT1_X, POS_DATE_DOT1_Y);
    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor);
    tft.setCursor(POS_DATE_DOT1_X, POS_DATE_DOT1_Y);
    tft.print(".");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}

//
// ************
//
void tft_DateDot2Refresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  int savX, savY;
  static unsigned long mSecs;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor(POS_DATE_DOT2_X, POS_DATE_DOT2_Y);
    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor );
    tft.setCursor(POS_DATE_DOT2_X, POS_DATE_DOT2_Y);
    tft.print(".");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}

//
// ************
//
void tft_DateRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, int day, int month, int year )
{
  static unsigned long mSecs;
  static int lastDay, lastMonth, lastYear;
  int savX, savY;
  char cOutput[5];


  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL ) {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    if( day != lastDay ) {
      lastDay = day;
      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_DATE_DAY_X, POS_DATE_DAY_Y);
      tft.print("  ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_DATE_DAY_X, POS_DATE_DAY_Y);
      sprintf(cOutput, "%02d", day);
      tft.print(cOutput);
      tft.setTextColor(textColor, bgColor);
    }

    if( month != lastMonth ) {
      lastMonth = month;
      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_DATE_MONTH_X, POS_DATE_MONTH_Y);
      tft.print("  ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_DATE_MONTH_X, POS_DATE_MONTH_Y);
      sprintf(cOutput, "%02d", month);
      tft.print(cOutput);
      tft.setTextColor(textColor, bgColor);
    }

    if( year != lastYear ) {
      lastYear = year;
      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_DATE_YEAR_X, POS_DATE_YEAR_Y);
      tft.println("    ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_DATE_YEAR_X, POS_DATE_YEAR_Y);
      sprintf(cOutput, "%04d", year);
      tft.print(cOutput);
      tft.setTextColor(textColor, bgColor);
    }

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor);
    mSecs = millis();
  }
}
//
// ************
//
void tft_WDayRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, int wDay )
{
  static unsigned long mSecs;
  static int lastWDay = -1;
  int savX, savY;
  const String DAY[] = {"So","Mo", "Di", "Mi", "Do", "Fr", "Sa", "--"};

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL ) {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    if( wDay != lastWDay ) {
      lastWDay = wDay;
      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_WDAY_X, POS_WDAY_Y);
      tft.print("  ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_WDAY_X, POS_WDAY_Y);
      tft.print(DAY[(wDay >= 0 && wDay < 7) ? wDay : 7]);
      tft.setTextColor(textColor, bgColor);
    }

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}
//
// ************
//
void tft_TimeRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, int hour, int minute, int second )
{
  static int lastHour = -1, lastMinute = -1, lastSecond = -1;
  static unsigned long mSecs;

  char cOutBuf[3];
  int savX, savY;

  if (millis() - mSecs >= DISPLAY_REFRESH_INTERVALL) {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    if (hour != lastHour) {
      lastHour = hour;
      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_TIME_HOUR_X, POS_TIME_HOUR_Y);
      tft.println("  ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_TIME_HOUR_X, POS_TIME_HOUR_Y);
      sprintf(cOutBuf, "%02d", hour);
      tft.print(cOutBuf);
      tft.setTextColor(textColor, bgColor);
    }

    if( minute != lastMinute ) {
      lastMinute = minute;
      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_TIME_MINUTE_X, POS_TIME_MINUTE_Y);
      tft.println("  ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_TIME_MINUTE_X, POS_TIME_MINUTE_Y);
      sprintf(cOutBuf, "%02d", minute);
      tft.print(cOutBuf);
      tft.setTextColor(textColor, bgColor);
    }

    if( second != lastSecond ) {
      lastSecond = second;
      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_TIME_SECOND_X, POS_TIME_SECOND_Y);
      tft.println("  ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_TIME_SECOND_X, POS_TIME_SECOND_Y);
      sprintf(cOutBuf, "%02d", second);
      tft.print(cOutBuf);
      tft.setTextColor(textColor, bgColor);
    }

    tft.setCursor( savX, savY );
    mSecs = millis();
  }
}
//
// ************
//
void tft_TimeQuoteRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  int savX, savY;
  static unsigned long mSecs;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL ) {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor(POS_TIME_QUOTE_X, POS_TIME_QUOTE_Y);
    tft.print(" ");

    tft.setTextColor(displayTextColor, displayBgColor);
    tft.setCursor(POS_TIME_QUOTE_X, POS_TIME_QUOTE_Y);
    tft.print(":");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}
//
// ************
//
void tft_TimeDotRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  int savX, savY;
  static unsigned long mSecs;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL ) {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor(POS_TIME_DOT_X, POS_TIME_DOT_Y);
    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor );
    tft.setCursor(POS_TIME_DOT_X, POS_TIME_DOT_Y);
    tft.print(":");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}
//
// ************
//
void tft_TempRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, int degree, int tenth )
{
  static int lastDegree, lastTenth;
  static unsigned long mSecs;
  static int DStatus;

  int savX, savY;
  char cOutBuf[3];


  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    if( degree != lastDegree ||
        tenth != lastTenth )
    {
      savX = tft.getCursorX();
      savY = tft.getCursorY();

      lastDegree = degree;
      lastTenth = tenth;

      tft.setTextSize( MEDIUM_TEXT_SIZE );

      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(  POS_TEMP_TEMP_X, POS_TEMP_TEMP_Y );
      tft.print(" ");
      sprintf(cOutBuf, "%2d", degree);
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(  POS_TEMP_TEMP_X, POS_TEMP_TEMP_Y );
      tft.print(cOutBuf);

      if( millis() - mSecs >= 1000 )
      {

        if( DStatus <= 0)
        {
          DStatus = 1;
          tft.print(".");
        }
        else
        {
          DStatus = 0;
          tft.print(" ");
        }
      }

      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(  POS_TEMP_TENTH_X, POS_TEMP_TENTH_Y );
      tft.print(" ");
      sprintf(cOutBuf, "%2d", tenth);
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(  POS_TEMP_TENTH_X, POS_TEMP_TENTH_Y );
      tft.print(cOutBuf);

      tft.setTextSize( SMALL_TEXT_SIZE );

      tft.setCursor( savX, savY );
      tft.setTextColor(textColor, bgColor);

    }

    mSecs = millis();
  }
}

void tft_TempDotRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  int savX, savY;
  static unsigned long mSecs;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextSize( MEDIUM_TEXT_SIZE );

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor(POS_TEMP_DOT_X, POS_TEMP_DOT_Y);
    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor );
    tft.setCursor(POS_TEMP_DOT_X, POS_TEMP_DOT_Y);
    tft.print(".");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);

    tft.setTextSize( SMALL_TEXT_SIZE );

    mSecs = millis();
  }
}
//
// ************
//
void tft_LoadRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, int aWeight, float aTemp )
{
  static int lastWeight;
  static float lastTemp;
  static unsigned long mSecs;
  int savX, savY;
  char cOutBuf[10];

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    if ( aWeight != lastWeight || aTemp != lastTemp ) {
      savX = tft.getCursorX();
      savY = tft.getCursorY();

      lastWeight = aWeight;
      lastTemp = aTemp;

      tft.setTextSize( MEDIUM_TEXT_SIZE );

      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(  POS_LOAD_LOAD_X, POS_LOAD_LOAD_Y );

      tft.print("     ");
      sprintf(cOutBuf, "%4dg", aWeight);
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(  POS_LOAD_LOAD_X, POS_LOAD_LOAD_Y );
      tft.print(cOutBuf);


      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(  POS_LOAD_LOAD_X, POS_LOAD_TENTH_Y );
      tft.print("     ");
//      sprintf(cOutBuf, "%5.2f", aTemp);
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(  POS_LOAD_LOAD_X, POS_LOAD_TENTH_Y );
//      tft.print(cOutBuf);
      tft.print(String(aTemp, 2));

      tft.setCursor( savX, savY );
      tft.setTextColor(textColor, bgColor);

      tft.setTextSize( SMALL_TEXT_SIZE );
    }

    mSecs = millis();
  }
}
//
// ************
//
void tft_LoadDotRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  int savX, savY;
  static unsigned long mSecs;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextSize( MEDIUM_TEXT_SIZE );

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor(POS_LOAD_DOT_X, POS_LOAD_DOT_Y);

    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor );
    tft.setCursor(POS_LOAD_DOT_X, POS_LOAD_DOT_Y);

    tft.print(".");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);

    tft.setTextSize( SMALL_TEXT_SIZE );
    mSecs = millis();
  }
}
//
// ************
//
void tft_IPDot1Refresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  static unsigned long mSecs;
  int savX, savY;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor( POS_IP_DOT1_X, POS_IP_DOT1_Y );
    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor );
    tft.setCursor( POS_IP_DOT1_X, POS_IP_DOT1_Y );
    tft.print(".");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}
//
// ************
//
void tft_IPDot2Refresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  static unsigned long mSecs;
  int savX, savY;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor( POS_IP_DOT2_X, POS_IP_DOT2_Y );
    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor );
    tft.setCursor( POS_IP_DOT2_X, POS_IP_DOT2_Y );
    tft.print(".");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}
//
// ************
//
void tft_IPDot3Refresh(int displayTextColor, int displayBgColor, int textColor, int bgColor )
{
  static unsigned long mSecs;
  int savX, savY;

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )
  {
    savX = tft.getCursorX();
    savY = tft.getCursorY();

    tft.setTextColor(displayBgColor, displayBgColor);
    tft.setCursor( POS_IP_DOT3_X, POS_IP_DOT3_Y );
    tft.print(" ");
    tft.setTextColor(displayTextColor, displayBgColor );
    tft.setCursor( POS_IP_DOT3_X, POS_IP_DOT3_Y );
    tft.print(".");

    tft.setCursor( savX, savY );
    tft.setTextColor(textColor, bgColor);
    mSecs = millis();
  }
}
//
// ************
//
void tft_IPRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, IPAddress ip)
{
  static int lastAByte, lastBByte, lastCByte, lastDByte;
  static unsigned long mSecs;
  int savX, savY;
  char cOutBuf[6];

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL ) {
    if( ip[0] != lastAByte ||
        ip[1] != lastBByte ||
        ip[2] != lastCByte ||
        ip[3] != lastDByte )
    {
      savX = tft.getCursorX();
      savY = tft.getCursorY();

      lastAByte = ip[0];
      lastBByte = ip[1];
      lastCByte = ip[2];
      lastDByte = ip[3];

      tft.setTextColor(displayBgColor, displayBgColor);

      tft.setCursor( POS_IP_ABYTE_X, POS_IP_ABYTE_Y );
      tft.print("   ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor( POS_IP_ABYTE_X, POS_IP_ABYTE_Y );
      sprintf(cOutBuf, "%03d", ip[0]);
      tft.print(cOutBuf);

      tft.setCursor( POS_IP_BBYTE_X, POS_IP_BBYTE_Y );
      tft.print("   ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor( POS_IP_BBYTE_X, POS_IP_BBYTE_Y );
      sprintf(cOutBuf, "%03d", ip[1]);
      tft.print(cOutBuf);

      tft.setCursor( POS_IP_CBYTE_X, POS_IP_CBYTE_Y );
      tft.print("   ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor( POS_IP_CBYTE_X, POS_IP_CBYTE_Y );
      sprintf(cOutBuf, "%03d", ip[2]);
      tft.print(cOutBuf);

      tft.setCursor( POS_IP_DBYTE_X, POS_IP_DBYTE_Y );
      tft.print("   ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor( POS_IP_DBYTE_X, POS_IP_DBYTE_Y );
      sprintf(cOutBuf, "%03d", ip[3]);
      tft.print(cOutBuf);

      tft.setCursor( savX, savY );
      tft.setTextColor(textColor, bgColor);
    }

    mSecs = millis();
  }
}
//
// ************
//
void tft_FreeMemRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, unsigned long freemem )
{
  static unsigned long lastFreemem;
  static unsigned long mSecs;
  int savX, savY;
  char cOutBuf[6];

  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL )  {
    if( freemem != lastFreemem ) {
      savX = tft.getCursorX();
      savY = tft.getCursorY();

      lastFreemem = freemem;

      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_FREE_MEM_X, POS_FREE_MEM_Y);
      tft.print("     ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_FREE_MEM_X, POS_FREE_MEM_Y);
      sprintf(cOutBuf, "%4lu", freemem);
      tft.print(cOutBuf);

      // 684 M
      //
      tft.setCursor( savX, savY );
      tft.setTextColor(textColor, bgColor);
    }

    mSecs = millis();
  }
}

//
// ************
//
void tft_FreeSDRefresh(int displayTextColor, int displayBgColor, int textColor, int bgColor, unsigned long freesd )
{
//  static unsigned long mSecs;
  static unsigned long lastFreeSD;
  int savX, savY;
  char cOutBuf[6];


//  if( millis() - mSecs >= DISPLAY_REFRESH_INTERVALL ) {
    if( freesd != lastFreeSD ) {
      savX = tft.getCursorX();
      savY = tft.getCursorY();

      lastFreeSD = freesd;

      tft.setTextColor(displayBgColor, displayBgColor);
      tft.setCursor(POS_FREE_SD_X, POS_FREE_SD_Y);
      tft.print("     ");
      tft.setTextColor(displayTextColor, displayBgColor );
      tft.setCursor(POS_FREE_SD_X, POS_FREE_SD_Y);
      sprintf(cOutBuf, "%4lu", freesd);
      tft.print(cOutBuf);

      // 684 M
      //
      tft.setCursor( savX, savY );
      tft.setTextColor(textColor, bgColor);
    }
//   mSecs = millis();
// }
}


/** ***********************************************
 * Get back the current Sketch name and version.
 */
String getSketchVersion() {
  return String(String(F(PRG_NAME_SHORT)) + " " +
         String(F(PRG_VERSION)) + F(" | CompTime: ")  +
         __DATE__ + F(" - ") + __TIME__);
}

/** ***********************************************
 * Get back the current Sketch name and version for display
 */
String getSketchVersion4Display() {
  return String(String(F(PRG_NAME_SHORT)) + " - " + String(F(PRG_VERSION)));
}

/** ***********************************************
 * Create a date string with format "DD.MM-YYY" from global 'tblock' variable.
 * @return: String Format: "DD.MM-YYYY"
 * @todo: adjust comment
 */
String getDate(const unsigned int aOrder = 0) {
  char s[12];
  switch (aOrder) {
    case 0 :
      sprintf(s, "%02d.%02d.%04d", tblock->tm_mday, 1 + tblock->tm_mon, 1900 + tblock->tm_year);
      break;
    case 1 :
      sprintf(s, "%04d.%02d.%02d", 1900 + tblock->tm_year, 1 + tblock->tm_mon, tblock->tm_mday);
      break;
    default :
      return "-";
  }
  return (String(s));
}

/** ***********************************************
 * Create a time string with format"hh:mm:ss" from global 'tblock' variable.
 * @return: String Format: "hh:mm:ss"
 */
String getTime() {
  char s[12];
  sprintf(s, "%02d:%02d:%02d", tblock->tm_hour, tblock->tm_min, tblock->tm_sec);
  return (String(s));
}

/** ***********************************************
 * Create a full timestamp string with format"hh:mm:ss" from global 'tblock' variable.
 * @return: String Format: "hh:mm:ss"
 */
String getDateAndTime(const unsigned int aOrder = 0) {
  return getDate(aOrder) + "   " + getTime();
}

/** ***********************************************
 * Convert the given  address array into a printable format
 * @return: "111.222.333.444" - Format (decimal)
 */
const String getCurrentIpAsString(IPAddress aIp) {
  char s[18];
  sprintf(s, "%d.%d.%d.%d", aIp[0], aIp[1], aIp[2], aIp[3]);
  return(String(s));
}

/** ***********************************************
 * Get the current time (UTC) from NTP server and fill the 'date" structure
 */
time_t getDateTimeNTP() {
  ulSecs2000_timer=getNTPTimestamp(ipAddrNTPServer)+3600;    // add +3600 sec (1h) for MET
  epoch_to_date_time(date, ulSecs2000_timer); // fresh up the date structure
  ulSecs2000_timer -= millis()/1000;          // keep distance to millis counter at now
  Serial << F("Current Time UTC from NTP server: ") << epoch_to_string(ulSecs2000_timer) << endl;
  ntpdRefreshTime = ulSecs2000_timer + NTP_REFRESH_AFTER; // setze erneutes Zeitholen
  return ulSecs2000_timer;
}

/** ***********************************************
 *  Starts the WiFi client and get the current time from a ntp server
 */
void WiFiStart() {
  int connectionAttempts = 0;
  bool tryConnect = 0;
  while (WiFiMulti.run() != WL_CONNECTED) {
    tryConnect = 1;
    Serial << F(".");
    connectionAttempts++;
    delay(500);
    if (connectionAttempts > 240) {   // 120 seconds
      Serial << endl << F("[Warning]: could not connect to WLAN") << endl;
      Serial << F("  ==> reset now!") << endl;
      ESP.restart();
      delay(1000);     // only for depricating the next print out.
    }
  }
  if (tryConnect) Serial << endl << F("WiFi (re)connected, local IP: ") << WiFi.localIP() << endl;
}


/** ***********************************************
 * Send data to a valid ThingsSpeak account
*/
void saveData2ThingsSpeak() {
  const int maxSendRepeats = 10;
  const int waitTimeBetweenSend_ms = 5000; // 5 sec.

  String request= "/update?api_key=" + thingSpeakServer[0].tsAPIKey;
  String val = "";

  char buf[20];
  for (unsigned int tss=0; tss < sizeof(thingSpeakServer)/sizeof(struct tsData); tss++) {
    for (unsigned int tsd=1; tsd < numberOfTSFields; tsd++) {
      if (thingSpeakServer[tss].tsDataSet[tsd].length() != 0) {
        val = thingSpeakServer[tss].tsDataSet[tsd];
        sprintf(buf, "&field%d=%s", tsd, val.c_str());
        request += String(buf);
      }
    }
  }

  Serial << F("Try to send to ThingsSpeak ") << endl;
#ifdef DEBUG
  Serial << F("REST request: ") << request << endl;
#endif

  HTTPClient http;
  int sendLoop = 0;
  do {
    http.begin(thingSpeakServer[0].tsServerIP.c_str(), 80, request);
    int httpCode = http.GET();
    if (httpCode) {
      // HTTP header has been send and Server response header has been handled
      Serial << F("[HTTP] GET... code: ") << httpCode;

      // don't found at server ?
      if (httpCode != 200) {
        Serial << F("   ... failed, no connection or no HTTP server\n") << http.getString() << endl;
      }
      else {
        Serial << F("   ... successfull") << endl;
        return;
      }
    }
    sendLoop++;
    delay(waitTimeBetweenSend_ms);
  } while ( sendLoop < maxSendRepeats);

  if (sendLoop >= maxSendRepeats) {
    Serial << F("Can't sent datagram to Thingspeak server, not reachable.") << endl;
  }
  Serial << endl;
}


/** ***********************************************
 * Simple median/average calculation
 * ------------------------------------
 * Getting array will be sorted and now get the average over the middle values
 */
float median(float *values, size_t arraySize) {
  float tmp = 0;     // set to 0, make the compiler happy :-)
  const size_t relVal = 2;   // +- 2 Werte + 1 für die Mittelwertberechnung

  for (size_t i=0; i < arraySize-2; i++) {
    for (size_t j=arraySize-1; j > i; j--) {
      if ( values[j] < values[j-1] ) {
        tmp = values[j];
        values[j] = values[j-1];
        values[j-1] = tmp;
      }
    }
  }

/*for (int i=0; i<arraySize; i++) {
    Serial << "i[" << i << "] = " << values[i] << endl;
  }
*/
  tmp = 0.0;
  for (size_t i=arraySize/2-relVal; i<arraySize/2+relVal+1; tmp +=values[i++]) {}
//  Serial << "tmp: " << tmp << endl;
  return tmp/(relVal*2+1);
}

/** ***********************************************
 *
 */
float messung() {
//  Serial << "measurementLoop: " << measurementLoop << endl;
  if (measurementLoop < MEASURING_COUNT) {
    if (hx711.readyToSend()) {
      dataArray[measurementLoop] = hx711.read()/divisor;
//      Serial.println(dataArray[measurementLoop]);
      measurementLoop++;
    }
  }
  else {
    measurementLoop = 0;
    float x = median(dataArray, MEASURING_COUNT);
    return round(factorCalibrate2g * (x-correkture2null));
  }
  return weightCurrent;
}

/** ***********************************************
 * Write out a simple string in case of task not found
 */
void handleNotFound(){
  server.send(404, "text/plain", "File Not Found\n\n");
}

/** ***********************************************
 * Write out a simple hello string
 */
void handleRoot() {
  server.send(200, "text/plain", "hello from esp8266!");
}

/** ***********************************************
 * Write out a simple string with the weight are measured
 */
void handleVersion() {
  server.send(200, "text/plain", getSketchVersion());
}

/** ***********************************************
 * Write out a simple string with the weight are measured
 */
void handleWeigth() {
  server.send(200, "text/plain", String(weightCurrent));
}

/** ***********************************************
 * Write out a simple string with the temperature are measured
 */
void handleTemperature() {
  server.send(200, "text/plain", String(temperature));
}

/** ***********************************************
 * Write out a full data set in json format
 */
void handleJSON() {
  StaticJsonBuffer<200> jsonBuf;

  JsonObject& root = jsonBuf.createObject();
  root["Version"] = getSketchVersion();
  root["Date"] = getDateAndTime();
  root["weight"] = weightCurrent;
  root["temperature"] = temperature;

  String outPrint;
  root.printTo(outPrint);
  server.send(200, "text/json", outPrint);
}
/*
 *
 */
bool readDS18B20() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  temperature_valid = false;

  if (!DS18B20.search(addr)) {
//    Serial << F("No more addresses.") << endl;
    DS18B20.reset_search();
    return temperature_valid;
  }

/*
  Serial << F("ROM =");
  for (i = 0; i < 8; i++) { Serial << F(" ") << String(addr[i], HEX); }
*/
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial << F("CRC is not valid!") << endl;
      return temperature_valid;
  }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10: type_s = 1; break;
    case 0x28: type_s = 0; break;
    case 0x22: type_s = 0; break;
    default:   Serial << F("Device is not a DS18x20 family device.") << endl;
               return temperature_valid;
  }

  DS18B20.reset();
  DS18B20.select(addr);
  DS18B20.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = DS18B20.reset();
  DS18B20.select(addr);
  DS18B20.write(0xBE);         // Read Scratchpad

//  Serial << F("  Data = ") << String(present, HEX) << F(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = DS18B20.read();
//    Serial << String(data[i], HEX) << F(" ");
  }
//  Serial << F("  CRC=") << String(OneWire::crc8(data, 8), HEX) << endl;

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }

  temperature_valid = true;
  temperature = (float)raw / 16.0;
//  Serial << F("  Temperature = ") << temperature << F(" Celsius, ") << endl;
  return temperature_valid;
}


void setup() {
  Serial.begin(115200);
  Serial.println(F("Start"));

  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

  tft_char_width = FIXED_FONT_CHAR_WIDTH;
  tft_char_height = FIXED_FONT_CHAR_HEIGHT;

  tft_rotation = ROTATION_90;       // ROTATION_NO;
  tft_text_size = SMALL_TEXT_SIZE;
  tft_lines = 0;
  tft_columns = 0;

  tft_hor_spacing = FIXED_HORIZONTAL_SPACING;
  tft_vert_spacing = FIXED_VERTICAL_SPACING;

  tft.begin();

  Serial.println(F("display_out"));
  tft.setRotation(tft_rotation);
  tft.setTextSize(tft_text_size);
  tft_MapTFTParams();
  tft_drawFrames( ILI9341_BLACK, ILI9341_WHITE, ILI9341_CYAN );
  delay(500);
  tft_drawFrameLabels();

    // set multiple access points for better mobility
  for (unsigned int i=0; i < sizeof(apList)/sizeof(struct accessPoint); i++) {
    Serial << F("add AP(") << i << F(") : ") << apList[i].SSID << endl;
    WiFiMulti.addAP(apList[i].SSID, apList[i].PASSWORD);
  };

  // get the mac address for identifying of the node.
  WiFi.macAddress(MACArray);
  macID = String(MACArray[0], HEX) + String(MACArray[1], HEX) +
          String(MACArray[2], HEX) + String(MACArray[3], HEX);
  macID.toUpperCase();
  Serial << F("MAC: ") << macID << F("   ====   ChipId: ") << ESP.getChipId() << endl;

  WiFiStart();
  getDateTimeNTP();

  server.on("/", handleRoot);
  server.on("/version", handleVersion);
  server.on("/weigth", handleWeigth);
  server.on("/temp", handleTemperature);
  server.on("/json", handleJSON);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  tsSendTime = ulSecs2000_timer + millis()/1000UL + 946684800UL;

//  EEPROM.begin(EEPROM_SIZE);
}


void loop(void)
{
  looptime = millis();

  rawtime = ulSecs2000_timer + millis()/1000UL + 946684800UL;  // in seconds since ...
  tblock = localtime(&rawtime);

  server.handleClient();

  weight = messung();
//  Serial << "weight:" << weight << "\t\tweigthCurrent: " << weightCurrent << endl;
  if (weight != weightCurrent) weightCurrent = weight;

  tft_DateRefresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, tblock->tm_mday, 1 + tblock->tm_mon, 1900 + tblock->tm_year);
  tft_DateDot1Refresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK );
  tft_DateDot2Refresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK );

  tft_WDayRefresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, tblock->tm_wday );

  tft_TimeRefresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, tblock->tm_hour, tblock->tm_min, tblock->tm_sec);
  tft_TimeQuoteRefresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK );
  tft_TimeDotRefresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK );

//  tft_TempRefresh(ILI9341_GREEN, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, 23, 3 );
//  tft_TempDotRefresh(ILI9341_GREEN, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK);

  tft_LoadRefresh(ILI9341_YELLOW, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, int(weightCurrent), temperature );
//  tft_LoadDotRefresh(ILI9341_YELLOW, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK);

  tft_IPRefresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, WiFi.localIP());
  tft_IPDot1Refresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK );
  tft_IPDot2Refresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK );
  tft_IPDot3Refresh(ILI9341_WHITE, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK );

  tft_FreeMemRefresh(ILI9341_GREEN, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, ESP.getFreeHeap() );

//  tft_FreeSDRefresh(ILI9341_GREEN, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, int(weightCurrent) );

  // send only one time per minute
/*
  if (rawtime - tsSendTime > 60 ) {
    thingSpeakServer[0].tsDataSet[3] = String(weightCurrent);
    thingSpeakServer[0].tsDataSet[4] = String(temperature);

    saveData2ThingsSpeak();
    tsSendTime = rawtime;
  }
*/
  if (!(tempLoopCnt++ % 4)) {
    readDS18B20();
  }


  looptime = millis() - looptime;

  tft_FreeSDRefresh(ILI9341_GREEN, ILI9341_BLACK, ILI9341_WHITE, ILI9341_BLACK, int(looptime) );

  Serial << F("Gewicht: ") << weightCurrent << F("g") << F("\tTemp:") << temperature << F("\tlooptime: ") << looptime << endl;

  delay( (looptime > 500) ? 500 : 500 - looptime) ;

}


