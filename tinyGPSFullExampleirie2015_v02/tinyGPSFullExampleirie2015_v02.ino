#include <SerialLCD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

//点灯時間定義
#define LIGHTUP_HOUR 8  //time use by UTC. 8 is 17 in JST
#define LIGHTUP_MIN 0
#define LIGHTDN_HOUR 13  //time use by UTC. 13 is 22 in JST
#define LIGHTDN_MIN 00

#define ONDAY_def  3 // 0　is Sunday, therefore 3 is Wednesday,
#define ON_HOUR 5l   //5l is 5 hours  It's lightingn time 

/*点灯式用
#define LIGHTUP_HOUR 17
#define LIGHTUP_MIN 18
*/

#define RL_SET 12
#define RL_RESET 13


/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 2, TXPin = 3; //ublox 4H Rx2TX3 , Grobe rx3tx2
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
//AltSoftSerial ss;

// initialize the library
SerialLCD slcd(6,7);//this is a must, assign soft serial pins

void setup()
{
   pinMode(RL_SET, OUTPUT);
    pinMode(RL_RESET, OUTPUT);
  digitalWrite(RL_SET,LOW);digitalWrite(RL_RESET,LOW);delay(500);
  digitalWrite(RL_RESET,HIGH);delay(500);digitalWrite(RL_RESET,LOW);
  
  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));
 ss.end();

/* 
 slcd.begin();
  // Print a message to the LCD.
  slcd.print("hello, world!");
  //slcd.end();
  */
  ss.begin(GPSBaud);

}

void loop()
{
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
 
 /*
  //printValue();
     slcdPrint();
 */
 //Light up ? 

  smartDelay(1000);
  SW_loop();
 
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void SlcdprintFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      slcd.print('*');
    slcd.print(' ');
  }
  else
  {
    int vala,valb;
    
    vala =  (int)val;
    valb=  (int)((val-vala)*10000);
    
    char sz[32];
    sprintf(sz, "%d.%d ", vala,valb);
  //  slcd.print(val,3);// slcd.print('.'); slcd.print(valb);
   slcd.print(sz);
    
  }
  
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void SlcdprintDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    slcd.print("********** ");
  }
  else
  {
    char sz[32];
    int heisei;
    heisei=27+(d.year()-2015);
    sprintf(sz, "%02d/%02d/H%02d ", d.month(), d.day(), heisei);
    slcd.print(sz);
  }
  
  if (!t.isValid())
  {
    slcd.print("******** ");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d%02d%02d ", t.hour(), t.minute(), t.second());
    slcd.print(sz);
  }

//  printInt(d.age(), d.isValid(), 5);
}


static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

static void printValue(void)
{
 Serial.println(gps.location.lat()); // Latitude in degrees (double)
Serial.println(gps.location.lng()); // Longitude in degrees (double)
Serial.print(gps.location.rawLat().negative ? "-" : "+");
Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
Serial.print(gps.location.rawLng().negative ? "-" : "+");
Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
Serial.println(gps.date.year()); // Year (2000+) (u16)
Serial.println(gps.date.month()); // Month (1-12) (u8)
Serial.println(gps.date.day()); // Day (1-31) (u8)
Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
Serial.println(gps.time.hour()); // Hour (0-23) (u8)
Serial.println(gps.time.minute()); // Minute (0-59) (u8)
Serial.println(gps.time.second()); // Second (0-59) (u8)
Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
Serial.println(gps.speed.knots()); // Speed in knots (double)
Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
Serial.println(gps.speed.mps()); // Speed in meters per second (double)
Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
Serial.println(gps.course.deg()); // Course in degrees (double)
Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
Serial.println(gps.altitude.meters()); // Altitude in meters (double)
Serial.println(gps.altitude.miles()); // Altitude in miles (double)
Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
Serial.println(gps.altitude.feet()); // Altitude in feet (double)
Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)
   
 
}

void slcdPrint(){
  ss.end();
 // slcd.begin();

  slcd.setCursor(0, 0);
 
 SlcdprintFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  // print the number of seconds since reset:
//slcd.println(gps.location.lat(),DEC); // Latitude in degrees (double)
  slcd.setCursor(8, 0);
//slcd.println(gps.location.lng(),DEC); // Longitude in degrees (double)
 SlcdprintFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  slcd.setCursor(0, 1);
SlcdprintDateTime(gps.date, gps.time);

// slcd.end();
  
  ss.begin(GPSBaud);
  
  
}


void SW_loop()
{
  int ONDAY; 
  static int Backup_time_stamp = 0;
 
  ONDAY = ONDAY_def;
  
  if( gps.date.isValid() == 0) return;
 
  Serial.println("youbi_hantei");
  int y = gps.date.year();
  int m = gps.date.month();
  int d = gps.date.day();
  int h = gps.time.hour();
  int mi = gps.time.minute();
    int s = gps.time.second();
    
    Serial.print(y);Serial.print(' ');   Serial.print(m);Serial.print(' ');    Serial.print(d);Serial.print(' ');
    Serial.print(h);Serial.print(' ');    Serial.print(mi);Serial.print(' ');    Serial.print(s);Serial.println(' ');
    
    if(is_suiyo(y,m,d,ONDAY) == 0) {Backup_time_stamp = 0; return;}
    if(Backup_time_stamp == 2) return; //今日は点灯済み
    
      int flag = 0;
      int utc_min,utc_on_min,utc_off_min;
      
      utc_min = h * 60 + mi;
      utc_on_min  = LIGHTUP_HOUR * 60 +LIGHTUP_MIN;
      utc_off_min = LIGHTDN_HOUR * 60 +LIGHTDN_MIN;
      
        Serial.print(utc_min);Serial.print("  ");
        Serial.print(utc_on_min);Serial.println(' ');
      
      if(utc_min >= utc_on_min && Backup_time_stamp== 0 ){ 
        Backup_time_stamp=1;
        Serial.println("light up!");
        digitalWrite(RL_SET,HIGH);delay(1000);digitalWrite(RL_SET,LOW);
      }
  
      if(utc_min >= utc_off_min && Backup_time_stamp == 1 ){
        Backup_time_stamp=2;
        digitalWrite(RL_RESET,HIGH);delay(500);digitalWrite(RL_RESET,LOW);
        Serial.println("light down");
      }
  
}


int youbi(int y,int m,int d)
{
  if(m==1 || m==2){
    y--;
    m+=12;
  }
  return ((y + y/4 - y/100 + y/400 + (13 * m + 8)/5 + d)%7);
}


//曜日判定　0:日曜～6:土曜
int is_suiyo(int _year, int _month, int _day, int ONday)
{
  int _youbi = youbi(_year,_month,_day);
 
  Serial.print("Youbi is "); Serial.println(_youbi);

  if(_youbi == ONday){
      Serial.print("Today is ON day!  ");
    return 1;
  }
  else 
    return 0;

}




