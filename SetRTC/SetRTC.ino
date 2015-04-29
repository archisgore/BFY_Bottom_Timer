#include <Wire.h>



int seconds; //00-59;
int minutes; //00-59;
int hours;//1-12 - 00-23;
int day;//1-7
int date;//01-31
int month;//01-12
int year;//0-99;

void setup() {
  // put your setup code here, to run once:
    ////////////////////////////////
  seconds = 00;
  minutes = 34;
  hours = 23;
  day = 1;
  date = 27;
  month = 4;
  year = 15;
  initChrono();//just set the time once on your RTC
  ///////////////////////////////

}



void loop() {
}



void set_date()
{
  Wire.beginTransmission(104);
  Wire.write(3);
  Wire.write(decToBcd(day));
  Wire.write(decToBcd(date));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}
void get_date()
{
  Wire.beginTransmission(104); 
  Wire.write(3);//set register to 3 (day)
  Wire.endTransmission();
  Wire.requestFrom(104, 4); //get 5 bytes(day,date,month,year,control);
  day   = bcdToDec(Wire.read());
  date  = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year  = bcdToDec(Wire.read());
}

void set_time()
{
   Wire.beginTransmission(104);
   Wire.write(0);
   Wire.write(decToBcd(seconds));
   Wire.write(decToBcd(minutes));
   Wire.write(decToBcd(hours));
   Wire.endTransmission();
}
void get_time()
{
  Wire.beginTransmission(104); 
  Wire.write(0);//set register to 0
  Wire.endTransmission();
  Wire.requestFrom(104, 3);//get 3 bytes (seconds,minutes,hours);
  seconds = bcdToDec(Wire.read() & 0x7f);
  minutes = bcdToDec(Wire.read());
  hours = bcdToDec(Wire.read() & 0x3f);
  
 


  
}
void initChrono()
{
  set_time();
  set_date();
}

byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}
