#include <avr/sleep.h>
#include <Time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>
#include <SD.h>



/*************************************************** 
B "Eff" Y Bottom Timer: Because 'Eff You!

This project began when I got mad at various bottom timers (dive computers)
out there for providing everything but what I needed. They have everything
from fancy cables, and wet contacts, and Comic-book heads-up displays
and science fiction stuff. What they don't have, is the damn reliability to
not flood constantly or run out of batteries all the time.

I need the following out of my computers:
1. Commodity parts - I want to buy batteries with a standard
profile (aka I should be able to replace the with whatever I find
that day.)

2. I'm sick of flooded compartments. Seriously. It's 2015. We should
have inductive charging by now.

3. BLE connectivity. No special addons. No special bullshit.

4. Funnily enough, I was willing to pay a high cost for all the above,
but decided I could build something better, cheaper, and simpler myself using off-the-shelf parts.

5. Remember settings across resets. RTCs have been in computers since the 60s. Damn!

For this code to work you need:
1. Any Arduino compatible board with 7 digital I/O pins to spare (analog pins double up as Digital pins), 
   and 2 I2C pins to spare typically two of the Analog pins.
2. The OpenROV IMU Sensor package which includes, compass, gyroscope, depth, temperature and accelerometer.
   Packs quite the punch. One module has all the sensors needed for a bottom timer.
3. Some sort of OLED SPI screen (I used Adafruit's 1.5" version, but doesn't matter.)


Optional:
1. I added a wireless Qi charger + LiPo battery + the adafruit trinket Lipo charger backpack.
   This means my computer is never opened. I place it on a Qi charger when I'm done with a dive,
   and the thing keeps on going.
2. I also have an SPI BLE interface (or you can just use something like Bluduino.
3. A battery-backed RTC so I don't have to keep setting the time every time the battery dies.

This code is demo-only - i.e. it shows depth, time and temperature on the OLED screen. But that's pretty much
all you need to build complex math, algorithms or whatever you wish. Thanks to the acceleremoter, you can
easily build a tap-based interface.

My current "prototype" costs me $200.

Additionally, I connected a spring switch to Pin 2 of the Arduino (hardware interrupt)
to wake it up by shaking. This means my low-power mode is REALLY low power with no mechanical contacts.

This code has been copied from various places but modified significantly. Plenty of exemplary Arduino
code is used as-is. It's really THAT simple and THAT cheap to build a decent bottom timer.

So go forth and build your own - of variying power and varying capacities.

 ****************************************************/

// You can use any (4 or) 5 pins 
#define sclk 13 //CL
#define mosi 11 //SI
#define miso 12 //SO
#define OLED_CS   10 //OC

#define dc   9  //DC
#define SD_CS 8  //SC
#define rst  7 //R

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#define IDLE_INTERVAL_BEFORE_SLEEP 10 //5 minutes == 300 seconds


// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#define WAKE_PIN 2

#define WRITE_TEXT(x, y, text) writeText(x, y, text)


const int IMUDevAddress = 0x76;  // 7-bit I2C address of the MS5803
const int RTCDevAddress = 104; //7-bit I2C address of the RTC

// Here are the commands that can be sent to the 5803

const byte Reset = 0x1E;
const byte D1_256 = 0x40;
const byte D1_512 = 0x42;
const byte D1_1024 = 0x44;
const byte D1_2048 = 0x46;
const byte D1_4096 = 0x48;
const byte D2_256 = 0x50;
const byte D2_512 = 0x52;
const byte D2_1024 = 0x54;
const byte D2_2048 = 0x56;
const byte D2_4096 = 0x58;
const byte AdcRead = 0x00;
const byte PromBaseAddress = 0xA0;

unsigned int CalConstant[8];  // Matrix for holding calibration constants

long AdcTemperature, AdcPressure;  // Holds raw ADC data for temperature and pressure
float Temperature, Pressure, TempDifference, Offset, Sensitivity;
float T2, Off2, Sens2;  // Offsets for second-order temperature computation
 
byte ByteHigh, ByteMiddle, ByteLow;  // Variables for I2C reads


int t_seconds; //00-59;
int t_minutes; //00-59;
int t_hours;//1-12 - 00-23;
int t_day;//1-7
int t_date;//01-31
int t_month;//01-12
int t_year;//0-99;


time_t timeOfLastInput;

char displaybuffer[20];
char filebuffer[20];

// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(OLED_CS, dc, mosi, sclk, rst);  

// Option 2: must use the hardware SPI pins 
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1351 tft = Adafruit_SSD1351(OLED_CS, dc, rst);


float const pow2_23 = pow(2, 23),
            pow2_33 = pow(2, 33),
            pow2_37 = pow(2, 37),
            pow2_16 = pow(2,16),
            pow2_7 = pow(2, 7),
            pow2_8 = pow(2, 8),
            pow2_15 = pow(2, 15),
            pow2_21 = pow(2, 21),
            pow2_15_multiplied_by_10000 = pow2_15 * 10000.00,
            nine_by_fivehundred = 9.0/500.0;
            
bool diveMode = false;
bool sleepMode = false;

File diveFile;


void setup(void) {
  Serial.begin(9600);
  Serial.println("Hello world!");
  setupSleep();
  setupOLED(); 
  setupSdCard();
  setupIMUSensor();
  setupMode();
  
  //Call this to reset the RTC
  //setupTime();
}

void setupTime() {
    t_seconds = 0;
    t_minutes = 35;
    t_hours = 20;
    t_day = 2;
    t_date = 28;
    t_month = 4;
    t_year = 2015;
    set_time();
    set_date();
}

void setupSleep() {
   pinMode(WAKE_PIN, INPUT_PULLUP);
   
   attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
   
   set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
   sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin                                   // wakeUpNow when pin 2 gets LOW
}

void setupMode() {
  tft.drawFastVLine(64, 0, 128, GREEN);
  tft.drawFastHLine(0, 64, 128, GREEN);
  
  tft.setTextColor(GREEN, BLACK);
  WRITE_TEXT(5, 0, "Temp");
  WRITE_TEXT(70, 0, "Depth");
  
  if (diveMode) { 
    WRITE_TEXT(5, 65, "Dive"); 
    tft.setTextColor(RED, BLACK);
  } else {
    WRITE_TEXT(5, 65, "Surface");
    tft.setTextColor(BLUE, BLACK);
  }
}

void setupOLED() {
   
  pinMode(OLED_CS, OUTPUT);
  digitalWrite(OLED_CS, HIGH);
     
  // initialize the OLED
  tft.begin();
  
  tft.fillScreen(BLACK);
}

void setupIMUSensor() {
  Wire.begin();
  // Reset the device and check for device presence 
  sendCommand(IMUDevAddress, Reset);
  delay(1000);
   
  // Get the calibration constants and store in array
  for (byte i = 0; i < 8; i++)
  {
    sendCommand(IMUDevAddress, PromBaseAddress + (2*i));
    Wire.requestFrom(IMUDevAddress, 2);
    while(Wire.available()){
      ByteHigh = Wire.read();
      ByteLow = Wire.read();
    }
    CalConstant[i] = (((unsigned int)ByteHigh << 8) + ByteLow);
  }
}

void setupSdCard() {

  if (!SD.begin(SD_CS)) {
    WRITE_TEXT(0, 0, "UNABLE TO OPEN SD CARD. Dive LOGGING will not work.");
    delay(20000);
  }
  //Serial.println("SD OK!");
}

void loop(){
  // Read the Device for the ADC Temperature and Pressure values
  sendCommand(IMUDevAddress, D1_512);
  delay(10);
  sendCommand(IMUDevAddress, AdcRead);
  Wire.requestFrom(IMUDevAddress, 3);
  while(Wire.available())
  {
    ByteHigh = Wire.read();
    ByteMiddle = Wire.read();
    ByteLow = Wire.read();
  }
  AdcPressure = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
  
//  Serial.print("D1 is: ");
//  Serial.println(AdcPressure);
  
  sendCommand(IMUDevAddress, D2_512);
  delay(10);
  sendCommand(IMUDevAddress, AdcRead);
  Wire.requestFrom(IMUDevAddress, 3);
  while(Wire.available())
  {
    ByteHigh = Wire.read();
    ByteMiddle = Wire.read();
    ByteLow = Wire.read();
  }
  AdcTemperature = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
 // Serial.print("D2 is: ");
//  Serial.println(AdcTemperature);
  
    
  // Calculate the Temperature (first-order computation)
  
  TempDifference = (float)(AdcTemperature - ((long)CalConstant[5] << 8));
  Temperature = (TempDifference * (float)CalConstant[6])/ pow2_23;
  float TemperatureMinus2000 = Temperature;
  Temperature = Temperature + 2000;  // This is the temperature in hundredths of a degree C
  
  // Calculate the second-order offsets
  
  if (Temperature < 2000.0)  // Is temperature below or above 20.00 deg C ?
  {
    T2 = 3 * TempDifference * TempDifference / pow2_33;
    Off2 = 1.5 * TemperatureMinus2000 * TemperatureMinus2000;
    Sens2 = 0.625 * TemperatureMinus2000 * TemperatureMinus2000;
  }
  else
  {
    T2 = (TempDifference * TempDifference) * 7 / pow2_37;
    Off2 = 0.0625 * TemperatureMinus2000 * TemperatureMinus2000; 
    Sens2 = 0.0;
  }
  
  // Check print the offsets
  
  //Serial.println("Second-order offsets are:");
  //Serial.println(T2);
  //Serial.println(Off2);
  //Serial.println(Sens2);
  
  
  // Print the temperature results
  
  //Temperature = Temperature / 100;  // Convert to degrees C
  Temperature = Temperature * nine_by_fivehundred + 32;
  //Serial.print("First-Order Temperature in Degrees C is ");
  //Serial.println(Temperature);
  //Serial.print("Second-Order Temperature in Degrees C is ");
  //Serial.println(Temperature - (T2 / 100));
  
  // Calculate the pressure parameters
  
  Offset = (float)CalConstant[2] * pow2_16;
  Offset = Offset + ((float)CalConstant[4] * TempDifference / pow2_7);

  Sensitivity = (float)CalConstant[1] * pow2_15;
  Sensitivity = Sensitivity + ((float)CalConstant[3] * TempDifference / pow2_8);
  
  // Add second-order corrections
  
  Offset = Offset - Off2;
  Sensitivity = Sensitivity - Sens2;
  
  // Calculate absolute pressure in bars

  Pressure = (float)AdcPressure * Sensitivity / pow2_21;
  Pressure = Pressure - Offset;
  Pressure = Pressure / pow2_15_multiplied_by_10000;
  
  // Convert to psig and display
  
  Pressure = Pressure - 1.015;  // Convert to gauge pressure (subtract atmospheric pressure)
  //Pressure = Pressure * 14.50377;  // Convert bars to psi
  //Serial.print("Pressure in psi is: ");
  //Serial.println(Pressure);
  //Serial.println();
  float depth = (Pressure * 33.33);
  
  if (diveMode && depth < 1) {
    diveFile.close();
    diveMode = false;
    timeOfLastInput = now();
    setupMode();
  } else if (!diveMode && depth > 1) {
    diveMode = true;
    sleepMode = false;
    diveFile = SD.open(getDiveFileName(), FILE_WRITE);
    setupMode();
  }
  if (!diveMode && !sleepMode && now() - timeOfLastInput > IDLE_INTERVAL_BEFORE_SLEEP) {
    goToSleep();
  }
  if (!sleepMode) {
    logDiveEntry(Temperature, depth);
    drawScreen(Temperature, depth);
  }
}

void drawScreen(int temp, int depth) {
  
  sprintf(displaybuffer, "%02d F", temp);
  WRITE_TEXT(5, 30, displaybuffer);
  
  sprintf(displaybuffer, "%03d ft", depth);
  WRITE_TEXT(75, 30, displaybuffer);
  
  get_time();
  sprintf(displaybuffer, "%02d:%02d:%02d", t_hours, t_minutes, t_seconds);
  WRITE_TEXT(5, 95, displaybuffer);
}

void logDiveEntry(float temp, float depth) {
  int buflen = sprintf(filebuffer, "%5.5d, %5.5d\n", temp, depth);
  diveFile.write(filebuffer, buflen);
  diveFile.flush();
}

void sendCommand(int device, byte command){
  Wire.beginTransmission(device);
  Wire.write(command);
  Wire.endTransmission();
}

void goToSleep() {
}


void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
  sleepMode = false;
  timeOfLastInput = now();
}

void writeText(int x, int y, char *text) {
  tft.setCursor(x,y);
  tft.print(text);
}

char* getDiveFileName() {
  return "log.csv";
}

void get_date()
{
  sendCommand(RTCDevAddress, 3);
  Wire.requestFrom(RTCDevAddress, 4); //get 5 bytes(day,date,month,year,control);
  while (Wire.available()) {
    t_day   = bcdToDec(Wire.read());
    t_date  = bcdToDec(Wire.read());
    t_month = bcdToDec(Wire.read());
    t_year  = bcdToDec(Wire.read());
  }
}

void get_time()
{
  sendCommand(RTCDevAddress, 0);
  Wire.requestFrom(RTCDevAddress, 3);//get 3 bytes (seconds,minutes,hours);
  while (Wire.available()) {
    t_seconds = bcdToDec(Wire.read() & 0x7f);
    t_minutes = bcdToDec(Wire.read());
    t_hours = bcdToDec(Wire.read() & 0x3f); 
  }
}

void set_time()
{
   Wire.beginTransmission(RTCDevAddress);
   Wire.write(0);
   Wire.write(decToBcd(t_seconds));
   Wire.write(decToBcd(t_minutes));
   Wire.write(decToBcd(t_hours));
   Wire.endTransmission();
}

void set_date()
{
  Wire.beginTransmission(RTCDevAddress);
  Wire.write(3);
  Wire.write(decToBcd(t_day));
  Wire.write(decToBcd(t_date));
  Wire.write(decToBcd(t_month));
  Wire.write(decToBcd(t_year));
  Wire.endTransmission();
}

byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}
