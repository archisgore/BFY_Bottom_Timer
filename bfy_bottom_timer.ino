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
out there for dropping the ball on the little things. I don't need science fiction
devices on the surface. I need a timer that works reliably. If I take a valve-cleaning
workshop and a reg servicing cert so I understand my gear, I should be able to dissect
my computer and know how it operates, what it's failure points are, etc. It is a critical
piece of gear that keeps me alive, and I want to know how it works.

Here's a few things I got mad about:
1. Most need specialized parts. I can't use the O-rings I have. I can't use
aquaseal. I don't have a choice on the material the housing is made of.
I hate weird proprietary batteries.

2. I'm sick of flooded compartments. Compartment floods, and the entire thousand
dollar device dies.

3. I'm sick of special software, special wires and special protocols. No offense, but it
is 2015 right now and every programmer worth having is currently being bid on for hundreds 
of thousands of dollars. This means that my expensive hardware is useless if the manufacturer 
goes out of business. Look at the battery compartment cover for your favourite computer right now.
In the next 30 minutes, find out where you can find a replacement.

4. It is not about cost, but it is about quality. "Nitrox capable" computers are twice as much as
"Air" computers. I hate the entire scuba industry for this one - nobody wants to make an honest buck
and nobody wants to pay for an honest service. Divers are petty deal-seekers, and manufacturers are
slimy snake oil salesmen. For the life of me, I couldn't just "pay 2000 dollars for a quality constructed
machine."

So I decided to build my own. It started as a weekend boast and then I got 
serious and designed a computer I wanted.

The ingredients:
1. Two of these (one for your computer, the other to wirelessly program it.) http://www.dfrobot.com/index.php?route=product/product&product_id=1122
2. Your power supply: https://learn.adafruit.com/adafruit-powerboost-1000c-load-share-usb-charge-boost/overview
3. Qi Pad for recharging without contact: https://www.adafruit.com/products/2117
4. OLED Screen (because OLED is a cool thing you want on your computers.) https://www.adafruit.com/products/1431
5. Slow vibration sensor (I use this to wake up the arduino from POWER_DOWN mode. Pretty cool - how much power you'll save.): https://www.adafruit.com/products/1767
6. ChronoDot real time clock (to ALWAYS keep time): https://www.adafruit.com/products/255
7. Battery (You can go smaller if you wish, or the large 6600mAH. It's all good.): https://www.adafruit.com/products/328
8. I2C level shifter (IMU sensor operates on 3V, the Nano on 5V logic. Being safe.): https://www.adafruit.com/products/757
9 The sensor (has everything you need on one circuit.) http://store.openrov.com/products/openrov-imu-depth-module
10. Micro SD Card (go big here): http://www.amazon.com/SanDisk-Memory-Adapter--SDSDQUAN-064G-G4A-Version/dp/B00M55C1I2/ref=sr_1_2?s=electronics&ie=UTF8&qid=1430285862&sr=1-2&keywords=micro-sd+cards

The procedure:

First, make sure one of your nano's can program the other. Follow these instructions: http://www.dfrobot.com/wiki/index.php/Bluno_SKU:DFR0267#Wireless_Programming_via_BLE

One you test this out, make sure you use the PERIPHERAL board for everything that follows. Keep the CENTRAL board safe for future use.

Next up, Connect the OLED Screen as follows:
*/
// You can use any (4 or) 5 pins 
#define sclk 13 //CL
#define mosi 11 //SI
#define miso 12 //SO
#define OLED_CS   10 //OC

#define dc   9  //DC
#define SD_CS 8  //SC
#define rst  7 //R

/*
Now format the SD card on your computer to MSDOS-FAT format and insert it into the OLED holder.

Next up, connect the OpenROV IMU Sensor to LV side of the Level Shifter. There are 4 wires and they are obvious.

Short the two grounds on the level shifter and hook them up to any ground on the nano. Apply 3V3 pin 
from Nano to the LV pin on the level shifter. Apply the 5V pin from the Nano to the HV pin. I find it
easiest to keep all wires color coded. SDA - White. SCL - Green, 5V/Vcc - RED, GND - Black. Pull out the same colored
wires on the HV side of the level shifter, as they go in on the LV side from the sensor. Basically, I start by connecting
the color coded wires on the sensor. Then I plug them into the level shifter's LV (Low Voltage) side. Keep VCC/GND where 
they are, and put WHITE on A1 and GREEN on A2.

On the other side, pull out a white wire from B1, and GREEN wire on B2.

Next up, put in your chronodot's battery. I clipped off all headers here to save space. You don't have to. Connect the I2C 
ports from the chronodot to the Nano. You'll notice the chronodot has the same pin labels on one side as our IMU sensor. 
Solder the same color-coded wires like above.

Now SHORT the wires from the chronodot to the HV side of the Level shifter (I2C devices share the same 4 wires).

Finally connect this Levelshifter+Chronodot+IMU assembly to the Nano thus:
LV -> 3V3
HV -> 5V
GND/GND -> GND
SDA/White/B1 -> A4
SCL/Green/B2 -> A5

Finally, solder the vibration switch between GND, and Pin2 or Digital 2 or D2.

Now plug in the USB Port and use this Arduino IDE and upload this program. For kicks, you can upload it over the BLE interface.

You should see your sensor working, and your OLED should begin displaying stuff. When your program goes to sleep, 
a little flick on the vibration switch will wake it up.

Finally comes power. Plug in the Qi bad to the microusb side of the PowerBoost 1000C, and the battery in that white connector thingy.

Pull out a Red ad Black wire from the side where it says + and Minus. Connect Black/GND/- to Arduino's GND. Connect Red/+/5V to Arduino's Vin (Nothing else!)

Now your board is powered from the battery. If you place your Qi pad on a wireless charger, the battery will charge.

I decided to coat everything in Epoxy to make them water-resistent and rigid, this ensure a slim chance of survival if the compartment/housing fails.


 ****************************************************/



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
