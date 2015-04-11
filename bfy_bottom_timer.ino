#include <SD.h>
#include <avr/sleep.h>
#include <Time.h>

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

#define dc   A0  //DC
#define SD_CS A1  //SC
#define rst  A2 //R

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>
#include <SD.h>

#define WRITE_TEXT(x, y, text) \
  tft.setCursor(x,y); \
  tft.print(text);


#include <Wire.h>

const int DevAddress = 0x76;  // 7-bit I2C address of the MS5803

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

char displaybuffer[100];

// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(OLED_CS, dc, mosi, sclk, rst);  

// Option 2: must use the hardware SPI pins 
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1351 tft = Adafruit_SSD1351(OLED_CS, dc, rst);

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

int wakePin = 2;                 // pin used for waking up
int sleepStatus = 0;             // variable to store a request for sleep
int count = 0;

float const pow2_23 = pow(2, 23),
            pow2_33 = pow(2, 33),
            pow2_37 = pow(2, 37),
            pow2_16 = pow(2,16),
            pow2_7 = pow(2, 7),
            pow2_8 = pow(2, 8),
            pow2_15 = pow(2, 15),
            pow2_21 = pow(2, 21),
            pow2_15_divided_by_10000 = pow2_15 / 10000;

void setup(void) {
  //Serial.begin(9600);
  //setupSleep();
  SPI.setClockDivider(0);
  setTime(0);
  setupOLED(); 
  setupSdCard();
  setupIMUSensor();
  
  setupScreen();
}

void setupScreen() {
  tft.drawFastVLine(64, 0, 128, GREEN);
  tft.drawFastHLine(0, 64, 128, GREEN);
  
  tft.setTextColor(RED, BLACK);
  WRITE_TEXT(5, 0, "Temp");
  WRITE_TEXT(70, 0, "Depth");
  WRITE_TEXT(5, 65, "Time");  
  tft.setTextColor(BLUE, BLACK);
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
  //Serial.println("initialized I2C");
  
  testdrawtext("Initializing IMU Sensor...", BLUE);
  // Reset the device and check for device presence 
  sendCommand(Reset);
  delay(1000);
   
  // Get the calibration constants and store in array
  for (byte i = 0; i < 8; i++)
  {
    sendCommand(PromBaseAddress + (2*i));
    Wire.requestFrom(DevAddress, 2);
    while(Wire.available()){
      ByteHigh = Wire.read();
      ByteLow = Wire.read();
    }
    CalConstant[i] = (((unsigned int)ByteHigh << 8) + ByteLow);
  }
  
  testdrawtext("Calibration successful.", BLUE);
}

void setupSdCard() {
  //Serial.print("Initializing SD card...");
  testdrawtext("Trying to open SD Card", RED);

  if (!SD.begin(SD_CS)) {
    //Serial.println("failed!");
    return;
  }
  //Serial.println("SD OK!");

  createFile();
  
  readAndShowFile();

}

void setupSleep() {
    pinMode(wakePin, INPUT);

   
  /* Now it is time to enable an interrupt. In the function call
   * attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
   *
   * B   Name of a function you want to execute while in interrupt A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level trigger
   *             CHANGE     a change in level trigger
   *             RISING     a rising edge of a level trigger
   *             FALLING    a falling edge of a level trigger
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */
 
  attachInterrupt(0, wakeUpNow, RISING); // use interrupt 0 (pin 2) and run function
                                      // wakeUpNow when pin 2 gets LOW
                                      
                         
}

void loop(){

  // Read the Device for the ADC Temperature and Pressure values
  
  sendCommand(D1_512);
  delay(10);
  sendCommand(AdcRead);
  Wire.requestFrom(DevAddress, 3);
  while(Wire.available())
  {
    ByteHigh = Wire.read();
    ByteMiddle = Wire.read();
    ByteLow = Wire.read();
  }
  AdcPressure = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
  
//  Serial.print("D1 is: ");
//  Serial.println(AdcPressure);
  
  sendCommand(D2_512);
  delay(10);
  sendCommand(AdcRead);
  Wire.requestFrom(DevAddress, 3);
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
  
  Temperature = Temperature / 100;  // Convert to degrees C
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
  Pressure = Pressure / pow2_15_divided_by_10000;
  
  // Convert to psig and display
  
  Pressure = Pressure - 1.015;  // Convert to gauge pressure (subtract atmospheric pressure)
  //Pressure = Pressure * 14.50377;  // Convert bars to psi
  //Serial.print("Pressure in psi is: ");
  //Serial.println(Pressure);
  //Serial.println();
    
  drawScreen(Temperature, Pressure * 33.33);
}

void drawScreen(float Temperature, float Depth) {
  sprintf(displaybuffer, "%2d C", (int)Temperature);
  WRITE_TEXT(5, 30, displaybuffer);
  
  sprintf(displaybuffer, "%3d ft", (int)Depth);
  WRITE_TEXT(75, 30, displaybuffer);
  
  sprintf(displaybuffer, "%2d:%2d:%2d", hour(), minute(), second());
  WRITE_TEXT(5, 95, displaybuffer);
}

void createFile() {
  //Serial.println("Creating divecomputer.txt in SD card");
  File f = SD.open("/divecomputer.txt", FILE_WRITE);
  f.write("Hello World!", sizeof('a') * 13);
  f.close();
}

void readAndShowFile() {
  //Serial.println("Opening root directory");
  File f = SD.open("/", FILE_READ);
  while (f != NULL) {
    //Serial.println(f.name());
    testdrawtext(f.name(), RED);
    f = f.openNextFile();
    delay(1000);
  }
}

void testdrawtext(char *text, uint16_t color) {
  /*
  tft.fillRect(0, 100, 128, 128, BLACK);
  tft.setCursor(0, 100);
  tft.setTextColor(color);
  tft.print(text);
  */
}
             // counter
 
void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}

void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
 
    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */
 
    attachInterrupt(0,wakeUpNow, RISING); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW
 
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
                             // wakeUpNow code will not be executed
                             // during normal running time.
                             
    count = 0;
 
}


void sendCommand(byte command){
  Wire.beginTransmission(DevAddress);
  Wire.write(command);
  Wire.endTransmission();
}





 
