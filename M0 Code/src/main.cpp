 

/*
Changed screen CS from 10 to 12

 by AcE Krystal @ 26-3-2019
 Version 0.04.1(Playground) 
 Project 201903p01s(02+03+04+buttons)

 Goals : 
 Done! s02 Learning possibility's and limits of "current sensed end stops" and stepping drifting using cheap servo's
 Done! s02 Make the code dynamic to auto adjust for different servo's of environment changes (at startup).
 Done! s03 Reading out 2 SHT31-D's with 1 microcontroller
 Done! Combining the 2 scripts
 Done! trigger open/close steps based on temperature changes measured.
 s04 

 Action Flow : 
 a. Do a startup sweep to learn and adjust nominal and max servo currents. (1.Close, 2.Open, 3. Close)
 b. Do a startup step test to learn and adjust minimal detectable step size. (4.Close, 5.Open, 6.Close)
 c. Start normal loop ()
 Formule concept used for stall detection = 
          difference = previousCurrent - newcurrent + tollerance;
          Stalled = true if difference < 0;

 Features :
 = Temperature and Humidity Sensor v0.1duo
 - 
 = Servo controller v0.4 
 - v0.1 Stall detection based on current draw.
 - v0.2 Parameters all collected at top for easy adjusting.
 - v0.3 Automaticly learns servo provile at statup and adjusts stall detection parameters based on learned servo (and job) profile.
 - v0.3 Movement actions now return avg. and max. load, clould be usefull for detecting if it dit some "work" in robotic arms?.
 - v0.3 Stall detection is much faster, smaller steps are possible.
 - v0.4 Minimal step size learning at startup. It now automaticly sets the smallest detectable step size.
 = Screen
 -


 Future idea's and functions : 
 1. Learn Servo characteristics during startup sweep (min current and max current) and handle small current changes in higher level with more weight then changes at the lower level currents.
 Solution for : known problem around starting while physicly already at the end, the startup peak current doesn't lower enough to regonize it rizing again when stalling.
 2. Trying to intergrate with the rest of features for this project, possible switching from delay to timers. 
    Test code together with :
    SHT31D sensors          [?]
    CapsensPad              [?]
    128x64 OLED             [?]
    conver to 328PB chip    [?] 
    
 */

//Possible thanks to :
// Atte Pitkänen [ inspiration for OLEDs Gaph : 
/* https://www.youtube.com/watch?v=6-SRpThOZ5I */ 

// Adafruit [ OLED SSD1306 library ] 
/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x64 pixel display using SPI to communicate
 4 or 5 pins are required to interface.

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
// =========== EEPROM ===============================
#include <Arduino.h>
#include <FlashAsEEPROM.h>

#define CONFIG_VERSION "al2"          // ID of the settings block
#define SERVOCONFIG_VERSION "al2"     // ID of the servo settings block
#define CONFIG_START 32               // Tell it where to store your config data in EEPROM
#define SERVOCONFIG_START 232         // Tell it where to store your config data in EEPROM

// General configuration settings ----------------
struct ConfigStoreStruct_DEFAULT {
  byte selectedProfile;                       // bytes used 1
  float arrayProfileSettings[3][3];           // bytes used 36
  unsigned int stabelizeSessionTempTime;      // bytes uses 4
  unsigned int graph1Interval;                // bytes uses 4
  unsigned int buttonAutoLockTime;            // bytes uses 4
  int saveServoProfilesToFlash;                  // bytes uses 4
  char version_of_program[4];                 // bytes used 4    // This is the last variable of the struct so when settings are saved, they will only be validated if they are stored completely.
  } settings = {
// The default values
  1,                  //selectedProfile (on startup)
  {{16,1,15,}, {34,1,15,}, {40,1,15,},},     // {PLA,PET,ABS} {{temp,range,interval}}         
  300,                //stabelizeTime
  5,                  //graphInterval
  120,                //buttonAutoLockTime
  true,               //saveServoProfilesToFlash
  CONFIG_VERSION
};

// Learned servo profile settings
const uint ServosNumbers = 2; // Total ammount of servo's that are being created.
struct ServoStoreStruct {
    // Servo Settings --------------------
    boolean servoProfileAlreadyKnown;           // Is true if servoProfiles are already learned (and saved).
    // Servo Define
    byte analogPin[ServosNumbers];              // Pin for Sensing Current draw. Going between servo negative and shurt resistor to ground (also add ~10K arduino protective resitor toward this pin).
    byte servoPin[ServosNumbers];               // Pin for control signal to Servo.
    // Servo Basic Settings
    byte posSignal[ServosNumbers];              // variable to store the stopping signal for the servo [default = 90].
    byte increaseFlow[ServosNumbers];           // Direction and speed to increase flow [default = 0].
    byte decreaseFlow[ServosNumbers];           // Direction and speed to decrease flow [default = 180].

    // Default Start parameters for Stall Detection --------------
    // These settings can be overwritten after the learning process and can even be stored in EEPROM
    int stepTolleranceLowC[ServosNumbers];      // Adjust to how sensitive stall detection should react at lower loads (more is less sensetive) [default analog = 30, digital = 10].
    int stepTolleranceHighC[ServosNumbers];     // Adjust to how sensitive stall detection should react at higher loads (more is less sensetive) [default analog= -25 digital = -25] If you go below 0 it will alway's trigger before max load is reached.
    int startupCurrent[ServosNumbers];          // Current draw that needs to be seen before it believes the servo is actually starting to turn (some servo's react slow and take some time before they actually start). [default = 15].
    int servoMinCurrent[ServosNumbers];
    int servoNominalCurrent[ServosNumbers];     // Stores the learned normal load operations current (low end current) of the servo device/setup. [example = 50].
    int servoMaxCurrent[ServosNumbers];         // Stores the learned high load operation current (high end current) of the servo device/setup. [example = 500].
    // Servo Advanced Settings  -------------- 
    boolean dynamicStepSize[ServosNumbers];     // At startup it will try to detect the smallest flowStepSize detectable by the stall detector and overwrite above set flowStepSize value with it. [default = true].
    int minFlowStepSize[ServosNumbers];         // Used when not using dynamicStepSize detection. Lower is smaller and more steps, but to small may not trigger a stall. [default = 10].
  
  char version_of_program[4];                   // This is the last variable of the struct so when settings are saved, they will only be validated if they are stored completely.
} servoSettings = {
  // Servo Define
    false,        //servoProfileAlreadyKnown; // Is true if servoProfiles are already learned (and saved).
    //2,          //ServosNumbers             // Total ammount of servo's that are being created.                 
    {A0, A1},     //analogPin                 // Pin for Sensing Current draw. Going between servo negative and shurt resistor to ground (also add ~10K arduino protective resitor toward this pin).
    {5, 9},       //servoPin                  // Pin for control signal to Servo.
    // Servo Basic Settings
    {90, 90},     //posSignal                 // variable to store the stopping signal for the servo [default = 90].
    {0, 0},       //increaseFlow              // Direction and speed to increase flow [default = 0].
    {180, 180},   //decreaseFlow              // Direction and speed to decrease flow [default = 180].

    // Default Start parameters for Stall Detection --------------
    // These settings can be overwritten after the learning process and can even be stored in EEPROM
    {10, 30},     //stepTolleranceLowC        // Adjust to how sensitive stall detection should react at lower loads (more is less sensetive) [default analog = 30, digital = 10].
    {-25, -25},   //stepTolleranceHighC       // Adjust to how sensitive stall detection should react at higher loads (more is less sensetive) [default analog= -25 digital = -25] If you go below 0 it will alway's trigger before max load is reached.
    {15, 15},     //startupCurrent            // Current draw that needs to be seen before it believes the servo is actually starting to turn (some servo's react slow and take some time before they actually start). [default = 15].
    {0, 0},       //servoMinCurrent
    {130, 130},   //servoNominalCurrent       // Stores the learned normal load operations current (low end current) of the servo device/setup. [example = 50].
    {500, 500},   //servoMaxCurrent           // Stores the learned high load operation current (high end current) of the servo device/setup. [example = 500].
    // Servo Advanced Settings  -------------- 
    {true, true}, //dynamicStepSize           // At startup it will try to detect the smallest flowStepSize detectable by the stall detector and overwrite above set flowStepSize value with it. [default = true].
    {10, 10},     //minFlowStepSize           // Used when not using dynamicStepSize detection. Lower is smaller and more steps, but to small may not trigger a stall. [default = 10].
  SERVOCONFIG_VERSION
};

void loadConfig();
void saveConfig();
void loadServoProfiles();
void saveServoProfiles();

// Reduce writes to Flash (using commit write only if Power Loss is detected) : 
volatile boolean saveToFlashQueue = false;  // keeps track when there is a change that needs to be stored to flash memory.
// Power Loss Detection
#include <Arduino.h>
//#include "avdweb_SAMDtimer.h"
volatile const byte LED1 = 1;
const byte LED2 = 6;
volatile const byte analogPowerLossDetectPin = A3; 
volatile boolean flashCommitDone = false;
void ISR_powerLoss_detector();
// Interrupt Timer (TC) for Power Loss Detection
volatile const byte LEDtimer = 13;
volatile byte x;
volatile short analogReadTest;
void timer3Init();


// =========== SENSORS ===============================
// SHT31-D Temp/Hum. Sensor's ==========================
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();
// ??? Time definement needed? #define TEMP_INTERVAL  3000
float temp[2] = {11.00, 12.00};         //Setup temperature result array (2 sensors
float hum[2] = {13.00, 14.00};          //Setup humidity result array (2 sensors)
const int sht31_On[2] = {0x44, 0x45};   //array of available sht31-d sensors, you can dissable sensors here by removing them from the array.
byte lastSensor = 0;                    // used to track what sensor was read the last time, usefull for switching LED's off.

void readTempHum(byte sensorID);
// timers -------

// Button Sensor's ==========================
const byte interruptPinRedButton = 1;
const byte interruptPinBueButton = 6;
volatile byte blueButton = LOW;           // Stores current button state (for long presses) and setting tapped variable
volatile byte redButton = LOW;            // Stores current button state (for long presses) and setting tapped variable
volatile byte blueButtonTapped = LOW;     // Stores is button was tapped for one tap functions
volatile byte redButtonTapped = LOW;      // Stores is button was tapped for one tap functions
boolean buttonLock = false;
//uint buttonAutoLockTime = 60000;
long buttonAutoLockTimer = 0;

void buttonDown();
void buttonUp();
int buttonPressType();

// =========== Calculators ===============================

// GUI Pages
byte currentMenuPage = 1;
const char* menuState[3] = {"System Settings:", "MainScreen", "Profile Settings:"};

// GUI Settings --------
const char* arrayProfileID[3] = {"PLA", "PET", "ABS"};
byte selectedProfileSetting = 0;                           // Selected setting to adjust.
const char* arrayProfileSettingNames[3] = {"Target Temp:", "Range:", "Interval:"};
byte selectedSystemSetting = 0;
const char* arraySystemSettingNames[5] = {"Stabelize Time:", "GraphInterval:", "AutoLock:", "Save Servo's", "Relearn Servo's"};
// Profile Settings --------

float lastTemp = 90;                                      // Used to calculate trend of Temp
float trentTemp = 0;
float lastTrendTemp = 0;
float setMinT2 = 0;
float setMaxT2 = 0;

// =========== ACTORS ===============================
// Servo's v0.4 ==========================
//#include <Servo.h>
#include "AcEservo.h"
AcEservo servo_[ServosNumbers];

// Interval Trend and Timer
long timerServoDelay = 0;                // Delay for interval time between steps.
const byte trendMultiplier = 5;          // The number of measurement times it will build a trend number out of (newTemp - oldTemp * this variable)

int servoControllSteps = 0;
int lastStepDirection = 0;
uint nextServoToOperate = 0;
uint setNextServoToOperate( uint currentServo, int direction);
void learnAllServoProfiles();

// Screen ==========================
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
/*#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
*/
// Comment out above, uncomment this block to use hardware SPI
#define OLED_DC     18
#define OLED_CS     12 // was 10 on breadboard
#define OLED_RESET  19
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);
 
  // AcE Krytal Playground
// Specials 
  // ButtonLock Message
    byte buttonLockMessage = 0;
    long buttonLockMessageTimer = 0;
    unsigned int buttonLockMessageTime = 5000; // show message 5 secconds
// Bar Defaults --------------
  const int barLength = 88;
  const int barHeight = 5;
// Bar1 --------------
  const int barT1StartHo = 25;
  const int barT1StartVe = 46;
// Bar2 --------------
  const int barT2StartHo = 25;
  const int barT2StartVe = 56;
// Graph1 --------------
  const int graph1StartHorizontal = 25;
  const int graph1StartVertical = 10;
  const int graph1Length = 80;      // Make sure its under 128 for byte-array sake, if bigger edit some var types.
  const float graph1Height = 26;
  float graph1BottomRange = 0;      // used to set bottom of the graph range
  float graph1TopRange = 100;       // used to set top of the graph range
  float readArray[graph1Length];    // Make an array of max 128 length. (Use int if array is bigger then 255)
  byte drawHeight;                  // Stores graph value location for writing to big line array.
  byte targetDrawHeight;
  byte minDrawHeight;               // Stores graph pixel location value for min temp record.
  byte maxDrawHeight;               // Stores graph pixel location value for max temp record.
  byte lastReadingNR = 0;           // Stores where the last value for the Graph was written, used to know where to start in the array. (Use int if array is bigger then 255)
  long lastUpdateGraph1 = 0;
  long stabilizeTimer = 0;          // Used for temp stabelization delay for timer stabelizeSessionTempTimer.   
  const unsigned int sessionResetbyTempDifference = 1;
  boolean stabilizing = false;
  boolean startSettingsGraph1 = true;
  boolean setTempReached = false;
  float maxTempinSession[2] = {0, 0};         // Minimum temp reached in a session.
  float minTempinSession[2] = {100, 100};     // Minimum temp reached in a session.
  float maxHuminSession[2] = {0, 0};
  float minHuminSession[2] = {100, 100};
  boolean updateGraph1Scale = true;

void drawHeaderProfile(int selector);
void drawHeaderSystemSettings();
void drawMenuSystemSettings(int selector);
void drawMenuProfileSettings(int selector);
void drawButtonLockPopup();
void drawProgressBar2(int horizontalStartPixel, int verticalStartPixel);
void drawSetTempRange2(void);
void drawTempRange2(int sensor, int horizontalStartPixel, int verticalStartPixel);
void drawHumRange2(int sensor, int horizontalStartPixel, int verticalStartPixel);
void updateProgressBar2(int sensorID, int horizontalStartPixel, int verticalStartPixel);
void updateGraph1(void);
void emptyGraph1(void);
void updateGraph1Scaling(void);
void updateGraph1Pointers(void);
void updateGraph1Addons(int sensorID, int horizontalStartPixel, int verticalStartPixel, int length, int height);

// Memmory Check ---------
int freeMemory();

void setup() {
  Serial.begin(115200);           //  setup serial
  delay(5000);


  // Factory Reset function -------------------
  pinMode(interruptPinRedButton, INPUT);
  pinMode(interruptPinBueButton, INPUT);
  boolean buttonStateRed = digitalRead(interruptPinRedButton);
  boolean buttonStateBlue = digitalRead(interruptPinBueButton);
  boolean doFactoryReset = false;
  if (buttonStateBlue == HIGH && buttonStateRed == HIGH){
    delay(5000);
    if (buttonStateBlue == HIGH && buttonStateRed == HIGH ){
      doFactoryReset = true;
      pinMode(LED2, OUTPUT);
      digitalWrite (LED2, HIGH );
      delay(10000);
      digitalWrite (LED2, LOW);
    }
  } 
  
  buttonAutoLockTimer = millis(); 

  //delay(8000);
  Serial.print(F("Mem>> ")); Serial.println (freeMemory());   
  Serial.println (F("loading screen stuff"));
//screen splash --------
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //display.begin(SSD1306_SWITCHCAPVCC);
   if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    while (1){ // Don't proceed, loop forever
      Serial.println(F("SSD1306 allocation failed")); 
    }
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  Serial.print(F("Mem>> ")); Serial.println (freeMemory());   
  Serial.println(F("Show display splash"));
  display.display();
// =========== EEPROM Setup ===============================
  Serial.print(F("Debug : servo Before loading mem Nominal/Max Current = "));Serial.print(servoSettings.servoNominalCurrent[0]);Serial.print(F("/"));Serial.println(servoSettings.servoMaxCurrent[0]);
  //loadConfig();
  //loadServoProfiles();
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  int loadingBlinks = 0;
  if (doFactoryReset == true ){    
  // We don't load any configs.
    Serial.println(F("Doing factory reset. (not loading from mem, if changes done mem wil be overwritten."));
    for (int i = 0; i < 4; i++){
      digitalWrite (LED1, HIGH );
      delay(70);
      digitalWrite (LED1, LOW);
      delay(70);
    }
  } else if (doFactoryReset == false){
  // We do load basic system configs.
        Serial.println(F("Normal startup, will look for existing System Config."));
      digitalWrite (LED2, HIGH );
      noInterrupts();
      loadConfig();
      interrupts();
      digitalWrite (LED2, LOW);
      delay(70);
      loadingBlinks = loadingBlinks + 2;
  }
  
  if (doFactoryReset == false && settings.saveServoProfilesToFlash == 1 ){
  // We do more clear loading blinks for (eventual remote) debug purpose
      Serial.println(F("Also looking for existing Servo Profiles."));
      digitalWrite (LED2, HIGH );
      noInterrupts();
      loadServoProfiles();
      interrupts();
      digitalWrite (LED1, LOW);
      delay(70);
      loadingBlinks = loadingBlinks + 2;
      
      while (loadingBlinks > 0){
      loadingBlinks --;
      digitalWrite (LED2, HIGH );
      delay(70);
      digitalWrite (LED2, LOW);
      delay(70);
      }
  }
  

  
  analogReadTest = analogRead(analogPowerLossDetectPin);

  Serial.print(F("Debug : servo After loading mem Nominal/Max Current = "));Serial.print(servoSettings.servoNominalCurrent[0]);Serial.print(F("/"));Serial.println(servoSettings.servoMaxCurrent[0]);
  Serial.print(F("Debug : Skipping Servo learning? servoProfileAlreadyKnown = "));Serial.println(servoSettings.servoProfileAlreadyKnown);
// ----------- Timer for Power Detection Setup ------------------
  timer3Init();
//delay(3000);

// =========== SENSORS Setup ===============================
// Button Sensor's ==========================
  pinMode(interruptPinRedButton, INPUT);
  pinMode(interruptPinBueButton, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinRedButton), buttonUp, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(interruptPinBueButton), buttonDown, CHANGE);

/*// SHT31-D Temp/Hum. Sensor ==========================
  Serial.print(F("Mem>> ")); Serial.println (freeMemory());   
  Serial.println(F("loading SHT31-D stuff"));
   delay(1000);
  Serial.println("Finding SHT31-D sensors : ");
      /byte error;
      for (int i = 0; i < (sizeof(sht31_On) / sizeof(sht31_On[0])); i++) {
        
        Serial.print("#");
        Serial.print(i);
        Serial.print(", with i2c addr: 0x");
        Serial.print(sht31_On[i], HEX);
        Wire.beginTransmission(sht31_On[i]);
        error = Wire.endTransmission();
        //if (! sht31.begin(i)) {   // Test i2c addr response
        if (error == 2) {   // Test i2c addr response
          //Serial.print(sht31.begin(i));
          Serial.print("Couldn't find SHT31-D : 0x");
          Serial.println(sht31_On[i], HEX);
        } else if (error == 0) {
          //Serial.print(sht31.begin(i));
          Serial.print("Found SHT31-D : 0x");
          Serial.println(sht31_On[i], HEX);
        } else {
          Serial.println("unknown response from i2c adress test");
          Serial.print("Error given = ");
          Serial.println(error);
        }
      }
      */
 

// =========== Calculators Setup ===============================
  setMinT2 = settings.arrayProfileSettings[settings.selectedProfile][0] - (settings.arrayProfileSettings[settings.selectedProfile][1]/2);
  setMaxT2 = settings.arrayProfileSettings[settings.selectedProfile][0] + (settings.arrayProfileSettings[settings.selectedProfile][1]/2);
// PID ==========================
  //myPID.SetMode(AUTOMATIC);
// =========== ACTORS Setup ===============================
// Servo's Setup ==========================
//delay(8000);
//for (uint i=0; i<(sizeof (servoSettings.analogPin)/sizeof(servoSettings.analogPin[0])); i++) {
for (uint i=0; i<(sizeof (servoSettings.analogPin)/sizeof(servoSettings.analogPin[0])); i++) {

  //uint i = 1;
    Serial.print("Creating Servo "); Serial.println(i); Serial.print (" on servo Pin : "); Serial.println (servoSettings.servoPin[i]);
    
    //AcEservo servo_[i] (                  // create servo object to control a servo
    servo_[i] = AcEservo ( 
      //Servo Define
      servoSettings.servoPin[i],            //int servoPin,
      servoSettings.analogPin[i],           //int analogSensePin,
      //Servo Settings
      servoSettings.posSignal[i],           //int centerSignal,
      servoSettings.decreaseFlow[i],        //int downSignal,
      servoSettings.increaseFlow[i],        //int upSignal,
      servoSettings.stepTolleranceLowC[i],  //int stepTolleranceLowC,
      servoSettings.stepTolleranceHighC[i], //int stepTolleranceHighC,
      //Servo Profile (Load or Default)
      servoSettings.startupCurrent[i],      //int startupCurrent,
      servoSettings.servoNominalCurrent[i], //int nominalCurrent,
      servoSettings.servoMaxCurrent[i]      //int maxCurrent,
    );
    Serial.print(F("servo mem Nominal/Max Current = "));Serial.print(servoSettings.servoNominalCurrent[i]);Serial.print(F("/"));Serial.println(servoSettings.servoMaxCurrent[i]);
    Serial.print(F("object has Nominal/Max Current = "));Serial.print(servo_[i].nominalCurrent);Serial.print(F("/"));Serial.println(servo_[i].maxCurrent);
      //DEBUG Memory
        Serial.println(F("// Debug memory : "));
        Serial.println(F("// Servo Settings --------------------"));
        Serial.println( servoSettings.servoProfileAlreadyKnown);           // Is true if servoProfiles are already learned (and saved).
        Serial.println(F("// Servo Define --------------"));
        Serial.println( servoSettings.analogPin[i]);              // Pin for Sensing Current draw. Going between servo negative and shurt resistor to ground (also add ~10K arduino protective resitor toward this pin).
        Serial.println( servoSettings.servoPin[i]);               // Pin for control signal to Servo.
        Serial.println(F("// Servo Basic Settings --------------"));
        Serial.println( servoSettings.posSignal[i]);              // variable to store the stopping signal for the servo [default = 90].
        Serial.println( servoSettings.increaseFlow[i]);           // Direction and speed to increase flow [default = 0].
        Serial.println( servoSettings.decreaseFlow[i]);           // Direction and speed to decrease flow [default = 180].
delay(50);
        Serial.println(F("// Default Start parameters for Stall Detection --------------"));
        // These settings can be overwritten after the learning process and can even be stored in EEPROM
        Serial.println( servoSettings.stepTolleranceLowC[i]);      // Adjust to how sensitive stall detection should react at lower loads (more is less sensetive) [default analog = 30, digital = 10].
        Serial.println( servoSettings.stepTolleranceHighC[i]);     // Adjust to how sensitive stall detection should react at higher loads (more is less sensetive) [default analog= -25 digital = -25] If you go below 0 it will alway's trigger before max load is reached.
        Serial.println( servoSettings.startupCurrent[i]);          // Current draw that needs to be seen before it believes the servo is actually starting to turn (some servo's react slow and take some time before they actually start). [default = 15].
        Serial.println( servoSettings.servoMinCurrent[i]);
        Serial.println( servoSettings.servoNominalCurrent[i]);     // Stores the learned normal load operations current (low end current) of the servo device/setup. [example = 50].
        Serial.println( servoSettings.servoMaxCurrent[i]);         // Stores the learned high load operation current (high end current) of the servo device/setup. [example = 500].
        Serial.println(F("// Servo Advanced Settings  --------------"));
        Serial.println( servoSettings.dynamicStepSize[i]);     // At startup it will try to detect the smallest flowStepSize detectable by the stall detector and overwrite above set flowStepSize value with it. [default = true].
        Serial.println( servoSettings.minFlowStepSize[i]);         // Used when not using dynamicStepSize detection. Lower is smaller and more steps, but to small may not trigger a stall. [default = 10].
        Serial.println(F("// SERVOCONFIG_VERSION ------------- "));
        Serial.println( servoSettings.version_of_program[0]);
        Serial.println( servoSettings.version_of_program[1]);
        Serial.println( servoSettings.version_of_program[2]);
        Serial.println( servoSettings.version_of_program[3]);
delay(50);

  }
  //delay(3000);
  if (servoSettings.servoProfileAlreadyKnown == false || settings.saveServoProfilesToFlash == 0){
    learnAllServoProfiles();
  } else {
    Serial.println(F("Servo Profiles loaded from memory."));
  }

// Screen ==========================
  //delay(500); // Pause for 2 seconds
  display.clearDisplay();
    //Startup Screen
    display.setCursor(0,0);             // Start at top-left corner
    //Starting Sensor
    display.setTextSize(1);             // Draw 2X-scale text
    display.setTextColor(WHITE);
    display.println(F("Starting Sensors"));
    //Name      
    display.setTextColor(WHITE);
    display.setTextSize(2);             // Draw 2X-scale text
    display.println(F("Enclosure Controller"));
    display.setTextSize(1);
    display.println(F("by AcE Krystal"));
      //Version
    display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
    display.print(F("version "));
    display.println(F("0.5"));

  display.display();
  delay(500);
  // Serial.print(F("Mem>> ")); Serial.println (freeMemory()); 

}
void loop() {

  // =========== SENSORS Loop ===============================
  // SHT31-D Temp/Hum. Sensor ==========================
  //Serial.print(F("Mem>> ")); Serial.print (freeMemory()); 
  //Serial.println(F(" loading SHT31-D Loop"));
  int numberOfsht31Sensors = (sizeof(sht31_On) / sizeof(sht31_On[0]));
  for (int i = 0; i < numberOfsht31Sensors; i++) {
        readTempHum (i);
        if (! isnan(temp[i])) {  // check if 'is not a number', if not its not a valid measurement and should be discarded
    //          Serial.print(F("Temp."));
    //          Serial.print(i);
    //          Serial.print(F(" *C = ")); Serial.println(temp[i]);
        } else { 
          Serial.print(F("Failed to read temperature from sensor: "));
          Serial.println(i);
        }
       
        if (! isnan(hum[i])) {  // check if 'is not a number', if not its not a valid measurement and should be discarded
    //          Serial.print(F("Hum."));
    //          Serial.print(i);
    //          Serial.print(F(" % = ")); Serial.println(hum[i]);
        } else { 
          Serial.print(F("Failed to read humidity from sensor: "));
          Serial.println(i);
        }
    //        Serial.println();
  }
 // Button Sensor's (and UI Menu) Loop ==========================
  //Serial.print(F("Mem>> ")); Serial.print (freeMemory()); 
  //Serial.println(F(" Button Loop"));
  if (settings.buttonAutoLockTime != 0 && !buttonLock){  // Is buttonAutoLock enabled (not 0)?
    if (millis() - buttonAutoLockTimer > settings.buttonAutoLockTime*1000){
      buttonLock = true;
    }
  }
  // Interactive Logics (User Interface).
  switch (currentMenuPage){
    case 0:   //"System Settings:"
             switch (buttonPressType()){   // 1=blueTab, 2=blueLong, 3=redTab, 4=redLong
        case 1:   //1=blueTab = - previous setting
          if (selectedSystemSetting > 0 ) {
            selectedSystemSetting--;
          } else {
            selectedSystemSetting = sizeof(arraySystemSettingNames)/sizeof(arraySystemSettingNames[0])-1;
            Serial.println(F("lowest system setting already selected"));
          }
          break;
        case 2:   //2=blueLong = back
          //selectedMenuPage = 0;   // return to main page level 0
          currentMenuPage = 1;    // return one page up
          break;
        case 3:   //3=redTab = + next setting
          if (selectedSystemSetting < (sizeof(arraySystemSettingNames)/sizeof(arraySystemSettingNames[0]))-1 ) {
            selectedSystemSetting++;
          } else {
            selectedSystemSetting = 0;
            Serial.println(F("highest system setting already selected"));
          }
          break;
        case 4:   //4=redLong = Enter
          currentMenuPage = 4;          // Enter Profile Editing Page
          //profileEditing = true;      // Enter Profile Editing mode
          break;
        case 5:   //5=dualLong = manual buttonlock.
          buttonLock = true;
          break;
        case 0:   //na.
          // no action by no pressing
          break;
      }
      redButtonTapped = LOW;    // reset button tap for next tap
      blueButtonTapped = LOW;    // reset button tap for next tap 
      break;
    case 1:   //"MainScreen":
      switch (buttonPressType()){   // 1=blueTab, 2=blueLong, 3=redTab, 4=redLong
        case 1:   //1=blueTab = - profile down.
          if (settings.selectedProfile > 0 ) {         
          settings.selectedProfile--;
          } else {
          settings.selectedProfile = sizeof(arrayProfileID)/sizeof(arrayProfileID[0])-1 ;  //If begin of menu is reached, go back to end of menu
          Serial.println(F("lowest profile already selected"));
          }
          setMinT2 = settings.arrayProfileSettings[settings.selectedProfile][0] - (settings.arrayProfileSettings[settings.selectedProfile][1]/2);
          setMaxT2 = settings.arrayProfileSettings[settings.selectedProfile][0] + (settings.arrayProfileSettings[settings.selectedProfile][1]/2);

          //setMinT2 = settings.arrayProfileTargetTemp[settings.selectedProfile] - (settings.arrayProfileRange[settings.selectedProfile]/2);
          //setMaxT2 = settings.arrayProfileTargetTemp[settings.selectedProfile] + (settings.arrayProfileRange[settings.selectedProfile]/2);
          setTempReached = false;
          saveConfig();               // Save settings to EEPROM (for persistence profile selection on next restart). 
          break;
        case 2:   //2=blueLong = Enter system settings.
          currentMenuPage = 0;
          break;
        case 3:   //3=redTab = + profile up.
          if (settings.selectedProfile < (sizeof(arrayProfileID)/sizeof(arrayProfileID[0]))-1 ) {
          settings.selectedProfile++;
          } else {
          settings.selectedProfile = 0;                                                     //If end of menu is reached, go back to start of menu
          Serial.println(F("highest profile already selected"));
          }
          setMinT2 = settings.arrayProfileSettings[settings.selectedProfile][0] - (settings.arrayProfileSettings[settings.selectedProfile][1]/2);
          setMaxT2 = settings.arrayProfileSettings[settings.selectedProfile][0] + (settings.arrayProfileSettings[settings.selectedProfile][1]/2);
          setTempReached = false;
          saveConfig();               // Save settings to EEPROM (for persistence profile selection on next restart).
          break;
        case 4:   //4=redLong = Enter profile settings. (and unlock buttonLock)
          if(buttonLock){
            buttonLock = false;
          } else {
            //profileEditing = true;      // Enter Profile Editing mode
            //selectedMenuPage = 1;
            currentMenuPage = 2;          // Set page for Profile Editing as current.
          }
          break;
        case 5:   //5=dualLong = Resetting Session.
          setTempReached = false;
          Serial.println(F("Resetting Session triggered by user."));
        case 0:   //na.
          // no action by no pressing
          break;
      }
      redButtonTapped = LOW;      // reset button tap for next tap
      blueButtonTapped = LOW;    // reset button tap for next tap
      break;
    case 2:   //"Profile Settings:"
      switch (buttonPressType()){   // 1=blueTab, 2=blueLong, 3=redTab, 4=redLong
        case 1:   //0=blueTab = - previous setting
          if (selectedProfileSetting > 0 ) {
            selectedProfileSetting--;
          } else {
            selectedProfileSetting = sizeof(arrayProfileSettingNames)/sizeof(arrayProfileSettingNames[0])-1;
            Serial.println(F("lowest profile setting already selected"));
          }
          break;
        case 2:   //1=blueLong = back
          //selectedMenuPage = 0;   // return to main page level 0
          currentMenuPage = 1;    // return one page up
          break;
        case 3:   //2=redTab = + next setting
          if (selectedProfileSetting < (sizeof(arrayProfileSettingNames)/sizeof(arrayProfileSettingNames[0]))-1 ) {
            selectedProfileSetting++;
          } else {
            selectedProfileSetting = 0;
            Serial.println(F("highest profile setting already selected"));
          }
          break;
        case 4:   //3=redLong = Enter
          currentMenuPage = 3;          // Enter Profile Editing Page
          //profileEditing = true;      // Enter Profile Editing mode
          break;
        case 0:   //na.
          // no action by no pressing
          break;
      }
      redButtonTapped = LOW;      // reset button tap for next tap
      blueButtonTapped = LOW;    // reset button tap for next tap
      break;
    case 3:   //"ProfileEditing:"
      switch (buttonPressType()){   // 1=blueTab, 2=blueLong, 3=redTab, 4=redLong
        case 1:  //1=blueTab = - decrease value
          if (settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] > 0){
            if ( selectedProfileSetting == 0){  // Target Temp -1 between 0/100
                settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting]--;
                setTempReached = false; // Also reset session because of target temp change.
            } 
            if ( selectedProfileSetting == 1){  // Range -0.1 between 0/100
                settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] = settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] - 0.1;
                setTempReached = false; // Also reset session because of range temp change.
            }
            if ( selectedProfileSetting == 2){  // Interval -1 second between 0/3600
                settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting]--;
            }   
          }
          break;
        case 2:   //2=blueLong = back, Exit profile editing menu.
          currentMenuPage = 2;    // return one page up
          saveConfig();               // Save settings to EEPROM (for persistence on next restart)
          break;
        case 3:   //3=redTab = + increase value
            if ( selectedProfileSetting == 0){  // Target Temp +1 between 0/100
                if (settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] < 100){
                settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting]++;
                setTempReached = false; // Also reset session because of target temp change.
              }
            } 
            if ( selectedProfileSetting == 1){  // Range +0.1 between 0/100
              if (settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] < 100){
                settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] = settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] + 0.1;
                setTempReached = false; // Also reset session because of range temp change.
              }            
            }
            if ( selectedProfileSetting == 2){  // Interval +1 second between 0/3600
              if (settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting] < 3600){
                settings.arrayProfileSettings[settings.selectedProfile][selectedProfileSetting]++;
              }
            }   
          break;
        case 4:   //4=redLong = Enter
          currentMenuPage = 2;    // return one page up
          saveConfig();               // Save settings to EEPROM (for persistence on next restart)
          break;
        case 0:
          // no action by no pressing
          break;
      } 
      redButtonTapped = LOW;    // reset button tap for next tap
      blueButtonTapped = LOW;    // reset button tap for next tap
    case 4:   //"SystemEditing:"
      switch (buttonPressType()){   // 1=blueTab, 2=blueLong, 3=redTab, 4=redLong
        case 1:  //1=blueTab = - decrease value
            if ( selectedSystemSetting == 0){  // Stabelize Time -1 between 0/3600
                if (settings.stabelizeSessionTempTime > 0){
                  settings.stabelizeSessionTempTime--;
                }
            } 
            if ( selectedSystemSetting == 1){  // GraphInterval Time -1 between 0/3600
                if (settings.graph1Interval > 0){
                  settings.graph1Interval--;
                }
            }
            if ( selectedSystemSetting == 2){  // buttonAutoLock Time -1 second between 0/3600
                if (settings.buttonAutoLockTime > 0){
                  settings.buttonAutoLockTime--;
                }
            }
            if ( selectedSystemSetting == 3){  // saveServoProfilesToFlash set to 0 to disable.
                if (settings.saveServoProfilesToFlash == 1){
                  settings.saveServoProfilesToFlash = 0;
                }
            }
            /*if ( selectedSystemSetting == 4){  // Resetting is a direct actions to reset function.
                if (settings.buttonAutoLockTime < 3600){
                  settings.buttonAutoLockTime++;
                }
            }*/    
          break;
        case 2:   //2=blueLong = back, Exit System editing menu.
          currentMenuPage = 0;    // return one page up
          saveConfig();               // Save settings to EEPROM (for persistence on next restart)
          break;
        case 3:   //3=redTab = + increase value
            if ( selectedSystemSetting == 0){  // stabelize Time +1 between 0/3600
                if (settings.stabelizeSessionTempTime < 3600){
                  settings.stabelizeSessionTempTime++;
                }
            } 
            if ( selectedSystemSetting == 1){  // graphInterval Time +1 between 0/3600
                if (settings.graph1Interval < 3600){
                  settings.graph1Interval++;
                }
            }
            if ( selectedSystemSetting == 2){  // buttonAutoLock Time +1 second between 0/3600
                if (settings.buttonAutoLockTime < 3600){
                  settings.buttonAutoLockTime++;
                }
            }
            if ( selectedSystemSetting == 3){  // saveServoProfilesToFlash set to 1 to enable.
                if (settings.saveServoProfilesToFlash == 0){
                  settings.saveServoProfilesToFlash = 1;
                }
            }
            /*if ( selectedSystemSetting == 4){  // Resetting is a direct actions to reset function.
                if (settings.buttonAutoLockTime < 3600){
                  settings.buttonAutoLockTime++;
                }
            }*/         
          break;
        case 4:   //4=redLong = Enter
          if ( selectedSystemSetting == 4){
            learnAllServoProfiles();
          }
          currentMenuPage = 0;    // return one page up
          saveConfig();               // Save settings to EEPROM (for persistence on next restart)
          break;
        case 0:
          // no action by no pressing
          break;
      } 
      redButtonTapped = LOW;    // reset button tap for next tap
      blueButtonTapped = LOW;    // reset button tap for next tap            
  }
      
  //delay(500);

  // PowerLossDetector (for writing to mem @ powerloss).
  Serial.print(analogReadTest); Serial.print(" = cap charge | write done by time interrupt? : "); Serial.print(flashCommitDone);Serial.print(" Chnges in MemQueue? = "); Serial.print(saveToFlashQueue);Serial.print(" saveServoProfilesToFlash = "); Serial.println(settings.saveServoProfilesToFlash);

 
  // =========== Calculations Loop ===============================  
  
  // Calculate new setMin and setMax temps. (now only if temp not reached yet. changing targetTemp wil also reset this.)
  if (!setTempReached){
  setMinT2 = settings.arrayProfileSettings[settings.selectedProfile][0] - (settings.arrayProfileSettings[settings.selectedProfile][1]/2);
  setMaxT2 = settings.arrayProfileSettings[settings.selectedProfile][0] + (settings.arrayProfileSettings[settings.selectedProfile][1]/2);
  }
  // Calculate step action to take v1.1 -------------- (without PID)
  servoControllSteps = 0;  // Reset before calulating potation new step if needed. 
  if ( (millis() - timerServoDelay) > (settings.arrayProfileSettings[settings.selectedProfile][2]*1000) ){
  // Calculate step action to take v1 -------------- (without PID)
    timerServoDelay = millis();
    trentTemp = ((temp[1] - lastTemp) * trendMultiplier);     // Calculate trend 5 points in the future.
    Serial.print(F("min / max TempinSession = "));Serial.print(minTempinSession[1]);Serial.print(F("/"));Serial.println(maxTempinSession[1]);
    Serial.print(F("min / max graph1Range = "));Serial.print(graph1BottomRange);Serial.print(F("/"));Serial.println(graph1TopRange);
    Serial.print(F("trendTemp = "));Serial.print(trentTemp);Serial.print(F(" compare = "));Serial.print(trentTemp+temp[1]); 
    Serial.print(F(" -> "));Serial.print(setMinT2); Serial.print(F("/"));Serial.print(setMaxT2); Serial.print(F(" <- "));Serial.println(temp[1]);
    if ( trentTemp + temp[1] <= setMinT2 ){                    // If temperature is less then set temperature.
      servoControllSteps = -1;  //to cold = close                // it should set the step controller to close 1 step.
      //Serial.print(temp[1]); Serial.print(F(" is <= ")); Serial.print(setMinT2); Serial.print(F(" must close 1 step!; stallFlag = "));Serial.println(stallFlag);
    } else if ( trentTemp + temp[1] >= setMaxT2 ) {          // If temperature is more then set temperature.
      servoControllSteps = 1;   //to hot = open                // it should set the step controller to open 1 step.
      //Serial.print(temp[1]); Serial.print(F(" is >= ")); Serial.print(setMaxT2); Serial.print(F(" must open 1 step!; stallFlag = "));Serial.println(stallFlag);
    } else {
      servoControllSteps = 0;   // good = nothing               // temperature is fine, do nothing.
      //Serial.print(temp[1]); Serial.println(F(" is within set reange, not servo action needed."));
    }
    lastTemp = temp[1];
    //lastTrendTemp = trentTemp;
  }

  // Caculate witch servo can make the desired servoControllSteps
  if (servoControllSteps){
    nextServoToOperate = setNextServoToOperate(nextServoToOperate, servoControllSteps);
  } 

  for (int i=1; i>-1; i--){
  // Update min/max of temp and hum for every sensor in session
    //Serial.print(F("Printing I = ")); Serial.println(i);
    if (temp[i] < minTempinSession[i]){
      minTempinSession[i] = temp[i];
      updateGraph1Scale = true;
    }
    if (temp[i] > maxTempinSession[i]){
      maxTempinSession[i] = temp[i];
      updateGraph1Scale = true;
    }

    if (hum[i] < minHuminSession[i]){
      minHuminSession[i] = hum[i];
      updateGraph1Scale = true;
    }
    if (hum[i] > maxHuminSession[i]){
      maxHuminSession[i] = hum[i];
      updateGraph1Scale = true;
    }
  }


   // Are we inside a 3D print session ?
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0); 
    if ( setMinT2 < temp[1] && temp[1] < setMaxT2 ){   // Is temperature between set temperatures?
    // Yes, we are 3D Printing
      if (!setTempReached && !stabilizing){         // If set temperatures was not yet reached before.
      // Start stabelization for new session!
        stabilizing = true;
        updateGraph1Scale = true;
        minTempinSession[1] = temp[1];
        maxTempinSession[1] = temp[1];
        emptyGraph1();
      } else if (setTempReached && !stabilizing) {
        // Running normal session.
        if (maxTempinSession[1]>setMaxT2){
          display.print(F("OK* hot!"));
        } else if (minTempinSession[1]<setMinT2){
          display.print(F("OK* cold!"));
        } else {
          display.print(F("OK")); 
        }
      } else {
        // still stabilizing within range             
        display.print(F("Stabilize"));
      }
    } else if ((temp[0]+sessionResetbyTempDifference) < setMinT2 && (temp[0]+sessionResetbyTempDifference) > temp[1]){  // Upper temp is more then x higher then Lower temp and not withing or higher then set temp range?
    // We are no longer (or yet) 3D printing. Get ready for new Session.  
        if (setTempReached){
          Serial.println(F("Automaticly detected out of session. Getting ready for new session."));
        }
        setTempReached = false;                                 // Not normal, Session not yet started (or reset for next session).
        startSettingsGraph1 = true;
        updateGraph1Scale = true;           
      display.print(F("Sleep")); 

    } else { 
    // temp is not in range, but is also not fully cooled down.
      //Serial.println(F("Waiting for temperature to be in range before starting stabilizing timer."));
      stabilizeTimer = millis();           
      if (!setTempReached && !stabilizing){
        if(temp[1]<setMinT2){
          display.print(F("Warming up"));
        } else {
          display.print(F("Cooling"));
        }
         
      } else if (setTempReached) {
        // Session started but we are no longer withing set temp range! (o.O) NhuuuuUUUuuu! 
        if (temp[1]> setMaxT2){
          display.print(F("!To HOT!"));
        } else if (temp[1]<setMinT2){
          display.print(F("!To COLD!"));
        }
      } else if (stabilizing){
        // still stabilizing but we have gone out of set temp range!
        Serial.print(F("Still Stabilizing, but we are outside set temperature range!"));
        display.print(F("Stabilize"));
      }
    }

    //Is temperature stabilized?
    if (stabilizing && millis()-stabilizeTimer  > settings.stabelizeSessionTempTime*1000){
      stabilizing = false;
      Serial.println(F("Nominal Temperatures Reached and stabelized , starting new session"));
      setTempReached = true;      //
      //Did we have big fluxtuations during stabilizing that changed our o we need to also reset the scaling again because of big stabilizing fluctiations?
      if (maxTempinSession[1] > setMaxT2+0.1 || minTempinSession[1] < setMinT2-0.1){
        updateGraph1Scale = true;
        emptyGraph1();
      }
      minTempinSession[1] = temp[1];  // Ressetting temp and hum for session.
      maxTempinSession[1] = temp[1];
      minHuminSession[1] = temp[1];
      maxHuminSession[1] = temp[1];
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);             
      display.println(F("SET")); 
    }
        
  // =========== ACTORS Loop ===============================
  // Servo's Loop ==========================
  
  if (servoControllSteps < 0){ // are we closing?
    servo_[nextServoToOperate].moveServo(servoSettings.minFlowStepSize[nextServoToOperate], servoSettings.decreaseFlow[nextServoToOperate], 1 );   // decrease flow with steps
  } else if (servoControllSteps > 0) { // are we opening?
    servo_[nextServoToOperate].moveServo(servoSettings.minFlowStepSize[nextServoToOperate], servoSettings.increaseFlow[nextServoToOperate], 1 );   // increase flow with steps
  }
  
  delay(100);
  

  // Screen Loop ==========================
  //Serial.print(F("Mem>> ")); Serial.print (freeMemory()); 
  //Serial.println(F(" Screen Loop"));
    // screen update --------------

    // Bars --------------
    drawProgressBar2(barT1StartHo, barT1StartVe);
    drawProgressBar2(barT2StartHo, barT2StartVe);

    updateProgressBar2(0, barT1StartHo, barT1StartVe);
    updateProgressBar2(1, barT2StartHo, barT2StartVe);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(116,barT1StartVe-10);             // Start in back-top of bar
      display.println(F("H%")); 
      display.setCursor(0,barT1StartVe-10);             // Start in front-top of bar 
      display.cp437(true); 
      display.write(248);
      display.println(F("C"));
    drawSetTempRange2();
    drawTempRange2(0, barT1StartHo, barT1StartVe);
    drawTempRange2(1, barT2StartHo, barT2StartVe);
    drawHumRange2(0, barT1StartHo, barT1StartVe);
    drawHumRange2(1, barT2StartHo, barT2StartVe);
    

  // Graph1 --------------
      //Serial.print(F("Mem>> ")); Serial.println (freeMemory());   
      //Serial.println(F("Screen Graph Loop"));
    
    
    if (startSettingsGraph1){
    // Starting up graph. Did not start printing, Stopped printing or Warming up/Cooling Down.
      startSettingsGraph1 = false;
      if (temp[1] > settings.arrayProfileSettings[settings.selectedProfile][0]){   // if current temp is higher then set temp.
      // use current temp as top and set temp as botting
        graph1BottomRange = 20 - sessionResetbyTempDifference;
        graph1TopRange = temp[1] ;
      } else {                                                  // if current temp is lower then set temp.
      // use current temp as bottom, set temp as top.
        graph1BottomRange = temp[1] - sessionResetbyTempDifference;
        graph1TopRange = setMaxT2;
      }
      }

      
  // Update Graph1
    if (millis() - lastUpdateGraph1 > settings.graph1Interval*1000){    // did the timer trigger for graph update?
    // Create a new graph update
      lastUpdateGraph1 = millis();
      // Create the newest graph1 value
      drawHeight = map((temp[1]*100), (graph1BottomRange*100), (graph1TopRange*100), 0, graph1Height ); // used *100 to turn floats 2 decimals into int because map can't handle floats.
      // Add the newest created graph1 value (drawHeight) to to oldest place in the graph1 Array.
      if (lastReadingNR < graph1Length-1){  // are we at the end of the array?
        lastReadingNR++;                      // no? then olderst value is in last point+1 .
        } else {
          lastReadingNR = 0;                    // yes? then oldest value is in start of the array.
          //Serial.print(F("lastReadingNR 0 = "));Serial.println(readArray[0])
        }
      readArray[lastReadingNR] = drawHeight;    // Overwrite the oldest value with the new value.
    }
    updateGraph1();
    if (updateGraph1Scale){          // New update needed in graph scaling?
    // Adjust graph1 zoom/scaling, based on minTemp or maxTemp in session.
      updateGraph1Scaling();
      updateGraph1Scale = false; 
    }
    updateGraph1Pointers();
    updateGraph1Addons(1, graph1StartHorizontal, graph1StartVertical, graph1Length, graph1Height);                   
  
    // User Interface building
  
  // Menu GUI
  switch (currentMenuPage){
    case 0:   //"System Settings:"
    if (!buttonLock){   // When not button locked, show as being able to interact.
        drawMenuSystemSettings(1);; // draw with selection on setting scrolling/changing
        drawHeaderSystemSettings(); // draw header (has no selection options)
      } else { // selections not shown by lock
        drawMenuSystemSettings(0);  // draw wihtout selection
        drawHeaderSystemSettings(); // draw header (has no selection options)
      }
      break;
    case 1:   //"MainScreen":
      if (!buttonLock){   // When not button locked, show as being able to interact.
        drawHeaderProfile(1);
      } else { // selections not shown by lock
        drawHeaderProfile(0);   // When button locked, show as no interaction possible.
      }
      //drawMenuLayer1();
      break;
    case 2:   //"Profile Settings:"
      if (!buttonLock){   // When not button locked, show as being able to interact.
        drawMenuProfileSettings(1);
        drawHeaderProfile(0);
      } else { // selections not shown by lock
        drawMenuProfileSettings(0);
        drawHeaderProfile(0);
      }
      break;
    case 3:   //"Change Profile Setting:"
      if (!buttonLock){   // When not button locked, show as being able to interact
        drawMenuProfileSettings(2);
        drawHeaderProfile(0);
      } else { // selections not shown by lock
        drawMenuProfileSettings(0);
        drawHeaderProfile(0);
      }
      break;
    case 4:   //"Change System Setting:"
      if (!buttonLock){   // When not button locked, show as being able to interact
        drawMenuSystemSettings(2);  // draw with selection on value scrolling/changing
        drawHeaderSystemSettings();
      } else { // selections not shown by lock
        drawMenuSystemSettings(0);
        drawHeaderSystemSettings();
        }
      break;


  }
  // Special GUI
  if (buttonLockMessage == 1 ){
  // Set timer for buttonlock Message
    buttonLockMessageTimer = millis();
    redButtonTapped = LOW;    // reset button tap for next tap
    blueButtonTapped = LOW;    // reset button tap for next tap 
    buttonLockMessage = 2;
  }
  if (buttonLockMessage == 2 && millis() - buttonLockMessageTimer < buttonLockMessageTime){
  // Button-Lock Message / instructions as pop-up.
    drawButtonLockPopup();
  } else {
    buttonLockMessage = 0;
  }
  // Render Screen
  display.display();
  display.clearDisplay(); // Clear display buffer
}

// =========== EEPROM Functions ===============================
// Option 1 : Arduino default internal version (atmega328)
/*
  void loadConfig() {  
    //Serial.print(F("Mem>> ")); Serial.print (freeMemory()); 
    Serial.println(F(" EEPROM Function"));
    // To make sure there are settings, and they are YOURS!
    // If nothing is found it will use the default settings.
    if (EEPROM.read(CONFIG_START + sizeof(settings) - 1) == settings.version_of_program[3] // this is '\0'
        EEPROM.read(CONFIG_START + sizeof(settings) - 2) == settings.version_of_program[2] &&
        EEPROM.read(CONFIG_START + sizeof(settings) - 3) == settings.version_of_program[1] &&
        EEPROM.read(CONFIG_START + sizeof(settings) - 4) == settings.version_of_program[0])
    { // reads settings from EEPROM
      for (unsigned int t=0; t<sizeof(settings); t++)
        *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
    } else {
      // settings aren't valid! will overwrite with default settings
      saveConfig();
    }
  }

  void saveConfig() {
    for (unsigned int t=0; t<sizeof(settings); t++)
    { // writes to EEPROM
      EEPROM.write(CONFIG_START + t, *((char*)&settings + t));
      // and verifies the data
      if (EEPROM.read(CONFIG_START + t) != *((char*)&settings + t))
      {
        // error writing to EEPROM
      }
    }
  }*/

// Option 2 : Adafruit_FRAM_SPI version
/*
  void loadConfig() {  
    //Serial.print(F("Mem>> ")); Serial.print (freeMemory()); 
    Serial.println(F(" EEPROM Function"));
    // To make sure there are settings, and they are YOURS!
    // If nothing is found it will use the default settings.
    if (//fram.read(CONFIG_START + sizeof(settings) - 1) == settings.version_of_program[3] // this is '\0'
        fram.read8(CONFIG_START + sizeof(settings) - 2) == settings.version_of_program[2] &&
        fram.read8(CONFIG_START + sizeof(settings) - 3) == settings.version_of_program[1] &&
        fram.read8(CONFIG_START + sizeof(settings) - 4) == settings.version_of_program[0])
    { // reads settings from EEPROM
      for (unsigned int t=0; t<sizeof(settings); t++)
        *((char*)&settings + t) = fram.read8(CONFIG_START + t);
    } else {
      // settings aren't valid! will overwrite with default settings
      saveConfig();
    }
  }

  void saveConfig() {
    for (unsigned int t=0; t<sizeof(settings); t++)
    { // writes to EEPROM
      fram.writeEnable(true);
      fram.write8(CONFIG_START + t, *((char*)&settings + t));
      fram.writeEnable(false);
      // and verifies the data
      if (fram.read8(CONFIG_START + t) != *((char*)&settings + t))
      {
        // error writing to EEPROM
        Serial.print(F("Error while trying to write to EEPROM!!!"));
      }
    }
  }
  */

// Option 3 : ATSAMD21 emulated EEPROM using FLASH (Arduino Zero / Adafruit M0)
// !!! FLASH HAS ONLY ~ 10.000 WRITE CYCLES !!! Be sure to prevent writing to it often, for example, write only at powerloss.
void loadConfig() {
 Serial.println("Loading system config from Flash memory");
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (//EEPROM.read(CONFIG_START + sizeof(settings) - 1) == settings.version_of_program[3] // this is '\0'
      EEPROM.read(CONFIG_START + sizeof(settings) - 2) == settings.version_of_program[2] &&
      EEPROM.read(CONFIG_START + sizeof(settings) - 3) == settings.version_of_program[1] &&
      EEPROM.read(CONFIG_START + sizeof(settings) - 4) == settings.version_of_program[0])
  { // reads settings from EEPROM
    Serial.println("Memory config found, now loading");
    for (unsigned int t=0; t<sizeof(settings); t++)
      *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
  } else {
    Serial.println("No memory config found, loading factory defaults");
    // settings aren't valid! will overwrite with default settings
    //saveConfig();
  }
}

void loadServoProfiles(){
   Serial.println("Loading servo profiles from Flash memory");
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (//EEPROM.read(SERVOCONFIG_START + sizeof(settings) - 1) == servoSettings.version_of_program[3] // this is '\0'
      EEPROM.read(SERVOCONFIG_START + sizeof(servoSettings) - 2) == servoSettings.version_of_program[2] &&
      EEPROM.read(SERVOCONFIG_START + sizeof(servoSettings) - 3) == servoSettings.version_of_program[1] &&
      EEPROM.read(SERVOCONFIG_START + sizeof(servoSettings) - 4) == servoSettings.version_of_program[0])
  { // reads settings from EEPROM
    Serial.println("Memory config found, now loading");
    for (unsigned int t=0; t<sizeof(servoSettings); t++)
      *((char*)&servoSettings + t) = EEPROM.read(SERVOCONFIG_START + t);
  } else {
    Serial.println("No memory config found, loading factory defaults");
    // settings aren't valid! will overwrite with default settings
    //saveServoProfiles();
  }
}

void saveConfig() {
  Serial.println("saving new config to memory");
  for (unsigned int t=0; t<sizeof(settings); t++)
  { // writes to EEPROM
    EEPROM.write(CONFIG_START + t, *((char*)&settings + t));
    // and verifies the data
    if (EEPROM.read(CONFIG_START + t) != *((char*)&settings + t))
    {
      Serial.println("Error during saving configuration to memory!!!");
      // error writing to EEPROM
    }else{
      //Serial.println("Configuration saved and validated with succes!");
    }
  }
  saveToFlashQueue = true;
}

void saveServoProfiles() {
  Serial.println("saving new servo profiles to FLASH memory");
  for (unsigned int t=0; t<sizeof(servoSettings); t++)
  { // writes to EEPROM
    EEPROM.write(SERVOCONFIG_START + t, *((char*)&servoSettings + t));
    // and verifies the data
    if (EEPROM.read(SERVOCONFIG_START + t) != *((char*)&servoSettings + t))
    {
      Serial.println("Error during saving servoSettings to memory!!!");
      // error writing to EEPROM
    }else{
      //Serial.println("servoSettings saved and validated with succes!");
    }
  }
  saveToFlashQueue = true;
}

void ISR_powerLoss_detector() {
  pinMode(analogPowerLossDetectPin, INPUT);
  analogReadTest = analogRead(analogPowerLossDetectPin);
  // Charge Cap : float-charge cap up to 90% (.9*1023 = 920.7) in 10ms sessions
  if (analogReadTest < 920){
    pinMode(analogPowerLossDetectPin, OUTPUT);
    digitalWrite(analogPowerLossDetectPin, HIGH);
    return;
  }
  
  // Read Cap : 
  //pinMode(analogPowerLossDetectPin, INPUT);
  if ( analogReadTest > 1000) {
    if (saveToFlashQueue){
      noInterrupts();                                     // Disable interrupts
      EEPROM.commit();                                  // Only now commit the changes to the actual flash storage
      pinMode(LED1, OUTPUT);
      digitalWrite (LED1, HIGH );
      flashCommitDone = true;
      while (1){
        Serial.println("Power Loss Detected!");        // Go in endless loop. (PREVENTS WRITING STUPID AMOUNT OF CYCLES TOT FLASH!)
        Serial.print(analogReadTest); Serial.print(" = cap charge | write done by time interrupt? : "); Serial.print(flashCommitDone);Serial.print(" Chnges in MemQueue? = "); Serial.print(saveToFlashQueue);Serial.print(" saveServoProfilesToFlash = "); Serial.println(settings.saveServoProfilesToFlash);
      }
    }
  }
  //Serial << analogRead(A0) << endl; 
}

void timer3Init(){

    pinMode(LEDtimer, OUTPUT);
  //noInterrupts(); // disable all interrupts
  // Set up the generic clock (GCLK4) used to clock timers
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(7);             // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |         // Enable GCLK4
                      GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                      GCLK_GENCTRL_ID(7);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization
 
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;             // Turn the power to the TC3 module on
  // Feed GCLK4 to TCC2 (and TC3)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 (and TC3)
                      GCLK_CLKCTRL_GEN_GCLK7 |     // Select GCLK4
                      GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK4 to TCC2 (and TC3)
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  TC3->COUNT16.COUNT.reg = 0x0000;                      // Set period register to 255 (only works in combination with counter set? CC? Need to do match.)
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);         // Wait for synchronization
  
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_OVF;   // Enable TC3 interrupts on overflow
  //TC3->COUNT.INTENSET.reg = /*TC_INTENSET_MC1 | TC_INTENSET_MC0 |*/ TC_INTENSET_OVF; // Enable TC3 interrupts
  
  //NVIC_DisableIRQ(TC3_IRQn);
  //NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 2);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 0 (highest)
  NVIC_EnableIRQ(TC3_IRQn);         // Connect TC3 to Nested Vector Interrupt Controller (NVIC)

  // Set the TC3 timer to tick at 2MHz, or in other words a period of 0.5us - timer overflows every 128us
  // timer counts up to (up to 255 in 128us)
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 |      // Set prescaler to 8, 16MHz/8 = 2MHz
                           TC_CTRLA_PRESCSYNC_PRESC |     // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;          // Set the counter to 8-bit mode
                           
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;               // Enable TC3
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  TC3->COUNT16.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                             TC_READREQ_ADDR(TC_COUNT16_COUNT_OFFSET);        // Offset of the 8 bit COUNT register
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for (read) synchronization

  //use this to disable the counter :
	//TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	//while(TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

//interrupts(); // enable all interrupts  
}
void TC3_Handler(){           // ISR TC3 overflow callback function
// This ISR is called every 15.54ms (~64Hz)
  if (x == 1) {
    // Toggle on
    digitalWrite(LEDtimer, HIGH);
    x = 0;
  } else {
    // Toggle off
    digitalWrite(LEDtimer, LOW);
    x = 1;
  }
  ISR_powerLoss_detector();
  //analogReadTest = analogRead(analogPowerLossDetectPin);
  REG_TC3_INTFLAG = TC_INTFLAG_OVF;
  TC3->COUNT16.COUNT.reg = 50000;          // Set new overflow count?
  //while (TC3->COUNT16.STATUS.bit.SYNCBUSY);     // Wait for synchronization
  //TC3->COUNT8.INTFLAG.reg = TC_INTFLAG_OVF;   // Rest the overflow interrupt flag
}


/*void resetFactorySettings(){
  for (int i = 0 ; i < fram.length() ; i++) {
    fram.writeEnable(true);
    fram.write(i, 0);
    fram.writeEnable(false);
    resetFunc(); //call reset  
  }
}
void(* resetFunc) (void) = 0; //declare reset function at address 0
*/

// =========== SENSORS Functions ===============================
// SHT31-D Temp/Hum. Sensor ==========================
void readTempHum(byte sensorID) {
  //Serial.print(F("Mem>> ")); Serial.print (freeMemory()); 
  //Serial.println(F(" Temp/Hum Function"));
  sht31.begin(sht31_On[sensorID]);               // select sensor to read
       temp[sensorID] = sht31.readTemperature();     // Fill variable t with temperature reading
       hum[sensorID] = sht31.readHumidity();        // Fill variable h with humidity reading
  lastSensor = sensorID;
}

// Button Sensor's Functions ==========================
void buttonUp(){
  if ( !redButton ){                  // Rigister only Low to High changes
     redButtonTapped = HIGH;          // Set flag that it was tapped (but was not yet pressed, prevents registering a tap when goign from High to Low again).
  }
  redButton = !redButton;         //
  Serial.print(F("Red Button Change!!! to "));Serial.println(redButton);
}
void buttonDown(){
  if ( !blueButton ){                 // Rigister only High tp Low to changes
      blueButtonTapped = HIGH;        // Set flag that it was tapped (but was not yet pressed, prevents registering a tap when goign from High to Low again).
  }
  blueButton = !blueButton;       // 
  Serial.print(F("Blue Button Change!!! to "));Serial.println(blueButton);
}

int buttonPressType(){       // 0=noPress, 1=blueTab, 2=blueLong, 3=redTab, 4=redLong, 5=dubbleTab, 6=dubbelLong.
  if (buttonLock){ // Are the buttons locked?
  // Yes, only enable red long press to unlock
    if (redButtonTapped || blueButtonTapped){
      if (redButton) {          // Check for long press red
        delay(500);
        if (redButton){
        Serial.println(F("long Red Press, Unlocking button Lock. ")); 
        buttonLock = false;
        buttonAutoLockTimer = millis(); // Update last time button pressed.
        return 0;           
        }
      }
      buttonLockMessage = 1; // Set this to 1 to start timer.
      Serial.print(F("Buttons are locked, use red long press to unlock. "));
    }
  } else {
  // No, return normal button functions.
    if (blueButtonTapped){ 
      if (blueButton) {             // Check for long press blue.
        delay(500);
        if (blueButton && redButton){
          Serial.print(F("long Double Press"));
          buttonAutoLockTimer = millis(); // Update last time button pressed.
          return 5;
        } else if (blueButton){
          Serial.println(F("long Blue Press")); 
          buttonAutoLockTimer = millis(); // Update last time button pressed.
          return 2;         
        } 
      }
      Serial.println(F("short Blue Press"));
      buttonAutoLockTimer = millis(); // Update last time button pressed.
      return 1;
    }  
    if (redButtonTapped){
      if (redButton) {          // Check for long press red
        delay(500);
        if (redButton && blueButton){
          Serial.print(F("long Double Press"));
          buttonAutoLockTimer = millis(); // Update last time button pressed.
          return 5;
        } else if (redButton){
          Serial.println(F("long Red Press")); 
          buttonAutoLockTimer = millis(); // Update last time button pressed.
          return 4;           
        }
      }
      Serial.println(F("short Red Press"));
      buttonAutoLockTimer = millis(); // Update last time button pressed.
      return 3;
    } 
  }
  return 0;
}

// =========== ACTORS Functions ===============================

// Servo Functions ==========================
void learnAllServoProfiles(){
  for (uint i=0; i<(sizeof (servo_)/sizeof(servo_[0])); i++) {
      Serial.println("Servo "); Serial.println(i); Serial.println (" Learning -------------------------");
      servo_[i].learnServoProfile();
    // take settings our of AcEservo librarie for storage in EEPROM/FLASH
    // Servo Define
    //int analogPin[i];               // Pin for Sensing Current draw. Going between servo negative and shurt resistor to ground (also add ~10K arduino protective resitor toward this pin).
    //int servoPin[i];                // Pin for control signal to Servo.
    // Servo Basic Settings
    //int posSignal[i];               // variable to store the stopping signal for the servo [default = 90].
    //int increaseFlow[i];            // Direction and speed to increase flow [default = 0].
    //int decreaseFlow[i];            // Direction and speed to decrease flow [default = 180].

    // Default Start parameters for Stall Detection --------------
    // These settings can be overwritten after the learning process and can even be stored in EEPROM
    servoSettings.stepTolleranceLowC[i] = servo_[i].stepTolleranceLowC;     // Adjust to how sensitive stall detection should react at lower loads (more is less sensetive) [default analog = 30, digital = 10].
    servoSettings.stepTolleranceHighC[i] = servo_[i].stepTolleranceHighC;    // Adjust to how sensitive stall detection should react at higher loads (more is less sensetive) [default analog= -25 digital = -25] If you go below 0 it will alway's trigger before max load is reached.
    servoSettings.startupCurrent[i] = servo_[i].startupCurrent;          // Current draw that needs to be seen before it believes the servo is actually starting to turn (some servo's react slow and take some time before they actually start). [default = 15].
    //servoSettings.servoMinCurrent[i] = servo_[i].             // Not yet in use
    servoSettings.servoNominalCurrent[i] = servo_[i].nominalCurrent;    // Stores the learned normal load operations current (low end current) of the servo device/setup. [example = 50].
    servoSettings.servoMaxCurrent[i] = servo_[i].maxCurrent;         // Stores the learned high load operation current (high end current) of the servo device/setup. [example = 500].
    // Servo Advanced Settings  -------------- 
    //servoSettings.dynamicStepSize[i] =      // At startup it will try to detect the smallest flowStepSize detectable by the stall detector and overwrite above set flowStepSize value with it. [default = true].
    servoSettings.minFlowStepSize[i] = servo_[i].minStepSize;         // Used when not using dynamicStepSize detection. Lower is smaller and more steps, but to small may not trigger a stall. [default = 10].
    Serial.print(F("servo LEARNED-mem Nominal/Max Current = "));Serial.print(servoSettings.servoNominalCurrent[i]);Serial.print(F("/"));Serial.println(servoSettings.servoMaxCurrent[i]);

  }
  servoSettings.servoProfileAlreadyKnown = true;
  saveServoProfiles();
  saveToFlashQueue = true;
}

uint setNextServoToOperate( uint currentServo, int stepDirection){
  for (uint i=1; i <= ((sizeof (servo_)/sizeof(servo_[0]))+1); i++ ){
    Serial.print(F("In setNextServoToOperate i = "));Serial.print(i);Serial.print(F(" currentServo = "));Serial.print(currentServo);Serial.print(F(" witch has stallFlag = "));Serial.println(servo_[currentServo].stallFlag);
    Serial.print(F(" lastStepDirection = "));Serial.print(lastStepDirection); Serial.print(F(" newStepDirection = "));Serial.println(stepDirection);
    if ( i > (sizeof (servo_)/sizeof(servo_[0]))) {
      Serial.print (F("All servo's ("));Serial.print(i-1);Serial.println(F(") seem to have reached there maximum position in this direction."));
      Serial.println (F("Servo's will only rotate again in oposite direction."));
      servoControllSteps = 0; // Cancel step direction by seting to 0.
      break;
    }
    if (stepDirection != lastStepDirection) {
      Serial.print(F("Changing servoStepDirection, re-using last used servo"));
      lastStepDirection = stepDirection;
      break;  // we are changing direction, use last servo again to help keep multiple servo's a bit more in sync.
    } else {
      if (currentServo + 1 < (sizeof (servo_)/sizeof(servo_[0])) ){
          currentServo++;
      } else {
        currentServo = 0;
      }
      if (servo_[currentServo].stallFlag != stepDirection){
        break;
      }
    }
  }

  return currentServo;
}

// Screen ==========================
// Startup dislay -------------- /*

// Display normal screen
void drawHeaderProfile(int selector){
  //if (selectedMenuPage == 0){                   // Show profile selection.
    display.setCursor(62,0);                // Start at top-center for 3 characters of 5 pixels
    display.setTextSize(1);
    if (selector == 1){
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.print(arrayProfileID[settings.selectedProfile]);
    display.print(settings.arrayProfileSettings[settings.selectedProfile][0],0);
    display.print(" ~");
    display.print(settings.arrayProfileSettings[settings.selectedProfile][1],1);
  //}
}

void drawHeaderSystemSettings(){
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print(F("System Settings: "));
}

void drawMenuSystemSettings(int selector) {
  display.fillRect(0,0,128,19,BLACK);   // Make empty back ground to draw the menu.
  display.setCursor(0,10);              // Start at top-left corner
  display.setTextSize(1);
  if (selector == 1){
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.print(arraySystemSettingNames[selectedSystemSetting]);
    display.setTextColor(WHITE);
  display.print(F(" ")); 
  if (selector == 2){
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  if ( selectedSystemSetting == 0){  // Target Temp
    display.print(settings.stabelizeSessionTempTime);
  } 

  if ( selectedSystemSetting == 1){  // graph1IntervalTime

    display.print(settings.graph1Interval);
  }

  if ( selectedSystemSetting == 2){  // buttonAutoLockTime
    display.print(settings.buttonAutoLockTime);
  }

  if ( selectedSystemSetting == 3){  // Save Servos
    if (settings.saveServoProfilesToFlash == 1){
      display.print("yes");
    } else {
      display.print("no");
    }
  }

  if ( selectedSystemSetting == 4){  // Relearn Servos
    
      display.print("Now!");

  }  
}

void drawMenuProfileSettings(int selector) {
  display.fillRect(0,0,128,16,BLACK);
  display.setCursor(0,10);              // Start at top-left corner
  display.setTextSize(1);
  if (selector == 1){
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.print(arrayProfileSettingNames[selectedProfileSetting]);
  display.setTextColor(WHITE);
  display.print(F(" "));
  if (selector == 2){
    display.setTextColor(BLACK, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  if ( selectedProfileSetting == 0){  // Target Temp
    display.print(settings.arrayProfileSettings[settings.selectedProfile][0],0);
  } 

  if ( selectedProfileSetting == 1){  // Range
    display.print(settings.arrayProfileSettings[settings.selectedProfile][1],1);
  }

  if ( selectedProfileSetting == 2){  // Interval
    display.print(settings.arrayProfileSettings[settings.selectedProfile][2],0);
  } 
  
}

// Special GUI ---------------
void drawButtonLockPopup() {
  display.fillRect(10,10,108,44,BLACK);   // Make empty back ground to draw the message
  display.drawRect(10,10,108,44,WHITE);   // Make line rectangle
  display.setCursor(22,13);              // Start at top-left corner
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Long press Red");
  display.setCursor(22,25); 
  display.println("  to unlock");
  display.setCursor(22,33); 
  display.println("   Buttons!");
  }


// Display Bars --------------
void drawProgressBar2(int horizontalStartPixel, int verticalStartPixel) {
     //display.drawRect(barT2StartHo, barT2StartVe, barT2Length, barT2Height, WHITE); // horizontalStart, verticalStart, length, drop
    display.drawLine(horizontalStartPixel, verticalStartPixel-2, barT2StartHo, verticalStartPixel+barHeight+2, WHITE); 
    display.drawLine(horizontalStartPixel+barLength, verticalStartPixel-2, horizontalStartPixel+barLength, verticalStartPixel+barHeight+2, WHITE); 
}

void drawSetTempRange2(void) {        //Draw the set temperature range with 4 little vertical lines
    int min = map(setMinT2, 0, 100, 0, barLength);
    int max = map(setMaxT2, 0, 100, 0, barLength);
    display.drawLine(min+barT2StartHo, barT2StartVe-1, min+barT2StartHo, barT2StartVe+barHeight+2, WHITE); // horizontalStart, verticalStart, horizontalEnd, verticalEnd, Color
    display.drawLine(min+barT2StartHo, barT2StartVe+barHeight+2, max+barT2StartHo, barT2StartVe+barHeight+2, WHITE); 
    display.drawLine(max+barT2StartHo, barT2StartVe-1, max+barT2StartHo, barT2StartVe+barHeight+2, WHITE); 

}

void drawTempRange2(int sensor, int horizontalStartPixel, int verticalStartPixel) {
    int min = map(minTempinSession[sensor], 0, 100, 0, barLength);
    int max = map(maxTempinSession[sensor], 0, 100, 0, barLength);
    display.drawLine(min+horizontalStartPixel, verticalStartPixel-2, min+horizontalStartPixel, verticalStartPixel, WHITE); // horizontalStart, verticalStart, horizontalEnd, verticalEnd, Color
    display.drawLine(min+horizontalStartPixel, verticalStartPixel+barHeight+2, min+horizontalStartPixel, verticalStartPixel+barHeight, WHITE); // horizontalStart, verticalStart, horizontalEnd, verticalEnd, Color
    display.drawLine(max+horizontalStartPixel, verticalStartPixel-2, max+horizontalStartPixel, verticalStartPixel, WHITE); 
    display.drawLine(max+horizontalStartPixel, verticalStartPixel+barHeight+2, max+horizontalStartPixel, verticalStartPixel+barHeight, WHITE); 

}

void drawHumRange2(int sensor, int horizontalStartPixel, int verticalStartPixel) {
    int min = map(minHuminSession[sensor], 0, 100, 0, barLength);
    int max = map(maxHuminSession[sensor], 0, 100, 0, barLength);
    display.drawPixel(min+horizontalStartPixel, verticalStartPixel + (barHeight/2), INVERSE); // horizontalStart, verticalStart, horizontalEnd, verticalEnd, Color
    display.drawPixel(max+horizontalStartPixel, verticalStartPixel + (barHeight/2), INVERSE); // horizontalStart, verticalStart, horizontalEnd, verticalEnd, Color
 }

void updateProgressBar2(int sensorID, int horizontalStartPixel, int verticalStartPixel) {
    //float x = PI;
    display.setTextSize(1);
    display.setTextColor(WHITE); 
    display.setCursor(0,verticalStartPixel-1);             // Start in front of bar 
    display.print(temp[sensorID],1);
    //Serial.print("waarde van temperature = "); Serial.println(temperature);
    display.setCursor(116,verticalStartPixel-1);             // Start in back of bar 
    display.print(hum[sensorID],0);
    //Serial.print("testing humidity update "); Serial.print(verticalStartPixel); Serial.print(" with value in update "); Serial.println(humidity);
    int fillTempLength = map(temp[sensorID], 0, 100, 0, barLength);
    display.fillRect(horizontalStartPixel, verticalStartPixel, fillTempLength , barHeight, WHITE);              //horizontalStart, verticalStart,horizontelLength, verticalHeight
    int fillHumLength = map(hum[sensorID], 0, 100, 0, barLength);
    display.fillRect(horizontalStartPixel, verticalStartPixel + (barHeight/2) , fillHumLength ,1 , INVERSE);    //horizontalStart, verticalStart,horizontelLength, verticalHeight
    delay(10);
}

// Display Graph --------------
void updateGraph1(void){
  // This graph is build up from the last added number in an array (witch can move).
  // It keeps track of where the last number was added and (in part 1) starts reading towards array value "0"
  // If it reaches the start of the array "0" it will continue with part 2 reading from the last array value 
  // towards the last added value place.
  // This way we don't have to do array shifting, we can just add new value's in the next array spot overwriting
  // the oldest value.
  //Serial.print(F("Mem>> ")); Serial.print (freeMemory()); 
  //Serial.println(F(" updateGraph1 Function"));
    int i = 0;
    int graphResultNR = 0;
    int writeLocation = 0;
    // Thist part is needed for option 3 single line graph
    //int lastWriteLocation = graph1Length + graph1StartHo;   // for option 3
    //int lastI = lastReadingNR;                              // For option 3
    // Reset / empty graph
   // part 1/2 - last added reading in the array to 0
    for (i = lastReadingNR; i > -1; i-- ){
      writeLocation = (graph1Length - 1) - graphResultNR + graph1StartHorizontal;
      // Option 1 : Draw single dot line graph
      display.drawPixel(writeLocation, graph1StartVertical + graph1Height - readArray[i], WHITE); // horizontalPoint, verticalPoint, Color
      // Option 2 : Draw filled graph
      //display.drawLine(writeLocation, graph1StartVe + graph1Height - readArray[i], writeLocation, graph1StartVe + graph1Height, WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
      // Option 3 : Draw single line graph
      //display.drawLine(writeLocation, graph1StartVe + graph1Height - readArray[i], lastWriteLocation, graph1StartVe + graph1Height - readArray[lastI] , WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
      //lastWriteLocation = writeLocation;
      //lastI = i;
      //Serial.print(graphResultNR); Serial.print(" part 1 : "); Serial.print(i); Serial.print(" - 0 waarde "); Serial.println(readArray[i]);
      graphResultNR++;
    }
   // part 2/2 - graphlength to lastreading
    for (i = graph1Length-1; i > lastReadingNR; i-- ){
      writeLocation = graph1Length - graphResultNR + graph1StartHorizontal;
      //Option 1 : Draw single dot line graph
      display.drawPixel(writeLocation, graph1StartVertical + graph1Height - readArray[i], WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
      // Option 2 : Draw filled graph
      //display.drawLine(writeLocation, graph1StartVe + graph1Height - readArray[i], writeLocation, graph1StartVe + graph1Height, WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
      // Option 3 : Draw single line graph
      //display.drawLine(writeLocation, graph1StartVe + graph1Height - readArray[i], lastWriteLocation, graph1StartVe + graph1Height - readArray[lastI] , WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
      //lastWriteLocation = writeLocation;
      //lastI = i;
      
      //Serial.print(graphResultNR); Serial.print("part 2 : ");Serial.print(i); Serial.print(" - "); Serial.print(lastReadingNR); Serial.print(" waarde "); Serial.println(readArray[i]);
      graphResultNR++;
    }
}

void emptyGraph1(void){
  for (int i = graph1Length-1; i > -1; i--){
    readArray[i] = 0;
  }
}

void updateGraph1Scaling(void){
  // update graph1 top scale value.
  if (maxTempinSession[1] < setMaxT2){         // is temp going obove nominal range?
    graph1TopRange = setMaxT2 + 0.1;              // Top graph1 is not adjusted, still within range.
  } else {
    graph1TopRange = maxTempinSession[1] + 0.1;      // Adjust top of graph1 to new maximal record.
  } 
  // update graph1 bottom scale value.
  if (minTempinSession[1] > setMinT2){         // Is temp going under nominal range?
    graph1BottomRange = setMinT2 - 0.1;           // Bottom graph1 is not adjusted, still within range.
  } else {
    graph1BottomRange = minTempinSession[1] - 0.1;   // Adjust bottom of graph1 to new minimal record.
  }
}

void updateGraph1Pointers(void){
  // Update Target temp line locations.
  targetDrawHeight = map((settings.arrayProfileSettings[settings.selectedProfile][0]*100), (graph1BottomRange*100), (graph1TopRange*100), 0, graph1Height );
  // Update min and max temp line locations. 
  minDrawHeight = map((minTempinSession[1]*100), (graph1BottomRange*100), (graph1TopRange*100), 0, graph1Height );
  maxDrawHeight = map((maxTempinSession[1]*100), (graph1BottomRange*100), (graph1TopRange*100), 0, graph1Height );
         
}

void updateGraph1Addons(int sensorID, int horizontalStartPixel, int verticalStartPixel, int length, int height){
  //Draw session min and max Temperature records as dots on both sides of the graph.
  //Edge lines 
      //left bottom
    display.drawLine(graph1StartHorizontal, graph1StartVertical + graph1Height, graph1StartHorizontal+4, graph1StartVertical + graph1Height, WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
    display.drawLine(graph1StartHorizontal, graph1StartVertical-3 + graph1Height, graph1StartHorizontal, graph1StartVertical + graph1Height, WHITE);
        //right bottom
    display.drawLine(graph1StartHorizontal + graph1Length, graph1StartVertical + graph1Height, graph1StartHorizontal + graph1Length-4, graph1StartVertical + graph1Height, WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
    display.drawLine(graph1StartHorizontal + graph1Length, graph1StartVertical-3 + graph1Height, graph1StartHorizontal + graph1Length, graph1StartVertical + graph1Height, WHITE);
        //left top
    display.drawLine(graph1StartHorizontal, graph1StartVertical, graph1StartHorizontal+4, graph1StartVertical, WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
    display.drawLine(graph1StartHorizontal, graph1StartVertical+3, graph1StartHorizontal, graph1StartVertical, WHITE);
        //right top
    display.drawLine(graph1StartHorizontal + graph1Length, graph1StartVertical, graph1StartHorizontal + graph1Length-4, graph1StartVertical, WHITE); // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
    display.drawLine(graph1StartHorizontal + graph1Length, graph1StartVertical+3, graph1StartHorizontal + graph1Length, graph1StartVertical, WHITE);
  //Target Temp pixels
    display.drawLine(graph1StartHorizontal + 1, graph1StartVertical + graph1Height - targetDrawHeight, graph1StartHorizontal + 3, graph1StartVertical + graph1Height - targetDrawHeight, WHITE);
    display.drawLine(graph1StartHorizontal + graph1Length-1 , graph1StartVertical + graph1Height - targetDrawHeight, graph1StartHorizontal + graph1Length-3 , graph1StartVertical + graph1Height - targetDrawHeight, WHITE);
  //Current record pixels
    //display.drawLine(graph1StartHorizontal, graph1StartVertical + graph1Height - maxDrawHeight, graph1StartHorizontal + 1, graph1StartVertical + graph1Height - maxDrawHeight, WHITE);
    //display.drawLine(graph1StartHorizontal + graph1Length, graph1StartVertical + graph1Height - maxDrawHeight, graph1StartHorizontal + graph1Length -1, graph1StartVertical + graph1Height - maxDrawHeight, WHITE);
  //Max record pixels
    display.drawLine(graph1StartHorizontal, graph1StartVertical + graph1Height - maxDrawHeight, graph1StartHorizontal + 1, graph1StartVertical + graph1Height - maxDrawHeight, WHITE);
    display.drawLine(graph1StartHorizontal + graph1Length, graph1StartVertical + graph1Height - maxDrawHeight, graph1StartHorizontal + graph1Length -1, graph1StartVertical + graph1Height - maxDrawHeight, WHITE);
  //Min record pixels
    display.drawLine(graph1StartHorizontal, graph1StartVertical + graph1Height - minDrawHeight, graph1StartHorizontal + 1, graph1StartVertical + graph1Height - minDrawHeight, WHITE);
    display.drawLine(graph1StartHorizontal + graph1Length, graph1StartVertical + graph1Height - minDrawHeight, graph1StartHorizontal + graph1Length -1, graph1StartVertical + graph1Height - minDrawHeight, WHITE);
  //Target Temp numbers
    display.setCursor(horizontalStartPixel+length+7,verticalStartPixel+(height/2)-(7/2));                // Start at top-center for 3 characters of 5 pixels
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print(settings.arrayProfileSettings[settings.selectedProfile][0],0);
    display.drawLine(horizontalStartPixel + length, verticalStartPixel + height - targetDrawHeight, horizontalStartPixel + length+5 , verticalStartPixel + (height/2), WHITE);      //Horizontal line to temp numbers
    display.drawLine(horizontalStartPixel + length+5 , verticalStartPixel + (height/2)-1, horizontalStartPixel + length+5 , verticalStartPixel + (height/2)+1, WHITE);  //Vertical line to temp numbers
  //Target Range numbers
  if (stabilizing || setTempReached){                                        // Only show details when in tange.
    display.setCursor(0,verticalStartPixel+(height/2)-(7/2));                // Start at top-center for 3 characters of 5 pixels
    display.setTextSize(1);
    display.setTextColor(WHITE); 
    display.print(maxTempinSession[1] - minTempinSession[1],1);
    //display.print(settings.arrayProfileSettings[settings.selectedProfile][1],1);
    // angle lines to number
      //Top maximal
      display.drawLine(horizontalStartPixel-5, verticalStartPixel+(height/2)-6, horizontalStartPixel-1, verticalStartPixel + height - maxDrawHeight, WHITE); //angle line
      display.drawLine(horizontalStartPixel-10, verticalStartPixel+(height/2)-6, horizontalStartPixel-6, verticalStartPixel+(height/2)-6, WHITE); //horizontal line 
      //Bottom minimal
      display.drawLine(horizontalStartPixel-5, verticalStartPixel+(height/2)+6, horizontalStartPixel-1, verticalStartPixel + height - minDrawHeight, WHITE); //angle line
      display.drawLine(horizontalStartPixel-10, verticalStartPixel+(height/2)+6, horizontalStartPixel-6, verticalStartPixel+(height/2)+6, WHITE); //horizontal line  
  }
  //vertival line with min max line on it
    //left bottom
    //display.drawLine(graph1StartHorizontal-5, graph1StartVertical + graph1Height-5, graph1StartHorizontal-5, graph1StartVertical + graph1Height, WHITE); // vertical line // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
    //display.drawLine(graph1StartHorizontal-5-1, graph1StartVertical + graph1Height - minDrawHeight, graph1StartHorizontal-5+1, graph1StartVertical + graph1Height - minDrawHeight, WHITE); // horizontal line
    //left top
    //display.drawLine(graph1StartHorizontal-5, graph1StartVertical, graph1StartHorizontal-5, graph1StartVertical+5, WHITE); // vertical line // horizontalPoint, verticalPoint, horizontalEnd, verticalEnd, Color
    //display.drawLine(graph1StartHorizontal-5-1, graph1StartVertical + height - maxDrawHeight, graph1StartHorizontal-5+1, graph1StartVertical+height - maxDrawHeight, WHITE); // horizontal line  
      
}

int freeMemory(){
  char top;
  #ifdef __arm__
    return &top - reinterpret_cast<char*>(sbrk(0));
  #elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
  #else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
  #endif  // __arm__
}

