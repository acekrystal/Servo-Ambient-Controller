/*
    SmartServo.h - Labrary for current sensing cheap continues servo's like micro _centerSignal's.
    Created by AcE Krystal (Wouter Horselenberg), 17-4-2019

    Creative commons share alike
*/

#include "Arduino.h"
#include <Servo.h>
#include "AcEservo.h"

AcEservo::AcEservo(//int servoPin, int analogSensePin,int centerSignal, int downSignal, int upSignal, int startupCurrent,int nominalCurrent, int maxCurrent, int minStepSize)
    //Servo Define
    int servoPin,
    int analogSensePin,
    //Servo Settings
    int centerSignal,
    int downSignal,
    int upSignal,
    int stepTolleranceLowC,
    int stepTolleranceHighC,
    //Servo Profile (Load or Default)
    int startupCurrent,
    int nominalCurrent,
    int maxCurrent)

    : Servo() 
    {
        //Servo Define
        _servoPin = servoPin;
        _analogSensePin = analogSensePin;
        //Servo Settings
        _centerSignal = centerSignal;
        _downSignal = downSignal;
        _upSignal = upSignal;
        _stepTolleranceLowC = stepTolleranceLowC;
        _stepTolleranceHighC = stepTolleranceHighC;
        //Servo Profile
        _startupCurrent = startupCurrent;
        _nominalCurrent = nominalCurrent;
        _maxCurrent = maxCurrent; 
        //_minStepSize = minStepSize;
        //Servo Status (Return)
        //int stallFlag,
        //int actionNominalLoad,
        //int actionMaxLoad)

    }

// Servo's ==========================

void AcEservo::learnServoProfile(){
  this->attach(_servoPin);      // attaches the servo on pin to the servo object;
  // Begin startup Sweep for Servo Profile Learning --------------
  Serial.println(F("Learning Servo and Use-Case Profile:"));
  Serial.print(F("Using low current tollerance : ")); Serial.println(_stepTolleranceLowC);
  Serial.print(F("Using high current tollerance : ")); Serial.println(_stepTolleranceHighC);
  Serial.print(F("Using start nominalCurrent : ")); Serial.println(_nominalCurrent);
  Serial.print(F("Using start maxCurrent : ")); Serial.println(_maxCurrent);
  int _nominalCurrentOpen = 0;      // Stores learning values for learning nominal Current usage while opening. [default = 0]
  int _nominalCurrentClose = 0;     // Stores learning values for learning nominal Current usage while closing. [default = 0]
  int _maxCurrentOpen = 0;          // Stores learning values for learning max Current usage while open. [default = 0]
  int _maxCurrentClose= 0;          // Stores learning values for learning max Current usage while closed. [default = 0]
  Serial.println("1 - Close (making sure to hit closing end)" );                     //Making sure it is fully closed so it can start do a full opening sweep
  AcEservo::moveServo(400, 110, 0);       //Turn fully closed slowly so it can start measuring a full opening sweep.
  delay(500);
  moveServo(100, 180, 0);       //Turn fully closed slowly so it can start measuring a full opening sweep.
  _maxCurrent = maxCurrentDetected;     // Quick adjust to servo max range;
  Serial.print(F("Learned Servo initial stalling maxCurrent : ")); Serial.println(_maxCurrent);
  delay(500);
  Serial.println("2 - Open");
  moveServo(400, 0, 1);        //Turn fully open slowly
  _nominalCurrentOpen = servoActionLoad;     //Write down the servoActionLoad just recorded for the slow full sweep action.
  delay(500);
  moveServo(100, 0, 0);          //Try to turn even further open on full power to learn MaxCurrentLoad (done by giving it an unreachable high Tollerance)
  _maxCurrentOpen = maxCurrentDetected;
  Serial.print(F("Learned Servo Opening : Nominal Open Load = ")); Serial.print(_nominalCurrentOpen); Serial.print(F("; Open Stalling Load = ")); Serial.println(_maxCurrentOpen);
  delay(500);
  Serial.println("3 - Close");
  moveServo(400, 180, 1);       //Turn fully closed slowly
  _nominalCurrentClose = servoActionLoad;    //Write down the servoActionLoad just recorded for the slow full sweep action.
  delay(500);
  moveServo(60, 180, 0);        //Try to turn even further open on full power to learn MaxCurrentLoad (done by giving it an unreachable high Tollerance)
  _maxCurrentClose = maxCurrentDetected;
  Serial.print(F("Learned Servo Closing : Nominal Close Load = ")); Serial.print(_nominalCurrentClose); Serial.print(F("; Close Stalling Load = ")); Serial.println(_maxCurrentClose);
  delay(500);
  //Comparing open and close sweep and using the highest numbers. This might need to add some code if your servo has higher load in 1 direction (lifting heavy load from ground).  
  if (_nominalCurrentOpen >= _nominalCurrentClose){
  _nominalCurrent = _nominalCurrentOpen;
  } else {
  _nominalCurrent = _nominalCurrentClose;
  }
  if (_maxCurrentOpen >= _maxCurrentClose){
  _maxCurrent = _maxCurrentOpen;
  } else {
  _maxCurrent = _maxCurrentClose;
  }
  Serial.print(F("Adjusting parameters to Nominal Load = ")); Serial.print(_nominalCurrent); Serial.print(F("; Stalling Load = ")); Serial.println(_maxCurrent);
  // End Startup sweep --------------
  // Step Size leanring --------------
  Serial.println("Trying to learning minimal detectable step size :");
  int minCloseStepSize = 0;               // Stores learning values for learning minimal step size while closed. [default = 0]
  int minOpenStepSize = 0;                // Stores learning values for learning minimal step size while open. [default = 0]
  Serial.println("4 - Close stepping"); // making sure it is closed and confirming sweep value's for stall detection
  moveServo(400, 180, 1);     //Turn fully closed slowly
  delay(200);
  stallFlag = 0;                          // Resetting stall flag for new detection.
  while (stallFlag == 0){                 // Increasing closing step size until stall is detectable.
      minCloseStepSize++;
      moveServo(minCloseStepSize, _downSignal, 1); // decrease flow with steps
      delay(100);
  }
  Serial.println("5 - Open stepping");
  moveServo(400, 0, 1);      //Turn fully open slowly
  delay(200);
  stallFlag = 0;                          // Resetting stall flag for new detection
  while (stallFlag == 0){                 // Increasing opening step size until stall is detectable.
      minOpenStepSize++;        
      moveServo(minOpenStepSize, _upSignal, 1); // increase flow with steps
      delay(100);
  }
  if (minCloseStepSize > minOpenStepSize){
      minStepSize = minCloseStepSize+2;
  } else {
      minStepSize = minOpenStepSize+2;
  }
  moveServo(400, 180, 1);       //Turn fully closed slowly
  Serial.print(F("Adjusting Minimal detectable step size to = "));Serial.println(_minStepSize);
  delay(100);
  
}


void AcEservo::moveServo (int multiplier, int directionSpeed, int enableStallStop){
  int servoActionCumelativeCurrent = 0;
  servoActionLoad = 0;
  maxCurrentDetected = 0;
  stallFlag = 0;

  // Start rotating the servo ------------------
  int current = 1;                                  // Set current to 1, not 0 for debug
  
    //myservo.write(directionSpeed);                // Tell rotating servo to start moving (write _centerSignal is stopping, so 100 or higher is good for turning)
    // Check when Servo actually starts trying to moving ------------------
    Serial.print(F("Waiting for Servo te start (trying to) move "));      
    this->attach(_servoPin);                                 
    this->write(directionSpeed);

    while (current < _startupCurrent){                          // Wait for servo to respond (they seem to have there own read cycles that can take up to ~15ms to actually start
        //noInterrupts();
        current = analogRead(_analogSensePin);          // read the analog current pin 
        //interrupts();
        Serial.print(current);                        
        Serial.print("."); 
        delay(10);                       
    }                                             // This part prevent the stall detection function measuring 0 current because of late starting servo's. It also helps making equal steps because timers start when there is actually current measured from the servo.
     // Servo Started, now continue ------------------
       Serial.println(F("Started!"));                 
    //delay(readDelay);                               // This fixes a bug found, giving a "0" when reading analogPin shortly after above while loop, read more : https://forum.arduino.cc/index.php?topic=603226.0 This is first set op 5ms for the first time, then back to 0ms.
    delay(16);                                     // Wait for peak startup to set in and fixes a bug found, giving a "0" when reading analogPin shortly after above while loop, read more : https://forum.arduino.cc/index.php?topic=603226.0 This is first set op 5ms
    //current = analogRead(analogPin);              // read the analog current pin
    _lastCurrent = 0;                        // read the input pin again (update for lastCurrent to give it a change to get higher value at spinup
  
  for (int a = 0; a < multiplier; a++){ 
     delay(8);
     // Calculate average Current load for whole changeFlow action. ------------------
     AcEservo::_stallSense(directionSpeed, enableStallStop);
     servoActionCumelativeCurrent = servoActionCumelativeCurrent + _lastCurrent;
     servoActionLoad = servoActionCumelativeCurrent / (a+1);
    // Serial.print(servoActionCumelativeCurrent);Serial.print(" a = "); Serial.print((a+1));Serial.print(" avg= "); Serial.println(servoActionLoad);
     // Keep track of maximum current draw for whole changeFlow action. ------------------

      
        if (stalled){
          Serial.println(F("Detected high current by stall detector, stopping current movement till new instructions."));                  // debug value
          break;
        }
  }
  this->write(_centerSignal);
  this->detach();                // For analog servo's that might drift somewhat it helps to quit the PWM signal. Makes it stay quite, not drift, and saves power and reduces heat.
  stalled = 0;                                  // reset stalled flag for new movements
}

void AcEservo::_stallSense(int lastDirection, int enableStallStop ){
  //noInterrupts();
  int current1 = analogRead(_analogSensePin);         // read the input pin 1st time for an average current, smoothing out possible spike fluctuations.
  //delay(5);
  int current2 = analogRead(_analogSensePin);         // read the input pin 2nd time for an average current, smoothing out possible spike fluctuations.
  //delay(5);
  int current3 = analogRead(_analogSensePin);         // read the input pin 3rd time for an average current, smoothing out possible spike fluctuations.
  //interrupts();
  int tolleranceCurrentProfileAdjust;           // will hold the calculated tollerence result that will be added to the difference. 
  int current = (current1 + current2 + current3) / 3; // calculate average current, smoothing out possible spike fluctuations.                     
  if (_lastCurrent == 0){
    _lastCurrent = current;
    Serial.print(current);                  // debug, helps with fine adjusting stall dector
    Serial.println(F(" = first current."));                     // debug, helps with fine adjusting stall dector
  } else {
    
    int difference = _lastCurrent - current;           // calculate difference between new and last current (is it going up? should result in negative numbers).
    tolleranceCurrentProfileAdjust = map(current, _nominalCurrent, _maxCurrent, _stepTolleranceLowC, _stepTolleranceHighC ); // puts measured current in a scale of nominal and max current, and maps it to the same place on the scale of low and high current tollerence.
    int differenceAdjust = difference + tolleranceCurrentProfileAdjust; // add suitable tollerence for this current level to the difference. Tollerence might also become negative at higher current levels.
    
    if (current > maxCurrentDetected){
      maxCurrentDetected = current;  
     }
         
    _lastCurrent = current;
    if (enableStallStop){                         // Actually stopping the servo when a stall is detected is dissabled a few times at startup for learning the max current draw.
      if (differenceAdjust < 0){                  // When calculated difference results in negative value, the servo seems stalled.
        this->write(_centerSignal);                        // Stop the rotating Servo.
        stalled = 1;                              // Raise the stalled flag for all functions.
        //stallCount++;                           // optional stall counter +1 stall detected.
        if ( lastDirection < _centerSignal ){                // Determin if servo is stallen in "open" position.
          stallFlag = 1;                         // Set stallFlag to "open" value for all other functions.
          Serial.print(F("Setting stallFlag to 1 with last turn direction : "));Serial.println(lastDirection);
        } else if ( lastDirection > _centerSignal ){         // Determin if servo is stallen in "closed" position.
          stallFlag = -1;                          // Set stallFlag to "closed" value for all other functions.
          Serial.print(F("Setting stallFlag to -1 with last turn direction : "));Serial.println(lastDirection);
        }
      }  
    }
    //delay (0); //Delay to compensate for dissableling the Serial.print debugging time in Servo steps (else the ON time for servo is to small) 
    // 0ms with debug, ~5ms? without
    // Plotting : 
        Serial.print(current);                  // debug, helps with fine adjusting stall dector
        Serial.print(F(" "));                     // debug, helps with fine adjusting stall dector
      Serial.print(difference);              // debug value
      Serial.print(F(" "));                     // debug value
  
      Serial.print(maxCurrentDetected);                 // debug value
      Serial.print(F(" "));                     // debug value
  //    Serial.print(tollerance);                 // debug value
  //    Serial.print(F(" "));                     // debug value
  //    Serial.print(differenceAdjust1);          // debug value
  //    Serial.print(F(" "));                     // debug value
  //      Serial.print(0);                          // debug value
  //      Serial.print(F(" "));                     // debug value   
  //      Serial.print(stallCount);                 // debug value
  //      Serial.print(F(" "));                     // debug value
  //    Serial.print(tolleranceCurrentProfileAdjust);  // debug value
  //    Serial.print(F(" "));                     // debug value
        Serial.println(differenceAdjust/2);       // debug, helps with fine adjusting stall dector
  }
}  