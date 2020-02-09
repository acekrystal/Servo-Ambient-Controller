/*
    SmartServo.h - Labrary for current sensing cheap continues servo's like micro 90's.
    Created by AcE Krystal (Wouter Horselenberg), 17-4-2019
   
        Features and versions : 
 - v0.1 Stall detection based on current draw.
 - v0.2 Parameters all collected at top for easy adjusting.
 - v0.3 Automaticly learns servo provile at statup and adjusts stall detection parameters based on learned servo (and job) profile.
 - v0.3 Movement actions now return avg. and max. load, clould be usefull for detecting if it dit some "work" in robotic arms?.
 - v0.3 Stall detection is much faster, smaller steps are possible.
 - v0.4 Minimal step size learning at startup. It now automaticly sets the smallest detectable step size.
 - v0.5 Finetune to support Analog continue's servo's.
 - v0.6 Converted to Library "AcEservo.h and .cpp"
   
    Creative commons share alike
*/
#ifndef AcEservo_h
#define AcEservo_h

#include "Arduino.h"
#include <Servo.h>

class AcEservo: public Servo {
    public:
        //AcEservo(){};   //default constructor must be followed by call to begin function
        AcEservo(//int servoPin, int analogSensePin,int centerSignal, int downSignal, int upSignal, int startupCurrent,int nominalCurrent, int maxCurrent, int minStepSize);
            //Servo Define
            byte servoPin=-1,
            byte analogSensePin=-1,
            //Servo Settings
            byte centerSignal=90,
            byte downSignal=0,
            byte upSignal=180,
            int stepTolleranceLowC=30,
            int stepTolleranceHighC=-25,
            //Servo Profile (Load or Default)
            int startupCurrent=15,
            int nominalCurrent=130,
            int maxCurrent=500);

        
        //Servo Status (Return)
        int stepTolleranceLowC;
        int stepTolleranceHighC;
        int startupCurrent;
        int nominalCurrent;
        int maxCurrent;

        int minStepSize;
        int stallFlag;
        int actionNominalLoad;
        int actionMaxLoad;
        int servoActionLoad;
        int maxCurrentDetected;
        int stalled;
        //Servo Public Functions
        void learnServoProfile();
        void moveServo(int multiplier, int directionSpeed, int enableStallStop);
    
    private:
        //Servo Define
        int _servoPin;
        int _analogSensePin;
        //Servo Settins
        int _centerSignal;
        int _downSignal;
        int _upSignal;
        //int _stepTolleranceLowC;
        //int _stepTolleranceHighC;
        //Servo Profile
        int _startupCurrent;
        //int _nominalCurrent;
        //int _maxCurrent;
        //int _minStepSize;
        //Servo Status
        //int _stallFlag;
        int _actionNominalLoad;
        int _actionMaxLoad;
        //Internal variables
        int _lastCurrent;
        void _stallSense(int lastDirection, int enableStallStop);
};

#endif