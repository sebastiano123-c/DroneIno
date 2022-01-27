#ifndef AUTOLEVELING_CLASS
  #define AUTOLEVELING_CLASS
  #include <Arduino.h>
  #include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
  #include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
  #include "../Config.h"
  #include "Models.h"

  class AutoLeveling {

    public:
      byte eepromData[36];
      boolean gyroAnglesSet;
      boolean autoLevel;                
      int calInt, start;
      int gyroAddress;
      int receiverInput[5];
      int batteryVoltage;
      int temperature;
      int esc1, esc2, esc3, esc4;
      volatile int receiverInputChannel1;
      volatile int receiverInputChannel2;
      volatile int receiverInputChannel3;
      volatile int receiverInputChannel4;
      float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
      float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
      float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
      float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
      float pidErrorTemp;
      float rollLevelAdjust, pitchLevelAdjust;
      long accX, accY, accZ, accTotalVector;
      unsigned long loopTimer;
      unsigned long timerChannel1, timerChannel2, timerChannel3, timerChannel4, escTimer, escLoopTimer;
      int16_t accAxis[4], gyroAxis[4];   
      int16_t gyroAxisCalibration[4];   
      double gyroPitch, gyroRoll, gyroYaw;

      void begin();          
      void readGyroscopeStatus();
      void setGyroscopeRegisters();
      void getBatteryVoltage();
      void getLoopTimer();
      void calibrateGyroscope();
      int convertReceiverChannel(byte function);
      void waitController();
      void setAutoLevelParameters();
      void droneStart();
      void setPID();
      void calculatePID();
      void batteryVoltageCompensation();
      void setEscPulses();

      AutoLeveling(boolean autoLeveling){
        boolean autoLevel = autoLeveling;
      }
  };
#endif