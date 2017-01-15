/*
   I2C communication as master
   one connection with BNO055
   one connection with robot (robot can be polled with timer or can request token with InputRobotRequestPin
*/
/* calibration result
 *  * Fully calibrated!
--------------------------------
Calibration Results: 
Accelerometer: 65508 65523 10 
Gyro: 65533 65535 0 
Mag: 182 65213 65263 
Accel Radius: 1000
Mag Radius: 460
 */
 
#define debugOn
//#define debugL2On
//#define debugLEDOn
#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
#include <BNO055SubsystemCommonDefine.h>
#include <BNO055Definitions.h>
#include <EEPROM.h>
//-- accelerometer and magnetometer
// powered by arduino 3.3v
#define sensor_id 0x01
#define systemLED 6
#define sysCalLED 5
#define magCalLED 4
#define UpdateBNOStatusDelay 500
#define pin1OpMode 8
#define pin2OpMode 7
uint8_t sysStatusFlag = 0x00;
uint8_t sysCalFlag = 0x00;
uint8_t magCalFlag = 0x00;
unsigned long refAccX;
unsigned long UpdateBNOStatus_time;
unsigned long UpdateLEDStatus_time;
float X;
float Y = 0;
int NOBeforeRotation = 0; // keep NO before rotation
int NOAfterRotation = 0;
int NOBeforeMoving = 0; // keep NO before moving straight
int NOAfterMoving = 0; // keep NO before moving straight
// compass calibration to be made

volatile boolean dataToRead = false;
uint8_t LOCAL_CTRL_REG[150];        // to store parameters


int x;
int y;
int z;
uint8_t statusBNO055;
float bias = 0;
unsigned int count = 0;
float translatedHeading = 0;
float relativeHeading = 0;
float startHeading = 0;
unsigned long avg = 0;
unsigned long savTime = 0;
uint8_t inputData[256];
uint8_t receivedCount = 0x00;
boolean flagOnRequest = false;
uint8_t dataIn[33];
uint8_t pollResponseExpectedLenght = 10;
boolean req = false;
unsigned long prevPollTimer;
unsigned long prevBNO055Timer;
unsigned long prevSentData;
unsigned long updateNOTimer;
boolean monitGyro = true;
boolean monitMagneto = true;
boolean statRobotRequest = 0;
uint8_t currentStatus = 0x00;
unsigned long startInterruptTime;
unsigned long interruptCount;
//Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_Address);
unsigned long countLoop = 0;
uint8_t calibData[22];
typedef struct {
  double x;
  double y;
  double z;
} vector;
typedef enum
{
  vector_ACCELEROMETER = 0x08,
  vector_MAGNETOMETER  = 0x0e,
  vector_GYROSCOPE     = 0x14,
  vector_EULER         = 0x1a,
  vector_LINEARACCEL   = 0x28,
  vector_GRAVITY       = 0x2e
} vector_type_t;

//Adafruit_BNO055::adafruit_bno055_reg_t regDef;
void setup() {
  // Wire.begin();
  Serial.begin(38400);
  pinMode(systemLED, OUTPUT);
  pinMode(sysCalLED, OUTPUT);
  pinMode(magCalLED, OUTPUT);
  pinMode(pin1OpMode, INPUT);
  pinMode(pin2OpMode, INPUT);
  digitalWrite(systemLED, 1);
  InitSubsystemParameters(true, 0, 0);
  //  LOCAL_CTRL_REG[selectedRange_Reg] = 0x01;         // default selected range
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  /* Initialise the sensor */

  if (!BNOInit())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
    LOCAL_CTRL_REG[BNO055Mode_Reg] = 0x00;
  }
  else
  {
    LOCAL_CTRL_REG[BNO055Mode_Reg] = BNOreadRegister(OPR_MODE_REG);
  }

  Serial.print("op mode:");
  Serial.println(LOCAL_CTRL_REG[BNO055Mode_Reg], HEX);
  delay(1000);
  EEPROM.get(eeAddress, bnoID);


  /*
     Look for the sensor's unique ID at the beginning oF EEPROM.
     This isn't foolproof, but it's better than nothing.
  */

  if (bnoID != sensor_id)
  {
    Serial.print("\nNo Calibration Data in EEPROM for this sensor ID:");
    Serial.println(bnoID);
    delay(500);
  }
  else
  {
    Serial.print("\nFound Calibration in EEPROM for this sensor ID:");
    Serial.println(bnoID);
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibData);
    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    BNORestoreCalibration(&calibData[0]);
    Serial.println("\n\nCalibration data loaded into BNO055");
    delay(200);
    BNOPrintCalibration();
    foundCalib = true;
  }
  UpdateBNOStatus();
  UpdateLEDStatus();
  delay(1000);
#if defined(debugOn)
  BNOdisplayCalStatus();
  BNOdisplaySensorStatus();
#endif
  if (digitalRead(pin1OpMode) == 1)
  {
    SetBNO055Mode(OP_MODE_COMPASS);
  }
  else {
    SetBNO055Mode(OP_MODE_NDOF);
  }
  //  SetBNO055Mode(OP_MODE_NDOF);
  LOCAL_CTRL_REG[BNO055Mode_Reg] = BNOreadRegister(OPR_MODE_REG);
  if (foundCalib  && digitalRead(pin1OpMode) == 0) {
    Serial.println("Move sensor slightly to calibrate magnetometers");
    unsigned int count = 0;
    while (BNOreadRegister(CALIB_STAT_REG) != 0xff && count < 60)
    {
#if defined(debugOn)
      BNOdisplayCalStatus();
      BNOdisplaySensorStatus();
#endif
      UpdateBNOStatus();
      UpdateLEDStatus();
      delay(1000);
      count++;
    }
    Serial.println("end calibration");
  }
  UpdateBNOStatus();
  UpdateLEDStatus();
  delay(500);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Optional: Display current status */
  // displaySensorStatus();
  //  WriteRegister(BNO055_Address, 0x10, 0b01000000);
  delay(20);
  Serial.println("BNO055 is started");
  pinMode(SensorOutputReadyPin, OUTPUT);
#if defined(BNO055PinInterrupt)
  pinMode(BNO055PinInterrupt, INPUT);
  attachInterrupt(BNO055PinInterrupt, Robot_BNO055DataReady, RISING);
#endif
  //    pinMode(MagnetoPowerPin, OUTPUT);

#if defined(SensorInputRobotRequestPin)
  pinMode(SensorInputRobotRequestPin, INPUT);
#endif
  Serial.println("ready");
  delay(1000);
  digitalWrite(SensorOutputReadyPin, HIGH);
  UpdateBNOStatus();
  UpdateLEDStatus();

  //  Serial.println("set IMU mode");
  //  SetBNO055Mode(OP_MODE_IMUPLUS);
}

void loop() {
  delayMicroseconds(1);

  if ((millis() - prevBNO055Timer) >= LOCAL_CTRL_REG[BNO055cycleDuration_Reg] && monitGyro == true ) //LOCAL_CTRL_REG[cycleDuration_Reg]
  {
    dataToRead = false;
    if (LOCAL_CTRL_REG[BNO055Mode_Reg] == OP_MODE_IMUPLUS)
    {
      double x, y, z;
      vector euler = {x, y, z};
      euler = GetVector(vector_EULER);
      //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      relativeHeading = euler.x;
      translatedHeading = relativeHeading + startHeading + permanentShiftHeading;
#if defined(debugOn)
      countLoop++;
      if (countLoop % 5000 == 0)
      {
        Serial.print("IMU:");
        Serial.println(translatedHeading);
      }
#endif
      if (translatedHeading >= 0)
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
      }
      else
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x01;
      }
      LOCAL_CTRL_REG[relativeHeading_Reg2] = uint8_t (abs(translatedHeading / 256));
      LOCAL_CTRL_REG[relativeHeading_Reg3] = uint8_t (abs(translatedHeading));
      //   prevtranslatedHeading=translatedHeading;
    }
    if (LOCAL_CTRL_REG[BNO055Mode_Reg] == OP_MODE_COMPASS)
    {
      double x, y, z;
      vector euler = {x, y, z};
      euler = GetVector(vector_EULER);
      //     imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      int compassHeading = euler.x;
      compassHeading = (compassHeading + permanentShiftHeading) % 360;
      LOCAL_CTRL_REG[compasHeading_Reg1] = uint8_t (compassHeading / 256);
      LOCAL_CTRL_REG[compasHeading_Reg2] = uint8_t (compassHeading);
#if defined(debugOn)
      countLoop++;
      if (countLoop % 5000 == 0)
      {
        Serial.print("compass:");
        Serial.println(compassHeading);
      }
#endif
    }
    if (LOCAL_CTRL_REG[BNO055Mode_Reg] == OP_MODE_NDOF)
    {
      double x, y, z;
      vector euler = {x, y, z};
      euler = GetVector(vector_EULER);
      //      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      int absoluteHeading = euler.x;
      translatedHeading = absoluteHeading + startHeading + permanentShiftHeading;
      absoluteHeading = (absoluteHeading + LOCAL_CTRL_REG[permanentNOShift_Reg1] * 256 + LOCAL_CTRL_REG[permanentNOShift_Reg2]) % 360;
      if (absoluteHeading >= 0)
      {
        LOCAL_CTRL_REG[absoluteHeading_Reg1] = 0x00;
      }
      else
      {
        LOCAL_CTRL_REG[absoluteHeading_Reg1] = 0x01;
      }
      LOCAL_CTRL_REG[absoluteHeading_Reg2] = uint8_t (abs(absoluteHeading / 256));
      LOCAL_CTRL_REG[absoluteHeading_Reg3] = uint8_t (abs(absoluteHeading));
      if (translatedHeading >= 0)
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
      }
      else
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x01;
      }
      LOCAL_CTRL_REG[relativeHeading_Reg2] = uint8_t (abs(translatedHeading / 256));
      LOCAL_CTRL_REG[relativeHeading_Reg3] = uint8_t (abs(translatedHeading));
      //   prevtranslatedHeading=translatedHeading;
#if defined(debugOn)
      countLoop++;
      if (countLoop % 5000 == 0)
      {
        Serial.print("NDOF absolute heading:");
        Serial.print(absoluteHeading);
        Serial.print(" relative heading:");
        Serial.println(translatedHeading);
      }
#endif
    }
#if defined(debugOn)
    if (countLoop % 20000 == 0)
    {
      BNOdisplayCalStatus();
      //            getCalibrationData();
      BNOdisplaySensorStatus();
    }
#endif
    //   imu::Vector<3> absoluteOrientation = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    //Serial.println(absoluteOrientation.y());
  }

  if ( (digitalRead(SensorInputRobotRequestPin) == HIGH) )
    /*
       The robot ask for the word
    */
  {
    int dataLen = Robot_PollSlave(LOCAL_CTRL_REG[robotAddress_Reg], currentStatus);  // request data from robot
    uint8_t receiveAddress = dataIn[0];
    if (receiveAddress == LOCAL_CTRL_REG[robotAddress_Reg])                     // check data is comming from robot
    {
      uint8_t cmd = dataIn[1];                                                  // this byte is a command switch
      uint8_t numberRegs = dataIn[2];
      uint8_t parameter = dataIn[2];
      switch (cmd)
      {
        case idleRequest:
          break;
        case setRegisterRequest:                                    // request is set registers values
          switch (numberRegs)                                       // number of consecutive registers to set
          {
            case 0xff:
              InitSubsystemParameters(true, 0, 0);                                        // reset default value
              break;
            default:
              if (numberRegs <= maxRegsNumberUpdate)
              {
                for (int i = 0; i < min(numberRegs, maxRegsNumberUpdate); i++)
                {
                  InitSubsystemParameters(false, dataIn[2 * i + 3], dataIn[2 * i + 4]);
                }
              }
              break;
          }
          break;
        case readRegisterRequest:
          {
            if (numberRegs <= maxRegsNumberRead)
            {
              Robot_SendRegistersValue(receiveAddress, numberRegs, dataIn);
            }
            break;
          }

        case startInitMonitorGyro:
          {
            double x, y, z;
            vector euler = {x, y, z};
            euler = GetVector(vector_EULER);
            //           imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            startHeading = -euler.x;
            SetBNO055Mode(IMUMode);
            monitGyro = true;
            LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg2] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg3] = 0x00;
            Serial.println("Start & Init monitor gyro");
            bitWrite(currentStatus, monitGyroStatusBit, 1);
            interruptCount = 0;
            break;
          }

        case startMonitorMagneto:
          {
            //           digitalWrite(MagnetoPowerPin, HIGH);
            SetBNO055Mode(compassMode);
            //          BNO055CalibrationStatus();
            monitMagneto = true;
            Serial.println("Start monitor magneto");
            bitWrite(currentStatus, monitMagnetoStatusBit, 1);
            break;
          }
        case setBNO055Mode:
          {
            //           digitalWrite(MagnetoPowerPin, HIGH);
            SetBNO055Mode(dataIn[2]);
            break;
          }

        default:
          {

          }
      }
    }
    req = !req;
  }

  if (millis() - UpdateBNOStatus_time >=  UpdateBNOStatusDelay)
  {
    UpdateBNOStatus();
    UpdateLEDStatus();
  }

}

void Robot_BNO055DataReady()
{
  //  detachInterrupt(BNO055InterruptNumber);
  Serial.print("i..");
  dataToRead = true;
  interruptCount++;
  // Serial.print(".");
}
void Robot_SendRegistersValue(uint8_t deviceAddress,  uint8_t numberRegs, uint8_t dataIn[33])
{
  Wire.beginTransmission(deviceAddress); // transmit to device #8
  Wire.write(deviceAddress);
  Wire.write(readRegisterResponse);
  Wire.write(numberRegs);
  for (int i = 0; i < numberRegs; i++)
  {
    Wire.write(dataIn[i + 3]);
    Wire.write(LOCAL_CTRL_REG[dataIn[i + 3]]);            // sends one byte
  }
  Wire.endTransmission();    // stop transmitting
}
int Robot_PollSlave(int deviceAddress, byte parameter) {

  int idx = 0;
  prevPollTimer = millis();
  //
  Wire.beginTransmission(deviceAddress);
  Wire.write(deviceAddress);
  Wire.write(idleRequest);
  Wire.write(parameter);
  Wire.write(pollResponseExpectedLenght);                   // Command Data = dummy zeroes
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, pollResponseExpectedLenght); // read a byte
  while (Wire.available()) {
    dataIn[idx] = Wire.read();
    idx++;
  }
#if defined(debugL2On)
  Serial.print("request from robot:" );
  Serial.println(deviceAddress, HEX);
  //  Serial.println(float(interruptCount * 1000 / (millis() - startInterruptTime)));
#endif
  return idx;
}
int UpdateNorthOrientation()
{
  int NO = 0;
#if defined(magnetoInstalled)
  compass.read();
  float northOrientation = compass.heading();
#else
  return 0;
#endif
#if defined(debugMagnetoOn)
  Serial.print(" magneto orientation: ");
  Serial.println(northOrientation);

  NO = int(northOrientation);
  LOCAL_CTRL_REG[headingNorthOrientation_Reg1] = uint8_t(NO / 256);
  LOCAL_CTRL_REG[headingNorthOrientation_Reg2] = uint8_t(NO);
#endif
  updateNOTimer = millis();
  return NO;

}
void InitSubsystemParameters(boolean allReg, uint8_t regNumber, uint8_t regValue)
{

  if (allReg || regNumber == BNO055cycleDuration_Reg)
  {
    Serial.print("BNO055 polling cycle:");
    if (allReg)
    {
      LOCAL_CTRL_REG[BNO055cycleDuration_Reg] = BNO055PollingCycle;         // default selected range
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[BNO055cycleDuration_Reg]);
  }
  //
  if (allReg || regNumber == robotAddress_Reg)
  {
    Serial.print("Robot addrsss:");
    if (allReg)
    {
      LOCAL_CTRL_REG[robotAddress_Reg] = robotI2CAddress;         // robot I2 address
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = int(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[robotAddress_Reg], HEX);
  }

  Serial.print("Permanent NO Shift:");
  LOCAL_CTRL_REG[permanentNOShift_Reg1] = permanentNOShift / 256;      // robot I2 address
  LOCAL_CTRL_REG[permanentNOShift_Reg2] = permanentNOShift;       // robot I2 address
  Serial.println(LOCAL_CTRL_REG[permanentNOShift_Reg1] * 256 + LOCAL_CTRL_REG[permanentNOShift_Reg2]);
}


void receiveEvent(int howMany) {
  //  Serial.println(howMany);
  while (Wire.available()) { // loop through all but the last
    inputData[receivedCount] = Wire.read(); // receive byte as a character
    //   Serial.print(inputData[receivedCount], HEX);        // print the character
    //   Serial.print("-");
    receivedCount++;
  }
  //  Serial.println();
}

void Robot_CalibrateGyro(uint8_t deviceAddress)
{

  Wire.beginTransmission(deviceAddress); // transmit to device #8
  Wire.write(deviceAddress);
  Wire.write(calibrateGyro);
  Wire.endTransmission();    // stop transmitting
}


void UpdateBNOStatus()
{
  LOCAL_CTRL_REG[BNO055SysStatus_Reg] = BNOreadRegister(SYS_STAT_REG);
  LOCAL_CTRL_REG[BNO055CalStatus_Reg] = BNOreadRegister(CALIB_STAT_REG);
  LOCAL_CTRL_REG[BNO055TestStatus_Reg] = BNOreadRegister(SELFTEST_RESULT_REG);
  LOCAL_CTRL_REG[BNO055SysError_Reg] = BNOreadRegister(SYS_ERR_REG);
  UpdateBNOStatus_time = millis();
}
void UpdateLEDStatus()
{
  // uint8_t sysStat = LOCAL_CTRL_REG[BNO055SysStatus_Reg];
  LOCAL_CTRL_REG[BNO055Mode_Reg] = BNOreadRegister(OPR_MODE_REG);
#if defined(debugLEDOn)
  Serial.print("LED Sys:");
  Serial.print(LOCAL_CTRL_REG[BNO055SysStatus_Reg], HEX);
  Serial.print(" Cal:");
  Serial.println(LOCAL_CTRL_REG[BNO055CalStatus_Reg], HEX);
#endif
  switch (LOCAL_CTRL_REG[BNO055SysStatus_Reg])
  {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
      digitalWrite(systemLED, 1);
      break;
    case 0x05:
      digitalWrite(systemLED, 0);
      break;
    case 0x06:
      digitalWrite(systemLED, !digitalRead(systemLED));
      break;
  }
  switch (LOCAL_CTRL_REG[BNO055CalStatus_Reg])
  {



    case 0x00:
      digitalWrite(sysCalLED, 0);
      break;
    case 0xff:
      digitalWrite(sysCalLED, 1);
      break;
    default:
      digitalWrite(sysCalLED, !digitalRead(sysCalLED));
      break;
  }
  switch (LOCAL_CTRL_REG[BNO055CalStatus_Reg] & 0x03)
  {

    case 0x00:
      digitalWrite(magCalLED, 0);
      break;
    case 0x03:
      digitalWrite(magCalLED, 1);
      break;
    case 0x01:
      if (magCalFlag % 3 == 0)
      {
        digitalWrite(magCalLED, !digitalRead(magCalLED));
      }
      magCalFlag++;
      break;
    case 0x02:
      if (magCalFlag % 2 == 0)
      {
        digitalWrite(magCalLED, !digitalRead(magCalLED));
      }
      magCalFlag++;
      break;
  }
  UpdateLEDStatus_time = millis();
}

