#include "Vector.h"
#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4IMU.h>
#include <string.h>
#include <Timer.h>

const uint16_t defaultSpeedPos = 75;
const uint16_t defaultSpeedNeg = -75;

const uint16_t threshold = 375;
uint16_t counterL = 0;
uint16_t counterR = 0;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proximitySensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4Encoders encoders;


int storageArray[100];
uint32_t timeStorageArray[100];
Vector<int> turnMemory;
Vector<uint32_t> turnMemoryTime;

Timer timer(MILLIS);
uint32_t prevTime;

#define NUM_SENSORS 3

unsigned int lineSensorValues[NUM_SENSORS];
unsigned int prevLineSensorValues[NUM_SENSORS];

bool finished = false;
uint8_t targetHouses = 1;  // default number of ghouses we are looking for.
uint8_t detectedHouses = 0;

const int DESIRED_LEFT_SPEED = 102;  // change this back to an even number
const int DESIRED_RIGHT_SPEED = 100;
int leftEncoderCount = 0;
int rightEncoderCount = 0;

uint16_t leftPWMMotorSpeed = 0;
uint16_t rightPWMMotorSpeed = 0;

//==========================================================================
// TURN SENSOR TEST!!!!!!!!!

// Turnsensor.h provides functions for configuring the
// Zumo 32U4's gyro, calibrating it, and using it to
// measure how much the robot has turned about its Z axis.
//
// This file should be included once in your sketch,
// somewhere after you define objects named buttonA,
// display, and imu.

// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;
uint32_t targetTurnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;


// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate() {
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

/* This should be called in setup() to enable and calibrate the
gyro.  It uses the display, yellow LED, and button A.  While the
display shows "Gyro cal", you should be careful to hold the robot
still.

The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  display.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease()) {
    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));
  }
  display.clear();
  targetTurnAngle = turnAngle;
}


//======================================================================
//TEST OVER

void setup() {
  Serial.begin(9600);

  turnMemory.setStorage(storageArray, 0);
  turnMemoryTime.setStorage(timeStorageArray, 0);

  lineSensors.initThreeSensors();
  proximitySensors.initThreeSensors();
  encoders.init();
  calibrateMotorSpeeds();

  //Song to ensure that the zumo has been flashed and is ready
  buzzer.playFrequency(440, 100, 7);

  while (!buttonB.getSingleDebouncedRelease()) {
    if (buttonA.getSingleDebouncedRelease()) {
      targetHouses++;
      if (targetHouses > 2) {
        targetHouses = 1;
      }

      for (uint8_t i = 0; i < targetHouses; i++) {
        delay(200);
        buzzer.playFrequency(440, 100, 10);
      }
    }
  }

  buzzer.playFrequency(440, 100, 7);

  buttonA.waitForButton();
  turnSensorSetup();
  calibrateSensors();

  timer.start();
}

void loop() {

  turnSensorUpdate();
  proximitySensors.read();
  int16_t proxReadingL = proximitySensors.countsFrontWithLeftLeds();
  int16_t proxReadingR = proximitySensors.countsFrontWithRightLeds();

  lineSensors.read(lineSensorValues);
  prevTime = timer.read();

  if (proxReadingL >= 6 && proxReadingR >= 6) {
    detectedHouses++;
    encounteredAHouse(prevTime);
    timer.stop();
    timer.start();
  }
  if (finished) {
    motors.setSpeeds(0, 0);
    revertFromMemory();
    return 0;
  }

  if (lineSensorValues[0] > threshold && lineSensorValues[1] > threshold && lineSensorValues[2] < threshold
      || (lineSensorValues[2] > threshold && lineSensorValues[1] > threshold && lineSensorValues[0] > threshold)) {
    counterL = 0;
    counterR++;
    if ((counterR % 3) == 0) {
      counterR = 0;
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(1);
      reverse();
      turnLeft();
      timer.stop();
      timer.start();
    } else {
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(0);
      reverse();
      turnRight();
      timer.stop();
      timer.start();
    }
  } else if ((lineSensorValues[2] > threshold && lineSensorValues[1] > threshold && lineSensorValues[0] < threshold)) {
    counterR = 0;
    counterL++;
    if ((counterL % 3) == 0) {
      counterL = 0;
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(0);
      reverse();
      turnRight();
      timer.stop();
      timer.start();
    } else {
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(1);
      reverse();
      turnLeft();
      timer.stop();
      timer.start();
    }
  }

  if (lineSensorValues[2] > threshold && lineSensorValues[1] <= threshold) {
    delay(100);
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[1] < threshold) {
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(2);
      bearLeft();
      timer.stop();
      timer.start();
    }
  }

  if (lineSensorValues[0] > threshold && lineSensorValues[1] <= threshold) {
    delay(100);
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[1] < threshold) {
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(3);
      bearRight();
      timer.stop();
      timer.start();
    }
  }

  motors.setSpeeds(leftPWMMotorSpeed, rightPWMMotorSpeed);
}

void reverse() {
  motors.setSpeeds(-leftPWMMotorSpeed, -rightPWMMotorSpeed);
  delay(300);
}

void shiftForwards() {
  motors.setSpeeds(leftPWMMotorSpeed, rightPWMMotorSpeed);
  delay(300);
}

void turn180() {
  reverse();
  turnLeft();
  turnLeft();
}

void bearLeft() {
  motors.setSpeeds(-leftPWMMotorSpeed, rightPWMMotorSpeed);
  delay(100);  // how long to bear
}

void bearRight() {
  motors.setSpeeds(leftPWMMotorSpeed, -rightPWMMotorSpeed);
  delay(100);  // how long to bear
}

void turnLeft() {
  turnSensorReset();
  motors.setSpeeds(0, 0);
  delay(300);
  motors.setSpeeds(-leftPWMMotorSpeed, rightPWMMotorSpeed);
  while ((int32_t)turnAngle < turnAngle45 * 2) {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
}

void turnRight() {
  turnSensorReset();
  motors.setSpeeds(0, 0);
  delay(300);
  motors.setSpeeds(leftPWMMotorSpeed, -rightPWMMotorSpeed);
  while ((int32_t)turnAngle > -turnAngle45 * 2) {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
}

void resetCounters() {
  counterL = 0;
  counterR = 0;
}

void calibrateSensors() {

  // Delay so the robot does not move while the user is still
  // touching the button.
  delay(1000);

  // We use the gyro to turn so that we don't turn more than
  // necessary, and so that if there are issues with the gyro
  // then you will know before actually starting the robot.

  turnSensorReset();

  // Turn to the left 90 degrees.
  motors.setSpeeds(-leftPWMMotorSpeed, rightPWMMotorSpeed);
  while ((int32_t)turnAngle < turnAngle45 * 2) {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn to the right 90 degrees.
  motors.setSpeeds(leftPWMMotorSpeed, -rightPWMMotorSpeed);
  while ((int32_t)turnAngle > -turnAngle45 * 2) {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn back to center using the gyro.
  motors.setSpeeds(-leftPWMMotorSpeed, rightPWMMotorSpeed);
  while ((int32_t)turnAngle < 0) {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Stop the motors.
  motors.setSpeeds(0, 0);
}

void revertFromMemory() {
  while (turnMemory.size() != 0) {
    switch (turnMemory[turnMemory.size() - 1]) {
      case 0:  // left turn
        turnLeft();
        shiftForwards();
        break;
      case 1:  // right turn
        turnRight();
        shiftForwards();
        break;
      case 2:
        bearRight();
        motors.setSpeeds(0, 0);
        break;
      case 3:
        bearLeft();
        motors.setSpeeds(0, 0);
        break;
      case 4:
        turn180();
        motors.setSpeeds(0, 0);
      default:
        motors.setSpeeds(0, 0);
        break;
    }
    motors.setSpeeds(-leftPWMMotorSpeed, -rightPWMMotorSpeed);
    delay(getRevertedDelayTime());
    motors.setSpeeds(0, 0);

    turnMemoryTime.pop_back();
    turnMemory.pop_back();
  }
}

uint32_t getRevertedDelayTime() {
  return turnMemoryTime[turnMemoryTime.size() - 1];
}

void calibrateMotorSpeeds() {
  Timer tempTimer(MILLIS);
  int leftSpeedError = 0;
  int rightSpeedError = 0;

  leftPWMMotorSpeed = DESIRED_LEFT_SPEED;
  rightPWMMotorSpeed = DESIRED_RIGHT_SPEED;
}

void encounteredAHouse(uint32_t prevTime) {
  if (detectedHouses == targetHouses) {
    finished = true;
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(5);
    return;
  } else {
    turn180();
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(4);
  }
}