#include "Vector.h"
#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4IMU.h>
#include <string.h>
#include <Timer.h>

const uint16_t threshold = 375;
uint16_t counterL = 0;
uint16_t counterR = 0;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proximitySensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4Encoders encoders;


int turnsArray[100];
uint32_t turnTimeArray[100];
Vector<int> turnsTaken;
Vector<uint32_t> turnsTakenTime;

Timer timer(MILLIS);
uint32_t prevTime;

#define NUM_SENSORS 3

unsigned int lineSensorValues[NUM_SENSORS];
unsigned int prevLineSensorValues[NUM_SENSORS];

bool finished = false;
bool simplify = false;
uint8_t targetHouses = 1;  // default number of ghouses we are looking for.
uint8_t foundHouses = 0;

const int DESIRED_LEFT_SPEED = 100;  // change this back to an even number
const int DESIRED_RIGHT_SPEED = 100;
int leftEncoderCount = 0;
int rightEncoderCount = 0;

uint16_t leftSpeed = 0;
uint16_t rightSpeed = 0;

// Start of turn sensor test - taken from Zumo 32U4 example "MazeSolver.ino"

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
//End of turn sensor test

void setup() {
  Serial.begin(9600);

  turnsTaken.setStorage(turnsArray, 0);
  turnsTakenTime.setStorage(turnTimeArray, 0);

  lineSensors.initThreeSensors();
  proximitySensors.initThreeSensors();
  encoders.init();
  calibrateMotorSpeeds();

  buzzer.playFrequency(500, 150, 12);

  while (!buttonB.getSingleDebouncedRelease()) {
    if (buttonA.getSingleDebouncedRelease()) {
      targetHouses++;
      if (targetHouses > 2) {
        targetHouses = 1;
      }
      for (uint8_t i = 0; i < targetHouses; i++) {
        delay(200);
        buzzer.playFrequency(600, 100, 10);
      }
    }
    if (buttonC.getSingleDebouncedRelease()) {
      if (simplifiedPath.size() > 0) {
        returnToFinish();
      } else {
        simplify = true;
      }
    }
  }

  buzzer.playFrequency(700, 200, 15);

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
    foundHouses++;
    houseFound(prevTime);
    timer.stop();
    timer.start();
  }
  if (finished) {
    if (simplify) {
      motors.setSpeeds(0, 0);
      simplifyPath();
      returnToStart();
    } else {
      motors.setSpeeds(0, 0);
      returnToStart();
      return;
    }
  }

  if (lineSensorValues[0] > threshold && lineSensorValues[1] > threshold && lineSensorValues[2] < threshold
      || (lineSensorValues[2] > threshold && lineSensorValues[1] > threshold && lineSensorValues[0] > threshold)) {
    counterL = 0;
    counterR++;
    if ((counterR % 3) == 0) {
      counterR = 0;
      turnsTakenTime.push_back(prevTime);
      turnsTaken.push_back(1);
      reverse();
      turnLeft();
      timer.stop();
      timer.start();
    } else {
      turnsTakenTime.push_back(prevTime);
      turnsTaken.push_back(0);
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
      turnsTakenTime.push_back(prevTime);
      turnsTaken.push_back(0);
      reverse();
      turnRight();
      timer.stop();
      timer.start();
    } else {
      turnsTakenTime.push_back(prevTime);
      turnsTaken.push_back(1);
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
      turnsTakenTime.push_back(prevTime);
      turnsTaken.push_back(2);
      shiftLeft();
      timer.stop();
      timer.start();
    }
  }

  if (lineSensorValues[0] > threshold && lineSensorValues[1] <= threshold) {
    delay(100);
    lineSensors.read(lineSensorValues);
    if (lineSensorValues[1] < threshold) {
      turnsTakenTime.push_back(prevTime);
      turnsTaken.push_back(3);
      shiftRight();
      timer.stop();
      timer.start();
    }
  }

  motors.setSpeeds(leftSpeed, rightSpeed);
}

void reverse() {
  motors.setSpeeds(-leftSpeed, -rightSpeed);
  delay(300);
}

void driveForwards() {
  motors.setSpeeds(leftSpeed, rightSpeed);
  delay(300);
}

void turnLeft() {
  turnSensorReset();
  motors.setSpeeds(0, 0);
  delay(300);
  motors.setSpeeds(-leftSpeed, rightSpeed);
  while ((int32_t)turnAngle < turnAngle45 * 2) {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
}

void turnRight() {
  turnSensorReset();
  motors.setSpeeds(0, 0);
  delay(300);
  motors.setSpeeds(leftSpeed, -rightSpeed);
  while ((int32_t)turnAngle > -turnAngle45 * 2) {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
}
void turnAround() {
  reverse();
  turnLeft();
  turnLeft();
}

void shiftLeft() {
  motors.setSpeeds(-leftSpeed, rightSpeed);
  delay(100);  // how long to bear
}

void shiftRight() {
  motors.setSpeeds(leftSpeed, -rightSpeed);
  delay(100);  // how long to bear
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
  motors.setSpeeds(-leftSpeed, rightSpeed);
  while ((int32_t)turnAngle < turnAngle45 * 2) {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn to the right 90 degrees.
  motors.setSpeeds(leftSpeed, -rightSpeed);
  while ((int32_t)turnAngle > -turnAngle45 * 2) {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn back to center using the gyro.
  motors.setSpeeds(-leftSpeed, rightSpeed);
  while ((int32_t)turnAngle < 0) {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Stop the motors.
  motors.setSpeeds(0, 0);
}

//SimplifyPath code taken and modified from Zumo32U4 example "MazeSolver.ino"
void simplifyPath() {
  // Only simplify the path if it is at least three instructions
  // long and the second-to-last turn was a 'B'.
  if (turnsTaken.size() < 3 || turnsTaken[turnsTaken.size() - 2] != 4) {
    return;
  }

  int16_t totalAngle = 0;

  for (uint8_t i = 1; i <= 3; i++) {
    switch (turnsTaken[turnsTaken.size() - i]) {
      case 0:
        totalAngle += 90;
        break;
      case 1:
        totalAngle -= 90;
        break;
      case 4:
        totalAngle += 180;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch (totalAngle) {
    case 0:
      turnsTaken[turnsTaken.size() - 3] = 5;
      break;
    case 90:
      turnsTaken[turnsTaken.size() - 3] = 0;
      break;
    case 180:
      turnsTaken[turnsTaken.size() - 3] = 4;
      break;
    case 270:
      turnsTaken[turnsTaken.size() - 3] = 1;
      break;
  }

  // The path is now two steps shorter.
  turnsTaken.pop_back();
  turnsTaken.pop_back();
}


void returnToStart() {
  while (turnsTaken.size() != 0) {
    switch (turnsTaken[turnsTaken.size() - 1]) {
      case 0:  // left turn
        turnLeft();
        driveForwards();
        break;
      case 1:  // right turn
        turnRight();
        driveForwards();
        break;
      case 2:
        shiftRight();
        motors.setSpeeds(0, 0);
        break;
      case 3:
        shiftLeft();
        motors.setSpeeds(0, 0);
        break;
      case 4:
        turnAround();
        motors.setSpeeds(0, 0);
      default:
        motors.setSpeeds(0, 0);
        break;
    }
    motors.setSpeeds(-leftSpeed, -rightSpeed);
    delay(getReturnDelayTime());
    motors.setSpeeds(0, 0);

    turnsTakenTime.pop_back();
    turnsTaken.pop_back();
  }
}

uint32_t getReturnDelayTime() {
  return turnsTakenTime[turnsTakenTime.size() - 1];
}

void calibrateMotorSpeeds() {
  Timer tempTimer(MILLIS);
  int leftSpeedError = 0;
  int rightSpeedError = 0;

  leftSpeed = DESIRED_LEFT_SPEED;
  rightSpeed = DESIRED_RIGHT_SPEED;
}

void houseFound(uint32_t prevTime) {
  if (foundHouses == targetHouses) {
    finished = true;
    turnsTakenTime.push_back(prevTime);
    turnsTaken.push_back(5);
    return;
  } else {
    turnAround();
    turnsTakenTime.push_back(prevTime);
    turnsTaken.push_back(4);
  }
}