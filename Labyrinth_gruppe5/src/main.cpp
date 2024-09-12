#include <HCSR04.h>
#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

// Variable for Pololu Robot
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
BumpSensors bumpSensors;
Encoders encoders;
uint16_t maxSpeed = 200;  
uint16_t minSpeed = 0;
uint16_t baseSpeed = 200; 

// Variable for Ultrasonic sensor
UltraSonicDistanceSensor distanceSensor_RIGHT(2, 3);  
UltraSonicDistanceSensor distanceSensor_LEFT(1, 0);
unsigned int distance_RIGHT;
unsigned int distance_LEFT;
unsigned distance;
const int SENSOR_MAX_RANGE = 400;

// Wall following PID
const double desiredPosition = (double)4;
const double kp = 4;
const double ki = 0;
const double kd = 150;  
double kiTotal = 0.0;
double previousError = 0.0;
bool leftActive = true;
int16_t leftSpeed;
int16_t rightSpeed;

// Timer (optional)
unsigned long myTime;
unsigned long startTime;
unsigned long driveTime = 15000;

bool foundLeft = false;
int32_t encodersSteps;

bool left_Mode = true;

void setup() {
  // We initialize serial connection so that we could print values from sensor.
  Serial.begin(9600);  

  // Initalize the bump sensor 
  bumpSensors.calibrate();

  // Select mode : Left or Right 
  while (true) {
    if (buttonA.getSingleDebouncedPress()) {
      left_Mode = true;
      break;
    }

    if (buttonC.getSingleDebouncedPress()) {
      left_Mode = false;
      break;
    }
  }

  // drive the robot forward for a while until it enters the arena
  motors.setSpeeds(100,100);
  delay(70);
}

// function to test the ultrasound sensors 
void displayMeasurement(unsigned long distance, String sensorName) {
  // Print the sensor reading value
  if (distance > SENSOR_MAX_RANGE || distance <= 0) {
    Serial.println("Sensor " + sensorName + " : Out of sensor range!");
  } else {
    Serial.println("Distance from Sensor " + sensorName + " to object: " + String(distance) + " cm");
  }
}

// Turn right function
void turnRight() {
  motors.setSpeeds(100, -100);
  delay(210);
  motors.setSpeeds(0, 0);
}

// Turn left function
void turnLeft() {
  motors.setSpeeds(-100, 100);
  delay(210);
  motors.setSpeeds(0, 0);
}

// Bounce back funtion 
void bounceBack() {
  motors.setSpeeds(0, 0);
  delay(100);
  motors.setSpeeds(-100, -100);
  delay(100);
  motors.setSpeeds(0, 0);
}

// Move forward function
void moveForward() {
  motors.setSpeeds(50, 50);
}


void loop() {
  //myTime = millis();  // Optional for timer usage
  if (left_Mode) {
    distance = distanceSensor_LEFT.measureDistanceCm();
    Serial.println("Sensor left : " + String(distance) + " cm" );    
  } else {
    distance = distanceSensor_RIGHT.measureDistanceCm();
    Serial.println("Sensor Right : " + String(distance) + " cm" );
  }

  // To avoid small hole between wall segments
  if (distance >= 150) {
    distance = 5;
  }

  // Read the bump sensor. Check if the robot bumps the wall
  bumpSensors.read();
  if (bumpSensors.rightIsPressed() || bumpSensors.leftIsPressed()) {
    // drive back for ca. 5 cm
    motors.setSpeeds(-100, -100);
    delay(200);

    // turn right or left depending on the mode 
    if (left_Mode) {
      motors.setSpeeds(0, 0);
      delay(5);
      turnRight();
      motors.setSpeeds(100, 100);
      delay(100);
    } else {
      motors.setSpeeds(0, 0);
      delay(5);
      turnLeft();
      motors.setSpeeds(100, 100);
      delay(100);
    }

  } else {
    // If the there is no wall in front then start the wall-following algorithm
    double error = desiredPosition - distance;

    // If we lost of track of side wall then, ...
    if (error <= -20) {
      if (left_Mode) {
        motors.setSpeeds(80, 180);  // 90, 180
        delay(100);                 // 100
      } else {
        motors.setSpeeds(180, 80);  // 90, 180
        delay(100);                 // 100
      }

    } else {
      // PID-Control System
      double proportional = kp * error;
      kiTotal += error;
      double integral = ki * kiTotal;

      float derivative = kd * (error - previousError);
      previousError = error;

      float pidResult = proportional + integral + derivative;

      if (left_Mode) {
        leftSpeed = (int16_t)baseSpeed + pidResult;
        rightSpeed = (int16_t)baseSpeed - pidResult;
      } else {
        leftSpeed = (int16_t)baseSpeed - pidResult;
        rightSpeed = (int16_t)baseSpeed + pidResult;
      }

      leftSpeed = constrain(leftSpeed, minSpeed, (int16_t)maxSpeed);
      rightSpeed = constrain(rightSpeed, minSpeed, (int16_t)maxSpeed);

      // Stop the robot when we press button c 
      if (buttonC.getSingleDebouncedPress()) {
        while (true) {
          motors.setSpeeds(0, 0);
          if (buttonA.getSingleDebouncedPress()) {
            break;
          }
        }
      } else {
        motors.setSpeeds(leftSpeed, rightSpeed);
      }
    }
  }
}
