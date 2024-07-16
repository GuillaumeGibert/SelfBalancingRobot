/**
 * @file    selfbalancing.ino
 * @author  Guillaume Gibert
 * @version V0.1
 * @date    2023/03/31
 * @brief   Description: this code drives the mbot robot to perform self balancing.
 *
 * Function List:
 * 1. void control::move(int, int)
 * 2. void control::stop(void) 
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "MeMCore.h"

// robot characteristics
double _wheelDiameter = 6.0; //cm
double _distanceBetweenWheels = 11.5; //cm
int _basicMotorSpeed = 100;

// Vars to change setup
double _Kp_angle = -0.018;
int _targetAccY = -200;

// declare motors
MeDCMotor _leftMotor(9);
MeDCMotor _rightMotor(10);

// PID
double _fps = 100.0;
double _maxWheelSpeed = 250;
double _pidKp = -0.018;
double _pidKd = 0.0;
double _pidKi = 0.0;
double _pidIntegral = 0.0;
double _pidPreviousError = 0.0;

// declare gyro
MeGyro _gyro;

void setup()
{
  // initialize the serial port
  Serial.begin(115200);
  
  // set no speed for left and right motors 
  _leftMotor.run((9)==M1?-(0):(0));
  _rightMotor.run((10)==M1?-(0):(0));
  
  // initialize the gyro
  _gyro.begin();
  
  // wait for 2 seconds
  delay(2000);
  
  // retrieve new values from gyro
  _gyro.update();
}

void loop()
{
  // initialzes the timer
  unsigned long startTime = micros();
  
  // retrieve new values from gyro
  _gyro.update();
  
  /*
  Serial.print("accX:");
  Serial.print(_gyro.getAccX());
  Serial.print(",");
  Serial.print("accY:");
  Serial.print(_gyro.getAccY());
  Serial.print(",");
  Serial.print("accZ:");
  Serial.println(_gyro.getAccZ());
  */

  int errorAngle = _targetAccY - _gyro.getAccY(); // there should be no acceleration along z, only along y

  double wheelSpeed = computePID(_targetAccY, _gyro.getAccY(), _maxWheelSpeed, -_maxWheelSpeed, _pidKp, _pidKd, _pidKi, 1.0/_fps);
  
  double leftWheelSpeed = wheelSpeed;
  double rightWheelSpeed  = leftWheelSpeed;

/*
  Serial.print("_gyro.getAccY():");
  Serial.print(_gyro.getAccY());
  Serial.print(",");
  Serial.print("rightWheelSpeed:");
  Serial.println(rightWheelSpeed);
*/
  
  // crop wheel speed
  if (leftWheelSpeed > 250)
    leftWheelSpeed = 250;
  if (rightWheelSpeed > 250)
    rightWheelSpeed = 250;
  if (leftWheelSpeed < -250)
    leftWheelSpeed = -250;
  if (rightWheelSpeed < -250)
    rightWheelSpeed = -250;

  // send the commands
  move(rightWheelSpeed, leftWheelSpeed);

  // estimates the loop duration and waits before starting a new loop
  unsigned long currentTime = micros(); // in microseconds
  unsigned long elapsedTime = currentTime - startTime; // in microseconds

  unsigned long waitingTime = 1000000/_fps - elapsedTime; // in microseconds

  if (waitingTime < 16384) //(see https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/)
     delayMicroseconds(waitingTime); // in microseconds
  else
     delay(waitingTime / 1000); // in milliseconds
}

void move(int leftMotorSpeed, int rightMotorSpeed)
{
  _leftMotor.run((9)==M1?-(leftMotorSpeed):(leftMotorSpeed));
  _rightMotor.run((10)==M1?-(rightMotorSpeed):(rightMotorSpeed));
}

// compute the PID command given an ordered value and the current one
double computePID(double targetValue, double currentValue, double pidMax, double pidMin, double pidKp, double pidKd, double pidKi, double pidDt)
{
  // Calculate error
  double error = targetValue - currentValue;
  
  // Proportional term
  double Pout = pidKp * error;
  
  // Integral term
  _pidIntegral += error * pidDt;
  double Iout = pidKi * _pidIntegral;
  
  // Derivative term
  if (pidDt == 0)
  {
    return 0.0;
  }
  
  double derivative = (error - _pidPreviousError) / pidDt;
  double Dout = pidKd * derivative;
  
  // Calculate total output
  double output = Pout + Iout + Dout;
  
  // Restrict to max/min
  if( output > pidMax )
    output = pidMax;
  else if( output < pidMin )
    output = pidMin;
  
  // Save error to previous error
  _pidPreviousError = error;
  
  return output;
}
