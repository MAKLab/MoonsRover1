//  Moon Rover1 code designed for Arduino Nano or similar
//  Pi can send serial commands in the form
//  "MAF255,MBF255,SP90,ST90," == Both motors forwards full speed, Both Servos center postion

#include <Wire.h>
#include <Servo.h>

Servo pan;  // create servo object to control a servo
Servo tilt; // same again

const bool debug = 1;

// twelve servo objects can be created on most boards
const int dir1PinA = 7;
const int dir2PinA = 8;
const int motorPwmPinA = 5;

const int dir1PinB = 11;
const int dir2PinB = 12;
const int motorPwmPinB = 6;

// Attach our motor pins to these pwm ports

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(115200); //Best to use 115200, its the native baud for Pi Serial
  Serial.println("Rover Activated");
  
  pan.attach(9);  // attaches the servo on pin 9 to the servo object
  tilt.attach(10);

  // Set PWM pins as outputs
  pinMode (motorPwmPinA, OUTPUT);  
  pinMode (motorPwmPinB, OUTPUT);

  // Set Motor Direction Pins as outputs
  pinMode(dir1PinA,OUTPUT);  // Motor A
  pinMode(dir2PinA,OUTPUT);

  pinMode(dir1PinB,OUTPUT);  // Motor B
  pinMode(dir2PinB,OUTPUT);
  
}

void loop() {

  if (Serial.available()){
    byte firstByte = Serial.read();
    
    // Check to see if it a motor
    if (firstByte == 'M'){ 

      //  Its a motor, lets nab some movement data
      byte motor = Serial.read();
      bool direction = Serial.read();
      int mSpeed = Serial.parseInt();

      // Lets turn that into movement

      // Is it motor A?
      if (motor == 'A'){

        Serial.print ("A");

        // Are we going forwards?
        if (direction == 'F'){
          digitalWrite(dir1PinA, LOW);
          digitalWrite(dir2PinA, HIGH); 
          Serial.print ("F");
        } 

        // perhaps reverse?
        else if (direction == 'R'){
          digitalWrite(dir1PinA, HIGH);
          digitalWrite(dir2PinA, LOW);
          Serial.print ("R");
        }
        analogWrite(motorPwmPinA, mSpeed);
        Serial.println (mSpeed, DEC);
      } 

      // Perhaps it's B then?
      else if (motor == 'B'){

        // Are we going forwards
        if (direction == 'F'){
          digitalWrite(dir1PinA, LOW);
          digitalWrite(dir2PinA, HIGH); 
          Serial.print("F");
        } 
        
        // perhaps reverse?
        else if (direction == 'R')  {
          digitalWrite(dir1PinB, HIGH);
          digitalWrite(dir2PinB, LOW);
          Serial.print("R");
        }
        analogWrite(motorPwmPinB, mSpeed);  
        Serial.println(mSpeed, DEC);          
      }
    }

    // OK perhaps it is a servo instead
    else if (firstByte == 'S'){
      byte servo = Serial.read();
      int degrees = Serial.parseInt();
      Serial.print("S");
      if (servo == 'P'){
        pan.write(degrees);
        Serial.print("P");
        Serial.println(degrees, DEC);
      } else if (servo == 'T'){
        Serial.print("T");
        tilt.write(degrees);     
        Serial.println(degrees, DEC);
      }
    }
  }
}

