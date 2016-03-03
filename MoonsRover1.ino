//  Moon Rover1 code designed for Arduino Nano or similar
//  Pi can send serial commands in the form
//  "MAF255,MBF255,SP90,ST90," == Both motors forwards full speed, Both Servos center postion

#include <Wire.h>
#include <Servo.h>

Servo pan;  // create servo object to control a servo
Servo tilt; // same again

unsigned long int timer; 

// const bool debug = 1;

// Attach our motor pins to these pwm ports
const int dir1PinA = 7;
const int dir2PinA = 8;
const int motorPwmPinA = 5;

const int dir1PinB = 11;
const int dir2PinB = 12;
const int motorPwmPinB = 6;

// The rover needs to stop if the Pi has stopped sending instructions to it
// timeOut decides how long we should wait
const int timeOut = 500; 

// The camera mount does not sit pointing straight ahead, we can adjust it with this offet
const int panOffset = -3;
const int tiltOffset = 0;


void setup() {
	Serial.begin(115200); //Best to use 115200, its the native baud for Pi Serial
	Serial.println("Rover Activated");

  // Attach the pan Servo and set to straight ahead
	pan.attach(9);              // attaches the servo on pin 9 to the servo object
  pan.write(90 + panOffset);  // move servo to starting position

  // Attach the tilt Servo and set to upright
	tilt.attach(10);
  tilt.write(90 + tiltOffset);

	// Set PWM pins as outputs
	pinMode(motorPwmPinA, OUTPUT);
	pinMode(motorPwmPinB, OUTPUT);

	// Set Motor Direction Pins as outputs
	pinMode(dir1PinA, OUTPUT);  // Motor A
	pinMode(dir2PinA, OUTPUT);

	pinMode(dir1PinB, OUTPUT);  // Motor B
	pinMode(dir2PinB, OUTPUT);

  timer = millis();  // We will use this timer to cut the motors if signal is lost to rPI

}

void loop() {

  // Check to see if we have had no instruction for a while (bad)
  if (timer + timeOut < millis()){
    makeSafe();
    timer = millis();  
    // lets not waste too much time switching
    // off motors that are already stopped
  }

  //  Check to see if we have some instructions.  If there is enough serial data
  //  for some reason I need more than one byte in the buffer?
	if (Serial.available()>3){
		byte firstByte = Serial.read();
		byte motor = 0;

		// Check to see if it a motor
		if (firstByte == 'M'){

			//  Its a motor, lets nab some movement data
			motor = Serial.read();          // collects the motor name (A or B)
			byte direction = Serial.read(); // collects the motor direction (F or R)
			int mSpeed = Serial.parseInt(); // collects the motor speed (0 - 255)
			
			// Lets turn that into movement

			// Is it motor A?
			if (motor == 'A'){

				// Serial.print("A");  // uncomment to debug

				// Are we going forwards?
				if (direction == 'F'){
					digitalWrite(dir1PinA, HIGH);
					digitalWrite(dir2PinA, LOW);
					// Serial.print("F"); // uncomment to debug
				}

				// perhaps reverse?
				else if (direction == 'R'){
					digitalWrite(dir1PinA, LOW);
					digitalWrite(dir2PinA, HIGH);
					// Serial.print("R"); // uncomment to debug
				}
				analogWrite(motorPwmPinA, mSpeed);
				// Serial.println(mSpeed, DEC); // uncomment to debug
			}

			// Perhaps it's B then?
			else if (motor == 'B'){

				// Are we going forwards
				if (direction == 'F'){
					digitalWrite(dir1PinB, HIGH);
					digitalWrite(dir2PinB, LOW);
					// Serial.print("F"); // uncomment to debug
				}

				// perhaps reverse?
				else if (direction == 'R')  {
					digitalWrite(dir1PinB, LOW);
					digitalWrite(dir2PinB, HIGH);
					// Serial.print("R"); // uncomment to debug
				}
				analogWrite(motorPwmPinB, mSpeed);
				// Serial.println(mSpeed, DEC); // uncomment to debug
			}
		}

		// OK perhaps it is a servo instead
		else if (firstByte == 'S'){
			byte servo = Serial.read();
			int degrees = Serial.parseInt();
			// Serial.print("S"); // uncomment to debug
			if (servo == 'P'){
				pan.write(degrees + panOffset);
				// Serial.print("P"); // uncomment to debug
				// Serial.println(degrees, DEC); // uncomment to debug
			}
			else if (servo == 'T'){
				tilt.write(degrees + tiltOffset);
        // Serial.print("T");    // uncomment to debug     
				// Serial.println(degrees, DEC); // uncomment to debug
			}
		}
  // We have had a successful communication, keep the rover moving
  // by reseting it's timer
  timer = millis(); 
	}
}

void makeSafe(){
  // Apply the breaks
  digitalWrite(dir1PinA, HIGH);
  digitalWrite(dir2PinA, HIGH);

  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, HIGH); 

  // Set the camera mount to safe position
  pan.write(90 + panOffset);
  tilt.write(90 + tiltOffset);
  
  // tidy up by switching off the PWM signal
  digitalWrite (motorPwmPinA, LOW);
  digitalWrite (motorPwmPinB, LOW);
}

//  LM298 Enable Pins
//  Enable Motor  HIGH–Enable   LOW – Disable Motor
//  Direction 1   IN1 – HIGH    IN2 – LOW
//  Direction 2   IN1 – LOW     IN2 – HIGH
//  Coasting      IN1 – LOW     IN2 – LOW
//  Break         IN1 – HIGH    IN2 – HIGH

