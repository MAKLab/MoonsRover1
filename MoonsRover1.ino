//  Moon Rover1 code designed for Arduino Nano or similar
//  Pi can send serial commands in the form
//  "MLF255,MRF255,SP90,ST90," == Both motors forwards full speed, Both Servos center postion

#include <Wire.h>
#include <Servo.h>

Servo pan;  // create servo object to control a servo
Servo tilt; // same again

unsigned long int timer; 

// Attach our motor Pins to these pwm ports
const int dir1PinL = 7;
const int dir2PinL = 8;
const int motorPwmPinL = 5;

const int dir1PinR = 11;
const int dir2PinR = 12;
const int motorPwmPinR = 6;

// And our Servos to these...
const int serPinP = 9;
const int serPinT = 10;

// The rover needs to stop if the Pi has stopped sending instructions to it
// timeOut decides how long we should wait
const int timeOut = 500; 

// The camera mount does not sit pointing straight ahead, we can adjust it with this offet
const int panOffset = -3;
const int tiltOffset = 0;


void setup() {

  // Activate Serial
	Serial.begin(115200); //Best to use 115200, its the native baud for Pi Serial
	Serial.println("Rover Activated");

  // Attach the pan Servo and set to straight ahead
	pan.attach(serPinP);              // attaches the servo on Pin 9 to the servo object
  pan.write(90 + panOffset);  // move servo to starting position

  // Attach the tilt Servo and set to upright
	tilt.attach(serPinT);
  tilt.write(90 + tiltOffset);

	// Set PWM Pins as outputs
	pinMode(motorPwmPinL, OUTPUT);
	pinMode(motorPwmPinR, OUTPUT);

	// Set Motor Direction Pins as outputs
	pinMode(dir1PinL, OUTPUT);  // Motor A
	pinMode(dir2PinL, OUTPUT);

	pinMode(dir1PinR, OUTPUT);  // Motor B
	pinMode(dir2PinR, OUTPUT);

  // set timer variable to Arduino clock
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
		// byte motor = 0;

		// Check to see if its a motor
		if (firstByte == 'M' || firstByte == 'm'){

			//  Its a motor, lets nab some movement data
			byte motor = Serial.read();          // collects the motor name (L or R)
			byte dir = Serial.read(); // collects the motor direction (F or B)
			int mSpeed = Serial.parseInt(); // collects the motor speed (0 - 255)
			
			// Lets turn that into movement

			// Is it motor L?
			if (motor == 'L' || motor == 'l'){

				// Are we going forwards?
				if (dir == 'F' || dir == 'f'){
					digitalWrite(dir1PinL, HIGH);
					digitalWrite(dir2PinL, LOW);
				}

				// perhaps reverse?
				else if (dir == 'B' || dir == 'b'){
					digitalWrite(dir1PinL, LOW);
					digitalWrite(dir2PinL, HIGH);
				}
       
				analogWrite(motorPwmPinL, mSpeed);
			}

			// Perhaps it's R then?
			else if (motor == 'R' || motor == 'r'){

				// Are we going forwards
				if (dir == 'F' || dir == 'f'){
					digitalWrite(dir1PinR, HIGH);
					digitalWrite(dir2PinR, LOW);
				}

				// perhaps reverse?
				else if (dir == 'R' || dir == 'r')  {
					digitalWrite(dir1PinR, LOW);
					digitalWrite(dir2PinR, HIGH);
				}
       
				analogWrite(motorPwmPinR, mSpeed);
			}
		}

		// OK perhaps it is a servo instead
		else if (firstByte == 'S' || firstByte == 's'){
      
			byte servo = Serial.read();
			int degrees = Serial.parseInt();

			if (servo == 'P' || servo == 'p'){
				pan.write(degrees + panOffset);
			}
			else if (servo == 'T' || servo == 't'){
				tilt.write(degrees + tiltOffset);
			}
		}
   
	// We have had a successful communication, keep the rover moving
  	// by reseting it's safety timeout
  	timer = millis(); 
	}
}

void makeSafe(){
  // Apply the breaks
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL, HIGH);

  digitalWrite(dir1PinR, HIGH);
  digitalWrite(dir2PinR, HIGH); 

  // Set the camera mount to safe position
  pan.write(90 + panOffset);
  tilt.write(90 + tiltOffset);
  
  // tidy up by switching off the PWM signal
  digitalWrite (motorPwmPinL, LOW);
  digitalWrite (motorPwmPinR, LOW);
}

//  LM298 Enable Pins
//  Enable Motor  HIGH–Enable   LOW – Disable Motor
//  Direction 1   IN1 – HIGH    IN2 – LOW
//  Direction 2   IN1 – LOW     IN2 – HIGH
//  Coasting      IN1 – LOW     IN2 – LOW
//  Break         IN1 – HIGH    IN2 – HIGH
