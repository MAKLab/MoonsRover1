//  Moon Rover1 code designed for Arduino Nano or similar
//  Pi can send serial commands in the form
//  "MLF255,MRF255,SP90,ST90," == Both motors forwards full speed, Both Servos center postion

#include <Wire.h>
#include <Servo.h>


// ***********************************************************************************************
// *                                    MPU6050 Stuff                                            *
// ***********************************************************************************************

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   * ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// ***********************************************************************************************
// *                                    Set up Servos                                            *
// ***********************************************************************************************

Servo pan;  // create servo object to control a servo
Servo tilt; // same again


// ***********************************************************************************************
// *                                    Variables                                                *
// ***********************************************************************************************

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float targetYaw;


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Just a timer for safety stuff
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

// ***********************************************************************************************
// *                                    Code                                                     *
// ***********************************************************************************************

void setup() {

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately

	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(39);
	mpu.setYGyroOffset(-16);
	mpu.setZGyroOffset(-22);
	mpu.setZAccelOffset(1238); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		// Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

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
	 if (!dmpReady) return;
	//Serial.print(".");
	// Check to see if we have had no instruction for a while (bad)
	if (timer + timeOut < millis()) {
		makeSafe();
		timer = millis();
	}  // end if safety

	//  Check to see if we have some instructions.  If there is enough serial data
	//  for some reason I need more than one byte in the buffer?
	if (Serial.available() > 3) {
		byte firstByte = Serial.read();
		// byte motor = 0;

		// Check to see if its a motor
		if (firstByte == 'M' || firstByte == 'm') {
			Serial.println(F("Motor"));

			//  Its a motor, lets nab some movement data
			byte motor = Serial.read();          // collects the motor name (L or R)
			byte dir = Serial.read(); // collects the motor direction (F or B)
			int mSpeed = Serial.parseInt(); // collects the motor speed (0 - 255)

			// Lets turn that into movement

			if (motor == 'a' || motor == 'A') {
				// code for all motors goes here!
				if (dir == 'F' || dir == 'f') {  // All Motors forward!
					digitalWrite(dir1PinL, HIGH);
					digitalWrite(dir2PinL, LOW);
					digitalWrite(dir1PinR, HIGH);
					digitalWrite(dir2PinR, LOW);
					analogWrite(motorPwmPinL, mSpeed);
					analogWrite(motorPwmPinR, mSpeed);
				}
				else if (dir == 'B' || dir == 'b') {
					// perhaps reverse?			

					digitalWrite(dir1PinL, LOW);
					digitalWrite(dir2PinL, HIGH);
					digitalWrite(dir1PinL, LOW);
					digitalWrite(dir2PinL, HIGH);
					analogWrite(motorPwmPinL, mSpeed);
					analogWrite(motorPwmPinR, mSpeed);
				} // end IF dir = B
				else if (dir == 'L' || dir == 'l' || dir == 'R' || dir == 'r'){
					// a turn?

					float targetYaw = getYaw();  // Find out what way we are facing
					Serial.println(targetYaw, 1);  // for debug
				}

				// Now lets set our target 
				if (dir == 'L' || dir == 'l') {

					if (targetYaw - mSpeed < -180){
						targetYaw = 180 + targetYaw + mSpeed;
					} else { 
						targetYaw -= mSpeed; 
					}

					digitalWrite(dir1PinL, HIGH);  // Left Forward
					digitalWrite(dir2PinL, LOW);   // Left Forward

					digitalWrite(dir1PinR, LOW);   // Right Back
					digitalWrite(dir2PinR, HIGH);  // Right Back

					analogWrite(motorPwmPinL, 255);
					analogWrite(motorPwmPinR, 255);

					while (targetYaw < getYaw()){

						// do nothing 
					}
					makeSafe();
				}
				else if (dir == 'R' || dir == 'r') {
					if (targetYaw + mSpeed > 180){
						targetYaw = -180 + targetYaw - mSpeed;
					}
					else {
						targetYaw += mSpeed;
					}

					digitalWrite(dir1PinR, HIGH);   // Right forward
					digitalWrite(dir2PinR, LOW);    // Right forward

					digitalWrite(dir1PinL, LOW);    // Left Back
					digitalWrite(dir2PinL, HIGH);   // Left Back

					analogWrite(motorPwmPinL, 255);
					analogWrite(motorPwmPinR, 255);

					while (targetYaw > getYaw()){
						// do nothing 
					}
					makeSafe();

				}  // End IF dir = R
			}  // end IF Motor = A
			else if (motor == 'L' || motor == 'l') {
				// Are we going forwards?
				if (dir == 'F' || dir == 'f') {
					digitalWrite(dir1PinL, HIGH);  // Forward
					digitalWrite(dir2PinL, LOW);   // Forward
				}  // end if dir = f
				// perhaps reverse?
				else if (dir == 'B' || dir == 'b') {
					digitalWrite(dir1PinL, LOW);
					digitalWrite(dir2PinL, HIGH);
				} // end if dir = r
				analogWrite(motorPwmPinL, mSpeed);
			}  // End IF Motor = L
			else if (motor == 'R' || motor == 'r') {
				// Are we going forwards
				if (dir == 'F' || dir == 'f') {
					digitalWrite(dir1PinR, HIGH);   //forward
					digitalWrite(dir2PinR, LOW);    //forward
				}
				// perhaps reverse?
				else if (dir == 'B' || dir == 'b') {
					digitalWrite(dir1PinR, LOW);
					digitalWrite(dir2PinR, HIGH);
				}

				analogWrite(motorPwmPinR, mSpeed);
			}  // End IF Motor = R


		}  // end firstbyte = m
		  
		// OK perhaps it is a servo instead
		else if (firstByte == 'S' || firstByte == 's') {

			byte servo = Serial.read();
			int degrees = Serial.parseInt();

			if (servo == 'P' || servo == 'p') {
				pan.write(degrees + panOffset);
			}
			else if (servo == 'T' || servo == 't') {
				tilt.write(degrees + tiltOffset);
			}
		}

		// We have had a successful communication, keep the rover moving
		// by reseting it's safety timeout
		timer = millis();
	}// end serial instruction recivied



} // End loop


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpDataReady() {
	mpuInterrupt = true;
}

void makeSafe() {
	// Apply the breaks
	digitalWrite(dir1PinL, HIGH);
	digitalWrite(dir2PinL, HIGH);

	digitalWrite(dir1PinR, HIGH);
	digitalWrite(dir2PinR, HIGH);

	// Set the camera mount to safe position
	// pan.write(90 + panOffset);
	// tilt.write(90 + tiltOffset);

	// tidy up by switching off the PWM signal
	digitalWrite(motorPwmPinL, LOW);
	digitalWrite(motorPwmPinR, LOW);
}

float getYaw() {  // True for right
	mpu.resetFIFO();
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	while (!mpuInterrupt && fifoCount < packetSize) {
		// do nothing for a little bit while we get a gyro reading
	}

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}

	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		//Serial.print("ypr\t");

		float Yaw = ypr[0] * 180 / M_PI;  // pick up last one
		Serial.println(Yaw);

		return Yaw;
	}

	// Do turning stuff here 

}

void setLeftMotor(int){}

//  LM298 Enable Pins
//  Enable Motor  HIGH–Enable   LOW – Disable Motor
//  Direction 1   IN1 – HIGH    IN2 – LOW
//  Direction 2   IN1 – LOW     IN2 – HIGH
//  Coasting      IN1 – LOW     IN2 – LOW
//  Break         IN1 – HIGH    IN2 – HIGH
