/*
MIT License
Copyright 2021 Michael Schoeffler (https://www.mschoeffler.com)
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
* Example source code of an Arduino tutorial on how to control an MG 996R servo motor. 
*/

#include <Servo.h>

// servo object representing the MG 996R servo

//Baby Head Pins or wherever we hook these up to
#define PINHEAD 0
#define CHATTERER 2
#define BUTTERBALL 4
#define ANGELIQUE 6
#define EMPTY 8
#define EMPTY 10

//total we have 12 servos
Servo servo;
Servo pinhead;
Servo chatterer;
Servo butterball;
Servo angelique;

// State variables
#define MOVING	LOW
#define STOPPED HIGH
volatile int state = STOPPED;
volatile int edgeDetected = -1; // 0 = none, 1 = rising, -1 = falling. init'd to -1 so that it'll actually start moving

int babies[] = {PINHEAD, CHATTERER, BUTTERBALL, ANGELIQUE}
Servo strollers[] = {pinhead, chatterer, butterball, angelique}

//TODO: get the real sensor values here
int hallSensor = 2;              // the pin that the sensor is attached to
int doorSensor = 4;
int val = 0;                 // variable to store the sensor status (value)

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Define the servo parameters
#define SERVO_FORWARD_PULSE_WIDTH  410
#define SERVO_STOP_PULSE_WIDTH     0
#define SERVO_BACKWARD_PULSE_WIDTH 205

// Define servo rotation parameters
#define STEP_DELAY    15  // Milliseconds per degree, approximate
#define NUM_ROTATIONS 3   // How many times an animal moves back and forth
#define STOP_DELAY    200 // How long to wait on stopped segments

// our servo # counter
uint8_t servonum = 0;

void setup() {
  // put your setup code here, to run once:
  servo.attach(3);
  pinhead.attach(PINHEAD);
  chatterer.attach(CHATTERER);
  angelique.attach(ANGELIQUE);
  butterball.attach(BUTTERBALL);

  pinMode(sensor, INPUT);    // initialize sensor as an input
	Serial.begin(9600);        // initialize serial
	Serial.println("Lets Go!");

  // Register pin change interrupt for motion sensor - both edges
	attachInterrupt (digitalPinToInterrupt (doorSensor), doorMotionISR, CHANGE);
  attachInterrupt (digitalPinToInterrupt (hallSensor), hallMotionISR, CHANGE);

	delay(10);
}


void doorMotionISR(){
	if(digitalRead(doorSensor) == HIGH){
		Serial.println("Motion detected by door");
		edgeDetected = 1;
		Serial.println("Rotate to this point.");
		// stopAll(); // moved out of interrupt context
	}
	else{
		Serial.println("Motion stopped.");
		// state = MOVING; // not yet
		edgeDetected = -1;
		triggerTime = millis();
	}
}

void hallMotionISR(){
	if(digitalRead(hallSensor) == HIGH){
		Serial.println("Motion detected by hallway");
		edgeDetected = 1;
		Serial.println("Rotate to this point.");
		// stopAll(); // moved out of interrupt context
	}
	else{
		Serial.println("Motion stopped.");
		// state = MOVING; // not yet
		edgeDetected = -1;
		triggerTime = millis();
	}
}


#define DELAY_MS 5000
#define MAX_DELAY_BETWEEN 7000
int currentTime = 0;
int triggerTime = 0;
int timeLastMoved = 0;
int thisDelay = MAX_DELAY_BETWEEN;


void loop() {
  // put your main code here, to run repeatedly:
  servo.write(0);    // move MG996R's shaft to angle 0°
  delay(1000);       // wait for one second
  servo.write(45);   // move MG996R's shaft to angle 45°
  delay(1000);       // wait for one second
  servo.write(90);   // move MG996R's shaft to angle 90°
  delay(1000);       // wait for one second
  servo.write(135);  // move MG996R's shaft to angle 135°
  delay(1000);       // wait for one second
  servo.write(180);  // move MG996R's shaft to angle 180°
  delay(1000);       // wait for one second

  currentTime = millis();
	int dTime = currentTime - triggerTime;
  int dTimeMtn = currentTime - timeLastMoved;
  
	if(edgeDetected == 1){
		stopAll();
		state = STOPPED;
		edgeDetected = 0;
	}
	if(edgeDetected == -1 && dTime > DELAY_MS){
		state = MOVING;
		edgeDetected = 0;
	}

	if(state == MOVING && dTimeMtn > thisDelay){
		// Choose fast or slow ones
		int index = random(0, 10);
		if(index <= 2){
      switch(index){
        case 0:
           moveUs(slowPins[0], slowPins[0], slowAngle);
           break;
        case 1:
          moveUs(slowPins[1], slowPins[1], slowAngle);
          break;
        default:
          moveUs(slowPins[0], slowPins[1], slowAngle);
      }
      timeLastMoved = millis();
      thisDelay = random(0,MAX_DELAY_BETWEEN);
		}
		else{
      //randomly make the animals move
		  int A = random(0, numberFastAnimals);
      int B = random(0, numberFastAnimals);
      moveUs(fastPins[A], fastPins[B], fastAngle);
      timeLastMoved = millis();
      thisDelay = random(0,MAX_DELAY_BETWEEN);
		}
	}
	else{ // ie, motion is detected
		  // only trigger on edges - not appropriate in this scope, moved to edge detection area
		/*
		   Serial.println("Stopping all animals.");
		   stopAll();
		 */
	}
}
