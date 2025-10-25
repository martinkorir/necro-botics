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

//Baby Head Pins or wherever we hook these up to
#define PINHEAD 9
#define CHATTERER 2
#define BUTTERBALL 4
#define ANGELIQUE 6

//Sensor pins
#define DOOR_TRIG 9
#define DOOR_ECHO 10
#define HALL_TRIG 11
#define HALL_ECHO 12

///-|___| hall
///-|w
///-|a
///-|l
///-|l
///-|___/ door 

//no movement, staring at wall
#define WALL 0
//moving towards door
#define TO_DOOR 1
//stopped at door
#define DOOR 2
//moving towards hall
#define TO_HALL 3
//stopped at hall
#define HALL 4

int pinheadState = WALL;

//total we have 8 servos, I don't quite know how they're going to move
Servo pinhead;
// Servo chatterer;
// Servo butterball;
// Servo angelique;

int babies[] = {
	PINHEAD,
	//CHATTERER,
	//BUTTERBALL,
	//ANGELIQUE
};
Servo strollers[] = {
	pinhead,
	//chatterer,
	// butterball,
	//angelique
};


void setup() {
	// put your setup code here, to run once:
	pinhead.attach(PINHEAD);
	//chatterer.attach(CHATTERER);
	//angelique.attach(ANGELIQUE);
	//butterball.attach(BUTTERBALL);

	//sensor pins
	pinMode(DOOR_TRIG, OUTPUT);
	pinMode(HALL_TRIG, OUTPUT);
	pinMode(DOOR_ECHO, INPUT);
	pinMode(HALL_ECHO, INPUT);

	//pinMode(sensor, INPUT);    // initialize sensor as an input
	Serial.begin(9600);  // initialize serial
	Serial.println("Lets Go!");

	delay(10);
}


// void doorMotionISR(){
// 	if(digitalRead(doorSensor) == HIGH){
// 		Serial.println("Motion detected by door");
// 		edgeDetected = 1;
// 		Serial.println("Rotate to this point.");
// 		// stopAll(); // moved out of interrupt context
// 	}
// 	else{
// 		Serial.println("Motion stopped.");
// 		// state = MOVING; // not yet
// 		edgeDetected = -1;
// 		triggerTime = millis();
// 	}
// }

// void hallMotionISR(){
// 	if(digitalRead(hallSensor) == HIGH){
// 		Serial.println("Motion detected by hallway");
// 		edgeDetected = 1;
// 		Serial.println("Rotate to this point.");
// 		// stopAll(); // moved out of interrupt context
// 	}
// 	else{
// 		Serial.println("Motion stopped.");
// 		// state = MOVING; // not yet
// 		edgeDetected = -1;
// 		triggerTime = millis();
// 	}
// }

///This is to  turn the babies head to its full range
void wander() {
	pinhead.write(0);    // move MG996R's shaft to angle 0°
	delay(1000);         // wait for one second
	pinhead.write(45);   // move MG996R's shaft to angle 45°
	delay(1000);         // wait for one second
	pinhead.write(90);   // move MG996R's shaft to angle 90°
	delay(1000);         // wait for one second
	pinhead.write(135);  // move MG996R's shaft to angle 135°
	delay(1000);         // wait for one second
	pinhead.write(180);  // move MG996R's shaft to angle 180°
	delay(1000);         // wait for one second
}

long doorPreviousMillis = 0;
long hallPreviousMillis = 0;
long interval = 1000;
float doorDuration, doorDistance;
float hallDuration, hallDistance;

void readDoor(unsigned long currentMillis) {
	if (currentMillis - doorPreviousMillis > interval) {
		// save the last time you blinked the LED
		doorPreviousMillis = currentMillis;

		digitalWrite(DOOR_TRIG, LOW);
		delayMicroseconds(2);
		digitalWrite(DOOR_TRIG, HIGH);
		delayMicroseconds(10);
		digitalWrite(DOOR_TRIG, LOW);

		doorDuration = pulseIn(DOOR_ECHO, HIGH);
		doorDistance = (doorDuration * .0343) / 2;
		Serial.print("Door Distance: ");
		Serial.println(doorDistance);
	}
}

void readHall(unsigned long currentMillis) {
	if (currentMillis - hallPreviousMillis > interval) {
		// save the last time you blinked the LED
		hallPreviousMillis = currentMillis;

		digitalWrite(HALL_TRIG, LOW);
		delayMicroseconds(2);
		digitalWrite(HALL_TRIG, HIGH);
		delayMicroseconds(10);
		digitalWrite(HALL_TRIG, LOW);

		hallDuration = pulseIn(HALL_ECHO, HIGH);
		hallDistance = (hallDuration * .0343) / 2;
		Serial.print("Door Distance: ");
		Serial.println(hallDistance);
	}
}


//hiii ok here's what im envisioning for the states 
// when the baby "sees" someone at the door, it should stay looking 
//at the door until it no longer detects someone there, regardless 
//if the hall sensor detects someone
//when its en route to either the door or the hall, it can switch 
//over to the door if someone is sensed, the door is the priority because 
//thats where people enter and exit
//the wall is its "neutral" state, we wnat itto rest there every once in a while
//otherwise we wnat it to periodically "scan" the room, but not too often 
//because its meant to be sneaky
//
//We don't want to attach any baby heads yet but essentially when we set the babies up, 
//we'll want their 0 degree to be the center of their face since this servon is a 180 servo
//it looks like we will have to hot glue the heads on site

void loop() {
	unsigned long currentMillis = millis();
	readDoor(currentMillis);
	currentMillis = millis();
	readHall(currentMillis);
}
