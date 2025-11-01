/*
	MIT License
	Copyright 2021 Michael Schoeffler (https://www.mschoeffler.com)
	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * TODO: states: she gets confused in the reset state
 * TODO: hardware - diva down!
 * TODO: capacitors?
 * TODO: fluff - different Scan algorithms
*/

#include <Servo.h>

// Debug macros
#define TEST_SERVOS 0
#define TEST_SENSORS 1
#define TEST_STATES 1

// Baby Head pins or wherever we hook these up to
#define PINHEAD 9
#define CHATTERER 10
#define BUTTERBALL 11
#define ANGELIQUE 6
#define SHITHEAD 5
#define CRUELLA 3

//Sensor pins
#define DOOR_TRIG 7
#define DOOR_ECHO 8
// "hall" is actually "couch"
#define HALL_TRIG 2
#define HALL_ECHO 4

// Sensor constants:
#define DELAY_SENSOR_READ 30 // <- ms
// #define THRESHOLD_DETECT 50 // <- cm, detection is True if sensor measurement < this 
#define THRESHOLD_DETECT_DOOR 30
#define THRESHOLD_DETECT_HALL 300
#define MAX_US_DURATION (4000UL * 58) // max range is 4m, so this is a good timeout value for pulseIn

// Servo constants:
#define MIN_WANDER_MS 13666
#define MAX_WANDER_MS 31666
#define ANGLE_HALL 10 // couch
#define ANGLE_DOOR 135
#define ANGLE_WALL 90

// Distance conversion macros
// ref: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
#define US_TO_IN(x) (x / 148)
#define US_TO_CM(x) (x / 58)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define CLAMP(val, low, high) (((val) <= (low)) ? (low) : MIN(val, high))

///-|___| hall
///-|w
///-|a
///-|l
///-|l
///-|___/ door 

enum {
	/* no movement, staring at wall */
	STATE_WALL = 0,
	/* scanning/wandering */
	STATE_SCAN,
	/* no movement, staring at hall */
	STATE_HALL,
	/* no movement, staring at door */
	STATE_DOOR,

	/* moving to a new position */
	STATE_MOVING,  // 4
	/* stay in current position until something happens */
	STATE_IDLE
};

struct spookyServo {
	Servo dev;
	int currentAngle;
	int goalAngle;
	// TODO: calibrate - may add an angleOffset field if needed, to account for different baby positions
};

int currentState = STATE_MOVING;

//total we have 8 servos, I don't quite know how they're going to move

Servo servo_pinhead;
Servo servo_chatterer;
Servo servo_butterball;
Servo servo_angelique;
Servo servo_shithead;
Servo servo_cruella;

struct spookyServo pinhead = {
	.dev = servo_pinhead,
	.currentAngle = 90,
	.goalAngle = 90
};
struct spookyServo chatterer = {
	.dev = servo_chatterer,
	.currentAngle = 90,
	.goalAngle = 90
};
struct spookyServo butterball = {
	.dev = servo_butterball,
	.currentAngle = 90,
	.goalAngle = 90
};
struct spookyServo angelique = {
	.dev = servo_angelique,
	.currentAngle = 90,
	.goalAngle = 90
};
struct spookyServo shithead = {
	.dev = servo_shithead,
	.currentAngle = 90,
	.goalAngle = 90
};
struct spookyServo cruella = {
	.dev = servo_cruella,
	.currentAngle = 90,
	.goalAngle = 90
};

struct spookyServo * strollers[] = {
	&pinhead,
	&chatterer,
	&butterball,
	&angelique,
	&shithead,
	&cruella,
};



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
#if 0
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
#endif

long doorPreviousMillis = 0;
long hallPreviousMillis = 0;
long interval = 1000;
float doorDuration, doorDistance;
float hallDuration, hallDistance;

/**
 * Trigger ultrasonic sensor and return the duration of the echo signal, in us. Blocking.
 *
 * @note not recommended to call this more often than every 60ms
 *
 * @param trig	trigger pin number
 * @param echo	echo pin number
 *
 * @retval echo signal duration, in us
 */
unsigned long read_sensor_duration(int trig, int echo){
	digitalWrite(trig, LOW);
	delayMicroseconds(2);
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);

	unsigned long duration = pulseIn(echo, HIGH, MAX_US_DURATION);
#if 0 
	Serial.println(duration);
	Serial.println("\n");
#endif

	return duration;
}

bool allServosDone(){
	int numServos = sizeof(strollers) / sizeof(strollers[0]);
	for(int i = 0; i<numServos; i++){
		if(strollers[i]->currentAngle != strollers[i]->goalAngle){
#if TEST_SERVOS
			Serial.print(i);
			Serial.print(": ");
			Serial.print(strollers[i]->currentAngle);
			Serial.print("->");
			Serial.println(strollers[i]->goalAngle);
#endif
			return false;
		}
	}
#if TEST_SERVOS
	Serial.println("All servos done");
#endif
	return true;
}

#define ANGLE_INCREMENT 1 // <- how far to move each servo each time
#define DELAY_BETWEEN_SERVOS 4 // <- how long to wait between switching movement to a different servo - cumulative delay will be numServos * this
#define DELAY_SERVO_TICK 4 // <- how long to wait between broader servo movements

void moveServosParallel(){
	int numServos = sizeof(strollers) / sizeof(strollers[0]);
	for(int i=0; i<numServos; i++){
		struct spookyServo *thisServo = strollers[i];
		if(thisServo->currentAngle < thisServo->goalAngle){
			thisServo->currentAngle = thisServo->currentAngle + ANGLE_INCREMENT;
			if(thisServo->currentAngle >= thisServo->goalAngle){
				thisServo->currentAngle = thisServo->goalAngle;
			}
		}
		else if(thisServo->currentAngle > thisServo->goalAngle){
			thisServo->currentAngle = thisServo->currentAngle - ANGLE_INCREMENT;
			if(thisServo->currentAngle <= thisServo->goalAngle){
				thisServo->currentAngle = thisServo->goalAngle;
			}
		}
		thisServo->dev.write(thisServo->currentAngle);
#if TEST_SERVOS
		Serial.print(thisServo->currentAngle);
		Serial.print(" -> ");
		Serial.println(thisServo->goalAngle);
#endif
		delay(DELAY_BETWEEN_SERVOS);
	}
	delay(DELAY_SERVO_TICK);
}

#define DELAY_SERIES_SERVOS 350
void moveServosSeries(){
	int numServos = sizeof(strollers) / sizeof(strollers[0]);
	for(int i=0; i<numServos; i++){
		strollers[i]->currentAngle = strollers[i]->goalAngle;
		strollers[i]->dev.write(strollers[i]->currentAngle);
		delay(DELAY_SERIES_SERVOS);
	}
}

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

void test_sensors(){
	unsigned long doorDuration = read_sensor_duration(DOOR_TRIG, DOOR_ECHO);
	unsigned long hallDuration = read_sensor_duration(HALL_TRIG, HALL_ECHO);
	Serial.print("Door duration: ");
	Serial.print(doorDuration);
	Serial.print(" us, (");
	Serial.print(US_TO_CM(doorDuration));
	Serial.println(" cm)");

	Serial.print("Hall duration: ");
	Serial.print(hallDuration);
	Serial.print(" us, (");
	Serial.print(US_TO_CM(hallDuration));
	Serial.println(" cm)");
}

/**
 * @brief Sets all `strollers` goal angle to the provided angle.
 */
void setNewServoGoals(int angle){
	if(angle > 180){
		angle = 180;
	}
	if(angle < 0){
		angle = 0;
	}
	int numServos = sizeof(strollers) / sizeof(strollers[0]);
	for(int i=0; i<numServos; i++){
		strollers[i]->goalAngle = angle;
	}
}

// https://www.handsontec.com/dataspecs/motor_fan/MG996R.pdf
void test_servos(){
	static int angle = 0;
	static int increment = 1;

	if(angle == 180){
		increment = -1;
	}
	if(increment == -1 && angle == 0){
		increment = 1;
	}

	angle += increment;
	Serial.print("Moving all servos to angle ");
	Serial.println(angle);

	int numServos = sizeof(strollers) / sizeof(strollers[0]);
	for(int i=0; i<numServos; i++){
		strollers[i]->dev.write(angle);
		delay(10);
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

void setup() {
	// put your setup code here, to run once:
	pinhead.dev.attach(PINHEAD);
	chatterer.dev.attach(CHATTERER);
	angelique.dev.attach(ANGELIQUE);
	butterball.dev.attach(BUTTERBALL);
	shithead.dev.attach(SHITHEAD);
	cruella.dev.attach(CRUELLA);

	//sensor pins
	pinMode(DOOR_TRIG, OUTPUT);
	pinMode(HALL_TRIG, OUTPUT);
	pinMode(DOOR_ECHO, INPUT);
	pinMode(HALL_ECHO, INPUT);

	//pinMode(sensor, INPUT);    // initialize sensor as an input
	Serial.begin(115200);  // initialize serial
	Serial.println("Lets Go!");

	// init the servo positions:
	int numServos = sizeof(strollers) / sizeof(strollers[0]);
	for(int i=0; i<numServos; i++){
		strollers[i]->dev.write(strollers[i]->goalAngle);
		delay(50);
	}
}

void loop() {
	static unsigned long lastSensorReadMs, nextWanderMs = 0;
	static bool doorTrigger, hallTrigger, wallTrigger = false;
	static bool wanderActive, wanderWaiting = false;
	unsigned long currentMillis = millis();
	// Handle sensor reads:
	if(currentMillis - lastSensorReadMs > 2 * DELAY_SENSOR_READ){
		unsigned long doorDistance = US_TO_CM(read_sensor_duration(DOOR_TRIG, DOOR_ECHO));
		if(doorDistance != 0){
			doorTrigger = doorDistance < THRESHOLD_DETECT_DOOR;
		}
		delay(DELAY_SENSOR_READ);
		unsigned long hallDistance = US_TO_CM(read_sensor_duration(HALL_TRIG, HALL_ECHO));
		if(hallDistance != 0){
			hallTrigger =  hallDistance < THRESHOLD_DETECT_HALL;
			if(hallTrigger){
				Serial.println("hallTrig");
			}
		}
		lastSensorReadMs = millis();
		// TODO: calibrate - this may need more logic depending on sensor noise 
		// (eg, average a few readings, throw out some, idk)
#if TEST_SENSORS
		Serial.print("door: ");
		Serial.print(doorDistance);
		Serial.print(", hall: ");
		Serial.println(hallDistance);
#endif
	}

	static bool printed = false;

	int numServos = sizeof(strollers) / sizeof(strollers[0]);
	bool doneMoving = allServosDone();
	switch(currentState){
		/* STATE_IDLE: hold still in one of the 3 positions, wait for sensor */
		case STATE_IDLE:
#if TEST_STATES
			if(!printed){
				Serial.println("STATE_IDLE");
				printed = true;
			}
#endif
			// look to door or hall if triggered:
			// TODO: should IDLE move to HALL if we came here from DOOR?
			if(doorTrigger || hallTrigger){
				if(doorTrigger){
					currentState = STATE_DOOR;
				}
				else{
					currentState = STATE_HALL;
				}
				printed = false;
				break;
			}
			else{
				if(wanderActive){
					printed = false;
					currentState = STATE_SCAN;
					break;
				}
				else if(wanderWaiting ){
					if (((long)(currentMillis - nextWanderMs)) > 0){
						Serial.println(currentMillis - nextWanderMs);
						printed = false;
						currentState = STATE_SCAN;
						break;
					}
				}
				else{
					printed = false;
					currentState = STATE_WALL;
				}
			}
			break;
		/** STATE_WALL: set up Wander timestamps, look forward */
		case STATE_WALL:
#if TEST_STATES
			if(!printed){
				Serial.println("STATE_WALL");
				printed = true;
			}
#endif
			if(!wanderWaiting){

				nextWanderMs = currentMillis + random(MIN_WANDER_MS, MAX_WANDER_MS);

				Serial.print("next wander timestamp: ");
				Serial.print(nextWanderMs);
				Serial.print(", current time ");
				Serial.println(currentMillis);

				wanderWaiting = true;
			}
			printed = false;
			setNewServoGoals(ANGLE_WALL);
			wallTrigger = true;
			currentState = STATE_MOVING;
			delay(13);
			break;
		case STATE_SCAN:
#if TEST_STATES
			if(!printed){
				Serial.println("STATE_SCAN");
				printed = true;
			}
#endif
			static int scanAngle = 0;
			static int direction = 1;
			wanderWaiting = false;
			wanderActive = true;
			/*
			if(doorTrigger){
				printed = false;
				wanderActive = false;
				currentState = STATE_DOOR;
				break;
			}
			else if(hallTrigger){
				// setNewServoGoals(ANGLE_HALL);
				printed = false;
				wanderActive = false;
				currentState = STATE_HALL;
				break;
			}
			else{
			*/
			if(direction >= 0){
				if(scanAngle >= 180){
					Serial.println("lookin");
					direction = -1;
				}
			}
			else{
				if(scanAngle <= 0){
					// Finished wandering:
					wanderActive = false;
					printed = false;
					scanAngle = 0;
					direction = 1;
					currentState = STATE_IDLE;
					// TODO: we may even have to let it linger here ..
					break;
				}
				Serial.println(scanAngle);
			}
			scanAngle += direction;
			setNewServoGoals(scanAngle);
			// delay(SERVO_SCAN_DELAY); // TODO: calibrate - scan speeds
			printed = false;
			currentState = STATE_MOVING;
			/* } */
			break;
		case STATE_HALL:
#if TEST_STATES
			if(!printed){
				Serial.println("STATE_HALL");
				printed = true;
			}
#endif
			printed = false;
			setNewServoGoals(ANGLE_HALL);
			wanderActive = false;
			currentState = STATE_MOVING;
			break;
		case STATE_DOOR:
#if TEST_STATES
			if(!printed){
				Serial.println("STATE_DOOR");
				printed = true;
			}
#endif
			setNewServoGoals(ANGLE_DOOR);
			currentState = STATE_MOVING;
			printed = false;
			wanderActive = false;
			break;
		case STATE_MOVING:
#if TEST_STATES
			if(!printed){
				Serial.println("STATE_MOVING");
				printed = true;
			}
#endif
			if(!doneMoving){
				if(wanderActive || wallTrigger){
				   moveServosParallel();
				}
				else{
					moveServosSeries();
				}
			}
			else{
				printed = false;
				currentState = STATE_IDLE;
				wallTrigger = false;
			}
			break;
		default:
			Serial.println("Invalid state");
			currentState = STATE_IDLE;
			break;
	}

	// delay(13);
	/*
	test_servos();
	delay(120);

	test_sensors();
	delay(120);
	*/
}
