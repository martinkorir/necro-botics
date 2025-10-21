//This is the code that we used to move our scary little animals when no motion detected


/*  
	Arduino with PIR motion sensor
	For complete project details, visit: http://RandomNerdTutorials.com/pirsensor
	Modified by Rui Santos based on PIR sensor by Limor Fried
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


//Animal pins:
#define FLUFFY 0
#define SWEATER 2
#define POLAR 4
#define DINO 6
#define PUPPY 8
#define EMPTY 10

// State variables
#define MOVING	LOW
#define STOPPED HIGH
volatile int state = STOPPED;
volatile int edgeDetected = -1; // 0 = none, 1 = rising, -1 = falling. init'd to -1 so that it'll actually start moving

int fastPins[] = {FLUFFY, SWEATER, POLAR};
int slowPins[] = {DINO, PUPPY};
int fastAngle = 5;
int slowAngle = 10;
int numberFastAnimals = sizeof(fastPins)/sizeof(int);
int numberSlowAnimals = sizeof(slowPins)/sizeof(int);

int sensor = 2;              // the pin that the sensor is attached to
int val = 0;                 // variable to store the sensor status (value)

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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
	//pinMode(led, OUTPUT);      // initalize LED as an output
	pinMode(sensor, INPUT);    // initialize sensor as an input
	Serial.begin(9600);        // initialize serial
	Serial.println("Hi queen");

	pwm.begin();

	/*
	 * In theory the internal oscillator (clock) is 25MHz but it really isn't
	 * that precise. You can 'calibrate' this by tweaking this number until
	 * you get the PWM update frequency you're expecting!
	 * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
	 * is used for calculating things like writeMicroseconds()
	 * Analog servos run at ~50 Hz updates, It is importaint to use an
	 * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
	 * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
	 *    the I2C PCA9685 chip you are setting the value for.
	 * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
	 *    expected value (50Hz for most ESCs)
	 * Setting the value here is specific to each individual I2C PCA9685 chip and
	 * affects the calculations for the PWM update frequency. 
	 * Failure to correctly set the int.osc value will cause unexpected PWM results
	 */
	pwm.setOscillatorFrequency(27000000);
	pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

	// Register pin change interrupt for motion sensor - both edges
	attachInterrupt (digitalPinToInterrupt (sensor), motionISR, CHANGE);

	delay(10);
}

void moveUs(int animalA, int animalB, int angle){
	Serial.print("Moving animal ");
	Serial.print(animalA);
	Serial.print(" and animal ");
	Serial.println(animalB);
	int delayTime = STEP_DELAY * angle;
	for(int i=0; i<NUM_ROTATIONS; i++){
		pwm.setPWM(animalA, 0, SERVO_FORWARD_PULSE_WIDTH); //full speed forward
		pwm.setPWM(animalB, 0, SERVO_STOP_PULSE_WIDTH); 
		delay(delayTime);
		pwm.setPWM(animalA, 0, SERVO_STOP_PULSE_WIDTH); //stops
		pwm.setPWM(animalB, 0, SERVO_FORWARD_PULSE_WIDTH);
		if(edgeDetected == 1){
      pwm.setPWM(animalB, 0, SERVO_STOP_PULSE_WIDTH);
			return;
		}
		delay(delayTime);
		pwm.setPWM(animalA, 0, SERVO_BACKWARD_PULSE_WIDTH); //full speed backward
		pwm.setPWM(animalB, 0, SERVO_STOP_PULSE_WIDTH); //full speed backward
		delay(delayTime);
		pwm.setPWM(animalA, 0, SERVO_STOP_PULSE_WIDTH); //stops
		pwm.setPWM(animalB, 0, SERVO_BACKWARD_PULSE_WIDTH); 
		if(edgeDetected == 1){
			return;
		}
		delay(delayTime);
	}
}

void stopAll(){
	// stop fast ones
	for(int i = 0; i < numberFastAnimals; i++){
		pwm.setPWM(fastPins[i], 0, 0); 
	}
	// stop slow ones
	for(int i = 0; i < numberSlowAnimals; i++){
		pwm.setPWM(slowPins[i], 0, 0); 
	}
}

#define DELAY_MS 5000
#define MAX_DELAY_BETWEEN 7000
int currentTime = 0;
int triggerTime = 0;
int timeLastMoved = 0;
int thisDelay = MAX_DELAY_BETWEEN;

void motionISR(){
	if(digitalRead(sensor) == HIGH){
		Serial.println("Motion detected!");
		edgeDetected = 1;
		Serial.println("Stopping all animals.");
		// stopAll(); // moved out of interrupt context
	}
	else{
		Serial.println("Motion stopped.");
		// state = MOVING; // not yet
		edgeDetected = -1;
		triggerTime = millis();
	}
}

void loop(){
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