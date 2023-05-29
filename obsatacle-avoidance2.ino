#include <Servo.h>
#include <NewPing.h>
#include "note.h"

//Info
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz

//Enable/Disable pin
const int enable = 12; 	//
const int buzzer = 6; 	//pwm 980

//L298N control pins
// Motor A (LEFT)
const int enA = 11; 	//pwm  490
const int in1 = 10; 	//pwm n/a din cauza servo
const int in2 = 9; 		//pwm n/a din cauza servo
// Motor B (RIGHT)
const int enB = 3; 		//pwm 490
const int in3 = 7;
const int in4 = 8;

//SR04 - sonar pins
const int trig = A1;
const int echo = A2;

//Servo PWM signal pin
const int servoPin = 5; 	//pwm 980

//PIR
const int pir = A3;

//CONFIG
const int stopDistance = 40; //cm
const int maxDrivingTime = 3000;
const int maxSearchigTime = 10000;
const int slowDownTime = 1500;
const int servoDelay = 200;
const int maxDistance = 200; //cm
const int lowSpeed = 45; //of 255
const int medSpeed = 120; //of 255
const int maxSpeed = 180; //of 255
const int boostSpeed = 100;
const int boostTime = 200; //ms
const int spinOffset = 20; //diferenta intre puterea motoarelor la viraj

//INIT
NewPing sonar(trig, echo, maxDistance);
Servo servoMotor;

//GLOBALS
unsigned long startTimeDriving = 0;
unsigned long currentDrivingTime = 0;
unsigned long startTimeSearching = 0;
unsigned long currentSearchingTime = 0;
boolean lookingForward = false;
boolean goesForward = false;
boolean isSearching = true;
boolean pirActivated = false;

//int triggerDistance = 60;
int hz;
int i;

void setup(){
	Serial.begin(9600);
	Serial.println("Robot initializing...");
	
	//Enable pin
	pinMode(enable, INPUT_PULLUP);
	pinMode(buzzer, OUTPUT);
	
	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);

	// Turn off motors - Initial state
	analogWrite(enA, 0);
	analogWrite(enB, 0);
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);

	//Setup servo
	servoMotor.attach(servoPin);
	lookForward();

	//Startup sound
	tone(buzzer, 400);
	delay(200);
	noTone(buzzer);
	delay(1000);
	startTimeDriving = millis();

	playSiren(2, 5);

	////PWM Fq pt D3 si D11
  	TCCR2B = TCCR2B & B11111000 | B00000111; // 30Hz

}

void loop() {
	//Pornit / Oprit
	if(digitalRead(enable) == HIGH){
		Serial.println("Robot dezactivat! Exit...");
		stopMotors();
		lookForward();
		delay(1000);
		return;
	}

	//Get curent data
	getDrivingTime();
	getSearchingTime();
	//Serial.print("Driving time = ");
	//Serial.println(currentDrivingTime);
	//Serial.print("Searching time = ");
	//Serial.println(currentSearchingTime);

	//Paza start
	if(goesForward && currentSearchingTime > maxSearchigTime){
		Serial.println("Intru in mod paza...");
		isSearching = false;
		goesForward = false;
		goBackward(medSpeed, 150); // frana
		stopMotors();
		playSiren(9, 2); // de pus un sunet la intrarea in paza
		delay(1000);
	}

	//Trigger iesire din paza by pir
	if(digitalRead(pir) == HIGH){
		if(!pirActivated && !isSearching){
			Serial.println("Trigger by pir declansat...");
			pirActivated = true;
			playSiren(9, 2);
			searchBestPath();
			delay(1000);
			isSearching = true;
		}
	}else{
		if(pirActivated){
			pirActivated = false;
		}
	}
	
	//In caz de obstacol (?? && currentSearchingTime > 500)
	if(goesForward && isSearching && getDistance(3) <= stopDistance){
		Serial.println("Obstacol detectat");
		goBackward(medSpeed, 300); // frana
		stopMotors();
		playSiren(10, 100);
		searchBestPath();
	}

	//Incetineste
	if(goesForward && currentDrivingTime > slowDownTime){
		Serial.println("Incetinesc");
		analogWrite(enA, 25);
		analogWrite(enB, 25);
	}
	
	//Schimba directia curenta daca a mers prea mult
	if(goesForward && isSearching && currentDrivingTime > maxDrivingTime){
		Serial.println("Am mers prea mult");
		goBackward(medSpeed, 250);
		stopMotors();
		searchBestPath();
	}

	//Totul bine, merg inainte
	if(isSearching && !goesForward){
		Serial.println("Go forward");
		goForward(lowSpeed);
		//Serial.println("Reset driving time");
		startTimeDriving = millis();
	}
}

boolean isEnabled(){
	if(digitalRead(enable) == HIGH){
		return false;
	}else{
		return true;
	}
}

void getSearchingTime(){
	if(isSearching){
		currentSearchingTime = millis() - startTimeSearching;
	}else{
		//Reset searching time
		startTimeSearching = millis();
		currentSearchingTime = 0;
	}
}

void getDrivingTime(){
	if(goesForward){
		currentDrivingTime = millis() - startTimeDriving;
	}
}

int getDistance(int count){
	delay(30);
	int currentDistance = 0;
	//if(count > 1){
	//	currentDistance = sonar.convert_cm(sonar.ping_median(count));//sonar.ping_cm();
	//}else{
		currentDistance = sonar.ping_cm();
	//}
	
//	Serial.print("getDistance ");
//	Serial.println(currentDistance);

	if(currentDistance == 0 || currentDistance > maxDistance){
		currentDistance = maxDistance;
	}
	return currentDistance;
}

void searchBestPath(){
	Serial.println("Caut calea cea mai buna...");
	boolean goodToGo = false;
	int distanceLeft = 0;
	int distanceForward = 0;
	int distanceRight = 0;

	distanceForward = getDistance(3);
	if(distanceForward >= 100){
		goodToGo = true;
		lookForward();
		return;
	}
	
	if(!goodToGo){
		lookLeft();
	  	delay(servoDelay);
		distanceLeft = getDistance(3);
		Serial.print("distanceLeft = ");
		Serial.println(distanceLeft);
		//delay(150);
		if(distanceLeft >= 100){
			//Serial.println("Turn left");
			turnLeft(maxSpeed, 200);
			lookForward();
			return;
		}
	}

	if(!goodToGo){
		lookRight();
	  	delay(servoDelay);
		distanceRight = getDistance(3);
		Serial.print("distanceRight = ");
		Serial.println(distanceRight);
		//delay(150);
		if(distanceRight >= 100){
			//Serial.println("Turn right");
			turnRight(maxSpeed, 200);
			lookForward();
			return;
		}
	}
	
	if(distanceLeft > distanceForward && distanceLeft > distanceRight){
		//Serial.println("Turn left");
		turnLeft(maxSpeed, 200);
	}else if(distanceRight > distanceForward){
		//Serial.println("Turn right");
		turnRight(maxSpeed, 200);
	}//else go forward, distanceForward e mai mare ca toate;
	
	lookForward();
}

void stopMotors(){
	//Serial.println("Stop motors");
	analogWrite(enA, 0);
	analogWrite(enB, 0);
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
	goesForward = false;
}

void goForward(int motorSpeed){
	if(!lookingForward){
		lookForward();
	}
	if(!goesForward){
		Serial.println("Go forward");
		//Boost
		analogWrite(enA, boostSpeed);
		analogWrite(enB, boostSpeed);
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
		digitalWrite(in3, HIGH);
		digitalWrite(in4, LOW);
		delay(boostTime);
		analogWrite(enA, motorSpeed);
		analogWrite(enB, motorSpeed);
		goesForward = true;
	}
}

void goBackward(int motorSpeed, int timeActive){
	//Serial.println("Go backward");
	analogWrite(enA, motorSpeed);
	analogWrite(enB, motorSpeed);
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
	delay(timeActive);
	goesForward = false;
}

void turnLeft(int motorSpeed, int timeActive){
	//Serial.println("Turn left");
	analogWrite(enA, motorSpeed - spinOffset);
	analogWrite(enB, motorSpeed + spinOffset);
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	delay(timeActive);
	stopMotors();
}

void turnRight(int motorSpeed, int timeActive){
	//Serial.println("Turn right");
	analogWrite(enA, motorSpeed + spinOffset);
	analogWrite(enB, motorSpeed - spinOffset);
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
	delay(timeActive);
	stopMotors();
}

void lookLeft(){
	lookingForward = false;
	servoMotor.write(160);
}

void lookForward(){
	if(!lookingForward){
		lookingForward = true;
		servoMotor.write(90);
	}
}

void lookRight(){
	lookingForward = false;
	servoMotor.write(20);
}

void playSiren(int siren, int count){
	Serial.print("Play siren: ");
	Serial.println(siren);
	switch(siren){
		case 0: 
			for(int c = 0; c <= count; c++){
				for(i = 600; i > 300; i--){
					digitalWrite(buzzer,HIGH);
					delayMicroseconds(i);
					digitalWrite(buzzer,LOW);
					delayMicroseconds(i);
				}
				for(i = 600; i < 300; i++){
					digitalWrite(buzzer,HIGH);
					delayMicroseconds(i);
					digitalWrite(buzzer,LOW);
					delayMicroseconds(i);
				}
			}
		break;
		
		case 1:
			for(int c = 0; c <= count; c++){
				for( hz = 440; hz < 1400; hz++){
					tone(buzzer, hz+2);
					delay(3); //default 1
				}
				noTone(buzzer);
			}
		break;
		
		case 2: 
			for(int c = 0; c <= count; c++){
				for(int i=100; i> 0; i--){
					digitalWrite(12, HIGH);
					delayMicroseconds(i*10);
					digitalWrite(12, LOW);
					delayMicroseconds(i*10);
				}
				noTone(buzzer);
			}
		break;
		
		case 3: 
			for(int c = 0; c <= count; c++){
				tone(12,400,9);
				delay(10);
				tone(12,600,8);
				delay(10);
				noTone(buzzer);
			}
		break;
		
		case 4:
			for(int c = 0; c <= count; c++){
				for( hz = 1500; hz > 500; hz--){
					tone(12, hz*1);
					delayMicroseconds(1);
				}
				noTone(12);
				delay(10);
			}
		break;
		
		case 5:
			for(int c = 0; c <= count; c++){
				for( hz = 800; hz > 100; hz--){
					tone(buzzer, hz*3);
				}
				for( hz = 100; hz <800; hz++){
					tone(buzzer, hz*3);
				}
				noTone(buzzer);
			}
		break;
		
		case 6:
			for(int c = 0; c <= count; c++){
				for( hz = 1000; hz > 500; hz--){
					tone(buzzer, hz);
				}
				for( hz = 500; hz <1000; hz++){
					tone(buzzer, hz);
				}
				noTone(buzzer);
			}
		break;
		
		case 7: 
			for(int c = 0; c <= count; c++){
				for( hz = 100; hz <800; hz++){
					tone(buzzer, hz*3);
				}
				noTone(buzzer);
			}
		break;
		
		case 8:
			for(int c = 0; c <= count; c++){
				for(hz = 440; hz < 1000; hz++){ //1400
					tone(buzzer, hz+5);
					delay(1); //1
				}
				for(hz; hz < 1100; hz++){ //1500
					tone(buzzer, hz);
					delay(12); //12
				}
				for(hz = 1100; hz > 440; hz--){ //1500
					tone(buzzer, hz);
					delay(4); //4
				}
				noTone(buzzer);
			}
		break;
		
		case 9: //count 5
			for(int c = 0; c <= count; c++){
				for(i = 300; i > 0; i--){
					digitalWrite(buzzer,HIGH);
					delayMicroseconds(i);
					digitalWrite(buzzer,LOW);
					delayMicroseconds(i);
				}
				for(i = 0; i < 300; i++){
					digitalWrite(buzzer,HIGH);
					delayMicroseconds(i);
					digitalWrite(buzzer,LOW);
					delayMicroseconds(i);
				}
			}
		break;
		
		case 10: //crrrrr
			for(int c = 0; c <= count; c++){
				tone(buzzer, 250);
				delay(5);
				noTone(buzzer);
				delay(5);
			}
		break;
	}
	
	noTone(buzzer);
	TCCR2B = TCCR2B & B11111000 | B00000111; // 30Hz
	//Enable pin - reset state
	pinMode(enable, INPUT_PULLUP);
}
