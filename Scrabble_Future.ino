// SCRABBLE FUTURE by Jeffrey Chow
//
// This code instructs a rotating scrabble board to rotate to the loudest knock it hears.


// Libraries
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *Nema = AFMS.getStepper(200, 1);


// Pin definitions
const int southPiezo = A1;      // piezo 1 connected to analog pin 1
const int northPiezo = A3;      // piezo 2 connected to analog pin 3
const int westPiezo = A2;       // piezo 3 connected to analog pin 2
const int eastPiezo = A0;       // piezo 4 connected to analog pin 0
const int threshold = 300;      // minimum value for a knock
const int homesensor = A5;       // hall effect sensor connected to analog pin 5

// Sensors with changing data
int southReading = 0;     // variable to store the value read on piezo 1
int northReading = 0;     // variable to store the value read on piezo 3
int westReading = 0;      // variable to store the value read on piezo 2
int eastReading = 0;      // variable to store the value read on piezo 0
int homesensorstate = 0;  // variable to store the value read on hall effect sensor

// Defining step sizes
void forwards() {
  Nema->onestep(FORWARD, DOUBLE);
}

void backwards() {
  Nema->onestep(BACKWARD, DOUBLE);
}

AccelStepper myNema(forwards, backwards);     // Run AccelStepper library and use the functions to step



// MAIN CODE

void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(homesensor, INPUT);   // set hall effect sensor as an input

  AFMS.begin();  // create with the default frequency 1.6KHz

  stepperHome();                // run homing sequence stepperHome to find the 0 position
  delay(1000);
}

void loop()
{

  // read piezo sensors:
  southReading = analogRead(southPiezo);
  northReading = analogRead(northPiezo);
  westReading = analogRead(westPiezo);
  eastReading = analogRead(eastPiezo);


  myNema.setAcceleration(100);    // set stepper motor accceleration
  myNema.setSpeed(50);            // set stepper motor speed in RPM

  // Does any of the readings from the piezo sensors exceed the threshold value?
  if ((southReading >= threshold) or (northReading >= threshold) or (westReading >= threshold)
      or (eastReading >= threshold) )  
  {
    // If the reading from the north piezo is the loudest
    if ((northReading > southReading) and (northReading > westReading) and (northReading > eastReading))   
    {
      Serial.println("Going to 0");
      myNema.move(0 - myNema.currentPosition());   // Calculate the number of steps required to move to North
      myNema.runToPosition();                      // Move the motor to the calculated position 
      myNema.setCurrentPosition(0);                // Set current position to North (0)
      Serial.println("At 0");

    }

    else if ((eastReading > southReading) and (eastReading > westReading) and (eastReading > northReading)) {
      Serial.println("Going to 90");
      myNema.move(50 - myNema.currentPosition());
      myNema.runToPosition();
      myNema.setCurrentPosition(50);
      Serial.println("At 90");

    }

    else if ((southReading > northReading) and (southReading > westReading) and (southReading > eastReading)) {
      Serial.println("Going to 180");
      myNema.move(100 - myNema.currentPosition());
      myNema.runToPosition();
      myNema.setCurrentPosition(100);
      Serial.println("At 180");

    }

    else if ((westReading > southReading) and (westReading > northReading) and (westReading > eastReading)) {
      Serial.println("Going to 270");
      myNema.move(-50 - myNema.currentPosition());
      myNema.runToPosition();
      myNema.setCurrentPosition(-50);
      Serial.println("At 270");

    }
  }
}



// MOTOR HOMING SEQUENCE
void stepperHome() {
  homesensorstate = digitalRead(homesensor);    // read hall effect sensor to detect a magnetic field
  myNema.setAcceleration(100);
  myNema.setSpeed(30);
  while (homesensorstate != LOW) {             // continuously step until a magnetic field is detected
    myNema.move(50);
    myNema.run();
    homesensorstate = digitalRead(homesensor);
  }
  myNema.setCurrentPosition(0);              // 0 position is found, print "home" in serial monitor
  Serial.println("Home");
}
