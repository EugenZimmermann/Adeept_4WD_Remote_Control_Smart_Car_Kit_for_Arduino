/***********************************************************
File name:  SmartCar.cpp

Description:
Remote controlled car based on the Adeept Smart Car Kit and AdeeptMotor.ino

The car allows to be operated in 4 modes:
Mode 1: Remote control mode with joysticks.
Mode 2: Remote control with tilt sensor.
Mode 3: Autonomic mode using the ultrasonic distance sensor(s).
Mode 4: Autonomic line following mode.

Features:
(x) Mode 1 implemented.
( ) Mode 2 implemented.
( ) Mode 3 implemented.
( ) Mode 4 implemented.

( ) All modes should take use of the ultrasonic distance sensor to prevent collision.
( ) PID correction of direction.
( ) Red break lights.
( ) Orange turn lights.
( ) ...

Author: Eugen Zimmermann
Date: 2021/01/09 
***********************************************************/
#include <Arduino.h>
#include <SPI.h>            // 
#include <RF24.h>           // RF communication
#include <Servo.h>          // steering direction (moving Ultrasonic Sensor)

#include <QTRSensors.h>     // QTR Line Follower Sensor
#include <PID_v1.h>         // PID controller for steering correction
#include <NewPing.h>        // Ultrasonic sensor

#include <TaskScheduler.h>  // Task scheduler

/*
 * prototype functions
 ********************************************************/
void comRF();
void handleControl();
void handleDirection();
void sendSerial();
void serialControl();
void controlUltraSonicSensor();
void measureDistance();
void readLineFollowerSensor();

void ctrlCar0(byte dirServoDegree, bool motorDir, int motorSpd); //manual mode
void ctrlCar1(byte dirServoDegree, bool motorDir, int motorSpd); //automatic mode
/*********************************************************/

/*
 * Task scheduler
 ********************************************************/
Scheduler taskManager;

#define updateInterval 100
#define updateIntervalSerial 1000 // not important tasks

Task tDrive(0, TASK_FOREVER, &handleControl, &taskManager, true);       // handle forward and backward movement based on mode
Task tDirection(0, TASK_FOREVER, &handleDirection, &taskManager, true); // handle steering direction based on mode

// sensor and calculations
Task tDistance(updateInterval, TASK_FOREVER, &measureDistance, &taskManager, true);                        // ultrasonic distance sensor
Task tUltraSonicSensor(updateInterval + 20, TASK_FOREVER, &controlUltraSonicSensor, &taskManager, false);  // scanning movement of ultrasonic sensor
Task tLineFollowerSensor(updateInterval + 20, TASK_FOREVER, &readLineFollowerSensor, &taskManager, false); // line follower sensor
// Task tPID

// communication
Task tRF(0, TASK_FOREVER, &comRF, &taskManager, true);                             // remote control
Task tSerial(updateIntervalSerial, TASK_FOREVER, &sendSerial, &taskManager, true); // print all kind of stuff to serial console
Task tSerialControl(updateIntervalSerial/2, TASK_FOREVER, &serialControl, &taskManager, true); // control/debug through serial connection
/*********************************************************/

/*
 * Drive modes and general variables
 ********************************************************/
int mode = 1;
int automatic = 0;      // bool if in automatic mode (obsolete?)

const int buzzerPin = 8; // define pin for buzzer
/*********************************************************/

/*
 * RF24 transmission
 ********************************************************/
RF24 radio(9, 53);           // define the object to control NRF24L01
byte addresses[6] = "00007"; // define communication address which should correspond to remote control

// data[0] = direction Y of joystick U1 (front/back)
// data[1] = direction X of joystick U2 (left/right)
// data[2] = mode (1 manual, 2 xxx, 3 manuel with gyroscope, 4 automatic)
// data[3] = fine tuning joystick U1
// data[4] = fine tuning joystick U2
// data[5] = direction Y of joystick U2 (servo of distance sensor)
// data[6] = data return channel to remote
// data[7] = data return channel to remote
// data[8] = data return channel to remote
int data[9] = {512, 512, 1, 0, 1, 1, 512, 512, 512}; // define array used to save the communication data
/*********************************************************/

/*
 * define variables for serial communication/commands
 ********************************************************/
unsigned char cComIn;
char cCommand;
unsigned long iComandParamter;
bool bIsParameter;
/*********************************************************/

/*
 * Ultrasonic sensor
 ********************************************************/
#define MAX_DIST 150 // define the range(cm) for ultrasonic ranging module, Maximum sensor distance is rated at 400-500cm.

// #define TRIG_PIN 15                 // define Trig pin for ultrasonic ranging module
// #define ECHO_PIN 14                 // define Echo pin for ultrasonic ranging module
// NewPing sonarFront(TRIG_PIN, ECHO_PIN, MAX_DIST); // NewPing setup of pins and maximum distance.
// float distance = -1;                // save the distance away from obstacle
// long minDistance = MAX_DIST;
// int currentAngle = 90;
// int minAngle = 90;
// int nextStep = 5;

#define TRIG_PIN_L 15                                // define Trig pin for ultrasonic ranging module
#define ECHO_PIN_L 14                                // define Echo pin for ultrasonic ranging module
NewPing sonarLeft(TRIG_PIN_L, ECHO_PIN_L, MAX_DIST); // NewPing setup of pins and maximum distance to left side.
float distanceLeft = -1;                             // save the distance away from obstacle
float distanceLeftLast = -1;
long minDistanceLeft = MAX_DIST;
bool approachLeft = false;

#define TRIG_PIN_R 17                                 // define Trig pin for ultrasonic ranging module
#define ECHO_PIN_R 16                                 // define Echo pin for ultrasonic ranging module
NewPing sonarRight(TRIG_PIN_R, ECHO_PIN_R, MAX_DIST); // NewPing setup of pins and maximum distance to right side.
float distanceRight = -1;                             // save the distance away from obstacle
float distanceRightLast = -1;
long minDistanceRight = MAX_DIST;
bool approachRight = false;
/*********************************************************/

/*
 * Line Follower Sensor
 ********************************************************/
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

bool calibratedQTR = false;     // shows if calibration was done after switching mode
/*********************************************************/

/*
 * PID for steering direction
 ********************************************************/
// create variables for
double directionServoSetpoint, directionMissmatch, directionCorrection;

// define aggressive and conservative PID parameters
double aggKp = 1.5, aggKi = 50, aggKd = 6;
double consKp = 2, consKi = 5, consKd = 1;

// create PID objects
PID directionPID(&directionMissmatch, &directionCorrection, &directionServoSetpoint, consKp, consKi, consKd, DIRECT);
/*********************************************************/

/*
 * Servos for direction and ultrasonic sensor
 ********************************************************/
Servo directionServo;                   // define servo to control turning of smart car
const int directionServoPin = 2;        // define pin for signal line of the last servo
int directionServoDegree = 78;          //default: 78
int directionServoOffset = 0;           // define a variable for deviation(degree) of the servo

Servo ultrasonicServo;              // define servo to control turning of ultrasonic sensor
int ultrasonicPin = 3;              // define pin for signal line of the last servo
int ultrasonicServoDegree = 78;
int ultrasonicServoOffset = 0;     // define a variable for deviation(degree) of the servo
/*********************************************************/

/*
 * motor control
 ********************************************************/
// define moving directions
#define FORWARD LOW
#define BACKWARD HIGH
int motorSpeed = 0;
bool motorDirection = FORWARD;

const int dirAPin = 7;    // define pin used to control rotational direction of motor A
const int pwmAPin = 6;    // define pin for PWM used to control rotational speed of motor A
const int dirBPin = 4;    // define pin used to control rotational direction of motor B
const int pwmBPin = 5;    // define pin for PWM used to control rotational speed of motor B

// const int snsAPin = 0;    // define pin for detecting current of motor A
// const int snsBPin = 1;    // define pin for detecting current of motor B
/*********************************************************/

/*
 * LED control
 ********************************************************/
const int RPin = A3;
const int GPin = A4;
const int BPin = A5;
/*********************************************************/

void setup()
{
    Serial.begin(9600); // initialize serial port
    delay(500);

    radio.begin();                 // initialize RF24
    radio.setRetries(0, 5);        // set retries times
    radio.setPALevel(RF24_PA_LOW); // set power
    // radio.openWritingPipe(addresses[1]);    // open delivery channel
    // radio.openReadingPipe(1,addresses[0]);
    radio.openReadingPipe(1, addresses); // open delivery channel
    radio.startListening();              // start monitoring

    directionServo.attach(directionServoPin); // attaches the servo on servoDirPin to the servo object
    directionServo.write(directionServoDegree + directionServoOffset);

    ultrasonicServo.attach(ultrasonicPin); // attaches the servo on ultrasonicPin to the servo object
    ultrasonicServo.write(ultrasonicServoDegree + ultrasonicServoOffset);

    pinMode(dirAPin, OUTPUT); // set dirAPin to output mode
    pinMode(pwmAPin, OUTPUT); // set pwmAPin to output mode
    pinMode(dirBPin, OUTPUT); // set dirBPin to output mode
    pinMode(pwmBPin, OUTPUT); // set pwmBPin to output mode

    pinMode(buzzerPin, OUTPUT); // set buzzerPin to output mode

    pinMode(RPin, OUTPUT); // set RPin to output mode
    pinMode(GPin, OUTPUT); // set GPin to output mode
    pinMode(BPin, OUTPUT); // set BPin to output mode

    analogWrite(GPin, 0);

    // configure PID controller for autonomous mode
    directionServoSetpoint = 512.0;
    directionPID.SetOutputLimits(0, 1024);
    directionPID.SetMode(AUTOMATIC);

    // configure the line follower sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
    qtr.setEmitterPin(48);

    Serial.println("Car booted");
}

void loop()
{
    taskManager.execute();
}

void calibrateLineFollowerSensor()
{
    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
    // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    Serial.println("Calibrate line follower sensor");
    for (uint16_t i = 0; i < 400; i++)
    {
        qtr.calibrate();
        if (i % 80 - 40 > 0)
        {
            analogWrite(RPin, 0);
            analogWrite(BPin, 255);
        }
        else
        {
            analogWrite(RPin, 255);
            analogWrite(BPin, 0);
        }
    }

    // print the calibration minimum values measured when emitters were on
    Serial.print("Sensor minimum: ");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    Serial.print("Sensor maximum: ");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
}

void readLineFollowerSensor()
{
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensorValues);

    directionServoSetpoint = map(position, 0, 5000, 1024, 0);

    // // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // // reflectance and 1000 means minimum reflectance, followed by the line
    // // position
    // for (uint8_t i = 0; i < SensorCount; i++)
    // {
    //     Serial.print(sensorValues[i]);
    //     Serial.print('\t');
    // }
    Serial.println("Postition: " + position);
}

void controlUltraSonicSensor()
{
    // if (currentAngle > 135)
    // {
    //     currentAngle = 135;
    //     nextStep = -5;
    // }

    // if(currentAngle < 45)
    // {
    //     currentAngle = 45;
    //     nextStep = 5;
    // }

    // currentAngle += nextStep;

    // // set next step
    // ultrasonicServo.write(currentAngle + ultrasonicServoOffset);

    // // int minIndex = getMin(&minDistance, sizeAngles);

    // // minDistance = minDistances[minIndex];
    // // minAngle = angles[minIndex];
}

void sendSerial()
{
    Serial.print("Radio [0]: ");
    Serial.print(data[0]);
    Serial.print("; [1]: ");
    Serial.print(data[1]);
    Serial.print("; [2]: ");
    Serial.print(data[2]);
    Serial.print("; [3]: ");
    Serial.print(data[3]);
    Serial.print("; [4]: ");
    Serial.print(data[4]);
    Serial.print("; [5]: ");
    Serial.print(data[5]);
    Serial.print("; [6]: ");
    Serial.print(data[6]);
    Serial.print("; [7]: ");
    Serial.print(data[7]);
    Serial.print("; [8]: ");
    Serial.println(data[8]);

    // Serial.print("Mode (RAW): " + String(mode));

    // Serial.println("UltraSonicSensor: LastMeasuredAngle: " + String(currentAngle) + " - NextStep: " + String(nextStep) + " - Distance: " + String(distance) + " - MinDistance: " + String(minDistance) + " - MinAngle: " + String(minAngle));
}

void serialControl(){
    // check for incomming commands
    while (Serial.available()) {
        cComIn = Serial.read();
        if (cComIn >= '0' && cComIn <='9')
        {
            iComandParamter = 10 * iComandParamter + (cComIn -'0');
            bIsParameter = true;
        }
        else if (cComIn == 13){
            switch (cCommand){
                case 'U':
                case 'u':
                    if (bIsParameter) {
                        if (iComandParamter >= 0 && iComandParamter<=1023){
                            ultrasonicServoDegree = iComandParamter;
                        }
                    }
                    break;
                case 'O':
                case 'o':
                    if (bIsParameter) {
                        if (iComandParamter >=0 && iComandParamter<=100) {
                            ultrasonicServoOffset = iComandParamter;
                        }
                    }
                    break;
                case 'P':
                case 'p':
                    // if (bIsParameter) {
                    //     if (iComandParamter == 0) {
                    //         pumpActive = false;
                    //         Serial.println("pump inactive");
                    //     }
                    //     else if (iComandParamter == 1) {
                    //         pumpActive = true;
                    //         Serial.println("pump active");
                    //     }
                    // }
                    // ETout.sendData(); 
                    break;
                case 'S':
                case 's':
                    break;
                case 'M':
                case 'm':
                    if (bIsParameter) {
                        switch (iComandParamter){
                            case 0:
                            case 1:
                                mode = iComandParamter;
                                Serial.println("Mode: " + String(mode));
                                // radio.stopListening();
                                // radio.write(mode, sizeof(mode));
                                // radio.startListening();
                        }
                    }
                    break;
                case 'D':
                case 'd':
                    measureDistance();
                    // Serial.println("Distance: " + String(distance));
                    break;
                case 'T':
                case 't':
                    break;
                case 'R':
                case 'r':
                    break;
                case 'L':
                case 'l':
                    if (bIsParameter) {
                        switch (iComandParamter){
                            case 1:
                                Serial.println("LED: red");
                                analogWrite(RPin, 255);
                                analogWrite(GPin, 0);
                                analogWrite(BPin, 0);
                                break;
                            case 2:
                                Serial.println("LED: green");
                                analogWrite(RPin, 0);
                                analogWrite(GPin, 255);
                                analogWrite(BPin, 0);
                                break;
                            case 3:
                                Serial.println("LED: blue");
                                analogWrite(RPin, 0);
                                analogWrite(GPin, 0);
                                analogWrite(BPin, 255);
                                break;
                            case 4:
                                Serial.println("LED: orange");
                                analogWrite(RPin, 255);
                                analogWrite(GPin, 165);
                                analogWrite(BPin, 0);
                                break;
                            case 0:
                                Serial.println("LED: off");
                                analogWrite(RPin, 0);
                                analogWrite(GPin, 0);
                                analogWrite(BPin, 0);
                                break;
                            default:
                                Serial.println("LED: white");
                                analogWrite(RPin, 255);
                                analogWrite(GPin, 255);
                                analogWrite(BPin, 255);
                                break;
                        }
                    }
            }
        }
        else {
            cCommand = cComIn;
            iComandParamter = 0; 
            bIsParameter = false;
        }
        ultrasonicServo.write(ultrasonicServoDegree + ultrasonicServoOffset);
    }
}

void handleControl() {
    if (!automatic)
    {
        if(tUltraSonicSensor.isEnabled())
        {
            tUltraSonicSensor.disable();
        }

        // calculate the steering angle of servo according to the direction joystick of remote control and the deviation
        directionServoDegree = map(data[0], 0, 1023, 135, 45) - (data[3] - 512) / 25;
        switch (mode)
        {
            case 1:
                ultrasonicServoDegree = map(data[5], 0, 1023, 135, 45) - (data[4] - 512) / 25;
                break;
            default:
                ultrasonicServoDegree = 90;
                break;
        }

        ultrasonicServo.write(ultrasonicServoDegree + ultrasonicServoOffset);

        // control the steering and travelling of the smart car
        // if (abs(motorSpeed)>30){
            if (motorDirection == FORWARD && (distanceLeft < 10 || distanceRight < 10))
                ctrlCar0(directionServoDegree, motorDirection, 0);
            else if (motorDirection == FORWARD && (distanceLeft < 30 || distanceRight < 30))
                ctrlCar0(directionServoDegree, motorDirection, min(motorSpeed, 90));
            else
                ctrlCar0(directionServoDegree, motorDirection, motorSpeed);
        // }
    }
    else {
        // tUltraSonicSensor.enableIfNot();

        // // According to the result of scanning control action of intelligent vehicles
        // if (minDistance < 20) {     // if the obstacle distance is too close, reverse the travelling direction
        //     if (minAngle < 90)       // choose to reverse direction according to the angle with obstacle
        //         ctrlCar1(135, BACKWARD, motorSpeed); // control steering and reversing smart car
        //     else
        //         ctrlCar1(45, BACKWARD, motorSpeed); // control steering and reversing smart car

        //     minDistance = MAX_DIST;
        // }
        // else if (minDistance < 50) {// if the obstacle distance is too close, reverse the travelling direction
        //     if (minAngle < 90)       // choose to reverse direction according to the angle with obstacle
        //         ctrlCar1(135, FORWARD, motorSpeed); // control steering and moving on
        //     else
        //         ctrlCar1(45, FORWARD, motorSpeed); // control steering and moving on

        //     minDistance = MAX_DIST;
        // }
        // else {                     // if the obstacle distance is not close, move on
        //     ctrlCar1(90, FORWARD, motorSpeed); // control the smart car move on
        // }
        // // ctrlCar1(90, FORWARD, 0); // make the smart car stop for preparation of next scanning
    }
}

void handleDirection()
{
    
}

void comRF(){
    // read all the data
    while (radio.available())
    {
        radio.read(data, sizeof(data)); // read data
    }

    if (data[2])
    {
        mode = data[2];
        // get the speed
        motorSpeed = data[1] - 512;
        motorDirection = motorSpeed > 0 ? BACKWARD : FORWARD;

        motorSpeed = constrain(motorSpeed, -511, 511);
        motorSpeed = map(motorSpeed * motorSpeed, 0, 262144, 0, 255);
        // motorSpeed = map(motorSpeed, 0, 512, 0, 255);

        switch (mode)
        {
        case 0:
            digitalWrite(RPin, HIGH);
            digitalWrite(GPin, HIGH);
            digitalWrite(BPin, HIGH);
            break;
        case 1:
            digitalWrite(RPin, LOW);
            digitalWrite(GPin, LOW);
            digitalWrite(BPin, LOW);
            break;
        case 2:
            digitalWrite(RPin, LOW);
            digitalWrite(GPin, HIGH);
            digitalWrite(BPin, HIGH);
            break;
        case 3:
            digitalWrite(RPin, HIGH);
            digitalWrite(GPin, LOW);
            digitalWrite(BPin, HIGH);
            break;
        case 4:
            digitalWrite(RPin, HIGH);
            digitalWrite(GPin, HIGH);
            digitalWrite(BPin, LOW);
            motorSpeed = 120;
            break;
        default:
            break;
        }

        automatic = mode > 3;
    }

    if (data[5] < 100) // control the buzzer
    {
        // tone(buzzerPin, 330);
    }
    else
    {
        noTone(buzzerPin);
    }
}

void ctrlCar0(byte dirServoDegree, bool motorDir, int motorSpd) {
    int trimDir = (data[7] - 512) / 25;
    directionServo.write(dirServoDegree + directionServoOffset);
    digitalWrite(dirAPin, motorDir);
    digitalWrite(dirBPin, motorDir);

    if ((dirServoDegree + trimDir) > 90) { // turning left
        int redux = (dirServoDegree + trimDir - 90) * 0.35; // multiplier is a magic number
        analogWrite(pwmAPin, motorSpd);
        analogWrite(pwmBPin, motorSpd * (45 - redux) / 45);
    }
    if ((dirServoDegree + trimDir) < 90) { // turning right
        int redux = (90 - trimDir - dirServoDegree) * 0.35; // multiplier is a magic number
        analogWrite(pwmAPin, motorSpd * (45 - redux) / 45);
        analogWrite(pwmBPin, motorSpd);
    }
    if ((dirServoDegree + trimDir) == 90) {
        analogWrite(pwmAPin, motorSpd);
        analogWrite(pwmBPin, motorSpd);
    }
}

void ctrlCar1(byte dirServoDegree, bool motorDir, int motorSpd) {
    directionServo.write(dirServoDegree + directionServoOffset);
    digitalWrite(dirAPin, motorDir);
    digitalWrite(dirBPin, motorDir);
    // analogWrite(pwmAPin, motorSpd);
    // analogWrite(pwmBPin, motorSpd);
}

void measureDistance()
{
    // unsigned long distanceTemp = sonarFront.ping_cm();
    // if ((distanceTemp > 0) & (distanceTemp < MAX_DIST))
    // {
        // if (currentAngle < 80)
        // {
        //     minDistanceLeft = distanceTemp;
        // }
        // else if (currentAngle > 100)
        // {
        //     minDistanceRight = distanceTemp;
        // }
        // else
        // {
        //     distance = distanceTemp;
        // }
    // }

    unsigned long distanceTempLeft = sonarLeft.ping_cm();
    unsigned long distanceTempRight = sonarRight.ping_cm();

    if ((distanceTempLeft > 0) & (distanceTempLeft < MAX_DIST))
    {
        distanceLeftLast = distanceLeft;
        distanceLeft = distanceTempLeft;
        approachLeft = distanceLeftLast - distanceLeft - 0.2 > 0;
    }

    if ((distanceTempRight > 0) & (distanceTempRight < MAX_DIST))
    {
        distanceRightLast = distanceRight;
        distanceRight = distanceTempRight;
        approachRight = distanceRightLast - distanceRight - 0.2 > 0;
    }
}

void checkPIDCorrection()
{
    // check if aggressive of conservative PID parameters are necessary
    double gapP1 = abs(directionServoSetpoint - 512);
    if (gapP1 < 100)
    { //we're close to setpoint, use conservative tuning parameters
        directionPID.SetTunings(consKp, consKi, consKd);
    }
    else
    { //we're far from setpoint, use aggressive tuning parameters
        directionPID.SetTunings(aggKp, aggKi, aggKd);
    }
    directionPID.Compute();
    // setPowerP1(BeakerOutputP1);
}