
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

const int griperLimitButtonCPin1 = 52;     // the number of the pushbutton pin
const int griperLimitButtonOPin2 = 46;      // the number of the LED pin


#define echoPin 48 // attach pin D48 Arduino to pin Echo of HC-SR04
#define trigPin 50 //attach pin D50 Arduino to pin Trig of HC-SR04

int motorRight1 = 2;  //motor right to pwm pin 2
int motorRight2 = 53; // motor right digital pin 53
int motorRight3 = 51; // mootoe right digital pin 51

int motorLeft1 = 3;   //motor left to pwm 3
int motorLeft2 = 49;  // motor left input to digital 49
int motorLeft3 = 47;  // motor right input to digital 47

int motorGrip1 = 4;
int motorGrip2 = 45;
int motorGrip3 = 43;

int i, n;
int previousTime;
float lastError;
////////////////////////////
float  Kp = 20;
float Ki = 10;
float  Kd = 100;
unsigned int sensorData[6];


int leftMBSpeed = 15
int rightMBSpeed =  15

int minMspeed =  -50
int maxMspeed = 50

int leftMotorSpeed;
int rightMotorSpeed;

//////////////////////////////////////////
int t = 250;
int sped = 125;

int pos = 150;    // variable to store the servo
///////////////////////////////////////

int buttonGripStateClose = 0;
int buttonGripStateOpen = 0;


int sensorValueLeft2 = 0;
int sensorValueLeft1 = 0;
int sensorValueRight1 = 0;
int sensorValueRight2 = 0;
int sensorValueFront1 = 0;
int sensorValueBack1 = 0;

void servo_up() ;
void read_sensor_func();
double weightAverage (void);
double PID (double error);

bool state = true;
int cm = 0;
/////////////////////////////////////////////
void setup() {
  // Serial.begin(9600);

  myservo.attach(6); //servo pin


  pinMode(motorGrip1, OUTPUT);
  pinMode(motorGrip2, OUTPUT);
  pinMode(motorGrip3, OUTPUT);

  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  pinMode(motorRight3, OUTPUT);
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorLeft3, OUTPUT);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  // griper limit switch
  pinMode(griperLimitButtonCPin1, INPUT);
  pinMode(griperLimitButtonOPin2, INPUT);

  //stop all motors

  digitalWrite(motorGrip1, LOW);
  digitalWrite(motorGrip2, LOW);
  digitalWrite(motorGrip3, LOW);

  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, LOW);
  digitalWrite(motorRight3, LOW);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorLeft3, LOW);

  ///////////////////////////////////////////////
  buttonGripStateClose = digitalRead(griperLimitButtonCPin1);
  buttonGripStateOpen = digitalRead(griperLimitButtonOPin2);

  servo_up();
  //openGrip();
}

void loop() {
  //cm = getDistance();
  //read_sensor_func ();
  //serialPrint_sensor();

  // servo_up();
  // servo_down();

  // put your main code here, to run repeatedly:
  //  buttonGripStateClose = digitalRead(griperLimitButtonCPin1);
  //  buttonGripStateOpen = digitalRead(griperLimitButtonOPin2);
  //
  //
  //
  //  pickDown();

  //  pickUp();

  //speedRight(120);
  //speedLeft(120);
  //speedBoth(100);

  //forwardRight();
  //forwardLeft();
  //backwardRight();
  //backwardLeft();
  //forward();
  //cm = getDistance();

  //delay();
  //maincontrolLogic();
  //maincontrolLogic1();
  //maincontrolLogic2();

  //cm = getDistance();
  //
  //if(cm<8){
  //  speedBoth(0);
  //  stopn();
  //  pickUp();
  //  }


  maincontrolLogic();
  maincontrolLogic();
  maincontrolLogic();
  maincontrolLogic();
  maincontrolLogic();
  maincontrolLogic();
}


void maincontrolLogic() {

  read_sensor_func();


  double error = weightAverage();
  int output = PID(error);

  leftMotorSpeed = leftMBSpeed + output;
  rightMotorSpeed = rightMBSpeed - output;

  int condition;

  if (leftMotorSpeed > 0 && rightMotorSpeed > 0)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxMspeed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxMspeed);

    condition = 1;
  }
  else if (leftMotorSpeed < 0 && rightMotorSpeed > 0)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, minMspeed, 0);
    leftMotorSpeed = leftMotorSpeed * -1;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxMspeed);

    condition = 2;
  }
  else if (leftMotorSpeed > 0 && rightMotorSpeed < 0)
  {
    rightMotorSpeed = constrain(rightMotorSpeed, minMspeed, 0);
    rightMotorSpeed = rightMotorSpeed * -1;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxMspeed);



    if (sensorValueLeft2 < t && sensorValueLeft1 < t && sensorValueRight1 < t && sensorValueRight2 < t && sensorValueFront1 < t && sensorValueBack1 < t) {
      speedBoth(0);
      stopn();
    }
    else if ( sensorValueLeft1 > t && sensorValueRight1 > t && sensorValueFront1 > t && sensorValueBack1 > t) {
      speedBoth(sped);
      forward();

    }

    else if (sensorValueLeft1 > t && sensorValueRight1 < t) {
      speedRight(rightMotorSpeed);
      forwardRight();
      speedLeft(leftMotorSpeed);
      backwardLeft();

    }
    else if (sensorValueLeft1 < t && sensorValueRight1 > t) {
      speedLeft(leftMotorSpeed);
      forwardLeft();
      speedRight(rightMotorSpeed);
      backwardRight();

    }

    else if (sensorValueLeft2 > t) {
      speedRight(rightMotorSpeed);
      forwardRight();
      speedLeft(leftMotorSpeed);
      backwardLeft();

    }
    else if (sensorValueRight2 > t) {
      speedLeft(leftMotorSpeed);
      forwardLeft();
      speedRight(rightMotorSpeed);
      backwardRight();

    }

    else {

    }
  }
}

double PID (double error)
{
  double errorSum;
  double differError;

  errorSum += error; // calculatr sum of error

  float currentError = weightAverage();
  int currentTime = millis();
  differError = (error - lastError) / (currentTime - previousTime); // calculate rate of change of error

  lastError = error;
  previousTime = currentTime;

  if (abs(error) > 1) // prevent too much error is carry forward over course correction
  {
    errorSum = 0;
  }

  float proportional = error * Kp;  // Calculate the components of the PID

  float integral = errorSum * Ki;

  float differential = differError * Kd;

  double output = proportional + integral + differential;  // Calculate the result

  return output;
}


double weightAverage (void)
{
  double weightedSum;
  double dataSum;
  weightedSum = 800;
  dataSum = 0;

  for (n = 0; n < 6; n++)
  {
    dataSum += sensorData[n];
  }
  // no error then value should be zero 
double wAerror  =0;

  if (weightedSum>800){
     wAerror  = (dataSum - weightedSum);
    }
  else{
     wAerror  = (weightedSum-dataSum);
    }
  

  return (wAerro);
}

/////////////////////////////////
void pickDown() {
  servo_down();
  openGrip();
  servo_up();
}

void pickUp() {
  openGrip();
  servo_down();
  closeGrip();
  servo_up();
}

//////////////////////////////

void closeGrip() {
  while (!buttonGripStateClose) {
    closeGripM();
    speedGripM(120);
    buttonGripStateClose = digitalRead(griperLimitButtonCPin1);
    //  Serial.print("GripStateClose = ");
    //  Serial.print(buttonGripStateClose);
    //  Serial.print("\t");
    //  Serial.print("GripStateOpen` = ");
    //  Serial.print(buttonGripStateOpen);
    //  Serial.print("\n");
  }

  speedGripM(0);
  stopGripM();
}

void openGrip() {
  while (!buttonGripStateOpen) {
    openGripM();
    speedGripM(120);
    buttonGripStateOpen = digitalRead(griperLimitButtonOPin2);
    // Serial.print("GripStateClose = ");
    //Serial.print(buttonGripStateClose);
    //Serial.print("\t");
    //Serial.print("GripStateOpen` = ");
    //Serial.print(buttonGripStateOpen);
    // Serial.print("\n");
  }

  speedGripM(0);
  stopGripM();

}



///////////////////////////////////////
void speedGripM(int speedValue) {
  analogWrite(motorGrip1, speedValue);
}

void closeGripM() {
  digitalWrite(motorGrip2, HIGH);
  digitalWrite(motorGrip3, LOW);
}
void openGripM() {
  digitalWrite(motorGrip2, LOW);
  digitalWrite(motorGrip3, HIGH);
}

void stopGripM() {
  digitalWrite(motorGrip2, LOW);
  digitalWrite(motorGrip3, LOW);
}

///////////////////////////////////////


void speedRight(int speedValue) {
  analogWrite(motorRight1, speedValue);
}

void speedLeft(int speedValue) {
  analogWrite(motorLeft1, speedValue);
}

void speedBoth(int speedValue) {
  speedRight(speedValue);
  speedLeft(speedValue);
}

void forwardRight() {
  digitalWrite(motorRight2, HIGH);
  digitalWrite(motorRight3, LOW);
}

void forwardLeft() {
  digitalWrite(motorLeft2, HIGH);
  digitalWrite(motorLeft3, LOW);
}

void backwardRight() {
  digitalWrite(motorRight2, LOW);
  digitalWrite(motorRight3, HIGH);
}

void backwardLeft() {
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorLeft3, HIGH);
}

void forward() {
  forwardRight();
  forwardLeft();
}

void backward() {
  backwardRight();
  backwardLeft();
}

void stopn() {
  digitalWrite(motorRight2, LOW);
  digitalWrite(motorRight3, LOW);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorLeft3, LOW);
  speedBoth(0);
}


void servo_up() {

  for (pos = 20; pos <= 180; pos += 3) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                        // waits 15ms for the servo to reach the position
  }
}

void servo_down() {

  for (pos = 180; pos >= 20; pos -= 3) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                        // waits 15ms for the servo to reach the position
  }
}


int getDistance() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  int dcm = (duration / 2) / 29.1;   // Divide by 29.1 or multiply by 0.0343
  //inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
  return dcm;
}

void read_sensor_func () {
  sensorValueLeft2  = analogRead(A7);
  sensorValueLeft1  = analogRead(A6);
  sensorValueRight1  = analogRead(A5);
  sensorValueRight2  = analogRead(A4);
  sensorValueFront1  = analogRead(A3);
  sensorValueBack1  = analogRead(A2); \

  sensorData[1]  = analogRead(A7);
  sensorData[2]  = analogRead(A6);
  sensorData[3]  = analogRead(A5);
  sensorData[4]  = analogRead(A4);
  sensorData[5]  = analogRead(A3);
  sensorData[6]  = analogRead(A2);

}

void serialPrint_sensor() {
  Serial.print("--Left-2--");
  Serial.print(sensorValueLeft2 );
  Serial.print("--Left-1--");
  Serial.print(sensorValueLeft1 );
  Serial.print("--Right-1--");
  Serial.print(sensorValueRight1 );
  Serial.print("--Right-2--");
  Serial.print(sensorValueRight2 );
  Serial.print("--Front-1--");
  Serial.print(sensorValueFront1 );
  Serial.print("--Back-1--");
  Serial.print(sensorValueBack1 );
  Serial.print("--Distance--");
  Serial.println(cm);
}
