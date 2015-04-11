//primary drive code for our robot
//code moves forward until obstacle deteted, then..
//backs up, looks left and right to find greatest distance
//turns and moves forward.

#include <Servo.h>

#define LEFT 0
#define RIGHT 1
#define SMALLMOVE 1000 //set delay for small movements (ms)

volatile byte runProg = 0; //variable to kill program

//motor pins
int motor1Pin1 = 3; //pin 2 on L293D
int motor1Pin2 = 4; // pin 7 on L293D
int enablePin = 9; // pin 1 on L293D

//ultrasound sensor
int trigPin = 7;
int echoPin = 6;

//IR sensor
int sensorPin = A0;

//servo1 (steering)
Servo srvSteer; // create a servo object for steering

//servo2 (sensor)
Servo srvSensor; //create a servo object for the sensor servo
int pos = 0; //variable to store the servo position

int firstRun=1;

void setup() {
  Serial.begin (38400);
  //setup an interupt pin
  pinMode(2, INPUT);      // Make digital 2 an input
  digitalWrite(2, HIGH);  // Enable pull up resistor   
  attachInterrupt(0,stopgo,FALLING);
  Serial.println("attached");
  // setup the motor pins
  pinMode(motor1Pin1, OUTPUT);  
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // set enablePin high so that motor can turn on:
  digitalWrite(enablePin, HIGH);
  
  //setup the ultrasound sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //setup the IR sensor
  pinMode(sensorPin,INPUT);
  
  //setup the sensor servo
  srvSensor.attach(11); //attach the servo to pin 11
  srvSensor.write(90); //ensure the servo is looking straight ahead

  //setup the steering servo
  srvSteer.attach(10); //attach the servo to pin 10
  srvSteer.write(90); //ensure we start straight
  Serial.println("running");
}
void loop() {
  if(runProg){
    int distance=0;
    motor1Forward();
    distance = getDistance();
    //A sensor value of 300 equates to about 20cm
    if(distance > 300){
      avoidCollision();
    }
  }
  else
  {
    motor1Stop();
  }
}

void stopgo(){
  Serial.println("interrupt fired");
  if(runProg == 0){
    runProg = 1;
  }
  else
  {
    runProg = 0;
    motor1Stop();
  }
}

void motor1Forward() {
  //Drive the main motor forward
  Serial.println("Driving");
  digitalWrite(motor1Pin1, HIGH); // set pin 2 on L293D low
  digitalWrite(motor1Pin2, LOW); // set pin 7 on L293D high
}

void motor1Backward() {
  //Drive the main motor backwards
  digitalWrite(motor1Pin1,LOW); //set pin 2 on L293D high
  digitalWrite(motor1Pin2,HIGH);
}

void motor1Stop() {
  //Stop the main drive motor
  //Set both pins low to stop motor
  digitalWrite(motor1Pin1,LOW);
  digitalWrite(motor1Pin2,LOW);
}

long getDistance_ultra() {
  long duration, distance;
  // send a pulse to the trigger pin
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(5); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5); // Added this line
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  digitalWrite(trigPin, LOW);  
  return distance;
}

int getDistance() {
  //get distance using the IR sensor
  int sensorVal = 0; //store the output
  int nAverage = 10; //number of reads to average
  float distance;
  int i; //counter
  for (i = 0; i < nAverage; i = i + 1) {
    sensorVal = sensorVal + analogRead(sensorPin);
  }
  sensorVal = sensorVal / nAverage;
  delay(100);
  distance = (log(sensorVal) * -44) + 269
  return(sensorVal);
}
  
int look(int direction) {
  //turn servo2 in direction and measure distance
  Serial.println("Looking");
  long distance;
  if(direction == LEFT){
    srvSensor.write(45); //look left
  }
  else
  {
    srvSensor.write(135); //look right
  }
  delay(500);
  distance = getDistance();
  delay(500);
  srvSensor.write(90); // return the sensor to straight
  return(distance);
}

int chooseDirection(){
  Serial.println("Choosing");
  //looks left then right and chooses the direction with the greatest distsnce
  int distL=0;
  int distR=0;
  //Serial.println("Looking Left");
  distL = look(LEFT);
  //Serial.println("Looking Right");
  distR = look(RIGHT);
  
  if(distL < distR){
    return LEFT;
  }
  else
  {
    return RIGHT;
  }
}

void turn(int direction){
  //turn the robot 45 degrees in direction
  Serial.println("turning");
  motor1Stop();
  if(direction == LEFT){
    srvSteer.write(45);
    delay(500);
  }
  else
  {
    srvSteer.write(135);
    delay(500);
  }
  motor1Forward();
  delay(SMALLMOVE);
  motor1Stop();
  srvSteer.write(90); //reset to center
  delay(500);
}

void avoidCollision(){
  //function to handle collision avoidance
  Serial.println("Avoiding collision");
  int direction;
  motor1Stop();
  motor1Backward();
  delay(SMALLMOVE);
  motor1Stop();
  direction = chooseDirection();
  turn(direction);
}

