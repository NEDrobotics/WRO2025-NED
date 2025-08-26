#include <Servo.h>
Servo servo1;
Servo servo2;

#define TRIG1_PIN 37
#define ECHO1_PIN 36

const int S1PIN = 10;
const int S2PIN = 11;

// Motor Driver pins for Motor 1
const int IN1_1 = 22;
const int IN2_1 = 23;
const int ENA_1 = 8;

// Motor Driver pins for Motor 2
const int IN1_2 = 24;
const int IN2_2 = 25;
const int ENA_2 = 9;

const int s1mid = 99;
const int s2mid = 86;

const int targetdistance = 15;
const int targetmargin = 3;
void setup() {

  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  
  servo1.attach(S1PIN);
  servo2.attach(S2PIN);
  // Initialize motor pins
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(ENA_1, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(ENA_2, OUTPUT);

  Serial.begin(9600);
}
int olddegree;
void loop() {

  long duration1, duration2, duration3;
  float distance1, distance2, distance3;
  int degree;
  digitalWrite(TRIG1_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG1_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1_PIN, LOW);
  duration1 = pulseIn(ECHO1_PIN, HIGH);
  distance1 = duration1 * 0.0343 / 2;
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.println(" cm");
  
  // BOTH MOTORS: FORWARD 3 seconds
  Serial.println("Motors forward");
  setMotors(true, 255);   // forward at speed 200 (0-255)
  //servo1.write(s1mid); // + left
  //servo2.write(s2mid); // - left
  if(distance1 < (targetdistance-targetmargin)){
    degree = abs(targetdistance - distance1)*2;
    degree = min(degree, 20);
    if(abs(degree-olddegree) > 3){
      servo1.write(s1mid-degree);
      servo2.write(s2mid+degree);
      olddegree = degree;
      Serial.println(degree);
    }
  }
  else if(distance1 > (targetdistance+targetmargin)){
    degree = abs(targetdistance - distance1)*2;
    degree = min(degree, 20);
    if(abs(degree-olddegree) > 3){
      servo1.write(s1mid+degree);
      servo2.write(s2mid-degree);
      olddegree = degree;
    }
  }
  else{
    olddegree = 0;
    servo1.write(s1mid);
    servo2.write(s2mid);
  }
  delay(50);
}

// Control both motors: direction and speed
// forward = true for forward, false for backward
void setMotors(bool forward, int speed) {
  // Motor 1
  digitalWrite(IN1_1, forward ? HIGH : LOW);
  digitalWrite(IN2_1, forward ? LOW  : HIGH);
  analogWrite(ENA_1, speed);
  // Motor 2
  digitalWrite(IN1_2, forward ? LOW : HIGH);
  digitalWrite(IN2_2, forward ? HIGH  : LOW);
  analogWrite(ENA_2, speed);
}
