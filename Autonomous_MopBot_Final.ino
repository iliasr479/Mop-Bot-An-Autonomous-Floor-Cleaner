// Libraries
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// sonar
#define trigPinf 3
#define echoPinf 2
#define trigPinl 22
#define echoPinl 23
#define trigPinr 24
#define echoPinr 25

// servo
Servo myservo;
Adafruit_MPU6050 mpu;
float distancef;
float distancel;
float distancer;
float z_angle = 0;
int count = 0;
int corner = 0;

// motors
int ENA = 5;
int ENB = 6;
int spd = 100; // range: 0 to 255
float rot = 80;
int mop_cw = 28;
int mop_ccw = 29;
int mop_speed = 11;
int motorRightA = 7; // Right motor forward
int motorRightB = 8; // Right motor backward
int motorLeftA = 9; // Left motor forward
int motorLeftB = 10; // Left motor backward

void forward() // move forward
{
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftB, LOW);
}


void stp() // stop
{
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftB, LOW);
}


void backward()
{
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorRightB, HIGH);
  digitalWrite(motorLeftB, HIGH);
}


void right() // turn right
{
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorRightB, HIGH);
  digitalWrite(motorLeftB, LOW);
}


void left() // turn left
{
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftB, HIGH);
}

int calcdisf() // measures the distance ahead
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinf, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinf, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinf, LOW);

  duration = pulseIn(echoPinf, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

int calcdisl() // measures the distance in left
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinl, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinl, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinl, LOW);

  duration = pulseIn(echoPinl, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

int calcdisr() // measures the distance in right
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinr, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinr, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinr, LOW);

  duration = pulseIn(echoPinr, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

float angle() // measures the turning angle in degrees
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float Z_rotation;
  static unsigned long prevTime = 0;
  unsigned long currentTime = millis();
  float dt = (float)(currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  /* Print out the values */
  Z_rotation = (g.gyro.z + 0.012) * 180 / 3.1416;

  z_angle = (z_angle + Z_rotation * dt);
  //Serial.print(z_angle);
  //Serial.println(" deg");
  return z_angle;
}

void zigzag() // function for navigating the room
{
  z_angle = 0;
    if (count % 2 == 0) {
      left();
    }
    else {
      right();
    }
    while (abs(angle()) <= rot)
    {
      if (count % 2 == 0) {
        left();
      }
      else {
        right();
      }
      if (abs(angle()) > rot)
        break;
    }
    z_angle = 0;
    stp();
    forward();
    delay(350);
    stp();
    distancef = calcdisf();
    distancel = calcdisl();
    distancer = calcdisr();
    if((distancef<30)&&((distancel<30)||(distancer<30)))
    {
      corner=corner+1;
    }
    if (count % 2 == 0) {
      left();
    }
    else {
      right();
    }
    while (abs(angle()) <= rot)
    {
      if (count % 2 == 0) {
        left();
      }
      else {
        right();
      }
      if (abs(angle()) > rot)
        break;
    }
    z_angle = 0;
    stp();
    forward();
    count = count + 1;
}

 int lookRight() // front sonar turns right and measures distance
{
    myservo.write(72); 
    delay(200);
    int distance = calcdisf();
    delay(100);
    myservo.write(122);
//    Serial.print(distance);
//    Serial.println(" cm"); 
    return distance;
}
int lookLeft()// front sonar turns left and measures distance
{
    myservo.write(172); 
    delay(200);
    int distance = calcdisf();
    delay(100);
    myservo.write(122); 
//    Serial.print(distance);
//    Serial.println(" cm");
    return distance;
    //delay(1000);
}
void mop()
{
  analogWrite(mop_speed, 50);
  digitalWrite(mop_cw, HIGH);
  digitalWrite(mop_ccw, LOW);
}

void stop_mop()
{
  digitalWrite(mop_cw, LOW);
  digitalWrite(mop_ccw, LOW);
}
void setup()
{
  //Serial.begin(9600);
  pinMode(trigPinf, OUTPUT);
  pinMode(echoPinf, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  Serial.begin(115200);
  myservo.attach(4);  
  myservo.write(122);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(100);
    }
  }
  //Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}


void loop()
{
  float distanceR = 0;
  float distanceL =  0;
  distancef = calcdisf();
  Serial.print(distancef);
  Serial.println(" cm (f)");
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
  if(corner<3)
  {
    forward();
    //mop();
  }
  else
  {
    stp();
    //stop_mop();
  }
  if (distancef < 30)
  {
    stp();
    delay(100);
    distancel = calcdisl();
    distancer = calcdisr();
    if((distancel<30)||(distancer<30))
    {
      corner=corner+1;
    }
    distanceR=lookRight();
    delay(200);
    distanceL = lookLeft();
    Serial.print(distanceR);
    Serial.println(" cm (R)");
    Serial.print(distanceL);
    Serial.println(" cm (L)");
    
    delay(200);
    if((distanceR<100)&&(distanceL<100))
    {
      if(corner<3)
      {
        zigzag();
      }
      else
      {
        stp();
      }
    }
    else
    {
      z_angle=0;
      left();
      while (abs(angle()) <= rot)
      {
        left();
        if (abs(angle()) > rot)
          break;
      }
      unsigned long start_time = millis();
      forward();
      delay(100);
      //edit
      while(calcdisr()<20)
      {
        forward();
        if(calcdisr()>20)
          break;
      }
      unsigned long real_time = millis();
      //delay(200);
      z_angle=0;
      right();
      while (abs(angle()) <= rot)
      {
        right();
        if (abs(angle()) > rot)
          break;
      }
      forward();
      delay(500);
     while(calcdisr()<20)
      {
        forward();
        if(calcdisr()>20)
          break;
      }
      //delay(500);
      z_angle=0;
      right();
      while (abs(angle()) <= rot)
      {
        right();
        if (abs(angle()) > rot)
          break;
      }
      //forward();
      //delay(200);
      forward();
      delay(real_time - start_time);
//      while(calcdisr()<20)
//      {
//        forward();
//        if(calcdisr()>20)
//          break;
//      }
      z_angle=0;
      left();
      while (abs(angle()) <= rot)
      {
        left();
        if (abs(angle()) > rot)
          break;
      }
      forward();
    }
  }
  
}
