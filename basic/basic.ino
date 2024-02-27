#include <QTRSensors.h>

#define kp 0.1
#define kd 2
#define ki 0

#define MaxSpeed 100
#define BaseSpeed 70

#define NUM_SENSORS 8

#define speedturn 50

#define rightMotor1 A1
#define rightMotor2 A2
#define rightMotorPWM 10

#define leftMotor1 A4
#define leftMotor2 A5
#define leftMotorPWM 11

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup(){
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  delay(3000);
    Serial.begin(9600);
    Serial.println();

  int i;
  for(int i=0;i<100;i++){
    if(i<25 || i>=75){
      move(1,70,1);
      move(0,70,0);
    }
    else{
      move(1,70,0);
      move(0,70,1);
    }
    qtr.calibrate();
    delay(20);
  }
  wait();
  delay(3000);
}

int lastError=0;
uint16_t position=qtr.readLineBlack(sensorValues);

void loop(){
  position=qtr.readLineBlack(sensorValues);
  if(position>6500){
    move(1,speedturn,1);
    move(0,speedturn,0);
    return;
  }
  if(position<500){
    move(1,speedturn,0);
    move(0,speedturn,1);
    return;
  }
  int error= position-3500;
  int motorSpeed=kp * error + kd * (error -lastError);
  lastError=error;

  int rightMotorSpeed=BaseSpeed + motorSpeed;
  int leftMotorSpeed= BaseSpeed - motorSpeed;

  if(rightMotorSpeed>MaxSpeed){
    rightMotorSpeed=MaxSpeed;
  }
  if(leftMotorSpeed>MaxSpeed){
    leftMotorSpeed=MaxSpeed;
  }
  if(rightMotorSpeed<0){
    rightMotorSpeed=0;
  }
  if(leftMotorSpeed<0){
    leftMotorSpeed=0;
  }

  move(1,rightMotorSpeed,1);
  move(0,leftMotorSpeed,1);
}

void wait(){
  analogWrite(leftMotorPWM,0);
  analogWrite(rightMotorPWM,0);
}

void move(int motor,int speed,int direction){
  boolean inPin1;
  boolean inPin2;

  if(direction==1){
    inPin1=HIGH;
    inPin2=LOW;
  }
  if(direction==0){
    inPin1=LOW;
    inPin2=HIGH;
  }
  if(motor==0){
    digitalWrite(leftMotor1,inPin1);
    digitalWrite(leftMotor2,inPin2);
    digitalWrite(leftMotorPWM,speed);
  }
  if(motor==1){
    digitalWrite(rightMotor1,inPin1);
    digitalWrite(rightMotor2,inPin2);
    digitalWrite(rightMotorPWM,speed);
  }
}

void sensor_calibrate(){
  for(uint16_t i=0;i<400;i++){
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN,LOW);
  Serial.begin(9600);

  for(uint8_t i=0;i<SensorCount;i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }

  Serial.println();

  for(uint8_t i=0;i<SensorCount;i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
