#include <QTRSensors.h>
#define enB 5 // enB motordriver pin
#define enA 6 // 6 ardiuno pin
#define in1 10
#define in2 9
#define in3 8
#define in4 7

QTRSensors qtr; //qtr sensor class instace

const uint8_t SensorCount = 8; // 8 sensors
uint16_t sensorValues[SensorCount]; //array
boolean mode = LOW; //Low for motor, High for sensor
int sensorFlag = LOW; //Low for Calibration, High for Not Calibration
const byte interruptPin = 2; //for push button
long unsigned int lastPress; // handling multiple presses
int debounceTime = 40;
boolean ledstate = LOW;
unsigned int ledtime = 0;


void SensorCal(){
  // analog type
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount); // built inside the library methods
  qtr.setEmitterPin(3);
  
  delay(500); //ms
  pinMode(LED_BUILTIN, OUTPUT); //changing the mode of pin ---- pin 13 as output
  digitalWrite(LED_BUILTIN, HIGH); // the LED will turn on

  for (uint16_t i=0; i<400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off
  // we need calibration to check which pins have the maximum value...

  Serial.begin(9600);
  for(uint8_t i=0; i<SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  }



void SensorData(){ 
  //getting the sensor data
  uint16_t position = qtr.readLineBlack(sensorValues);  // under which position the line is
  
  for(uint8_t i=0; i<SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // to see the sensor value
  }
  Serial.println(position); 
  
  delay(250);
  if(ledtime>6){
    ledstate=!ledstate; // change the state
    ledtime = 0; // after every 1.5 sec the led will change its state.
    }
  digitalWrite(LED_BUILTIN, ledstate);
  ledtime++; // else increase counter
  }
  
// motor control funtion
void forward(){
  analogWrite(enA,255); //PWM function [8bits] .  .. 0 means 0V, 255 means 5V.
  analogWrite(enB,255);
  
  digitalWrite(in1,HIGH); //motor 1
  digitalWrite(in2,LOW);  //motor 1
 
  digitalWrite(in3,HIGH); //motor 2
  digitalWrite(in4,LOW);  //motor 2
  }

void back(){
  analogWrite(enA,255); //PWM function [8bits] .  .. 0 means 0V, 255 means 5V.
  analogWrite(enB,255);
  
  digitalWrite(in1,LOW); //motor 1
  digitalWrite(in2,HIGH);  //motor 1
 
  digitalWrite(in3,LOW); //motor 2
  digitalWrite(in4,HIGH);  //motor 2
  }  

void right(){
  analogWrite(enA,255); //PWM function [8bits] .  .. 0 means 0V, 255 means 5V.
  analogWrite(enB,255);
  
  digitalWrite(in1,HIGH); //motor 1
  digitalWrite(in2,LOW);  //motor 1
 
  digitalWrite(in3,LOW); //motor 2
  digitalWrite(in4,LOW);  //motor 2
  }

void left(){
  analogWrite(enA,255); //PWM function [8bits] .  .. 0 means 0V, 255 means 5V.
  analogWrite(enB,255);
  
  digitalWrite(in1,LOW); //motor 1
  digitalWrite(in2,LOW);  //motor 1
 
  digitalWrite(in3,HIGH); //motor 2
  digitalWrite(in4,LOW);  //motor 2
  }

void off(){
  analogWrite(enA,0); //PWM function [8bits] .  .. 0 means 0V, 255 means 5V.
  analogWrite(enB,0);
  
  digitalWrite(in1,LOW); //motor 1
  digitalWrite(in2,LOW);  //motor 1
 
  digitalWrite(in3,LOW); //motor 2
  digitalWrite(in4,LOW);  //motor 2
  }  

void motorTest(){
  forward();
  Serial.println("F");
  delay(1000); // 1 sec forward

  back();
  Serial.println("B");
  delay(1000);

  right();
  Serial.println("R");
  delay(1000);

  left();
  Serial.println("L");
  delay(1000);

  off();
  Serial.println("stop");
  delay(1000);
  }




void setup() {
  // put your setup code here, to run once:
  pinMode(5,OUTPUT);  // for motor driver
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP); // inside
  attachInterrupt(digitalPinToInterrupt(interruptPin), changeMode, FALLING);  // tell ardiuno that pin 2 is for interrupt pin ..... conversion......High-->to-->Low
}

void loop() {
  // put your main code here, to run repeatedly:
  if(mode==LOW){
    
    if(Serial.available()>0){  // valid data check
      
        int inbyte = Serial.read(); // whatever we get
        switch(inbyte){
            case '0':
            Serial.println("L");
            left();
            delay(1000);
            break;
            case '1':
            Serial.println("R");
            right();
            delay(1000);
            break;
            case '2':
            Serial.println("U");
            forward();
            delay(1000);
            break;
            case '3':
            Serial.println("D");
            off();
            delay(1000);
            break;
          }
      }
    }

    else if(mode==HIGH){

      if(sensorFlag==LOW){  // sensor calibration .... 1st time
          Serial.println("IR Test");
          SensorCal();
          sensorFlag=HIGH;
        }
        else{
            SensorData(); // unless we turn off it will be always here.
          }
      }
  
}


void changeMode(){

  if((millis()-lastPress) > debounceTime){
      lastPress = millis();
      if(digitalRead(interruptPin) == 0){
          mode=!mode;
        }
      else if(digitalRead(interruptPin) == 1){
          mode=mode;
        }
    }
  }
