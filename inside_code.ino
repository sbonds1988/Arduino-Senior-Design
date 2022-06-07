#include <SoftwareSerial.h>           //for ZigBees
#include <OneWire.h>                  //from temperature code
#include <DallasTemperature.h>        //from temperature code
#include "MS5837.h"                   //from pressure code
#include <Wire.h>                     //from pressure code

#define sensorPin A0                  //sonar sensor pin
#define ONE_WIRE_BUS 8                //temperature sensor pin

MS5837 sensor;                        //from pressure code

SoftwareSerial mySerial {0,1};        //for ZigBees (11,12 for uno; 0,1 for nano)
OneWire oneWire(ONE_WIRE_BUS);        //from temperature code
DallasTemperature sensors(&oneWire);  //from temperature code

int pin = 0;                          //pin number of SMA switch
int power = 255;                      //for swim function/SMA 5-step power function
int distance = 0;                     //for sonar sensor function
int signalPin = 9;                    //signal MOSFET pin
int powerPin = A2;                    //battery MOSFET pin

bool started = false;                 //true: Message is started
bool ended  = false;                  //true: Message is finished

char incomingByte;                    //variable to store the incoming byte
char msg[3];                          //message from ZigBee

const int SWIM = 1;                   //number received from outside ZigBee
const int SENSOR = 2;                 //number received from outside ZigBee
const int TMP = 3;                    //number received from outside ZigBee
const int SR = 4;                     //number received from outside ZigBee
const int PRS = 5;                    //number received from outside ZigBee

byte index;                           //index of array

float temp;                           //temperature sensor reading transmitted
float pressure;                       //storeSonar function value
float pressure1;                      //storePressure function value
float sonar;                          //sonar (distance) reading transmitted
float dp;                             //depth reading transmitted

void setup() {
  Serial.begin(38400);                //baud rate must be the same as is on ZigBee module
  pinMode(powerPin, OUTPUT);          //SMA power switch
  pinMode(signalPin, OUTPUT);         //SMA signal switch
  mySerial.begin(38400);              //for ZigBee
  pinMode(pin, OUTPUT);               //for SMA swim function
  digitalWrite(signalPin, LOW);        //added 11/17 after Polk meeting

  sensors.begin();                        //for temperature sensor
  Wire.begin();                           //from pressure code
  sensor.setModel(MS5837::MS5837_02BA);   //from pressure code
  sensor.init();                          //from pressure code (enter pin number)
  sensor.setFluidDensity(997);            //from pressure code, kg/m^3
                                          //997 freshwater, 1029 seawater
}

void loop() {
  while (mySerial.available() > 0) {
    incomingByte = mySerial.read();   //incoming message
    if (incomingByte == '<')          //starts message
    {
      started = true;
      index = 0;
      msg[index] = '\0';              //discard incomplete packet
    }
    else if (incomingByte == '>')     //ends message
    {
      ended = true;                   //exit while loop
    }
    else
    {
      if (index < 4)                  //room to read message
      {
        msg[index] = incomingByte;    //add char to array
        index++;
        msg[index] = '\0';            //add NULL to end
      }
    }
  }

  if (started && ended)     //message is complete
  {
    int value = atoi(msg);  //read input
    Serial.println(value);  //only for debugging
    index = 0;
    msg[index] = '\0';
    started = false;
    ended = false;

    if (value == SWIM) {
      //digitalWrite(powerPin, HIGH);
      for (int i = 0; i < 10; i++) {
        digitalWrite(signalPin, HIGH);
        delay(1500);
        digitalWrite(signalPin, LOW);
        delay(1500);

        //        analogWrite(signalPin, power);  // 24.5A, 12V
        //        delay(800); // .8 seconds at 255
        //
        //        power = 187; // 18A/24.5A = x/255   (9V)
        //        analogWrite(signalPin, power);
        //        delay(500); // .5 seconds at 18V
        //
        //        power = 130; // 12.5A/24.5A = x/255   (6V)
        //        analogWrite(signalPin, power);
        //        delay(400); // .4 seconds at 12.5V
        //
        //        power = 73; // 7A/24.5A = x/255   (4V)
        //        analogWrite(signalPin, power);
        //        delay(300); //.3 seconds at 7V
        //
        //        power = 42; // 4A/24.5A = x/255   (1.5V)
        //        analogWrite(signalPin, power);
        //        delay(200); // .2 seconds at 4V
        //
        //        power = 0; // 0A, 0V cooling period
        //        analogWrite(signalPin, power);
        //        delay(60000); // 1 second off
      }
      //digitalWrite(powerPin, LOW);
      Serial.print("1");  //shows when swim function ends
      while (mySerial.available() > 0) {
        mySerial.read();
      }
    }
    if (value == SENSOR) {
      temp = storeTemperature();         //temperature value
      float pressure = storePressure();  //pressure value
      sonar = storeSonar();              //distance value from sonar sensor
      float dp = storeDepth();           //depth value from pressure sensor
      transmit('t', temp);
      delay(2000);
      transmit('p', pressure);
      delay(2000);
      transmit('n', sonar);
      delay(2000);
      transmit('d', dp);
      delay(2000);

      Serial.print("2");
    }
    if (value == TMP) {
      temp = storeTemperature();
      transmit('t', temp);
      delay(1000);

      Serial.print("3");
    }
    if (value == SR) {
      sonar = storeSonar();
      transmit('n', sonar);
      delay(1000);

      Serial.print("4");
    }
    if (value == PRS) {
      pressure = storePressure();  //pressure value
      dp = storeDepth();           //depth value from pressure sensor
      transmit('x', pressure);
      delay(1000);
      transmit('y', dp);
      delay(1000);
      Serial.print("5");
    }
  }
}

void transmit(char inp, float measurement) { //sending message
  mySerial.print('<');
  Serial.print('<');
  delay(1000);
  mySerial.print(inp);
  Serial.print(inp);
  delay(1000);
  mySerial.print(measurement);
  Serial.print(measurement);
  delay(1000);
  mySerial.println('>');
  Serial.println('>');
  delay(1000);
}

float storeTemperature() {
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  return temperature;
  delay(1000);
}

float storePressure() {
  sensor.read();
  pressure1 = sensor.pressure();
  float depth = sensor.depth();
  return pressure1;
  delay(1000);
}

float storeSonar() {
  sonar = analogRead(sensorPin);
  return sonar;
  delay(1000);
}

float storeDepth() {
  sensor.read();
  float depth = sensor.depth();
  return depth;
  delay(1000);
}