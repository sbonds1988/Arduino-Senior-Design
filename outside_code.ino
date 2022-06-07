#include <SoftwareSerial.h>
int incoming = 0;
SoftwareSerial mySerial {0, 1};

int input;

const int SWIM = 1;   //user enters 1 for swim function
const int SENSOR = 2; //user enters 2 for swim function

bool started = false; //true: Message is started
bool ended  = false;  //true: Message is finished

char incomingByte ;   //variable to store the incoming byte
char msg[6];          //message from ZigBee (4 possibilities + brackets)

byte index;           //index of array

void setup() {
  Serial.begin(9600); //baud rate must be the same as is on ZigBee module
  mySerial.begin(9600);
  input = 0;
}

void loop() {
  delay(500); // 11:43
  Serial.println("Please choose a function:\n1 - Swim\n2 - Sensor Reading\n");

  while (Serial.available() == 0) { //  +
  }

  input = Serial.parseInt();
  if (input == SENSOR) { // remove SWIM
    transmit(input);
    for (int i = 0; i < 3; i++) {
      delay(500);
      input = 0;
      receive();
    }
    Serial.println("\nSensor if statement complete.\n");
  }
  else if (input == SWIM) {
    transmit(input);
    delay(500);
    input = 0;
    receive();
    while (Serial.available() > 0) {  // added 10-26
      Serial.read();    // added 10-26
    }
  }
  else {
    Serial.println("**Invalid command!**\n");
  }
  input = 0;

  delay(1000);
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void transmit(int inp) {  //parameter sent by user
  mySerial.print('<');    //starting symbol (ZigBee)
  mySerial.print(inp);    //message
  mySerial.println('>');  //ending symbol

  Serial.print('<');      //starting symbol (Serial Monitor)
  Serial.print(inp);      //message
  Serial.println('>');    //ending symbol
}

void receive() {
  while (mySerial.available() > 0) {
    incomingByte = mySerial.read();   //read incoming byte
    //      Serial.println(incomingByte);
    if (incomingByte == '<')          //starts message
    {
      started = true;
      index = 0;
      msg[index] = '\0';               //throw away any incomplete packet
    }
    else if (incomingByte == '>')      //ends message
    {
      ended = true;
      break;                           //exit loop
    }
    else                               //read message
    {
      if (index < 7)                   //make sure there is room
      {
        msg[index] = incomingByte;     //add char to array
        index++;
        msg[index] = '\0';             //add NULL to end
      }
    }
  }
  if (started && ended) { // 11:56 before change ctrl-z

    //Temperature
    if (msg[0] == 't') {
      Serial.print("Temperature = ");
      for (int i = 1; i < sizeof(msg); i++) {
        Serial.print(msg[i]);
      }
      Serial.println(" C");
      delay(1000);
    }

    //Pressure
    else if (msg[0] == 'p') {
      Serial.print("Pressure = ");
      for (int i = 1; i < sizeof(msg); i++) {
        Serial.print(msg[i]);
      }
      Serial.println(" mbar");
      delay(1000);
    }

    //Sonar
    else if (msg[0] == 'n') {
      Serial.print("Distance = ");
      for (int i = 1; i < sizeof(msg); i++) {
        Serial.print(msg[i]);
      }
      Serial.println(" cm");
      delay(1000);
    }

    index = 0;
    msg[index] = '\0';
    started = false;
    ended = false;
  }
}