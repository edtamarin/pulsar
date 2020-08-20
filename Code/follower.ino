#include <IRremote.h>

bool running = false;
/*
int IR_PIN = A3;
IRrecv irrecv(IR_PIN);
decode_results results;
#define POWER 0x10EFD827 
*/

  // define motor controls
#define E1 PD3  // Enable Pin for motor 1
#define E2 10  // Enable Pin for motor 2
#define I1 4  // Control pin 1 for motor 1
#define I2 2  // Control pin 2 for motor 1
#define I3 A1  // Control pin 1 for motor 2
#define I4 A0  // Control pin 2 for motor 2

int sensorPins[] = {5,6,7,8,9,A5,11,12}; // from right to left
int sensorResult[8];

int max_speed = 255;
float factor = 5.5;
int oldLine;
IRrecv irrecv(A3);
decode_results results;
const uint16_t POWER = 0xD827;
const uint16_t CENTRE = 0x20DF;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //delay(2000);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  int line;
  int rightMotorSpeed;
  int leftMotorSpeed;
  stateSelect(); // check for a signal
  if (running){
    readSensors();
    line = sensorResult[7]+2*sensorResult[6]+3*sensorResult[5]+4*sensorResult[4]+5*sensorResult[3]+6*sensorResult[2]+7*sensorResult[1]+8*sensorResult[0];
    line = 10* line / (sensorResult[7]+sensorResult[6]+sensorResult[5]+sensorResult[4]+sensorResult[3]+sensorResult[2]+sensorResult[1]+sensorResult[0]);
    if (line<=0){
      line = oldLine;
    }
    Serial.println(line);
    if (line<=35){
      moveRobot(max_speed,max_speed,1);
    }else if (line>=55){
      moveRobot(max_speed,max_speed,2);
    }else{
      moveRobot(max_speed,max_speed,0);
    }
     //moveRobot(leftMotorSpeed,rightMotorSpeed,0);
     oldLine = line;
  }else{
    moveRobot(0,0,0); // stop
  }
}

void moveRobot(int speedLeft, int speedRight, int dir){ // moving the robot
  // I use differential steering, speed is important
  analogWrite(E1, speedRight); // right
  analogWrite(E2, speedLeft); // left
      // always go forward
  if (dir == 0){ // forward
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  }else if (dir == 1){ // left
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);    
  }else{ // right
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);    
  }

}

void readSensors(){
  for (int i =0; i<8; i++){
    pinMode(sensorPins[i], OUTPUT );
    digitalWrite(sensorPins[i], HIGH );  
    delayMicroseconds(10);
    pinMode(sensorPins[i], INPUT );
    long time = micros();
    //time how long the input is HIGH, but quit after 1ms as nothing happens after that
    while (digitalRead(sensorPins[i]) == HIGH && micros() - time < 125); 
    int diff = micros() - time;
    // if line detected (this usually happens at above 100 us
    if (diff>=100){
      sensorResult[i] = 1;
    }else{
      sensorResult[i] = 0;
    }
  }
}

void stateSelect(){
  if (irrecv.decode(&results)) 
  {
    Serial.println(results.value);
    uint16_t resCode = (results.value & 0xFFFF);
    if (resCode == POWER) 
    {
      running = true;
    }
    if (resCode == CENTRE){
      running = false;
    }
    irrecv.resume();
  }
}

