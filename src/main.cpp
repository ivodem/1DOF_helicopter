#include <Arduino.h>

// Example code from Arduino.cc
int encoderPin1 = 2;
int encoderPin2 = 3;
volatile int lastEncoded = 0;   // load the variable from RAM rather than a storage register
volatile long encoderValue = 0; // value can be changed beyond the control of the code section
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
double angle;
int offset = 0;
String str;
double START_ANGLE = -45.0;
void updateEncoder();

void setup()
{
  Serial.begin(115200);
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  // the model of encoder is with open-collector output
  digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); // turn pullup resistor on
  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  
}
void loop()
{
  if (lastMSB != encoderValue){
    angle = (360.0/1600.0) * (double)encoderValue + START_ANGLE;
    Serial.println(str + "value: "+ encoderValue + " angle: " + angle);
    lastMSB = encoderValue;
  }
  delay(10);
}
void updateEncoder()
{
  int MSB = digitalRead(encoderPin1);     // MSB = most significant bit
  int LSB = digitalRead(encoderPin2);     // LSB = least significant bit
  int encoded = (MSB << 1) | LSB;         // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue++;
  lastEncoded = encoded; // store this value for next time
}
