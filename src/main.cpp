#include <Arduino.h>
#include <util/atomic.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <Arduino_FreeRTOS.h>

// Pin definitions
#define ENC_A 2
#define ENC_B 3

// Constants
#define START_ANGLE -45.0

// Function declarations
void updateEncoder(), motorControl(int);
double PID(double);

// Global Encoder variables
volatile int lastEncoded = 0;   // load the variable from RAM rather than a storage register
volatile long encoderValue = 0; // value can be changed beyond the control of the code section
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
int offset = 0;

// Global Motor controller variables
int pwm;

void setup()
{
  Serial.begin(115200);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  // the model of encoder is with open-collector output
  digitalWrite(ENC_A, HIGH); // turn pullup resistor on
  digitalWrite(ENC_B, HIGH); // turn pullup resistor on
  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  
}
void loop()
{
  if (lastMSB != encoderValue){
    String str;
    double angle;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      angle = (360.0/1600.0) * (double)encoderValue + START_ANGLE;
      pwm = (int)fabs(PID(angle));
      if (pwm > 255)
      {
        pwm = 255;
      }
    }
    Serial.println(str + "value: "+ encoderValue + " angle: " + angle + " pwm: "+ pwm);
    lastMSB = encoderValue;
  }
  delay(10);
}

double PID(double input_angle){
  // PID variables
  double set_point = 100;
  double error, error_integral = 0, error_derivative, last_error=0;
  double u, kp, ki, kd;
  double current_time, previous_time = 0, elapsed_time;
  kp = 2;
  ki = 1;
  kd = 1;
  // Get the time information
  current_time = millis();
  elapsed_time = current_time - previous_time;
  // Calculate the error
  error = set_point - input_angle;
  error_integral += error * elapsed_time;
  error_derivative = (error - last_error)/ elapsed_time;
  // Calculate the output
  u = (kp * error) + (ki * error_integral) + (kd * error_derivative);
  last_error = error;
  previous_time = current_time;

  return u;
}
void motorControl (int input_pwm){

}
void updateEncoder()
{
  int MSB = digitalRead(ENC_A);     // MSB = most significant bit
  int LSB = digitalRead(ENC_B);     // LSB = least significant bit
  int encoded = (MSB << 1) | LSB;         // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue++;
  lastEncoded = encoded; // store this value for next time
}
