#include <Arduino.h>
#include <util/atomic.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <Arduino_FreeRTOS.h>

// Pin definitions
static const int ENC_A = 2;
static const int ENC_B = 3;
static const int led_pin = LED_BUILTIN;

// Constants
static const int START_ANGLE = -45.0;
static const uint8_t BUF_LEN = 20;

// Settings

// Function declarations
void updateEncoder();
void motorControl(int);
double PID(double);


// Global Encoder variables
volatile int lastEncoded = 0;   // load the variable from RAM rather than a storage register
volatile long encoderValue = 0; // value can be changed beyond the control of the code section
double angle; // The current angle reading from the encoder 
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
int offset = 0;


// Global PID controller variables
static unsigned int set_point; // Set point in degrees
int pwm; // The output motor speed

// LED global variables
static unsigned int led_delay = 500; //ms

void PID(void *parameter){
  // PID variables
  double error, error_integral = 0, error_derivative, last_error=0;
  double u, kp, ki, kd;
  uint32_t current_time, previous_time = 0, elapsed_time;
  kp = 2;
  ki = 1;
  kd = 1;
  // Get the time information
  
  current_time = xTaskGetTickCount();
  elapsed_time = current_time - previous_time;
  // Calculate the error
  error = set_point - angle;
  error_integral += error * elapsed_time;
  error_derivative = (error - last_error)/ elapsed_time;
  // Calculate the output
  u = (kp * error) + (ki * error_integral) + (kd * error_derivative);
  last_error = error;
  previous_time = xTaskGetTickCount();
}

void toggleLED(void *parameter){
  while(1){
    digitalWrite(led_pin, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

void readDelay(void *parameter){
  char c;
  char buf[BUF_LEN];
  uint8_t idx = 0;

  // Clear whole buffer
  memset(buf, 0, BUF_LEN);

  while(1){

    // Read char from serial
    if(Serial.available() > 0) {
      
      int freqin;
      
      char str[45]; //Create buffer for output
      freqin =  Serial.parseInt();
      if (freqin != 0){
        led_delay = (1.0f/freqin)*1000;
        sprintf(str,"led_delay updated to: %i Hz and %i ms", freqin, led_delay);
        Serial.println(str);
      }
      
      //c = Serial.read();

      // Update delay variable and reset buffer
    //   if (c == '\n')
    //   {
    //     led_delay = atoi(buf);
    //     Serial.print("led_delay updated to: ");
    //     Serial.println(led_delay);
    //     memset(buf, 0, buf_len);
    //     idx = 0;
    //   }
    // } else {
    //   if(idx < buf_len - 1){
    //     buf[idx] = c;
    //     idx++;
    //   }
    }
  }
}

void writeSerial(void *parameter){
  while (1)
  {
    if (lastMSB != encoderValue){
      String str;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        angle = (360.0/1600.0) * (double)encoderValue + START_ANGLE;
        //Uncomment for use with PID
        //pwm = (int)fabs(PID(angle));
        // if (pwm > 255)
        // {
        //   pwm = 255;
        // }
      }
      //Serial.println(str + "value: "+ encoderValue + " angle: " + angle + " pwm: "+ pwm);
      Serial.println(str + "value: "+ encoderValue + " angle: " + angle);
      lastMSB = encoderValue;
      }
  }
}


void setup()
{
  Serial.begin(9600);
  Serial.print("LED BLINK, CUSTOM LED DELAY\n\n");
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(led_pin, OUTPUT);

  // the model of encoder is with open-collector output
  digitalWrite(ENC_A, HIGH); // turn pullup resistor on
  digitalWrite(ENC_B, HIGH); // turn pullup resistor on
  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  
  xTaskCreate(
              toggleLED,
              "Toggle LED",
              1024,
              NULL,
              1,
              NULL
  );

  xTaskCreate(  // Use xTaskCreate() in vanilla FreeRTOS
              readDelay,     // Function to be called
              "Read Delay",  // Name of task
              1024,           // Stack size
              NULL,           // Parameter to pass
              1,              // Task priority
              NULL           // Task handle
  );

  xTaskCreate(  // Use xTaskCreate() in vanilla FreeRTOS
              writeSerial,     // Function to be called
              "Write encoder",  // Name of task
              1024,           // Stack size
              NULL,           // Parameter to pass
              1,              // Task priority
              NULL           // Task handle
  );
  vTaskStartScheduler();
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop()
{
  
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
