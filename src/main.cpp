/*
  TODO:
    - Rewrite SerialWrite to send a data string every 10-20Hz or tick (15ms)
    - Add SerialRead function:
      * Implement Get On/Off button state
      * Implement GetSetPoint value
      * Implement GetManualOrAutoMode value
      * Implement Get Kp, Ki and Kd value
      * Implement Get Hz value
    - Ensure encoder values can be read at 100 Hz minimun (needed for the PID to perform at 100Hz)
    - Add comments for function descriptions
    - Add more tests
*/
#include <Arduino.h>
#include <util/atomic.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h> // add the FreeRTOS functions for Semaphores (or Flags).
#include <task.h>

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t interruptSemaphore;

TaskHandle_t xPIDHandle = NULL;

// Pin definitions
static const uint8_t ENC_A = 2;
static const uint8_t ENC_B = 3;
static const uint8_t led_pin = LED_BUILTIN;

// Constants
static const int START_ANGLE = -45.0;
static const uint8_t BUF_LEN = 20;

// Settings

// Function declarations
void updateEncoder();
void motorControl(int);

// Global timer values
const uint16_t t1_load = 0;
const uint16_t t1_comp = 16000;
static uint16_t ms = 0;
static uint16_t i = 0;
// Global Encoder variables
volatile int lastEncoded = 0;   // load the variable from RAM rather than a storage register
volatile long encoderValue = 0; // value can be changed beyond the control of the code section
double angle;                   // The current angle reading from the encoder
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
int offset = 0;

// Global PID controller variables
static unsigned int set_point; // Set point in degrees
static unsigned int frequency_ms;
static unsigned int frequency_hz = 100;
int pwm; // The output motor speed

// LED global variables
static unsigned int led_delay = 500; // ms

// Test variables
#define TESTING
static unsigned int task1 = 0; // task1 count
static unsigned int task2 = 0; // task2 count
static unsigned int task3 = 0; // task3 count
bool printed = false;


void intTask(void *parameter){
  while(1){
    vTaskSuspend(NULL);
    task1++;
    i++;
  }
}
void PID(void *parameter)
{
  // PID variables
  double error, error_integral = 0, error_derivative, last_error = 0;
  double u, kp, ki, kd;
  uint16_t current_time, previous_time = 0, delta_time;
  kp = 2;
  ki = 1;
  kd = 1;
  // Get the time information
  while (1)
  {
    vTaskSuspend(NULL);
    task2++;
    current_time = ms;
    delta_time = current_time - previous_time;
    // if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
    // {
      
    //   // We were able to obtain or "Take" the semaphore and can now access the shared resource.
    //   // We want to have the Serial Port for us alone, as it takes some time to print,
    //   // so we don't want it getting stolen during the middle of a conversion.
    if(printed == false){
      Serial.println(delta_time);
    }
    
      
    //   xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
    // }
    // Calculate the error
    error = set_point - angle;
    error_integral += error * delta_time;
    error_derivative = (error - last_error) / delta_time;
    // Calculate the output
    u = (kp * error) + (ki * error_integral) + (kd * error_derivative);
    last_error = error;
    previous_time = ms;
    //vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void toggleLED(void *parameter)
{
  while (1)
  {
    task3++;
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
      {
        // We were able to obtain or "Take" the semaphore and can now access the shared resource.
        // We want to have the Serial Port for us alone, as it takes some time to print,
        // so we don't want it getting stolen during the middle of a conversion.
        // print out the state of the button:
        //Serial.println("Entering LED task");

        xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
      }
    
    digitalWrite(led_pin, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

void readDelay(void *parameter)
{
  char c;
  char buf[BUF_LEN];
  uint8_t idx = 0;

  // Clear whole buffer
  memset(buf, 0, BUF_LEN);

  while (1)
  {

    // Read char from serial
    if (Serial.available() > 0)
    {

      int freq_in;

      char str[45]; // Create buffer for output
      freq_in = Serial.parseInt();
      if (freq_in != 0)
      {
        led_delay = (1.0f / freq_in) * 1000;
        sprintf(str, "led_delay updated to: %i Hz and %i ms", freq_in, led_delay);
        Serial.println(str);
      }
    }
  }
}

void writeSerial(void *parameter)
{
  while (1)
  {
    if (lastMSB != encoderValue)
    {
      String str;
      // ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      angle = (360.0 / 1600.0) * (double)encoderValue + START_ANGLE;
      // Uncomment for use with PID
      // pwm = (int)fabs(PID(angle));
      //  if (pwm > 255)
      //  {
      //    pwm = 255;
      //  }
      //}
      // Serial.println(str + "value: "+ encoderValue + " angle: " + angle + " pwm: "+ pwm);
      Serial.println(str + "value: " + encoderValue + " angle: " + angle);
      lastMSB = encoderValue;
    }
  }
}

void setup()
{

  /* Timer 1 setup */
  TCCR1A = 0x00; // Set Timer1 control register A to 0000 0000
  //Set the prescaler of Timer1 to 1 and turn on CTC mode (Timer1 control register B = 0000 1001)
  TCCR1B = 0x09; // WGM12 = 1 CS12 = 0 CS11 = 0 CS10 = 1
  TCNT1 = t1_load; // Reset Timer1
  OCR1A = t1_comp; // Set Timer1 compare value to every 1ms
  TIMSK1 = 0x02; // Enable Timer1 compare interrupt TIMSK1 = 0000 0010

  //Serial communication settings
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if (xSerialSemaphore == NULL) // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage the Serial Port
    if ((xSerialSemaphore) != NULL)
      xSemaphoreGive((xSerialSemaphore)); // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  interruptSemaphore = xSemaphoreCreateBinary();

  //Serial.print("LED BLINK, CUSTOM LED DELAY\n\n");
  // Pin mode settings for encoder and emergency stop button
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

  xTaskCreate(    // FreeRTOS task creation
      PID,        // Function to be called
      "Run PID",  // Name of task
      2048,       // Stack size
      NULL,       // Parameter to pass
      1,          // Task priority
      &xPIDHandle // Task handle
  );

  xTaskCreate(    // FreeRTOS task creation
      intTask,    // Function to be called
      "interruptTask", // Name of task
      1024,       // Stack size
      NULL,       // Parameter to pass
      1,          // Task priority
      NULL        // Task handle
  );

  // xTaskCreate(  // FreeRTOS task creation
  //             writeSerial,     // Function to be called
  //             "Write encoder",  // Name of task
  //             1024,           // Stack size
  //             NULL,           // Parameter to pass
  //             1,              // Task priority
  //             NULL           // Task handle
  // );
  frequency_ms = ((1.0f / frequency_hz) * 1000);
  vTaskStartScheduler();
  // Delete "setup and loop" task
  vTaskDelete(NULL);
  
  
}

void loop()
{
}

ISR(TIMER1_COMPA_vect){
  ms++;
  if(ms % frequency_ms == 0){
    //xSemaphoreGiveFromISR(interruptSemaphore, NULL);
    BaseType_t check_yield_required;
    check_yield_required = xTaskResumeFromISR(xPIDHandle);
    portYIELD_FROM_ISR();
  }
  #ifdef TESTING
  // Create better test for timing
      if (ms >= 10000 && printed == false){
        printed = true;
        if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
        {
          Serial.println("Tasks count in the last 10 seconds:");
          Serial.print("Task 1 (FRQ) executed: "); Serial.print(task1); Serial.print("\t\t"); Serial.print((double)task1/10); Serial.println(" Hz.");
          Serial.print("Task 2 (PID) executed: "); Serial.print(task2); Serial.print("\t\t"); Serial.print((double)task2/10); Serial.println(" Hz.");
          Serial.print("Task 3 (LED) executed: "); Serial.print(task3); Serial.print("\t\t"); Serial.print((double)task3/10); Serial.println(" Hz.");
          xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
        }
  #endif
  }
}

void updateEncoder()
{
  int MSB = digitalRead(ENC_A);           // MSB = most significant bit
  int LSB = digitalRead(ENC_B);           // LSB = least significant bit
  int encoded = (MSB << 1) | LSB;         // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue++;
  lastEncoded = encoded; // store this value for next time
}
