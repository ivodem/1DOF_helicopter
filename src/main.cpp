/*
  TODO:
    - Rewrite SerialWrite to send a data string every 10-20Hz or tick (15ms)
    - Add SerialRead function:
      * Implement Get On/Off button state
      * Implement GetSetPoint value
      * Implement GetManualOrAutoMode value
      * Implement Get Kp, Ki and Kd value
      * Implement Get Hz value
    - Make SerialRead and SerialWrite timing correctly
    - Ensure encoder values can be read at 100 Hz minimun (needed for the PID to perform at 100Hz)
    - Add comments for function descriptions
    - Add more tests
*/
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <util/atomic.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h> // add the FreeRTOS functions for Semaphores (or Flags).
#include <task.h>

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

TaskHandle_t xPIDHandle = NULL;

// Pin definitions
static const uint8_t ENC_A = 2;
static const uint8_t ENC_B = 3;
static const uint8_t EMS = 19;
static const uint8_t MOTOR = 9;
static const uint8_t LED_PIN = LED_BUILTIN;

// Function declarations
void updateEncoder();
void PID(void *);
void toggleLED(void *);
void readSerial(void *);
void writeSerial(void *);
void emergencyStop();
int freqToTime(int);
void recvWithStartEndMarkers();
void parseData();

// Settings
static const int START_ANGLE = -22.0;

// Constants

// Global timer values
const uint16_t t1_load = 0;
const uint16_t t1_comp = 16000;
static uint16_t ms = 0;

// Global Encoder variables
volatile int lastEncoded = 0;   // load the variable from RAM rather than a storage register
volatile long encoderValue = 0; // value can be changed beyond the control of the code section
double angle;                   // The current angle reading from the encoder
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
const int offset = 0; // The offset of the 1DOF arm from start to hovering at 0 degrees

// Global PID controller variables
double set_point; // Set point in degrees
double error;
int pwm; // The output motor speed
double u;
static unsigned int pid_freq_ms;
static unsigned int pid_freq_hz = 50;

// LED global variables
static unsigned int led_delay = 500; // ms

// Serial Read variables
const byte numChars = 48;
char receivedChars[numChars];
boolean newData = false;
char tempChars[numChars];        // temporary array for use when parsing
// Variables to hold the data from serial
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;



// Test variables
// #define TESTING
#ifdef TESTING
static unsigned int task1 = 0; // task1 count
static unsigned int task2 = 0; // task2 count
static unsigned int task3 = 0; // task3 count
static unsigned int task4 = 0; // task3 count
bool printed = false;
#endif

void setup()
{

  /* Timer 1 settings */
  TCCR1A = 0x00; // Set Timer1 control register A to 0000 0000
  // Set the prescaler of Timer1 to 1 and turn on CTC mode (Timer1 control register B = 0000 1001)
  TCCR1B = 0x09;   // WGM12 = 1 CS12 = 0 CS11 = 0 CS10 = 1
  TCNT1 = t1_load; // Reset Timer1
  OCR1A = t1_comp; // Set Timer1 compare value to every 1ms
  TIMSK1 = 0x02;   // Enable Timer1 compare interrupt TIMSK1 = 0000 0010

  /* Serial communication settings */
  Serial.begin(9600); // Baud rate 9600
  while (!Serial)
    ; // Wait for serial port to connect.

  // Create semaphore for Serial port (a shared resource between serialWrite() and serialRead())
  if (xSerialSemaphore == NULL)
  {
    xSerialSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore
    if ((xSerialSemaphore) != NULL)
      xSemaphoreGive((xSerialSemaphore)); // Make the Serial Port available for use
  }

  /* Pin setup */
  // Pin mode settings for encoder and emergency stop button
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(EMS, INPUT_PULLUP);
  pinMode(MOTOR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // analogWrite(MOTOR, 0); // Ensure motor is turned off
  //  the model of encoder is with open-collector output
  digitalWrite(ENC_A, HIGH); // turn pullup resistor on
  digitalWrite(ENC_B, HIGH);
  // Create interrupts for Encoder
  // Call updateEncoder() on high/low change
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(INT0, updateEncoder, CHANGE);
  attachInterrupt(INT1, updateEncoder, CHANGE);

  // Create interrupt for the Emergency Stop Button
  // on interrupt 2 (pin 21)
  attachInterrupt(INT2, emergencyStop, FALLING); // Will also trigger on wire break

  /* FreeRTOS setup */
  xTaskCreate(    // FreeRTOS task creation
      PID,        // Function to be called
      "Run PID",  // Name of task
      2048,       // Stack size
      NULL,       // Parameter to pass
      1,          // Task priority
      &xPIDHandle // Task handle
  );

  xTaskCreate(
      toggleLED,
      "Toggle LED",
      1024,
      NULL,
      1,
      NULL);

  xTaskCreate(
      readSerial,
      "Read from Serial",
      1024,
      NULL,
      1,
      NULL);

  xTaskCreate(
      writeSerial,
      "Write to serial",
      1024,
      NULL,
      1,
      NULL);

  /* Initialise variable */
  pid_freq_ms = freqToTime(pid_freq_hz);
  set_point = -10.0;
  // Start FreeRTOS scheduler
  vTaskStartScheduler();

  // Delete setup() task
  vTaskDelete(NULL);
}

/**
 * @brief An empty function because the FreeRTOS program runs in tasks
 *
 */
void loop()
{
}

/**
 * @brief Converts frequency (Hz) to time (ms) delay to be used to change the PID frequency
 *
 * @param freq Frequency in Hz
 * @return int Time delay in milliseconds
 */
int freqToTime(int freq)
{
  int time;
  time = ((1.0f / freq) * 1000);
  return time;
}

/**
 * @brief This function is
 *
 * @param parameter
 */
void PID(void *parameter)
{
  // PID variables
  double error_integral = 0, error_derivative, last_error = 0;
  double kp, ki, kd;
  int16_t current_time, previous_time = 0, delta_time;
  // Below kind of works
  // kp = 12;
  // ki = 0.1;
  // kd = 1;
  // New PID:
  kp = 15; // 30 = oscillation
  ki = 2;
  kd = 0;
  set_point = -15;
  const int PWM_HIGH = 255;
  const int PWM_LOW = 230;
  
  // Get the time information
  while (1)
  {
    vTaskSuspend(NULL);
#ifdef TESTING
    task1++;
#endif
    angle = (360.0 / 1600.0) * (double)encoderValue + START_ANGLE;
    current_time = ms;
    delta_time = current_time - previous_time;
    //  Calculate the error
    //  error = set_point - angle;
    //  error_integral += error * delta_time;
    //  error_derivative = (error - last_error) / delta_time;
    error = set_point - angle;
    error_integral += error * delta_time;
    error_derivative = (error - last_error) / delta_time;
    last_error = error;
    // Calculate the output
    u = (kp * error) + (ki * error_integral) + (kd * error_derivative);
    if (error_integral > 4000) error_integral = 4000;
    if (error_integral < -4000) error_integral = -4000;

    if ((int)u < PWM_HIGH && (int)u > PWM_LOW)
    {
      analogWrite(MOTOR, u); // Write pwm signal to motor
      pwm = (int)u;
    }
    else if ((int)u > PWM_HIGH)
    {
      analogWrite(MOTOR, PWM_HIGH);
      pwm = PWM_HIGH;
    }
    else
    {
      analogWrite(MOTOR, PWM_LOW);
      pwm = PWM_LOW;
    }
    previous_time = ms;
  }
}

void toggleLED(void *parameter)
{
  while (1)
  {
#ifdef TESTING
    task2++;
#endif
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

void readSerial(void *parameter)
{
  while (1)
  {
#ifdef TESTING
    task3++;
#endif

    // Take semaphore and read values from serial
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
    {
      recvWithStartEndMarkers(); //Receive data from serial
      if (newData == true)
      {
        strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        // because strtok() used in parseData() replaces the commas with \0
        parseData();
        newData = false;
      }
      xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(freqToTime(20) / portTICK_PERIOD_MS);
  }
}
void writeSerial(void *parameter)
{
  while (1)
  {
#ifdef TESTING
    task4++;
#endif
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE)
    {
      // Concatentate string using sprintf and write the string to serial port
      // sprintf(str, "angle:%f,pwm:%d,error:%f", angle, pwm, error);
      char str[25];  // String buffer for writing to serial
      char str2[30]; // String buffer for writing to serial
      memset(str, 0, 25);
      memset(str2, 0, 25);
      char buf_angle[5];               // String buffer for dtostrf()
      dtostrf(angle, 5, 1, buf_angle); // Convert double to String (char[])
      sprintf(str, "angle:%s,pwm:%d\n", buf_angle, pwm);
      // sprintf(str2, "e_deri:%d,e_integr:%d,u:%d", error_;
      Serial.print(str);
      xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
    }
    lastMSB = encoderValue;
    vTaskDelay(freqToTime(2) / portTICK_PERIOD_MS);
  }
}
/**
 * @brief Construct a new ISR object
 *
 */
ISR(TIMER1_COMPA_vect)
{
  ms++;
  if (ms % (int)((1.0f / pid_freq_ms) * 1000) == 0)
  {
    BaseType_t check_yield_required;
    check_yield_required = xTaskResumeFromISR(xPIDHandle);
    portYIELD_FROM_ISR();
  }
#ifdef TESTING
  // TODO: Create better test for timing
  if (ms >= 10000 && printed == false)
  {
    printed = true;
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
    {
      Serial.println("Tasks count in the last 10 seconds:");
      Serial.print("Task 1 (PID) executed: ");
      Serial.print(task1);
      Serial.print("\t\t");
      Serial.print((double)task1 / 10);
      Serial.println(" Hz.");
      Serial.print("Task 2 (LED) executed: ");
      Serial.print(task2);
      Serial.print("\t\t");
      Serial.print((double)task2 / 10);
      Serial.println(" Hz.");
      Serial.print("Task 3 (SRX) executed: ");
      Serial.print(task3);
      Serial.print("\t\t");
      Serial.print((double)task3 / 10);
      Serial.println(" Hz.");
      Serial.print("Task 4 (STX) executed: ");
      Serial.print(task4);
      Serial.print("\t\t");
      Serial.print((double)task4 / 10);
      Serial.println(" Hz.");
      xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
    }
  }
#endif
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
/**
 * @brief Stops all functions to "shutdown" the system
 *
 */
void emergencyStop()
{
  /* TODO
  - Add setting the PWM to 0 (stopping the fan)
  - Write message to serial to notify InduSoft that EMS has been pressed
  */
  analogWrite(MOTOR, 0);
  TIMSK1 = 0x00;         // Stop timer
  detachInterrupt(INT0); // Detach both encoder interrupts
  detachInterrupt(INT1);
  vTaskEndScheduler(); // End FreeRTOS scheduler
  Serial.println("Emergency Stop!");
}

/**
 * @brief Receive data from Serial with start and end markers
 *
 */
void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '{';             
  char endMarker = '}';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

void parseData()
{ // split the data into its parts

  char *strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ","); // get the first part - the string
  strcpy(messageFromPC, strtokIndx);   // copy it to messageFromPC

  strtokIndx = strtok(NULL, ",");   // this continues where the previous call left off
  integerFromPC = atoi(strtokIndx); // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  floatFromPC = atof(strtokIndx); // convert this part to a float
}