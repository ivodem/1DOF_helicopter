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

// Settings
static const int START_ANGLE = -45.0;

// Constants

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
const int offset = 0; // The offset of the 1DOF arm from start to hovering at 0 degrees

// Global PID controller variables
static unsigned int set_point; // Set point in degrees
static unsigned int pid_freq_ms;
static unsigned int pid_freq_hz = 100;
int pwm; // The output motor speed
double error;

// LED global variables
static unsigned int led_delay = 500; // ms

// Test variables
#define TESTING

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
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(EMS, INPUT_PULLUP);
  pinMode(MOTOR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  analogWrite(MOTOR, 0); // Ensure motor is turned off

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
 * @brief Converts frequency (Hz) to time (ms) delay
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
  double u, kp, ki, kd;
  uint16_t current_time, previous_time = 0, delta_time;
  kp = 2;
  ki = 1;
  kd = 1;
  // Get the time information
  while (1)
  {
    vTaskSuspend(NULL);
    task1++;
    current_time = ms;
    delta_time = current_time - previous_time;
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
    {

      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.

      xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
    }
    // Calculate the error
    error = set_point - angle;
    error_integral += error * delta_time;
    error_derivative = (error - last_error) / delta_time;
    // Calculate the output
    u = (kp * error) + (ki * error_integral) + (kd * error_derivative);
    last_error = error;
    previous_time = ms;

    if (u > 255)
    {
      u = 255;
    }
    else if (u < 125)
    {
      u = 125;
    }
    analogWrite(MOTOR, u); // Write pwm signal to motor
  }
}

void toggleLED(void *parameter)
{
  while (1)
  {
    task2++;
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
    {
      xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
    }

    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

void readSerial(void *parameter)
{
  static const uint8_t BUF_LEN = 45;
  char c;
  char str[BUF_LEN]; // Create buffer for output
  uint8_t idx = 0;

  while (1)
  {
    // Clear whole buffer
    memset(str, 0, BUF_LEN);
    task3++;
    // Take semaphore and read values from serial
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
    {
      if (Serial.available() > 0)
      {
        char str[BUF_LEN];
        pid_freq_hz = Serial.parseInt();
        if (pid_freq_hz != 0)
        {
          led_delay = freqToTime(pid_freq_hz) / 2;
          sprintf(str, "led_delay updated to: %i Hz and %i ms", pid_freq_hz, led_delay);
          Serial.println(str);
        }
      }

      xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(freqToTime(21) / portTICK_PERIOD_MS);
  }
}
void writeSerial(void *parameter)
{
  while (1)
  { 
    task4++;
    if (lastMSB != encoderValue)
    {
      char str[60];

      angle = (360.0 / 1600.0) * (double)encoderValue + START_ANGLE;
      // Uncomment for use with PID
      // pwm = (int)fabs(PID(angle));
      //  if (pwm > 255)
      //  {
      //    pwm = 255;
      //  }
      if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
      {
        // Concatentate string using sprintf and write the string to serial port
        sprintf(str, "angle:%f,pwm:%d,error:%f", angle, pwm, error);
        Serial.write(str);

        xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
      }
      lastMSB = encoderValue;
    }
    vTaskDelay(freqToTime(21) / portTICK_PERIOD_MS);
  }
}
/**
 * @brief Construct a new ISR object
 *
 */
ISR(TIMER1_COMPA_vect)
{
  ms++;
  if (ms % (int)((1.0f / 100) * 1000) == 0)
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
  TIMSK1 = 0x00;         // Stop timer
  detachInterrupt(INT0); // Detach both encoder interrupts
  detachInterrupt(INT1);
  vTaskEndScheduler(); // End FreeRTOS scheduler
  Serial.println("Emergency Stop!");
}
