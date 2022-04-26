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
TaskHandle_t xWriteHandle = NULL;
TaskHandle_t xReadHandle = NULL;

// Pin definitions
static const uint8_t ENC_A = 2;
static const uint8_t ENC_B = 3;
static const uint8_t EMS = 19;
static const uint8_t MOTOR = 9;

// Function declarations
void updateEncoder();
void PID(void *);
void toggleLED(void *);
void serialRead(void *);
void serialWrite(void *);
void emergencyStop();
int freqToTime(int);
void recvWithStartEndMarkers();
void parseData();

// Settings
static const int START_ANGLE = -30.0;

// Constants

// Global general variables
int running = 1;
int auto_mode = 1;

// Global timer values
const uint16_t t1_load = 0;
const uint16_t t1_comp = 16000;
static uint16_t ms = 0;
static uint16_t pid_delay = 0;

// Global Encoder variables
volatile int lastEncoded = 0;   // load the variable from RAM rather than a storage register
volatile long encoderValue = 0; // value can be changed beyond the control of the code section
double angle;                   // The current angle reading from the encoder
long lastencoderValue = 0;
const int offset = 0; // The offset of the 1DOF arm from start to hovering at 0 degrees

// Global PID controller variables
double kp, ki, kd;
int set_point; // Set point in degrees
double error;
int pwm; // The output motor speed
double u;
static unsigned int pid_freq_ms;
static unsigned int pid_freq_hz = 50;

// Serial Read variables
const byte numChars = 48;
char receivedChars[numChars];
boolean newData = false;
char tempChars[numChars]; // temporary array for use when parsing

// Test variables
//#define TESTING
#ifdef TESTING
static unsigned int task1 = 0; // task1 count
static unsigned int task2 = 0; // task2 count
static unsigned int task3 = 0; // task3 count
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
    pinMode(LED_BUILTIN, OUTPUT);

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
        serialRead,
        "Read from Serial",
        1024,
        NULL,
        1,
        &xReadHandle);

    xTaskCreate(
        serialWrite,
        "Write to serial",
        1024,
        NULL,
        1,
        &xWriteHandle);

    /* Initialise variable */
    pid_freq_ms = freqToTime(pid_freq_hz);
    set_point = 0;
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
 * @brief The PID, outputs pwm signal to the fan
 *
 * @param parameter
 */
void PID(void *parameter)
{
    // PID variables
    double error_integral = 0, error_derivative = 0, last_error = 0;
    int16_t current_time = 0, previous_time = 0, delta_time = 0;
    kp = 15; // 30 = oscillation
    ki = 2;
    kd = 0;
    const int PWM_HIGH = 255;
    const int PWM_LOW = 240;
    int step = 0;
    // Get the time information
    if (auto_mode == 1)
    {
        set_point = 0;
    }
    
    while (1)
    {
        vTaskSuspend(NULL);
#ifdef TESTING
        task1++;
#endif
        if (auto_mode == 1)
        {
            if (pid_delay >= 20000)
            {
                switch (step)
                {
                case 0:
                    set_point = 15;
                    break;
                case 1:
                    set_point = -15;
                    break;
                case 2:
                    set_point = 10;
                    break;
                case 3:
                    set_point = -10;
                    break;
                case 4:
                    set_point = 0;
                    break;
                default:
                    break;
                }
                step ++;
                if (step > 4)
                {
                    step = 0;
                }
                pid_delay = 0;
            }
            
            
            
        }
        
        // Calculate angle
        angle = (360.0 / 1600.0) * (double)encoderValue + START_ANGLE;
        // Calculate time difference
        current_time = ms;
        delta_time = current_time - previous_time;
        //  Calculate the error
        error = (double)set_point - angle;
        error_integral += (error * delta_time);
        error_derivative = (error - last_error) / delta_time;
        last_error = error;
        if (error_integral > 4000)
            error_integral = 4000;
        if (error_integral < -4000)
            error_integral = -4000;

        // Calculate the output
        u = (kp * error) + (ki * error_integral) + (kd * error_derivative);

        // Limit the PWM output to the fan
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
        // Get the end time
        previous_time = ms;
    }
}

void serialRead(void *parameter)
{
    while (1)
    {
#ifdef TESTING
        task2++;
#endif
        // Take semaphore and read values from serial
        if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE)
        {
            recvWithStartEndMarkers(); // Receive data from serial
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
        vTaskDelay(freqToTime(10) / portTICK_PERIOD_MS); // Do ~10 times per second (~10Hz)
    }
}
void serialWrite(void *parameter)
{
    while (1)
    {
#ifdef TESTING
        task3++;
#endif
        if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
        {
            // Concatentate string using sprintf and write the string to serial port
            // sprintf(str, "angle:%f,pwm:%d,error:%f", angle, pwm, error);
            char str[25]; // String buffer for writing to serial
            memset(str, 0, 25);
            char buf_angle[5];               // String buffer for dtostrf()
            dtostrf(angle, 5, 1, buf_angle); // Convert double to String (char[])
            // Data will be send in the following format:
            // angle,pwm,sp,error
            sprintf(str, "%s,%d,%d;\n", buf_angle, pwm, set_point);
            Serial.print(str);
            xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
        }
        vTaskDelay(freqToTime(2) / portTICK_PERIOD_MS); // Do ~2 times per second (~2Hz)
    }
}
/**
 * @brief Construct a new ISR object for Timer 1
 *
 */
ISR(TIMER1_COMPA_vect)
{
    ms++;
    pid_delay++;
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
            Serial.print("Task 2 (SRX) executed: ");
            Serial.print(task2);
            Serial.print("\t\t");
            Serial.print((double)task2 / 10);
            Serial.println(" Hz.");
            Serial.print("Task 3 (STX) executed: ");
            Serial.print(task3);
            Serial.print("\t\t");
            Serial.print((double)task3 / 10);
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
    int encoded = (MSB << 1) | LSB;         
    // BITWISE shift to left 1 bit, e.g.: 1 becomes 10
    // BITWISE OR the two values to join them together
    int sum = (lastEncoded << 2) | encoded; 
    // BTWISE shift left two bits and OR to add it to the previous encoded value
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

/**
 * @brief Takes the read serial data and splits the data in to its parts
 */

void parseData()
{
    // Data will look like this
    //{'off/on','manual/auto', 'set_point', 'kp','ki','kd'}
    char *strtokIndx; // this is used by strtok() as an index
    digitalWrite(LED_BUILTIN, HIGH);
    strtokIndx = strtok(tempChars, ","); // get the first part - the off = 0, on = 1
    running = atoi(strtokIndx);          // Convert this part to integer

    strtokIndx = strtok(NULL, ","); // manual = 0, auto = 1
    auto_mode = atoi(strtokIndx);   // convert this part to integer

    strtokIndx = strtok(NULL, ","); // frequency
    pid_freq_hz = atoi(strtokIndx); // convert this part to integer
    pid_freq_ms = freqToTime(pid_freq_hz);

    if (running == 0)
    {
        analogWrite(MOTOR, 0);
        digitalWrite(LED_BUILTIN, LOW);
        // vTaskSuspend(xWriteHandle);
        vTaskSuspend(xPIDHandle);
        TIMSK1 = 0x00;         // Stop timer
        detachInterrupt(INT0); // Detach both encoder interrupts
        detachInterrupt(INT1);
    }
    else if (running == 1)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        // vTaskResume(xWriteHandle);
        vTaskResume(xPIDHandle);
        TIMSK1 = 0x02;                                // Enable Timer1 compare interrupt TIMSK1 = 0000 0010
        attachInterrupt(INT0, updateEncoder, CHANGE); // Re-attach both encoder intterupts
        attachInterrupt(INT1, updateEncoder, CHANGE);
    }
    if (auto_mode == 0)
    {                                   // If the system is in manual mode
        strtokIndx = strtok(NULL, ","); // the setpoint
        set_point = atof(strtokIndx);   // convert this part to float

        strtokIndx = strtok(NULL, ","); // Kp
        kp = atof(strtokIndx);          // convert this part to float

        strtokIndx = strtok(NULL, ","); // Ki
        kd = atof(strtokIndx);          // convert this part to float

        strtokIndx = strtok(NULL, ","); // Kd
        ki = atof(strtokIndx);          // convert this part to float
    }
}
