/*

V4  - Cleanup and add homing capability, verified working with "Project_Hayley_V4_IK_user_interface_V04.py"
V5  - Corrected encoder pins sequence for Axis 1-6, compatible with "Project_Hayley_V4_IK_user_interface_V06.py"
V6  - attempt to concise the code (using a struct for stepper_position_data), compatible with "Project_Hayley_V4_IK_user_interface_V07.py"
V7  - Successfully added FreeRTOS queue. Compatible with "Project_Hayley_V4_IK_user_interface_V07.py"
V8  - Removal of homing and recalibration logic here as the user interface side (Python) will be zeroing to home base on encoder "home" readings
    - Compatible with "Project_Hayley_V4_IK_user_interface_V08.py" 
V9  - major change attempt to switch steps calculation from python/raspberry side to ESP32
    - The zero angles/position data need to be hardcoded on ESP32 instead
    - motor steps calculation parked under run_stepper() 
    - python/raspberry side need to send in desired relative angular positions instead of sending the relative steps
    - updated to send encoder as float instead of as long
V10 - Cleanup
    - as a consequence of V9, recalibrate & homing functions are now removed
    - combined some struct declarations for efficiency. e.g. struct with relative_target, encoder; struct with motor, distance_to_go
    - add encoder-stepper close loop, Compatible with "Project_Hayley_V4_IK_user_interface_V11.py"
V11 - attempt to add semaphore method, so as to recognize the last row before proceed with stepper motor execution
    - this is done via array of dummy angles 888.8 sent from raspberry pi to ESP32, and processed as boolean (round(converter.valueReading[5] * 10) == 8888)
    - Compatible with "Project_Hayley_V4_IK_user_interface_V11.py" 
V12 - Project revamp with updated hardware (new abosolute encoder V2 and new motor controller board - "stepper_with_TMC2209_controller_board_V1")
    - update default values under start_axis_degAngle[no_of_steppers]
    - Compatible with "Project_Hayley_V5_IK_user_interface_V01.py"     
V13 - update default values under start_axis_degAngle[no_of_steppers]
    - add stepper motor disabling logic
    - added I2C to handle different data e.g. angles vs changing stepper mode
    - Compatible with "Project_Hayley_V5_IK_user_interface_V04.py"  
V14 - major revamp to go dual core mode
    - Core 0 → I2C + 6 absolute encoders + target reception from Raspberry Pi
    - Core 1 → 6 stepper motors running AccelStepper + closed-loop PID position control
    - Compatible with "Project_Hayley_V5_IK_user_interface_V04.py"  
V15 - cleanup based on a working V14
    - added "QueueHandle_t jointQueue;" in order to manage stack of back-to-back angle move request from rapberry pi
*/

#include <Wire.h>
#include <AccelStepper.h>
#include <SPI.h>
#include <algorithm>

/*PINS
   Arduino SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = 10
   STM32 SPI pins
   MOSI = PA7, MISO = PA6, SCK = PA5, CS = PA4
   ESP32 SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = see below
*/

// START DEFINE SECTION FOR MOTOR CONTROLLER V3
//#define SLAVE_ADDRESS 0x04 // Nano IoT 2
#define SLAVE_ADDRESS 0x05 //ESP32S3

#define MSG_STEPPER_ARRAY  0x01
#define MSG_MODE_STRING    0x02

// Define motor interface type
#define motorInterfaceType 1 //AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins)
#define no_of_steppers 6
// #define max_speed 4200 // 2-Jan-26
#define max_speed 10000 // 3-Jan-26
// #define motor_acceleration 8000.0 // 2-Jan-26
#define motor_acceleration 4000.0 // 3-Jan-26

// ----- PID gains per joint -----
// float Kp = 2.5;
// float Ki = 0.0;
// float Kd = 0.15;
float Kp = 1.2;
float Ki = 0.0;
float Kd = 0.6;

float integral[6] = {0};
long prevError[6] = {0};

// ----- Shared feedback data -----
volatile long targetSteps[no_of_steppers];
SemaphoreHandle_t feedbackMutex;

// Motor driver settings
const float micro_steps[no_of_steppers] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0}; 
const float gear_reduction[no_of_steppers] = {-30.0, -30.0, -30.0, -30.0, -30.0, -1.0}; 
const float steps_per_rotation[no_of_steppers] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}; 
const int stepper_pins[no_of_steppers] = {17, 3, 10, 15, 6, 4}; //stepper pin id
const int stepper_direction[no_of_steppers] = {18, 46, 14, 16, 7, 5}; //stepper direction id
const int stepper_enable[no_of_steppers] = {1, 2, 42, 41, 40, 39}; //stepper enable - GND/low to enable
const byte axis_angle_pins[no_of_steppers] = {21, 47, 48, 45, 35, 36}; //CS pins for 6 encoders + 1 spare (sprare is GPIO 36)
const int steps_error_limit = 5; // set microstep tolerance limit allow by PID that will mark the target as complete

// Encoder reading settings
const int windowSize = 10; // Adjust the window size based on your requirements
float sensorValues[no_of_steppers][windowSize];
float axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
float accepted_axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
int currentIndex = 0;
uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)
uint16_t command = 0b1111111111111111; //read command (0xFFF)

// use this to find encoder default kinematically zeroed position 
//const float start_axis_degAngle[no_of_steppers] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 3-Oct-25

// kinematically zeroed position 
const float start_axis_degAngle[no_of_steppers] = {-174.02, 121.2, 91.92, 104.86, 9.30, -3.96}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 20-Dec-25

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
AccelStepper *steppers[no_of_steppers] = {NULL, NULL, NULL, NULL, NULL, NULL};

// Queue handle
QueueHandle_t jointQueue;

// https://medium.com/@sandhan.sarma/sending-floats-over-the-i2c-bus-between-two-arduinos-part-2-486db6dc479f
union BytesToFloat {
    // 'converts' incoming bytes to float array
    byte valueBuffer[no_of_steppers*8];
    float valueReading[no_of_steppers];    
} converter;


// float struct to store angles 
struct {
   float angles_array[no_of_steppers]; // unit: °
} relative_target, encoder; 


//struct motor_steps {
struct {
   long steps[no_of_steppers]; // unit: steps
} motor, distance_to_go; 


// ----- Core 0 Task: I2C + Encoder reading -----
void taskCore0(void *parameter) {
  SPI.begin(13, 12, 11); //SCLK, MISO, MOSI, SS
  Wire.begin(SLAVE_ADDRESS);   // initialize i2c as slave
  delay(100);
  Wire.onReceive(receiveData); // receive interrupt callback (triggered by I2C-Master)
  Wire.onRequest(sendEncoderData);
  Serial.println("I2C & SPI Ready!");
  delay(100);

  for (int i = 0; i <= (no_of_steppers-1); i++) {
    pinMode(axis_angle_pins[i], OUTPUT); // set all CS pin as output
  }

  feedbackMutex = xSemaphoreCreateMutex();
  
  while (true) {
    // 1. Read encoders 
    xSemaphoreTake(feedbackMutex, portMAX_DELAY);
    read_angle_Register(); // Encoder read out should be encoder.angles_array[i]
    xSemaphoreGive(feedbackMutex);

    vTaskDelay(2 / portTICK_PERIOD_MS); // should be 1000 Hz on ESP32, thus 2 ms periodic encoder prompt
  }
}



// ----- Core 1 Task: Stepper motion + PID closed loop -----
void taskCore1(void *parameter) {

  for (int i = 0; i <= (no_of_steppers-1); i++) {
    pinMode(stepper_enable[i], OUTPUT); //TMC2209 enable - GND/low to enable
    digitalWrite(stepper_enable[i], LOW); // Set the pin to LOW - GND/low to enable
    steppers[i] = new AccelStepper(motorInterfaceType, stepper_pins[i], stepper_direction[i]);
  }

  for (int x = 0; x <= (no_of_steppers-1); x++) {
    if (steppers[x]) {
        steppers[x]->setMaxSpeed(max_speed); //steps per second
        steppers[x]->setAcceleration(motor_acceleration); //desired acceleration in steps per second per second
    }
  }

  while (true) {
    // PID outer loop to compute correction
    if (xQueueReceive(jointQueue, &converter.valueReading, portMAX_DELAY) == pdTRUE) {

      // Load target positions
      xSemaphoreTake(feedbackMutex, portMAX_DELAY);
      for (int x = 0; x <= (no_of_steppers-1); x++) {
        targetSteps[x] = converter.valueReading[x];
        integral[x] = 0;
        prevError[x] = 0;
      }
      xSemaphoreGive(feedbackMutex);

      // Execute until all joints reach target
      bool reachedAll = false;
      while (!reachedAll) {
        reachedAll = true;

      for (int x = 0; x <= (no_of_steppers-1); x++) {
        xSemaphoreTake(feedbackMutex, portMAX_DELAY);

        long actual = encoder.angles_array[x];
        long target = targetSteps[x];

        long error_angle = target - actual;

        // convert error in angle to error in steps
        long error = round(error_angle/(360.0/steps_per_rotation[x]) * gear_reduction[x] * micro_steps[x] );
        
        // set steps_error_limit value here to exit stepper run loop
        if (abs(error) > steps_error_limit) reachedAll = false;

        integral[x] += error;
        long derivative = error - prevError[x];
        prevError[x] = error;

        long correction = (Kp * error) + (Ki * integral[x]) + (Kd * derivative);

        steppers[x]->move(correction); 
        steppers[x]->run();

        xSemaphoreGive(feedbackMutex);

        Serial.print("Axis ");
        Serial.print(x);
        Serial.print(" actual: ");
        Serial.println(encoder.angles_array[x]);
        Serial.print("Axis ");
        Serial.print(x);
        Serial.print(" target: ");
        Serial.println(targetSteps[x]);
        // Serial.print("correction: ");
        // Serial.println(correction);

      }
      vTaskDelay(1 / portTICK_PERIOD_MS);
      }
    }
  }
}

// ----- Launch tasks on ESP32-S3 cores -----
TaskHandle_t handleCore0;
TaskHandle_t handleCore1;

void setup() {
  Serial.begin(9600);    //Serial.begin(115200); // start serial for output
  while(!Serial);  // wait for USB serial connection
  delay(100);

  jointQueue = xQueueCreate(100, sizeof(converter.valueReading));
  assert(jointQueue); // optional check to crash early if creation fails

  xTaskCreatePinnedToCore(taskCore0, "Core0_Encoders_I2C", 4096, NULL, 2, &handleCore0, 0);
  xTaskCreatePinnedToCore(taskCore1, "Core1_Steppers_PID", 4096, NULL, 2, &handleCore1, 1);
}


void loop() {
  vTaskDelay(1000);
}

void receiveData(int byteCount){

  if (byteCount < 2) return;

  uint8_t msgType = Wire.read();
  uint8_t length  = Wire.read();

  switch (msgType) {

    // -----------------------------
    // Stepper motor array
    // -----------------------------
    case MSG_STEPPER_ARRAY: {
      Serial.println("stepper triggered");
      for(uint8_t index = 0; index<byteCount; index++){
          converter.valueBuffer[index] = Wire.read();
      }
      if (xQueueSend(jointQueue, &converter.valueReading, 0) == pdTRUE) {
        Serial.println("Queued new joint command");
      } else {
        Serial.println("Queue full! Command dropped");
      }
      break;
    }

    // -----------------------------
    // Mode string
    // -----------------------------
    case MSG_MODE_STRING: {
      char mode[32];

      for (int i = 0; i < length && i < 31; i++) {
        mode[i] = Wire.read();
      }
      mode[length] = '\0';

      Serial.print("Mode set to: ");
      Serial.println(mode);

      if (strcmp(mode, "disable_stepper") == 0) {
        // enable disable_stepper mode
        for (int i = 0; i <= (no_of_steppers-1); i++) {
          digitalWrite(stepper_enable[i], HIGH); // Set the pin to HIGH - HIGH to disable
        }
      }
      else if (strcmp(mode, "enable_stepper") == 0) {
        // enable enable_stepper mode
        for (int i = 0; i <= (no_of_steppers-1); i++) {
          digitalWrite(stepper_enable[i], LOW); // Set the pin to LOW - GND/low to enable
        }
      }
      break;
    }
    default:
    Serial.println("Unknown message type");
  }

}

// test reference from https://stackoverflow.com/questions/40196733/how-to-pass-arduino-struct-to-raspberry-pi-via-i2c
void sendEncoderData(){
  Wire.write((byte *)&encoder, sizeof(encoder));
}


void read_angle_Register()
{
  for (int i = 0; i <= (no_of_steppers-1); i++) {
    
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    //SPI.setClockDivider(SPI_CLOCK_DIV64);
      
    //--sending the command
    digitalWrite(axis_angle_pins[i], LOW);
    SPI.transfer16(command);
    digitalWrite(axis_angle_pins[i], HIGH);
  
    //--receiving the reading
    digitalWrite(axis_angle_pins[i], LOW);
    rawData = SPI.transfer16(command);
    digitalWrite(axis_angle_pins[i], HIGH);
    
    SPI.endTransaction();
  
    rawData = rawData & 0b0011111111111111; //removing the top 2 bits (PAR and EF)
  
    axis_degAngle[i] = (float)rawData / 16383.0 * 360.0; //16384 = 2^14, 360 = 360 degrees, however for real resolution, take 2^14 - 1 = 16383
    updateMovingAverage(i, axis_degAngle[i]);

    // Note: if getMovingAverage(i) == 360.00, it is likely to be false/no reading
    if ((axis_degAngle[i] != 0.00) && (getMovingAverage(i) != 0.00) && (getMovingAverage(i) != 360.00) && (getMovingAverage(i)<(axis_degAngle[i]+1.0)) && (getMovingAverage(i)>(axis_degAngle[i]-1.0))){
      accepted_axis_degAngle[i] = getMovingAverage(i);
      encoder.angles_array[i] = fmod(((accepted_axis_degAngle[i]-start_axis_degAngle[i])+180.0),360.0)-180.0;
    }
  }
}

void updateMovingAverage(int axis, float newValue) {
  // Update the array with the new value
  sensorValues[axis][currentIndex] = newValue;

  // Move to the next index in a circular manner
  if (axis==(no_of_steppers-1)){
    currentIndex = (currentIndex + 1) % windowSize;
  }
}

float getMovingAverage(int axis) {
  // Calculate the average of the values in the array
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += sensorValues[axis][i];
  }
  float local_window = windowSize;
  float moving_average = sum / local_window;
  return moving_average;
}



