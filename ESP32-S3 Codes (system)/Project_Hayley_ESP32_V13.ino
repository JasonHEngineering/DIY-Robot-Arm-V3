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
    - Compatible with "Project_Hayley_V5_IK_user_interface_V03.py"  
*/

#include <Wire.h>
#include <AccelStepper.h>
#include <SPI.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"

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
#define max_speed 4200
#define motor_acceleration 8000.0


bool task_running = false; // Track if task has started
bool distance_flag = false;

// Track the number of items processed so far
int processed_item_count = 0;
int steps_without_yield = 0; // Step counter for conditional yielding
const int max_steps_before_yield = 10000; // Yield after 10000 steps

const float micro_steps[no_of_steppers] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0}; 
const float gear_reduction[no_of_steppers] = {-30.0, -30.0, -30.0, -30.0, -30.0, -1.0}; 
const float steps_per_rotation[no_of_steppers] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}; 
//const int stepper_pins[no_of_steppers] = {4, 6, 43, 1, 15, 17}; //stepper pin id
//const int stepper_direction[no_of_steppers] = {5, 7, 44, 2, 16, 18}; //stepper direction id
//const byte axis_angle_pins[no_of_steppers] = {37, 38, 45, 36, 48, 47}; //GPIO chip select pins for all 6 encoders


const int stepper_pins[no_of_steppers] = {17, 3, 10, 15, 6, 4}; //stepper pin id
const int stepper_direction[no_of_steppers] = {18, 46, 14, 16, 7, 5}; //stepper direction id
const int stepper_enable[no_of_steppers] = {1, 2, 42, 41, 40, 39}; //stepper enable - GND/low to enable
const byte axis_angle_pins[no_of_steppers] = {21, 47, 48, 45, 35, 36}; //CS pins for 6 encoders + 1 spare (sprare is GPIO 36)


const int windowSize = 10; // Adjust the window size based on your requirements
float sensorValues[no_of_steppers][windowSize];

// use this to find kinematically zeroed position 
//const float start_axis_degAngle[no_of_steppers] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 3-Oct-25

//const float start_axis_degAngle[no_of_steppers] = {181.3, 10.88, 18.94, 282.6, 81.9, 83.8}; //Angle in degrees, encoder values when in kinematic zeroed position
//const float start_axis_degAngle[no_of_steppers] = {177.13, 31.45, -1.25, -74.72, 77.65, 87.49}; //Angle in degrees, encoder values when in kinematic zeroed position
//const float start_axis_degAngle[no_of_steppers] = {177.74, 26.82, -4.59, -71.87, 80.01, 86.92}; //Angle in degrees, encoder values when in kinematic zeroed position - 19-Apr-25
// const float start_axis_degAngle[no_of_steppers] = {184.06, 118.38, 88.77, 105.73, 10.64, 349.74}; //Angle in degrees, encoder values when in kinematic zeroed vertical position - 3-Oct-25
//const float start_axis_degAngle[no_of_steppers] = {185.94, 122.09, 91.41, 104.89, 9.09, 345.60}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 3-Oct-25
//const float start_axis_degAngle[no_of_steppers] = {185.98, 121.2, 91.92, 104.86, 9.30, 356.04}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 17-Dec-25
const float start_axis_degAngle[no_of_steppers] = {-174.02, 121.2, 91.92, 104.86, 9.30, -3.96}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 20-Dec-25

float axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
float accepted_axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
int currentIndex = 0;
uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)
uint16_t command = 0b1111111111111111; //read command (0xFFF)

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
AccelStepper *steppers[no_of_steppers] = {NULL, NULL, NULL, NULL, NULL, NULL};

bool flag = false;

// Define the size of your data structure
// typedef struct {
//     uint8_t data[7]; // Example: 10-byte I2C data buffer
// } i2c_data_t;

// Queue handle
QueueHandle_t i2c_data_queue;

// Semaphore to signal the start of the motor processing task
SemaphoreHandle_t start_task_semaphore;

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


void setup() {
  
  SPI.begin(13, 12, 11); //SCLK, MISO, MOSI, SS
  Serial.begin(9600);    //Serial.begin(115200); // start serial for output
  delay(100);

  for (int i = 0; i <= (no_of_steppers-1); i++) {
    pinMode(axis_angle_pins[i], OUTPUT); // set all CS pin as output
    pinMode(stepper_enable[i], OUTPUT); //TMC2209 enable - GND/low to enable
    digitalWrite(stepper_enable[i], LOW); // Set the pin to LOW - GND/low to enable
    steppers[i] = new AccelStepper(motorInterfaceType, stepper_pins[i], stepper_direction[i]);
  }

  Wire.begin(SLAVE_ADDRESS);   // initialize i2c as slave
  Wire.onReceive(receiveData); // receive interrupt callback (triggered by I2C-Master)
  Wire.onRequest(sendEncoderData);
  Serial.println("Ready!");


  // // Create a binary semaphore for task start synchronization
  // start_task_semaphore = xSemaphoreCreateBinary();
  // if (start_task_semaphore == NULL) {
  //     Serial.println("Error creating semaphore");
  //     while (1);
  // }

  // // Take the semaphore initially to block the motor task
  // xSemaphoreTake(start_task_semaphore, 0);

  // // Create the queue to hold incoming I2C data
  // i2c_data_queue = xQueueCreate(20, sizeof(converter.valueReading)); // Queue can hold 10 items

  // if (i2c_data_queue == NULL) {
  //     Serial.println("Error creating the queue");
  //     while (1);
  // }

  // Create the task to process the data
  //xTaskCreate(i2c_data_processing_task, "I2C Data Processing Task", 2048, NULL, 1, NULL);

  for (int x = 0; x <= (no_of_steppers-1); x++) {
    if (steppers[x]) {
        steppers[x]->setMaxSpeed(max_speed); //steps per second
        steppers[x]->setAcceleration(motor_acceleration); //desired acceleration in steps per second per second
    }
  }
  Serial.flush();
}


void loop() {
  
  if(flag) run_stepper();

  for (int x = 0; x <= (no_of_steppers-1); x++) {
    steppers[x]->run();
    if ((steppers[0]->distanceToGo() == 0) && (steppers[1]->distanceToGo() == 0) && (steppers[2]->distanceToGo() == 0) && (steppers[3]->distanceToGo() == 0) && (steppers[4]->distanceToGo() == 0) && (steppers[5]->distanceToGo() == 0)) {
          for (int i = 0; i < (windowSize*4); i++) { // this is for reading a few times for moving average
              read_angle_Register();
            }
        // report_angle(); //reporting of angles into serial com
    }
  }
}

// // Task to process incoming data
// void i2c_data_processing_task(void *param) {

//     while (1) {

//       Serial.println("Task waiting for semaphore...");

//       if (xSemaphoreTake(start_task_semaphore, portMAX_DELAY) == pdTRUE) {
//             Serial.println("Semaphore taken, task starting");
//             task_running = false; // Reset the task state

//       // Wait until the start signal is received
//     // xSemaphoreTake(start_task_semaphore, portMAX_DELAY);
//     // task_running = false; // Reset task running state

//         // while data is available in the queue
//         while (xQueueReceive(i2c_data_queue, &converter.valueReading, portMAX_DELAY)) {


//             // Print the current item being processed
//             processed_item_count++;
//             Serial.print("Processing item #");
//             Serial.println(processed_item_count);   

//             Serial.println(" ");
//             Serial.flush();

//             // Serial.println("converter.valueReading");

//             // for (int i = 0; i <= (no_of_steppers-1); i++) {
//             //     Serial.println(converter.valueReading[i]);
//             //     delay(3);
//             // }
//             // Serial.println(" ");
//             // Serial.flush();

//             delay(250);

//             distance_flag = false;
//             while (distance_flag == false) {
//               for (int i = 0; i < (windowSize*4); i++) { // this is for reading a few times for moving average
//                   read_angle_Register();
//                 }
//               distance_flag = report_stepper_distance_to_go();
//             }

//             if (round(converter.valueReading[5] * 10) != 8888) {
//               run_stepper();  
//             }

//             distance_flag = false;

//             while (distance_flag == false) {

//               distance_flag = report_stepper_distance_to_go();

//               for (int x = 0; x < no_of_steppers; x++) {
//                   steppers[x]->run();
//                 }
//             }

//             //delay(250);

//             for (int i = 0; i < (windowSize*4); i++) { // this is for reading a few times for moving average
//                 read_angle_Register();
//                 delay(3);
//               }

//             if (round(converter.valueReading[5] * 10) != 8888) {
//               run_stepper();  
//             }

//             distance_flag = false;

//             while (distance_flag == false) {

//               distance_flag = report_stepper_distance_to_go();

//               for (int x = 0; x < no_of_steppers; x++) {
//                   steppers[x]->run();
//                 }

//               // Wait for a short time before moving the next step (10 ms)
//               //vTaskDelay(pdMS_TO_TICKS(1)); // Yield control to the scheduler
//               // Instead of a fixed delay, yield control back to the scheduler
//               //taskYIELD();

//               // Count steps and yield every max_steps_before_yield iterations
//               steps_without_yield++;
//               if (steps_without_yield >= max_steps_before_yield) {
//                   vTaskDelay(pdMS_TO_TICKS(10)); // Yield for a minimal time (1 tick)
//                   steps_without_yield = 0; // Reset counter
//               }

//             }
//             //delay(100);
//         }
//         xSemaphoreTake(start_task_semaphore, 0);
//       }
//     }
// }

bool report_stepper_distance_to_go() {
  for (int x = 0; x < no_of_steppers; x++) {
    distance_to_go.steps[x] = steppers[x]->distanceToGo();
    // Serial.println((String)"Motor "+ String(x+1) + (String)" has remaining steps: " + distance_to_go.stepper_distance_to_go[x]); 
    // delay(2); // need this delay for serial print to work properly
  }

  return (
  distance_to_go.steps[0] == 0 && 
  distance_to_go.steps[1] == 0 && 
  distance_to_go.steps[2] == 0 &&
  distance_to_go.steps[3] == 0 &&
  distance_to_go.steps[4] == 0 &&
  distance_to_go.steps[5] == 0 
  );
}




void run_stepper(){
  
  float time_travel = 0.1;
  float speed_for_LERP = 5000.0;
  bool speed_flag[no_of_steppers] = {false, false, false, false, false, false};

  // sort and find the shortest time possible
  for (int x = 0; x < no_of_steppers; x++) {
    
    relative_target.angles_array[x] = converter.valueReading[x]-encoder.angles_array[x];
    motor.steps[x] = round(relative_target.angles_array[x]/(360.0/steps_per_rotation[x]) * gear_reduction[x] * micro_steps[x] );

    
    // if (x==5){
    //   Serial.println("converter.valueReading[5]");
    //   Serial.println(converter.valueReading[5]);
    //   Serial.println("encoder.angles_array[5]");
    //   Serial.println(encoder.angles_array[5]);      
    //   Serial.println("relative_target.angles_array[5]");
    //   Serial.println(relative_target.angles_array[5]);    
    //   Serial.println("motor.steps[5]");
    //   Serial.println(motor.steps[5]);    
    // }


    while (speed_flag[x]!= true) {
      float set_speed = motor.steps[x]/time_travel;
      if ((set_speed < speed_for_LERP) && (set_speed > -speed_for_LERP)) {
        speed_flag[x] = true;}
      else {
        time_travel+=0.1; //add time travel allowed
      }  
    }
  }
    //report_relative_target_angle();
    report_motor_steps();

    //Serial.println((String)"Time travel allowed is: "+time_travel);
  
  for (int x = 0; x < no_of_steppers; x++) {
    // moveTo() is absolute, from initial zeroed position, method use before encoder available
    // As of V5 switch to move() as a relative movement to current position per calculation from Project_Hayley_V4_IK_user_interface_V07.py
    steppers[x]->move(motor.steps[x]); 
    float set_speed = motor.steps[x]/time_travel;
    steppers[x]->setMaxSpeed(set_speed); // steppers[x]->setSpeed(set_speed);
  }

    flag = false;

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
      Serial.print("stepper triggered");
      for(uint8_t index = 0; index<byteCount; index++){
          converter.valueBuffer[index] = Wire.read();
      }
      flag = true;
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



    // if (byteCount>6){
    //   Serial.println("stepper triggered");
    //   for(uint8_t index = 0; index<byteCount; index++){
    //       converter.valueBuffer[index] = Wire.read();
    //   }
    //   flag = true;
    // }

  // // Condition to determine when to start the task
  // bool start_task_condition = false;

  // // hint for not triggering onreceive accidentally by onrequest 
  // // https://forum.arduino.cc/t/raspberrypi-i2c-arduino-onreceive-onrequest-collision/321731/3

  //   if (byteCount>6){
  //     Serial.println("stepper triggered");
  //     for(uint8_t index = 0; index<byteCount; index++){
  //         converter.valueBuffer[index] = Wire.read();
  //         Serial.println(converter.valueBuffer[index]);
  //     }

  //   //Serial.println(converter.valueReading[5]);
  //   if (round(converter.valueReading[5] * 10) == 8888) {
  //     start_task_condition = true;
  //     Serial.println("start_task_condition");
  //     Serial.println(start_task_condition);
  //   }
  //   else {
  //     start_task_condition = false;
  //     Serial.println("start_task_condition");
  //     Serial.println(start_task_condition);
  //   }
  //   // Check if last row has been received to start the task
  //   if (start_task_condition == 1) { 

  //     // Signal the task to start (if not already running)
  //     if (!task_running) {

  //         if (xSemaphoreGive(start_task_semaphore) == pdTRUE) {
  //             Serial.println("Semaphore given to start task");
  //             task_running = true;
  //         } else {
  //             Serial.println("Failed to give semaphore");
  //         }

  //         // xSemaphoreGive(start_task_semaphore); // Signal the task to start
  //         // task_running = true; // Mark the task as running
  //     }
  //   }
  //   // else if (start_task_condition == 0) {
  //   else {
  //     xQueueSend(i2c_data_queue, &converter.valueReading, 0);
  //   }
      
  //   }
}

// test reference from https://stackoverflow.com/questions/40196733/how-to-pass-arduino-struct-to-raspberry-pi-via-i2c
void sendEncoderData(){
  Wire.write((byte *)&encoder, sizeof(encoder));
}

void report_angle()
{
  for(int j = 0; j <= (no_of_steppers-1); j++) {
      //Serial.println(String(j+1) + " " + getMovingAverage(j));
      Serial.println(String(j+1) + " " + encoder.angles_array[j]);
    }
    Serial.flush();
}


void report_relative_target_angle()
{
  Serial.println("relative_target.angles_array");
  for(int j = 0; j <= (no_of_steppers-1); j++) {
      Serial.println((String)"Axis "+String(j+1) + " target angle is: " + relative_target.angles_array[j]); 
      delay(2); // need this delay for serial print to work properly
    }
    Serial.println(" ");
    Serial.flush();
}

void report_motor_steps()
{
  for(int j = 0; j <= (no_of_steppers-1); j++) {
      Serial.println((String)"Motor "+String(j+1)+" steps is: "+motor.steps[j]);
      delay(2); // need this delay for serial print to work properly
    }
    Serial.println(" ");
    Serial.flush();
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



