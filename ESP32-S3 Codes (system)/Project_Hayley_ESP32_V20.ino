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
    - added "QueueHandle_t jointQueue;" in order to manage stack of back-to-back angle move request from raspberry pi
V16 - create standalone Kp, Ki, Kd constants for each axis
    - Introduce slight speed balancing between the axis motion, pegged to the axis with the longer travel time
    - Compatible with "Project_Hayley_V5_IK_user_interface_V04.py"  
V17 - Introduce servo claw functionality by adding 1 more float data into "union BytesToFloat" 
    - Attempt to split acceptable custom step error/s for each axis, rather than a fixed single value
    - Compatible with "Project_Hayley_V5_IK_user_interface_V05.py"  
V18 - Added magnet_direction[i] to reflect magnet direction. recap motor direction is determined by gear_reduction[x]
    - Modified axis 6 to have correction[x] = error[x]*0.5 rather than PI loop
    - Speed up movement by reducing delay, and remove LERP speed control, for now
    - Compatible with "Project_Hayley_V5_IK_user_interface_V06.py"  
V19 - Cleanup, including fixing USB-serial cannot detect bug 
    - Replaced <AccelStepper.h> with direct register access
    - Compatible with "Project_Hayley_V5_IK_user_interface_V06.py"
V20 - Added ESP-NOW communication protocol for use with PS controller
    - Added safety angle limits motor cut-off
    - Compatible with "Project_Hayley_V5_IK_user_interface_V08.py"

*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
//#include <AccelStepper.h>
#include <SPI.h>
#include <algorithm>
#include "driver/mcpwm_prelude.h"

#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_task_wdt.h"

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

#define MSG_STEPPER_ARRAY 0x01
#define MSG_MODE_STRING   0x02
#define MSG_PS_CONTROLLER 0x03
#define MSG_SPEED         0x04

// Define motor interface type
#define motorInterfaceType 1 //AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins)
#define no_of_steppers 6
// #define max_speed 4200 // 2-Jan-26
#define max_speed 20000 // 3-Jan-26
#define motor_acceleration 8000.0 // 2-Jan-26
// #define motor_acceleration 80000.0 // 3-Jan-26

// Define spped for non accelstepper.h method
//#define LAUNCH_SPEED        4000.0f   // 16-Apr-26
//#define FULL_SPEED          32000.0f  // 16-Apr-26
#define LAUNCH_SPEED        1000.0f   // 18-Apr-26
#define FULL_SPEED          1000.0f  // 18-Apr-26
#define LAUNCH_STEPS        0       // 19-Apr-26
#define LAUNCH_INTERVAL_US  ((unsigned long)(1000000.0f / LAUNCH_SPEED))  // 250 µs
#define FULL_INTERVAL_US    ((unsigned long)(1000000.0f / FULL_SPEED))    //  31 µs

#define HIGH_SPEED_RATIO 1.0
#define LOW_SPEED_RATIO 0.2

#define SERVO_GPIO 38 // 20 is actually using USB D+ pin

static float speed_ratio = 1.0;

// ============================================================
//  6-axis independent motor control via ESP-NOW joystick
//  Axis 1 → dpad left(0x08)/right(0x04)
//  Axis 2 → dpad up(0x01)/down(0x02)
//  Axis 3 → lx   (-508 to +512)
//  Axis 4 → ly   (-508 to +512)
//  Axis 5 → rx   (-508 to +512)
//  Axis 6 → ry   (-508 to +512)
// ============================================================

// --- Per-axis independent motion state ---
struct AxisState {
  bool          active;
  bool          forward;
  unsigned long intervalUs;   // µs between steps
  unsigned long lastStepUs;
};

static AxisState axisState[no_of_steppers] = {};

// --- Speed config per axis ---
// Tune these per axis based on load, gear ratio, and feel
const float AXIS_MAX_SPEED[no_of_steppers] = {
  1000.0f,   // Axis 1 — dpad left/right
  1000.0f,   // Axis 2 — dpad up/down
  1000.0f,   // Axis 3 — lx
  1000.0f,   // Axis 4 — ly
  1000.0f,   // Axis 5 — rx
   500.0f,   // Axis 6 — ry (lower — no gearing, gear_reduction = -1)
};

const float AXIS_MIN_SPEED[no_of_steppers] = {
  100.0f,    // Axis 1
  100.0f,    // Axis 2
  100.0f,    // Axis 3
  100.0f,    // Axis 4
  100.0f,    // Axis 5
  100.0f,    // Axis 6
};


// --- Angle limits per axis ---
const int AXIS_MAX_ANGLE[no_of_steppers] = {
  100,   // Axis 1 — dpad left/right
  90,   // Axis 2 — dpad up/down
  90,   // Axis 3 — lx
  120,   // Axis 4 — ly
  100,   // Axis 5 — rx
  179,   // Axis 6 — ry (lower — no gearing, gear_reduction = -1)
};

const int AXIS_MIN_ANGLE[no_of_steppers] = {
  -100,   // Axis 1 — dpad left/right
  -90,   // Axis 2 — dpad up/down
  -90,   // Axis 3 — lx
  -120,   // Axis 4 — ly
  -100,   // Axis 5 — rx
  -179,   // Axis 6 — ry (lower — no gearing, gear_reduction = -1)
};


// Deadzone for analog sticks — values within ±DEADZONE are ignored
#define ANALOG_DEADZONE  30

// // dpad produces fixed speed — no proportional scaling needed
// #define DPAD_SPEED_FAST       600.0f
// #define DPAD_SPEED_SLOW       100.0f

// ============================================================
//  setAxisVelocity() — set speed and direction for one axis
//  speed = 0 stops the axis
// ============================================================

void setAxisVelocity(int axis, float speed, bool forward) {
  if (speed <= 0.0f) {
    axisState[axis].active = false;
    return;
  }
  axisState[axis].intervalUs = (unsigned long)(1000000.0f / (speed*speed_ratio));
  axisState[axis].forward    = forward;
  axisState[axis].active     = true;
}

// ============================================================
//  mapAnalogToSpeed() — maps joystick value to speed
//  input range: -508 to +512
//  deadzone applied, output is always positive (direction separate)
// ============================================================

float mapAnalogToSpeed(int16_t value, float minSpeed, float maxSpeed) {
  if (abs(value) <= ANALOG_DEADZONE) return 0.0f;

  // Normalize to 0.0–1.0 outside deadzone
  float absVal  = abs(value) - ANALOG_DEADZONE;
  float maxVal  = 512.0f - ANALOG_DEADZONE;   // use 512 as positive max
  float ratio   = constrain(absVal / maxVal, 0.0f, 1.0f);

  return minSpeed + ratio * (maxSpeed - minSpeed);
}

mcpwm_cmpr_handle_t comparator;
mcpwm_oper_handle_t oper;
mcpwm_timer_handle_t timer;
mcpwm_gen_handle_t generator;


// ----- PID gains per joint -----
const float Kp[no_of_steppers] = {1.2, 1.2, 1.2, 1.2, 1.2, 0.005}; 
const float Ki[no_of_steppers] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
const float Kd[no_of_steppers] = {0.6, 0.6, 0.6, 0.6, 0.6, 0.001}; 

float time_travel[no_of_steppers] = {0.01};
float speed_for_LERP = 25000.0;
volatile long correction[6] = {0};

float integral[no_of_steppers] = {0};
long prevError[no_of_steppers] = {0};
long error[no_of_steppers] = {0};

// ----- Shared feedback data -----
volatile float targetSteps[no_of_steppers];
SemaphoreHandle_t feedbackMutex;

// Motor driver settings
const float micro_steps[no_of_steppers] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0}; 
//const float micro_steps[no_of_steppers] = {16.0, 16.0, 16.0, 16.0, 16.0, 16.0}; 
const float gear_reduction[no_of_steppers] = {30.0, -30.0, -30.0, 30.0, -30.0, -1.0}; // -ve sign to flip direction of physical motor
const float steps_per_rotation[no_of_steppers] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}; 
const int magnet_direction[no_of_steppers] = {-1, 1, 1, -1, 1, 1}; //magnet direction 
const int stepper_pins[no_of_steppers] = {17, 3, 10, 15, 6, 4}; //stepper pin id
const int stepper_direction[no_of_steppers] = {18, 46, 14, 16, 7, 5}; //stepper direction id
const int stepper_enable[no_of_steppers] = {1, 2, 42, 41, 40, 39}; //stepper enable - GND/low to enable
const byte axis_angle_pins[no_of_steppers] = {21, 47, 48, 45, 35, 36}; //CS pins for 6 encoders + 1 spare (sprare is GPIO 36)
//const int steps_error_limit[no_of_steppers] = {40, 40, 40, 40, 40, 8}; // set microstep tolerance limit allow by PID that will mark the target as complete
int steps_error_limit[no_of_steppers] = {40, 40, 40, 40, 40, 8}; // set microstep tolerance limit allow by PID that will mark the target as complete

//  Pin mask table
static uint32_t stepMasks[no_of_steppers];
//static uint32_t allowMasks[no_of_steppers];

// Encoder reading settings
const int windowSize = 5; // Adjust the window size based on your requirements
float sensorValues[no_of_steppers][windowSize];
float axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
float accepted_axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
int currentIndex = 0;
uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)
uint16_t command = 0b1111111111111111; //read command (0xFFF)

// use this to find encoder default kinematically zeroed position 
//const float start_axis_degAngle[no_of_steppers] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 3-Oct-25

// kinematically zeroed position 
//const float start_axis_degAngle[no_of_steppers] = {-174.02, 121.2, 91.92, 104.86, 9.30, -3.96}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 20-Dec-25 - by estimation only
//const float start_axis_degAngle[no_of_steppers] = {-187.61, 121.91, 92.53, -106.88, 10.89, -4.71}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 2-Feb-26 - by using 3 mm pins
const float start_axis_degAngle[no_of_steppers] = {-187.61, 121.91, 92.53, -106.88, 10.89, 116.29}; //Angle in degrees, encoder values when in kinematic zeroed horizontal position - 14-Jun-26 - axis 6 adjust for zero tablet pointer

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
//AccelStepper *steppers[no_of_steppers] = {NULL, NULL, NULL, NULL, NULL, NULL};

// Queue handle
QueueHandle_t jointQueue;


// ------------------------------------------------------------
// ESP-NOW PACKET — must match sender struct exactly
// ------------------------------------------------------------
typedef struct __attribute__((packed)) {
  int16_t  lx; // Axis 3
  int16_t  ly; // Axis 4
  int16_t  rx; // Axis 5
  int16_t  ry; // Axis 6
  uint16_t L2; 
  uint16_t R2;
  uint32_t buttons;
  uint8_t  dpad; // Axis 1 - left & right, Axis 2 - up and down
  uint8_t  misc;
} ControllerPacket;

// global state for PS control
volatile bool     espnowActive   = false;  // tracks ESP-NOW on/off
volatile bool     newPacket      = false;  // set true when ESP-NOW data arrives
ControllerPacket  rxData         = {};
volatile unsigned long lastRxTime = 0;
volatile bool     i2cNewCmd      = false;


// https://medium.com/@sandhan.sarma/sending-floats-over-the-i2c-bus-between-two-arduinos-part-2-486db6dc479f
union BytesToFloat {
    // 'converts' incoming bytes to float array, the +1 is for the servo
    byte valueBuffer[(no_of_steppers+1)*8];
    float valueReading[no_of_steppers+1];    
} converter;

// float struct to store angles 
struct __attribute__((packed)) {
  float angles_array[no_of_steppers]; // unit: °
  uint8_t status_flags;               // 1 byte
} encoder; 


//  Per-motor state
struct Motor {
  long absTarget;
  long stepsDone;
  long bresenham;
  bool forward;
};
static Motor motors[no_of_steppers];

//  Active move globals
static volatile bool moveComplete = false;
static volatile bool moveActive   = false;
static volatile long masterSteps  = 0;
static volatile long masterDone   = 0;

//struct motor_steps 
// struct {
//    long steps[no_of_steppers]; // unit: steps
// } motor; 

bool allWithinRange(const long arr[6]) {
    for (int i = 0; i < 6; ++i) {
        if (std::abs(arr[i]) > steps_error_limit[i]) {
            return false;
        }
    }
    return true;
}

// ----- function for motor direction and stepping -----
inline void setDir(int i, bool forward) {
  digitalWrite(stepper_direction[i], forward ? HIGH : LOW);
}

inline void pulseStepMulti(uint32_t mask) {
  if (!mask) return;
  GPIO.out_w1ts = mask;
  delayMicroseconds(1);
  GPIO.out_w1tc = mask;
}

void startMove(long targets[no_of_steppers]) {
  masterSteps = 0;

  for (int i = 0; i < no_of_steppers; i++) {

    //axisState[i].active     = true;

    motors[i].absTarget = abs(targets[i]);
    motors[i].stepsDone = 0;
    motors[i].bresenham = 0;
    motors[i].forward   = (targets[i] >= 0);

    setDir(i, motors[i].forward);

    if (motors[i].absTarget > masterSteps)
      masterSteps = motors[i].absTarget;
  }

  masterDone = 0;
  moveActive = (masterSteps > 0);

  Serial.print("Move started. Master steps: ");
  Serial.println(masterSteps);
}

bool runMove() {
  if (!moveActive) return false;

  static unsigned long lastStepTime = 0;
  unsigned long now = micros();

  bool nearEndpoint = (masterDone < LAUNCH_STEPS) ||
                      ((masterSteps - masterDone) < LAUNCH_STEPS);
  unsigned long interval = nearEndpoint ? LAUNCH_INTERVAL_US : FULL_INTERVAL_US;

  if ((now - lastStepTime) < interval) return true;
  lastStepTime = now;

  masterDone++;

  uint32_t batchMask = 0;
  for (int i = 0; i < no_of_steppers; i++) {
    if (motors[i].stepsDone >= motors[i].absTarget) continue;
    motors[i].bresenham += motors[i].absTarget;
    if (motors[i].bresenham >= masterSteps) {
      motors[i].bresenham -= masterSteps;
      motors[i].stepsDone++;
      batchMask |= stepMasks[i];
    }
  }

  pulseStepMulti(batchMask);

  if (masterDone >= masterSteps) {
    moveActive   = false;
    moveComplete = true;
    // Serial.println removed — Serial inside step loop blocks timing
    return false;
  }

  return true;
}

// ----- function for servo -----
void setServoUs(uint32_t us)
{
  // MCPWM timer resolution = 1 MHz → 1 tick = 1 µs
  mcpwm_comparator_set_compare_value(comparator, us);
}


// ------------------------------------------------------------
// ESP-NOW CALLBACKS
// ------------------------------------------------------------
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len) {
  // Guard: ignore packets if ESP-NOW mode was deactivated mid-flight
  if (!espnowActive) return;

  if (len == sizeof(ControllerPacket)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRxTime = millis();
    newPacket  = true;  // signal main loop to process

    // Serial.print("LX: "); Serial.print(rxData.lx);
    // Serial.print(" LY: "); Serial.print(rxData.ly);
    // Serial.print(" RX: "); Serial.print(rxData.rx);
    // Serial.print(" RY: "); Serial.println(rxData.ry);
    // Serial.print("L2: "); Serial.print(rxData.L2);
    // Serial.print(" R2: "); Serial.print(rxData.R2);
    // Serial.print(" buttons: "); Serial.println(rxData.buttons, HEX);
    // Serial.print(" dpad: "); Serial.println(rxData.dpad, HEX);
    // Serial.print(" misc: "); Serial.println(rxData.misc, HEX);
    // Serial.print(" BTN: "); Serial.println(rxData.buttons, HEX);

  }
}

// ------------------------------------------------------------
// ESP-NOW LIFECYCLE HELPERS
// ------------------------------------------------------------
void startEspNow() {
  if (espnowActive) return;  // already running, nothing to do

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed");
    return;
  }

  else {
    Serial.println("ESP-NOW init success");
  }

  esp_now_register_recv_cb(onDataRecv);
  espnowActive = true;
  Serial.println("[ESP-NOW] Started");
  Serial.print("[ESP-NOW] MAC: ");
  Serial.println(WiFi.macAddress());
}

void stopEspNow() {
  if (!espnowActive) return;  // already stopped, nothing to do

  esp_now_unregister_recv_cb();
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);       // release radio so other modes can use it if needed
  espnowActive = false;
  motorsStop(); // safe: de-energise motors when link intentionally dropped
  Serial.println("[ESP-NOW] Stopped");
}


// ============================================================
//  runAllAxes() — generate pulses for all active axes
//  Call this every loop iteration — uses micros() for timing
// ============================================================

void runAllAxes() {
  unsigned long now = micros();

  uint32_t batchMask = 0;
  //uint32_t blocked_batchMask = 0;

  for (int i = 0; i < no_of_steppers; i++) {
    if (!axisState[i].active) continue;

    if ((now - axisState[i].lastStepUs) >= axisState[i].intervalUs) {
      axisState[i].lastStepUs = now;
      setDir(i, axisState[i].forward);

      //batchMask |= stepMasks[i];

      bool forward_read = axisState[i].forward; 
      if (magnet_direction[i] == 1) {
          forward_read = !forward_read;
      }

      // Block only the direction that pushes further into the limit
      // Reverse direction is always allowed — motor can move away from limit
      if ((encoder.angles_array[i] > AXIS_MAX_ANGLE[i]) &&  forward_read)  continue; // at max, trying to go further → block
      if ((encoder.angles_array[i] < AXIS_MIN_ANGLE[i]) && !forward_read)  continue; // at min, trying to go further → block

      batchMask |= stepMasks[i];   // ← only reached if not blocked



      // if ((encoder.angles_array[i] > AXIS_MAX_ANGLE[i]) && (forward_read) {
      //   allowMasks[i] = (1UL << stepper_pins[i]);
      //   batchMask &= ~allowMasks[i];   
      // };
      // else if ((encoder.angles_array[i] < AXIS_MIN_ANGLE[i]) && (forward_read) {
      //   allowMasks[i] = (1UL << stepper_pins[i]);
      //   batchMask &= ~allowMasks[i];   
      // };      
      // if ((encoder.angles_array[0] > AXIS_MAX_ANGLE[0]) && (axisState[0].forward)) continue; // Trial to stop movement once limit hits
      // if ((encoder.angles_array[1] > AXIS_MAX_ANGLE[1]) && (!axisState[1].forward)) continue; 
      // if ((encoder.angles_array[2] > AXIS_MAX_ANGLE[2]) && (!axisState[2].forward)) continue; 
      // if ((encoder.angles_array[3] > AXIS_MAX_ANGLE[3]) && (axisState[3].forward)) continue; 
      // if ((encoder.angles_array[4] > AXIS_MAX_ANGLE[4]) && (!axisState[4].forward)) continue; 
      // if ((encoder.angles_array[5] > AXIS_MAX_ANGLE[5]) && (axisState[5].forward)) continue; 

      // batchMask |= stepMasks[i];
      // batchMask &= ~allowMasks[i];      
    }
  }

  // Fire all due axes simultaneously in one register write
  pulseStepMulti(batchMask);
}

// ------------------------------------------------------------
// FAILSAFE — stop motors if ESP-NOW link goes silent
// Adjust timeout to your needs (ms)
// ------------------------------------------------------------
#define ESPNOW_TIMEOUT_MS  1000

void checkEspNowFailsafe() {
  if (!espnowActive) return;
  if (millis() - lastRxTime > ESPNOW_TIMEOUT_MS) {
    motorsStop();
    // NOTE: does not deactivate ESP-NOW — it will resume driving
    // as soon as packets arrive again (sender reconnects, etc.)
  }
  
  // // activate motor stop if axis limits reached
  // for (int i = 0; i < no_of_steppers; i++) {
  //   if ((encoder.angles_array[i] > AXIS_MAX_ANGLE[i]) && (axisState[i].forward)) {
  //     motorsStop();
  //   }
  // }

}

void motorsStop() {
  // TBD script to stop motors / home
  Serial.println("STOP");
  for (int i = 0; i < no_of_steppers; i++) {
    axisState[i].active = false;
  }
}



// --- Claw servo control ---
void openclaw() {
  setServoUs(900);
  vTaskDelay(pdMS_TO_TICKS(500));
}

void closeclaw() {
  setServoUs(1250);
  vTaskDelay(pdMS_TO_TICKS(500));
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
  
    axis_degAngle[i] = (float)rawData / 16383.0 * 360.0 ; //16384 = 2^14, 360 = 360 degrees, however for real resolution, take 2^14 - 1 = 16383
    updateMovingAverage(i, axis_degAngle[i]);

    // Note: if getMovingAverage(i) == 360.00, it is likely to be false/no reading
    if ((axis_degAngle[i] != 0.00) && (getMovingAverage(i) != 0.00) && (getMovingAverage(i) != 360.00) && (getMovingAverage(i)<(axis_degAngle[i]+1.0)) && (getMovingAverage(i)>(axis_degAngle[i]-1.0))){
      accepted_axis_degAngle[i] = getMovingAverage(i)* magnet_direction[i];

      // V20
      // encoder.angles_array[i] = fmod(((accepted_axis_degAngle[i]-start_axis_degAngle[i])+180.0),360.0)-180.0;

      float delta = accepted_axis_degAngle[i] - start_axis_degAngle[i];
      float wrapped = fmod(fmod(delta, 360.0) + 360.0, 360.0); // now in [0, 360)
      if (wrapped > 180.0)
          wrapped -= 360.0;                                      // fold to (-180, 180]
      encoder.angles_array[i] = wrapped;

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


// ----- Core 0 Task: I2C + Encoder reading -----
void taskCore0(void *parameter) {
  // SPI.begin(13, 12, 11); //SCLK, MISO, MOSI, SS
  // Wire.begin(SLAVE_ADDRESS);   // initialize i2c as slave
  // delay(100);
  // Wire.onReceive(receiveData); // receive interrupt callback (triggered by I2C-Master)
  // Wire.onRequest(sendEncoderData);
  // Serial.println("I2C & SPI Ready!");
  // delay(100);

  for (int i = 0; i <= (no_of_steppers-1); i++) {
    pinMode(axis_angle_pins[i], OUTPUT); // set all CS pin as output
  }

  
  
  while (true) {
    // 1. Read encoders 
    xSemaphoreTake(feedbackMutex, portMAX_DELAY);
    read_angle_Register(); // Encoder read out should be encoder.angles_array[i]
    xSemaphoreGive(feedbackMutex);

    vTaskDelay(1 / portTICK_PERIOD_MS); // should be 1000 Hz on ESP32, thus 2 ms periodic encoder prompt
  }
}

// ----- Core 1 Task: Stepper motion + PID closed loop -----
void taskCore1(void *parameter) {

  // Pin setup
  for (int i = 0; i <= (no_of_steppers - 1); i++) {
    pinMode(stepper_enable[i], OUTPUT);   //TMC2209 enable - GND/low to enable
    digitalWrite(stepper_enable[i], LOW); // Set the pin to LOW - GND/low to enable
  }
  for (int i = 0; i < no_of_steppers; i++) {
    stepMasks[i] = (1UL << stepper_pins[i]);
    gpio_set_direction((gpio_num_t)stepper_pins[i], GPIO_MODE_OUTPUT);
    pinMode(stepper_direction[i], OUTPUT);
  }

  esp_task_wdt_delete(NULL);   // timing-critical task — no watchdog

  // while (true) {

  // // Temporary simulate espnowActive == true
  // // espnowActive = true;
  // Serial.print("espnowActive: ");
  // Serial.println(espnowActive);

  //   if (espnowActive) {

  //     Serial.print("newPacket: ");
  //     Serial.println(newPacket);

  //     if (newPacket) {
  //       newPacket = false;
  //       //applyControllerPacket(rxData);
  //       // Serial.printf("[ESP-NOW] LX:%d LY:%d RX:%d RY:%d L2:%d R2:%d BTN:0x%08X DPAD:0x%02X MISC:0x%02X\n",
  //       // rxData.lx, rxData.ly, rxData.rx, rxData.ry,
  //       // rxData.L2, rxData.R2, rxData.buttons, rxData.dpad, rxData.misc);

  //       uint32_t batchMask = 0;

  //       if (rxData.ry > 100) {
  //         setDir(5, true);
  //         batchMask |= stepMasks[5];
  //         pulseStepMulti(batchMask);
  //       }

  //       else if (rxData.ry < -100) {
  //         setDir(5, false);
  //         batchMask |= stepMasks[5];
  //         pulseStepMulti(batchMask);
  //       }

        
  //     }

  //     // 3. Failsafe: stop motors if sender goes silent
  //     checkEspNowFailsafe();

  //     //vTaskDelay(1 / portTICK_PERIOD_MS); // should be 1000 Hz on ESP32, thus 2 ms periodic encoder prompt
  //     //vTaskDelay(1); // should be 1000 Hz on ESP32, thus 2 ms periodic encoder prompt
  //     taskYIELD();   // yield without fixed time penalty

  //   }
while (true) {
    if (espnowActive) {

      // Parse packet and update velocity targets
      if (newPacket) {
        newPacket = false;

      // // --- Axis 3 — lx ---
      // float spd3 = mapAnalogToSpeed(rxData.lx, AXIS_MIN_SPEED[2], AXIS_MAX_SPEED[2]);
      // setAxisVelocity(2, spd3, rxData.lx >= 0);

      // // --- Axis 4 — ly ---
      // float spd4 = mapAnalogToSpeed(rxData.ly, AXIS_MIN_SPEED[3], AXIS_MAX_SPEED[3]);
      // setAxisVelocity(3, spd4, rxData.ly >= 0);

      // // --- Axis 5 — rx ---
      // float spd5 = mapAnalogToSpeed(rxData.rx, AXIS_MIN_SPEED[4], AXIS_MAX_SPEED[4]);
      // setAxisVelocity(4, spd5, rxData.rx >= 0);

      // // --- Axis 6 — ry ---
      // float spd6 = mapAnalogToSpeed(rxData.ry, AXIS_MIN_SPEED[5], AXIS_MAX_SPEED[5]);
      // setAxisVelocity(5, spd6, rxData.ry >= 0);

      // Serial.print("LX: "); Serial.print(rxData.lx);
      // Serial.print(" LY: "); Serial.print(rxData.ly);
      // Serial.print(" RX: "); Serial.print(rxData.rx);
      // Serial.print(" RY: "); Serial.println(rxData.ry);

      // --- Axis 1 — rx ---
      float spd1 = mapAnalogToSpeed(rxData.rx, AXIS_MIN_SPEED[0], AXIS_MAX_SPEED[0]);
      setAxisVelocity(0, spd1, rxData.rx <= 0);

      // --- Axis 2 — ry ---
      float spd2 = mapAnalogToSpeed(rxData.ry, AXIS_MIN_SPEED[1], AXIS_MAX_SPEED[1]);
      setAxisVelocity(1, spd2, rxData.ry <= 0);

      // --- Axis 3 — lx ---
      float spd3 = mapAnalogToSpeed(rxData.ly, AXIS_MIN_SPEED[2], AXIS_MAX_SPEED[2]);
      setAxisVelocity(2, spd3, rxData.ly >= 0);

      // --- Axis 4 — ly ---
      float spd4 = mapAnalogToSpeed(rxData.lx, AXIS_MIN_SPEED[3], AXIS_MAX_SPEED[3]);
      setAxisVelocity(3, spd4, rxData.lx >= 0);

      // --- Axis 5 — dpad up/down ---
      if      (rxData.dpad & 0x02) setAxisVelocity(4, AXIS_MAX_SPEED[4], true);   // right = forward
      else if (rxData.dpad & 0x01) setAxisVelocity(4, AXIS_MAX_SPEED[4], false);  // left  = reverse
      else if (rxData.buttons & 0x0001) setAxisVelocity(4, AXIS_MIN_SPEED[4], true);   // right = forward
      else if (rxData.buttons & 0x0008) setAxisVelocity(4, AXIS_MIN_SPEED[4], false);  // left  = reverse
      else                       setAxisVelocity(4, 0.0f, true);         // stop

      // --- Axis 6 — dpad left/right ---
      if      (rxData.dpad & 0x04) setAxisVelocity(5, AXIS_MAX_SPEED[5], true);   // up   = forward
      else if (rxData.dpad & 0x08) setAxisVelocity(5, AXIS_MAX_SPEED[5], false);  // down = reverse
      else if (rxData.buttons & 0x0002) setAxisVelocity(5, AXIS_MIN_SPEED[5], true);   // up   = forward
      else if (rxData.buttons & 0x0004) setAxisVelocity(5, AXIS_MIN_SPEED[5], false);  // down = reverse
      else                       setAxisVelocity(5, 0.0f, true);         // stop

      // --- Homing ---
      if (rxData.misc & 0x05) {

        for (int x = 0; x < no_of_steppers; x++) {
          float error_angle = encoder.angles_array[x];
          error[x] = round(error_angle / (360.0 / steps_per_rotation[x])
                          * gear_reduction[x] * micro_steps[x]);

          if (std::abs(error[x]) < steps_error_limit[x]) continue; 
          setAxisVelocity(x, AXIS_MAX_SPEED[x], (error[x] < 0) ? true:false);
        }


      // --- Claw servo control ---
      if (rxData.buttons & 0x0080) {
        openclaw(); 
      } else if (rxData.buttons & 0x0040) {
        closeclaw();
      }

        // // --- Compute initial error ---
        // xSemaphoreTake(feedbackMutex, portMAX_DELAY);
        // for (int x = 0; x < no_of_steppers; x++) {
        //   float error_angle = -(encoder.angles_array[x]);
        //   error[x] = round(error_angle / (360.0 / steps_per_rotation[x])
        //                   * gear_reduction[x] * micro_steps[x]);
        // }
        // xSemaphoreGive(feedbackMutex);

        // // --- Start first move ---
        // startMove(error);
      }

      // // --- Axis 5 — dpad up/down ---
      // if      (rxData.dpad & 0x02) setAxisVelocity(4, DPAD_SPEED_FAST, true);   // right = forward
      // else if (rxData.dpad & 0x01) setAxisVelocity(4, DPAD_SPEED_FAST, false);  // left  = reverse
      // else if (rxData.buttons & 0x0001) setAxisVelocity(4, DPAD_SPEED_SLOW, true);   // right = forward
      // else if (rxData.buttons & 0x0008) setAxisVelocity(4, DPAD_SPEED_SLOW, false);  // left  = reverse
      // else                       setAxisVelocity(4, 0.0f, true);         // stop

      // // --- Axis 6 — dpad left/right ---
      // if      (rxData.dpad & 0x04) setAxisVelocity(5, DPAD_SPEED_FAST, true);   // up   = forward
      // else if (rxData.dpad & 0x08) setAxisVelocity(5, DPAD_SPEED_FAST, false);  // down = reverse
      // else if (rxData.buttons & 0x0002) setAxisVelocity(5, DPAD_SPEED_SLOW, true);   // up   = forward
      // else if (rxData.buttons & 0x0004) setAxisVelocity(5, DPAD_SPEED_SLOW, false);  // down = reverse
      // else                       setAxisVelocity(5, 0.0f, true);         // stop

      // // --- Axis 5 — buttons up/down ---
      // if      (rxData.buttons & 0x0008) setAxisVelocity(4, DPAD_SPEED_SLOW, true);   // right = forward
      // else if (rxData.buttons & 0x0001) setAxisVelocity(4, DPAD_SPEED_SLOW, false);  // left  = reverse
      // else                       setAxisVelocity(4, 0.0f, true);         // stop

      // // --- Axis 6 — buttons left/right ---
      // if      (rxData.buttons & 0x0004) setAxisVelocity(5, DPAD_SPEED_SLOW, true);   // up   = forward
      // else if (rxData.buttons & 0x0002) setAxisVelocity(5, DPAD_SPEED_SLOW, false);  // down = reverse
      // else                       setAxisVelocity(5, 0.0f, true);         // stop

        //applyControllerPacket(rxData);
      }

      // Generate pulses at correct rate for all active axes
      runAllAxes();

      checkEspNowFailsafe();

      taskYIELD();   // yield without fixed time penalty
    }

    // Wait for new target from queue
    if (xQueueReceive(jointQueue, &converter.valueReading, 0) == pdTRUE) {

      bool isLastItem = false;
      UBaseType_t remaining = uxQueueMessagesWaiting(jointQueue);
      isLastItem = (remaining == 0);

      if (isLastItem) {
          Serial.println("This is the last item in the queue.");
          steps_error_limit[0] = 40; // tighten error at the last robotic step only
          steps_error_limit[1] = 40; // tighten error at the last robotic step only
          steps_error_limit[2] = 40; // tighten error at the last robotic step only         
          steps_error_limit[3] = 40; // tighten error at the last robotic step only
          steps_error_limit[4] = 40; // tighten error at the last robotic step only
          steps_error_limit[5] = 8; // tighten error at the last robotic step only   
      }
      else {
        //steps_error_limit[no_of_steppers] = {120, 120, 120, 120, 120, 24}; // otherwise loose
        steps_error_limit[0] = 120; // tighten error at the last robotic step only
        steps_error_limit[1] = 120; // tighten error at the last robotic step only
        steps_error_limit[2] = 120; // tighten error at the last robotic step only         
        steps_error_limit[3] = 120; // tighten error at the last robotic step only
        steps_error_limit[4] = 120; // tighten error at the last robotic step only
        steps_error_limit[5] = 24; // tighten error at the last robotic step only   
      }

      // --- Compute initial error ---
      xSemaphoreTake(feedbackMutex, portMAX_DELAY);
      for (int x = 0; x < no_of_steppers; x++) {
        targetSteps[x]  = converter.valueReading[x];
        float actual     = encoder.angles_array[x];
        float error_angle = (float)targetSteps[x] - actual;
        error[x] = round(error_angle / (360.0 / steps_per_rotation[x])
                         * gear_reduction[x] * micro_steps[x]);

        // Debug print — add temporarily to diagnose
        Serial.print("Axis "); Serial.print(x);
        Serial.print("  target: ");    Serial.print(targetSteps[x]);
        Serial.print("  actual: ");    Serial.print(actual);
        Serial.print("  err_steps: "); Serial.println(error[x]);
      }
      xSemaphoreGive(feedbackMutex);

      // --- Start first move ---
      startMove(error);

      // --- Run until all joints within range ---
      bool reachedAll = false;
      while (!reachedAll) {

        runMove();   // call as fast as possible — no delay here

        if (moveComplete) {
          moveComplete = false;   // acknowledge
          Serial.println("Move complete.");   // serial is safe here

          // Recalculate error from fresh encoder readings
          xSemaphoreTake(feedbackMutex, portMAX_DELAY);
          for (int x = 0; x < no_of_steppers; x++) {
            float actual      = encoder.angles_array[x];
            float error_angle = (float)targetSteps[x] - actual;
            error[x] = round(error_angle / (360.0 / steps_per_rotation[x])
                             * gear_reduction[x] * micro_steps[x]);
          }
          xSemaphoreGive(feedbackMutex);

          // Check if we are close enough to stop
          if (allWithinRange(error)) {
            reachedAll = true;
          } else {
            startMove(error);   // correction move with fresh error
          }
        }
      }

      // --- Claw servo control ---
      // if (converter.valueReading[6] == 0.0) {
      //   setServoUs(900);
      //   vTaskDelay(pdMS_TO_TICKS(500));
      // } else if (converter.valueReading[6] == 1.0) {
      //   setServoUs(1250);
      //   vTaskDelay(pdMS_TO_TICKS(500));
      // }

      if (converter.valueReading[6] == 0.0) {
        openclaw(); 
      } else if (converter.valueReading[6] == 1.0) {
        closeclaw();
      }

    }
  }
}

// ----- Launch tasks on ESP32-S3 cores -----
TaskHandle_t handleCore0;
TaskHandle_t handleCore1;

void setup() {
  Serial.begin(9600);    //Serial.begin(115200); // start serial for output
  //while(!Serial);  // wait for USB serial connection
  delay(100);

  SPI.begin(13, 12, 11); //SCLK, MISO, MOSI, SS
  Wire.begin(SLAVE_ADDRESS);   // initialize i2c as slave
  delay(100);
  Wire.onReceive(receiveData); // receive interrupt callback (triggered by I2C-Master)
  Wire.onRequest(sendEncoderData);
  Serial.println("I2C & SPI Ready!");
  delay(100);

  // flags for stepper enable state and PS controller state
  encoder.status_flags = 0b00000001;
  //startEspNow();

  feedbackMutex = xSemaphoreCreateMutex();

  jointQueue = xQueueCreate(200, sizeof(converter.valueReading));
  assert(jointQueue); // optional check to crash early if creation fails

  // This section below sre settings for servo
  // 1 MHz timer resolution
  mcpwm_timer_config_t timer_cfg = {};
  timer_cfg.group_id = 0;
  timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_cfg.resolution_hz = 1000000;
  timer_cfg.period_ticks = 20000;   // 20 ms → 50 Hz
  timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;

  mcpwm_new_timer(&timer_cfg, &timer);

  mcpwm_operator_config_t oper_cfg = {};
  oper_cfg.group_id = 0;
  mcpwm_new_operator(&oper_cfg, &oper);
  mcpwm_operator_connect_timer(oper, timer);

  mcpwm_comparator_config_t cmp_cfg = {};
  mcpwm_new_comparator(oper, &cmp_cfg, &comparator);

  mcpwm_generator_config_t gen_cfg = {};
  gen_cfg.gen_gpio_num = SERVO_GPIO;
  mcpwm_new_generator(oper, &gen_cfg, &generator);

  // HIGH at timer start
  mcpwm_generator_set_action_on_timer_event(
    generator,
    MCPWM_GEN_TIMER_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      MCPWM_TIMER_EVENT_EMPTY,
      MCPWM_GEN_ACTION_HIGH));

  // LOW when compare matches
  mcpwm_generator_set_action_on_compare_event(
    generator,
    MCPWM_GEN_COMPARE_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      comparator,
      MCPWM_GEN_ACTION_LOW));

  mcpwm_timer_enable(timer);
  mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

  // Center servo
  setServoUs(1250);

  xTaskCreatePinnedToCore(taskCore0, "Core0_Encoders_I2C", 32768, NULL, 2, &handleCore0, 0);
  xTaskCreatePinnedToCore(taskCore1, "Core1_Steppers_PID", 32768, NULL, 2, &handleCore1, 1);
  vTaskDelete(NULL);

}


void loop() {
  vTaskDelay(200); // was 200
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
      // Serial.println("stepper triggered");
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
    // Mode string to enable/disable stepper motors
    // -----------------------------
    case MSG_MODE_STRING: {
    Serial.print("byteCount="); Serial.println(byteCount);
    Serial.print("length byte="); Serial.println(length);
      char mode[32];

      for (int i = 0; i < length && i < 31; i++) {
        mode[i] = Wire.read();
      }
      mode[length] = '\0';

      Serial.print("Mode set to: ");
      Serial.println(mode);

      // Serial.println(strcmp(mode, "enable_stepper"));   

      if (strcmp(mode, "disable_stepper") == 0) {
        // enable disable_stepper mode
        for (int i = 0; i <= (no_of_steppers-1); i++) {
          digitalWrite(stepper_enable[i], HIGH); // Set the pin to HIGH - HIGH to disable
        }
        setStepperEnabled(false);
      }
      else if (strcmp(mode, "enable_stepper") == 0) {
        // enable enable_stepper mode
        for (int i = 0; i <= (no_of_steppers-1); i++) {
          digitalWrite(stepper_enable[i], LOW); // Set the pin to LOW - GND/low to enable
        }
        setStepperEnabled(true);
      }
      break;
    }

    // -----------------------------
    // Mode string to enable/disable PS control
    // -----------------------------
    case MSG_PS_CONTROLLER: {
    Serial.print("byteCount="); Serial.println(byteCount);
    Serial.print("length byte="); Serial.println(length);
      char mode[32];

      for (int i = 0; i < length && i < 31; i++) {
        mode[i] = Wire.read();
      }
      mode[length] = '\0';

      Serial.print("Mode set to: ");
      Serial.println(mode);

      // Serial.println(strcmp(mode, "enable_PS_control"));   

      if (strcmp(mode, "enable_PS_control") == 0) {
        // enable PS_control mode
        Serial.print("Enabling PS control mode");
        startEspNow();
        setPsEnabled(true);
      }
      else if (strcmp(mode, "disable_PS_control") == 0) {
        // disable PS_control mode
        Serial.print("Disabling PS control mode");
        stopEspNow();
        setPsEnabled(false);        
      }
      break;
    }


    // -----------------------------
    // Mode string to toggle motor speed
    // -----------------------------
    case MSG_SPEED: {
    Serial.print("byteCount="); Serial.println(byteCount);
    Serial.print("length byte="); Serial.println(length);
      char mode[32];

      for (int i = 0; i < length && i < 31; i++) {
        mode[i] = Wire.read();
      }
      mode[length] = '\0';

      Serial.print("Mode set to: ");
      Serial.println(mode);

      // Serial.println(strcmp(mode, "enable_PS_control"));   

      if (strcmp(mode, "set_low_speed") == 0) {
        // enable PS_control mode
        Serial.print("Set to low speed");
        speed_ratio = LOW_SPEED_RATIO;

      }
      else if (strcmp(mode, "set_high_speed") == 0) {
        // disable PS_control mode
        Serial.print("Set to high speed");
        speed_ratio = HIGH_SPEED_RATIO;
   
      }
      break;
    }

    default:
    Serial.println("Unknown message type");
  }

}

void setStepperEnabled(bool enabled) {
    if (enabled)
        encoder.status_flags |= (1 << 0);   // set bit 0
    else
        encoder.status_flags &= ~(1 << 0);  // clear bit 0
}

void setPsEnabled(bool enabled) {
    if (enabled)
        encoder.status_flags |= (1 << 1);   // set bit 1
    else
        encoder.status_flags &= ~(1 << 1);  // clear bit 1
}

// test reference from https://stackoverflow.com/questions/40196733/how-to-pass-arduino-struct-to-raspberry-pi-via-i2c
void sendEncoderData(){
  Wire.write((byte *)&encoder, sizeof(encoder)); // 24 bytes floating + 1 = 25 bytes 
}





