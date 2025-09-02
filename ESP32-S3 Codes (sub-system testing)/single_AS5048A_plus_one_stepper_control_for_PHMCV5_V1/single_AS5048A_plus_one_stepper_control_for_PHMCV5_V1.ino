/*

V1 -  Initial version with Proj Hayley Motor Controller V5

*/

#include <SPI.h>
/*PINS
   Arduino SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = 10
   STM32 SPI pins
   MOSI = PA7, MISO = PA6, SCK = PA5, CS = PA4
   ESP32 SPI pins
   MOSI = 11, MISO = 12, SCK = 13, CS = see below

*/

// Include the libraries
#include <Wire.h>
#include <AccelStepper.h>
#include <SPI.h>

// START DEFINE SECTION FOR MOTOR CONTROLLER V3
//#define SLAVE_ADDRESS 0x04 // Nano IoT 2
#define SLAVE_ADDRESS 0x05 //ESP32S3

// Define motor interface type
#define motorInterfaceType 1 //AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins)
#define no_of_steppers 6
#define max_speed 1000
#define motor_acceleration 2000.0


bool task_running = false; // Track if task has started
bool distance_flag = false;

// Track the number of items processed so far
int processed_item_count = 0;
int steps_without_yield = 0; // Step counter for conditional yielding
const int movesteps = 500; // Yield after 10000 steps

const float micro_steps[no_of_steppers] = {16.0, 16.0, 16.0, 16.0, 16.0, 16.0}; 
const float gear_reduction[no_of_steppers] = {30.0, 30.0, 30.0, 30.0, 30.0, 1.0}; 
const float steps_per_rotation[no_of_steppers] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}; 

const int stepper_pins[no_of_steppers] = {17, 3, 10, 15, 6, 4}; //stepper pin id
const int stepper_direction[no_of_steppers] = {18, 46, 14, 16, 7, 5}; //stepper direction id
const int stepper_enable[no_of_steppers] = {1, 2, 42, 41, 40, 39}; //stepper enable - GND/low to enable

const byte axis_angle_pins[no_of_steppers] = {21, 47, 48, 45, 0, 35}; //CS pins for 6 encoders + 1 spare (sprare is GPIO 36)

const int windowSize = 100; // Adjust the window size based on your requirements
float sensorValues[no_of_steppers][windowSize];
//const float start_axis_degAngle[no_of_steppers] = {181.3, 10.88, 18.94, 282.6, 81.9, 83.8}; //Angle in degrees, encoder values when in kinematic zeroed position
//const float start_axis_degAngle[no_of_steppers] = {177.13, 31.45, -1.25, -74.72, 77.65, 87.49}; //Angle in degrees, encoder values when in kinematic zeroed position
const float start_axis_degAngle[no_of_steppers] = {177.74, 26.82, -4.59, -71.87, 80.01, 86.92}; //Angle in degrees, encoder values when in kinematic zeroed position - 19-Apr-25
float axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
float accepted_axis_degAngle[no_of_steppers] = {0, 0, 0, 0, 0, 0}; //Angle in degrees
int currentIndex = 0;
uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)
uint16_t command = 0b1111111111111111; //read command (0xFFF)

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
AccelStepper *steppers[no_of_steppers] = {NULL, NULL, NULL, NULL, NULL, NULL};

// float struct to store angles 
struct {
   float angles_array[no_of_steppers]; // unit: °
} relative_target, encoder; 


void setup()
{

  SPI.begin(13, 12, 11); //SCLK, MISO, MOSI, SS
  Serial.begin(115200);
  Serial.println("AS5048A - Magnetic position encoder");
  delay(100);

  for (int i = 0; i <= (no_of_steppers-1); i++) {
    pinMode(axis_angle_pins[i], OUTPUT); // set all CS pin as output
    pinMode(stepper_enable[i], OUTPUT); //TMC2209 enable - GND/low to enable
    digitalWrite(stepper_enable[i], LOW); // Set the pin to LOW - GND/low to enable
  }

  // Create stepper instances
  for (int i = 0; i <= (no_of_steppers-1); i++) {
    pinMode(stepper_pins[i], OUTPUT); // set all CS pin as output
    pinMode(stepper_direction[i], OUTPUT); // set all CS pin as output
    steppers[i] = new AccelStepper(motorInterfaceType, stepper_pins[i], stepper_direction[i]);
  }

  for (int x = 0; x <= (no_of_steppers-1); x++) {
    if (steppers[x]) {
        steppers[x]->setMaxSpeed(max_speed); //steps per second
        steppers[x]->setAcceleration(motor_acceleration); //desired acceleration in steps per second per second
      steppers[x]->moveTo(-movesteps); // initial travel
    }
  }
  Serial.flush();
}

void loop()
{
  read_angle_Register(); //read the position of the magnet
  // report_angle(); //reporting of angles into serial com

  for (int x = 0; x <= (no_of_steppers-1); x++) {

    if (steppers[x]->distanceToGo() == 0) { 
      // Change direction once the motor reaches target position
      steppers[x]->moveTo(-steppers[x]->currentPosition());
      report_angle(); //reporting of angles into serial com
      delay(100);
    }

    // Move the motor one step
    steppers[x]->run();
  }

  //Serial.println(steppers[0]->currentPosition());

}

void read_angle_Register() {
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
    // sum += sensorValues[i];
  }
  
  float local_window = windowSize;
  
  return sum / local_window;
}

void report_angle()
{
  for(int j = 0; j <= (no_of_steppers-1); j++) {
      Serial.println(String(j+1) + " " + getMovingAverage(j));
      //Serial.println(String(j+1) + " " + encoder.angles_array[j]);
    }
    Serial.flush();
}
