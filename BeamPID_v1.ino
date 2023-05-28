#include <PID_v1.h>

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define SERIAL SerialUSB    
#else
  #define SERIAL Serial
#endif

const uint8_t DXL_ID = 102;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
double distance;

double targ = 13.5;
double pos = 180;

double P = 6.0; // 6 to 7
double I = 3.5;
double D = 0.15;

double setpoint = 13.5, input, output;
PID PID1(&input, &output, &setpoint, P, I, D, DIRECT);


void setup() {
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-57,45);
  PID1.SetSampleTime(10);

}

void loop() {

  // Sense object distance // HC_SR04 example code
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2.0;

  // Sensor has trouble seeing objects under ~2mm 
  if (distance > 100){ 
    distance = 0.0;
  }

  // Calculate output
  input = distance;
  PID1.Compute();
  
  // Balance at 180 degrees // Output adjusts position to move object
  pos = 180 + output;

  // Set goal position // Standard Dynamixel arduino function
  dxl.setGoalPosition(DXL_ID, pos, UNIT_DEGREE);

  // Print for debugging
  SERIAL.print("Dist: ");SERIAL.print(distance);
  SERIAL.print(" POS: ");SERIAL.print(pos);
  SERIAL.println();
  

}
