/* Mobile Robot Arduino Controller
 * Uses 2 motors to move and turn a mobile robot with PID controllers
 * Both wheels have quadrature encoders that use the Encoders.h library
 * 
 * Separate controllers allow the robot to turn and move. Each controller has:
 * - PI controller for position (in inches or degrees)
 * - PD controller for velocity (in in/s or deg/s)
 * - Saturation constant and anti-windup for integrators
 *  
 * The Arduino can receive the following commands over Serial or I2C:
 * - 'Fxxx': Move Forward xxx inches
 * - 'Bxxx': Move Backward xxx inches
 * - 'Rxxx': Turn Right xxx inches
 * - 'Lxxx': Turn Left xxx inches
 * - 'D': Print controller variables to Serial
 * - 'E': Turn on (enable) the motors
 * - 'X': Turn off (disable) the motors
 * - 'Txxxyyy': Conduct an open loop test for 1 second, with xxx PWM counts on left wheel and yyy counts on right wheel
 * - 'OK': Clears any previous errors
 * */
 //I2C Definitions
#include <Wire.h>
#define SLAVE_ADDRESS 0x04

//Control variables
const unsigned int Ts = 10; // sample time in ms
const float freq = 1000 / Ts; // in Hz
const bool SERIAL_OUTPUT = true; // set to true to send data out serial line
const bool SERIAL_DIAGNOSE = false; // set to true to send back what was recieved
bool complete_flag = false;

// Scaling factors
const float EncoderScaling = 2 * 3.1416 / 3200; // Encoder counts to radians
const float MoveScaling = EncoderScaling * 5.9 / 2; // Encoder counts to inches (r=5.9 in)
const float TurnScaling = EncoderScaling * 5.81 / 10.71 * 360 / 2 / 3.1416; // Encoder counts to degrees (d=10.915 in)

// saturation constants
const float U_MAX = 125; // Max PWM voltage
float V_MAX = 20;  // Max velocity for distance controller (in/s)
const float W_MAX = 120; // Max velocity for turn controller (deg/s)

// Digital output
const int ENABLE_PIN = 4;
const int PWMA = 9; // Pulse width modulated voltage
const int PWMB = 10;
const int DIRA = 7; // Direction (HIGH is positive direction)
const int DIRB = 8;
const int NSF  = 12; // Status Flag
const int EncoderPinLA = 2;
const int EncoderPinLB = 5;
const int EncoderPinRA = 3;
const int EncoderPinRB = 6;
const int EncoderPinLVCC = 13;
const int TaskCompleted = 11;

// Motor output
int enable_ctrl = false;
float uL = 0;  // Left Wheel
float uR = 0;  // Right Wheel

// Quadrature encoder pins and variables
#include <Encoder.h>
Encoder EncoderL(EncoderPinLA, EncoderPinLB); // First pin (white) has interrupt, second (yellow) does not
Encoder EncoderR(EncoderPinRA, EncoderPinRB); // First pin (white) has interrupt, second (yellow) does not
long EncoderLTicks = 0;
long EncoderRTicks = 0;

// Time variables
unsigned long last_time = 0;
unsigned long current_time = 0;
int problem = 0;

// data requests variables
bool data_request = false;
bool continuous_data_request = false; // Used for continuous requests ('T' and 'E' commands)
int cdr_i = 0;
int cdr_max = 100;

// turn variables
float turn_radius = 0;
bool do_turn = false;
float circle_distance=0;

// Open Loop Test (T) variables
bool run_test = false;
const int start_iterations = 0;
const int stop_iterations = start_iterations + 1 * freq; // 1 second
const int finish_iterations = stop_iterations + freq / 2; // 0.5 seconds after pulse stops
unsigned int i = 0;

// Serial Communication
int incomingByte = 0;
String inputString = "";         // a string to hold incoming data
String tempstring = "";
bool stringComplete = false;  // whether the string is complete

//I2C Temp Byte

byte piControl;
byte parameter;

// I2C Communication
#include <Wire.h>
const int i2c_address = 4;        // slave address
bool writeOnFinish = false;       // write positions when controller finishes
const float finishAccuracy = 0.01;// write positions when within 1% of target


// Define controllers
class Controller {
  public:
    //Controller();
    //void run_pid();
    float ref;
    float y_new;
    float y_old;
    float e_int;
    float u;
    float vel;
    float P;
    float I;
    float D;
    float u_maximum;
    float err;

    // Constructor
    Controller(float _P, float _I, float _D, float _u_maximum) {
      P = _P;
      I = _I;
      D = _D;
      u_maximum = _u_maximum;
    }

    int sgn(float val) {
      if (val < 0) return -1;
      if (val > 0) return 1;
      return 0;
    }

    void run_pid() {
      vel = (y_new - y_old) * freq;  // Calculate velocity
      y_old = y_new;

      if (enable_ctrl) {

        err = ref - y_new;
        e_int = e_int + err / freq;
        u = P * err + D * vel + I * e_int;  // PID function

        // Anti-windup - when u is above maximum, keep integral error low
        if (abs(u) >= u_maximum) {
          u = u_maximum * sgn(u);
          e_int = e_int - err / freq;
        }
      }
      return;
    }
};

// Controller gains and variables
Controller ctrl_turn_dist(10.0, 15.0, 0, W_MAX); // degrees to degrees/s
Controller ctrl_turn_vel(1, 0, 0, U_MAX);  // degrees/s to PWM (gain ~= 1 PWM/deg/s)
Controller ctrl_move_dist(12, 12, 0, V_MAX);  // in to in/s
Controller ctrl_move_vel(6, 0, 0, U_MAX);  // in/s to PWM      (gain ~= 6 PWM/in/s)


void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void setup() {

  // Setup Motor Pin Modes
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(EncoderPinLVCC, OUTPUT);
  pinMode(TaskCompleted, OUTPUT);

  //Set Encoder VCC Pin High
  digitalWrite(EncoderPinLVCC, HIGH);

  // Setup Motors
  digitalWrite(ENABLE_PIN, HIGH);
  TCCR1B = (TCCR1B & 0b11111000) | 0x04; // change PWM frequency  http://playground.arduino.cc/Main/TimerPWMCheatsheet
 //Set task pin low so pi knows arduino is ready for instructions
  
  // Begin Serial (for PC/Bluetooth communication)
  Serial.begin(115200);

  // Begin Wire (for i2c communication)
  Wire.begin(i2c_address);      // join i2c bus with address #4
  Wire.onRequest(requestEvent); // register event for writing
  Wire.onReceive(receiveEvent); // register event for reading

  last_time = millis();


  // Print Serial output header
  if (SERIAL_OUTPUT) {
    Serial.print("Time");  // Overall Time
    Serial.print("\t");

    Serial.print("Move Ref");  // Position reference
    Serial.print("\t");
    Serial.print("Move Pos");  // Actual position
    Serial.print("\t");
    Serial.print("Move Err");  // Position error integral
    Serial.print("\t");
    Serial.print("Move VRef");  // Velocity reference
    Serial.print("\t");
    Serial.print("Move Vel");  // Actual Velocity
    Serial.print("\t");
    Serial.print("Move PWM");  // PWM counts for move
    Serial.print("\t");

    Serial.print("Turn Ref");  // Angle reference
    Serial.print("\t");
    Serial.print("Turn Pos");  // Actual angle
    Serial.print("\t");
    Serial.print("Turn Err");  // Angle error integral
    Serial.print("\t");
    Serial.print("Turn VRef");  // Angle Velocity reference
    Serial.print("\t");
    Serial.print("Turn Vel");  // Actual Angle Velocity
    Serial.print("\t");
    Serial.print("Turn PWM");  // PWM counts for turn
    Serial.print("\t");

    Serial.print("L Volt");  // Left Motor PWM counts
    Serial.print("\t");
    Serial.print("R Volt");  // Right Motor PWM counts
    Serial.print("\t");

    Serial.print("Exe Time");  // Time for each iteration
    Serial.print("\t");
    Serial.println("Error");
  }
}


void loop() {
 
  //Serial.println(piControl);
  //if (stringComplete) {  // Parse input string into command
//    if (SERIAL_DIAGNOSE) {
//      Serial.print("got: ");
//      Serial.println(inputString);
//    }
//
//    // Acknowledge an error if it happened
//    if (problem != 0 && inputString.substring(0, 2) == "OK") {
//      Serial.println("Error acknowledged");
//      problem = 0;
//    }
    problem = 0;
    Serial.println(toUpperCase(inputString.charAt(0)));
    switch (toUpperCase(inputString.charAt(0))) {
      case 'F': // Go forward in inches
        
        Serial.println("we did pt 2");
        tempstring = inputString.substring(1);
        Serial.println(tempstring);
        // Serial.println(tempstring);
        reset_controllers();
        ctrl_move_dist.ref = tempstring.toFloat();
        complete_flag = false;
        break;
      case 'R': // Go backward in inches
        
        tempstring = inputString.substring(1);
        // Serial.println(tempstring);
        reset_controllers();
        ctrl_move_dist.ref = -1 * tempstring.toFloat();
        break;
      case 'A': // Turn right, in degrees
       
        tempstring = inputString.substring(1);
        // Serial.println(tempstring);
        reset_controllers();
        ctrl_turn_dist.ref = -1 * tempstring.toFloat();
        break;
      case 'B': // Turn left, in degrees
       
        tempstring = inputString.substring(1);
        // Serial.println(tempstring);
        reset_controllers();
        ctrl_turn_dist.ref = tempstring.toFloat();
        break;
      case 'C': // Turn in a circle of a certain radius
       
        tempstring = inputString.substring(1);
        reset_controllers();
        turn_radius =  tempstring.toFloat();
        ctrl_move_dist.ref = 2*3.141*turn_radius*12;
        do_turn = true;
        continuous_data_request = true;
        cdr_max=750;
        cdr_i=0;
        break;
      case 'D': // Print data to Serial output
         
        Serial.println("Case D: ");
        data_request = true;
        break;
      case 'E': // Enable motors and (optionally) start continuous data request
        Serial.println("we did it boyz");
        enable_ctrl = true;
        if (inputString.length() > 1) {
          continuous_data_request = true;
          tempstring = inputString.substring(1);
          cdr_max = tempstring.toInt();
          cdr_i = 0;
        }
        break;
      case 'X': // Disable motors
       
        // reset_controllers();
        enable_ctrl = false;
        
        break;
      case 'T': // Initialize open loop test
       
        // do an open loop PWM pulse for 1 second, with continuous data request
        // e.g. T-50025 -> pulse motor L at uL=-50 and motor R at uR=25 for 1 sec
        reset_controllers();
        if (inputString.charAt(1) != '0') {
          uL = inputString.substring(1, 4).toInt();
        } else {
          uL = inputString.substring(2, 4).toInt();
        }
        if (inputString.charAt(4) != '0') {
          uR = inputString.substring(4, 7).toInt();
        } else {
          uR = inputString.substring(5, 7).toInt();
        }
        if (abs(uL) > U_MAX | abs(uR) > U_MAX) {
          problem = 98;
          uL = 0;
          uR = 0;
        }

        run_test = true;
        i = 0;
        cdr_i = 0;
        continuous_data_request = true;
        break;
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  //}   //  end if stringComplete

  

  if (! digitalRead(NSF)) { // Motor driver error, check if motor driver has power
    problem = 99; 
    Serial.println("Motor bridge driver error");
  }


//  if ( abs(ctrl_move_dist.err) < .01 & abs(ctrl_turn_dist.err) <.01) {
//    complete_flag = true;
//  }
  
  if (problem == 0) {

    // Run open loop test (T)
    if (run_test) {
      if (i == start_iterations) {
        writeVoltage('L', uL);
        writeVoltage('R', uR);
      }

      if (i == stop_iterations) {
        writeVoltage('L', 0);
        writeVoltage('R', 0);
      }

      if (i == finish_iterations) {
        continuous_data_request = false;
        run_test = false;
        //Serial.print(EncoderLTicks);
        //Serial.print("\t");
        //Serial.println(EncoderRTicks);
      }

      i = i + 1;
    } // end run_test

    // Read encoders and update position
    EncoderLTicks = EncoderL.read();
    EncoderRTicks = EncoderR.read();
    ctrl_move_dist.y_new = (float)(EncoderLTicks + EncoderRTicks) / 2 * MoveScaling;
    ctrl_turn_dist.y_new = (float)(EncoderRTicks - EncoderLTicks) / 2 * TurnScaling;

    // Run distance (i.e. position) controllers
    ctrl_move_dist.run_pid();
    ctrl_turn_dist.run_pid();

    if (do_turn) {
      ctrl_turn_dist.ref = (180/3.14)*ctrl_move_dist.y_new/(12*turn_radius);
       if (ctrl_move_dist.ref - ctrl_move_dist.y_new < .01 ){
           do_turn=false; 
       }
    }

    
    // Set velocity reference and update current velocity
    ctrl_move_vel.ref = ctrl_move_dist.u;      // velcity reference ref = distance output u
    ctrl_move_vel.y_new = ctrl_move_dist.vel;  // velcity position y_new = distance velocity vel
    ctrl_turn_vel.ref = ctrl_turn_dist.u;      // velcity reference ref = distance output u
    ctrl_turn_vel.y_new = ctrl_turn_dist.vel;  // velcity position y_new = distance velocity vel
    
    // Run velocity controllers
    ctrl_move_vel.run_pid();
    ctrl_turn_vel.run_pid();
  }

  // Calculate and set motor voltages
  if (enable_ctrl && problem == 0 && !run_test) {
    uL = ctrl_move_vel.u - ctrl_turn_vel.u;
    uR = ctrl_move_vel.u + ctrl_turn_vel.u; // if turn_vel > 0: turn left
    writeVoltage('L', uL);
    writeVoltage('R', uR);
  } else if (problem != 0 || (!enable_ctrl  & !run_test)) {
    uL = 0;
    uR = 0;
    writeVoltage('L', uL);
    writeVoltage('R', uR);
  }

  // Write Serial outputs
  if (SERIAL_OUTPUT && (data_request || continuous_data_request)) {

    Serial.print(millis());  // Overall Time (ms)
    Serial.print("\t");

    Serial.print(ctrl_move_dist.ref);  // Position reference
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_move_dist.y_new);  // Actual position
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_move_dist.e_int);  // Position error integral
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_move_dist.u);  // Desired Velocity
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_move_dist.vel);  // Actual Velocity
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_move_vel.u);  // PWM counts for move
    Serial.print("\t");
    Serial.print("\t");

    Serial.print(ctrl_turn_dist.ref);  // Angle reference
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_turn_dist.y_new);  // Actual angle
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_turn_dist.e_int);  // Angle error integral
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_turn_dist.u);  // Desired Angle Velocity
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_turn_dist.vel);  // Actual Angle Velocity
    Serial.print("\t");
    Serial.print("\t");
    Serial.print(ctrl_turn_vel.u);  // PWM counts for turn
    Serial.print("\t");
    Serial.print("\t");

    Serial.print(uL);  // Left Motor PWM counts
    Serial.print("\t");
    Serial.print(uR);  // Right Motor PWM counts
    Serial.print("\t");
    Serial.print("\t");

    Serial.print(millis() - last_time);  // Time for each iteration (ms)
    Serial.print("\t");
    Serial.print("\t");
    Serial.println(problem);
    data_request = false;

    if (continuous_data_request) {
      cdr_i = cdr_i + 1;
      if (cdr_i >= cdr_max) {
        continuous_data_request = false;
      }
    }

    inputString = "";
  }

  // Update times
  current_time = millis();
  while ((current_time - last_time) < Ts) {
    current_time = millis();
    
  }
  last_time = current_time;

  //
}
// end main loop


// Set all references and error integrals to 0; set encoder ticks to 0
void reset_controllers() {
  ctrl_turn_dist.ref = 0;
  ctrl_turn_dist.y_old = 0;
  ctrl_turn_dist.e_int = 0;
  ctrl_turn_vel.ref = 0;
  ctrl_turn_vel.y_old = 0;
  ctrl_turn_vel.e_int = 0;
  ctrl_move_dist.ref = 0;
  ctrl_move_dist.y_old = 0;
  ctrl_move_dist.e_int = 0;
  ctrl_move_vel.ref = 0;
  ctrl_move_vel.y_old = 0;
  ctrl_move_vel.e_int = 0;
  EncoderL.write(0);
  EncoderR.write(0);
  writeOnFinish = true;
}


// Write PWM counts to motor
void writeVoltage(char motorID, float u_float) {
  int u = (int)u_float;
  int DIR = LOW;
  if (u < 0 && motorID == 'L' || u > 0 && motorID == 'R') {
    DIR = HIGH;
  }
  u = min(abs(u), 255);
  if (motorID == 'L') {
    digitalWrite(DIRA, DIR);
    analogWrite(PWMA, u);
  } else if (motorID == 'R') {
    digitalWrite(DIRB, DIR);
    analogWrite(PWMB, u);
  }
}


// Create inputString one character at a time
void addChar(char c) {
  // if not a newline: add to string; otherwise, finish the string
  if (c != '\n' && (int)c != 0) {
    inputString += c;
  } else {
    stringComplete = true;
  }
}

// serial read command
void serialEvent() {
while(true)
 // while (Serial.available()) 
 {
    //addChar((char)Serial.read());
    //receiveEvent(1);
    piControl = piControl;
  }
}

// i2c read command
void receiveEvent(int howMany) {
  while (Wire.available()) {
    //addChar(Wire.read());

    byte inputNumber = Wire.read();
    if(inputNumber == 0xAF){ //Signature Byte
      piControl = Wire.read();
      parameter = Wire.read();
      Serial.println(piControl);
      Serial.println(parameter); 
    }
  }
  //Create input string based on I2C Command from Pi
  inputString = "";
  inputString += (String(piControl, HEX));
  inputString += String(parameter);
  Serial.println(inputString);
}

// i2c write command - write current position
void requestEvent() {
  if (writeOnFinish) {
    // combine move and turn positions
    if (abs(ctrl_move_dist.u) < 2 & abs(ctrl_move_dist.vel) < 0.1 & abs(ctrl_turn_dist.u) < 10 & abs(ctrl_turn_dist.vel) < 0.5 ) {
      // Write 2 bytes for encoder ticks - high and then low bytes
      byte toPi[4] = {EncoderLTicks / 256, EncoderLTicks % 256, EncoderRTicks / 256, EncoderRTicks % 256};
      Wire.write(toPi, 4);
      writeOnFinish = false;
    }
  }
}
