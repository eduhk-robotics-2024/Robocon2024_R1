// ASCII ART Generated with: https://textkool.com/en/ascii-art-generator?hl=default&vl=default&font=ANSI%20Regular&text=Define%20Motor%20Pins
//-------------------------------------------------Include Library-----------------------------------------------------------//


// ██████  ███████ ███████ ██ ███    ██ ███████     ██████  ███████ ██████ 
// ██   ██ ██      ██      ██ ████   ██ ██          ██   ██ ██           ██ 
// ██   ██ █████   █████   ██ ██ ██  ██ █████       ██████  ███████  █████  
// ██   ██ ██      ██      ██ ██  ██ ██ ██          ██           ██ ██     
// ██████  ███████ ██      ██ ██   ████ ███████     ██      ███████ ███████ 
//-------------------------------------------------Define PS2 Related Stuff-----------------------------------------------------------//
#include <PS2X_lib.h>
#define PS2_CLK 13  // PS2 Controller Clock pin
#define PS2_DAT 12  // PS2 Controller Data pin
#define PS2_CMD 11  // PS2 Controller Command pin
#define PS2_SEL 10  // PS2 Controller Select pin

PS2X ps2x;         // PS2X library instance for PS2 controller
int error = 0;     // Variable to store PS2 controller initialization error
byte vibrate = 0;  // Vibration control variable
byte type = 0;     // Vibration ps2 controller type
//----------------------------------------------------------------------------------------------------------------------------------//
// ██████  ███████ ██████      ███████ ███████ ████████     ██    ██ ██████ 
// ██   ██ ██           ██     ██      ██         ██        ██    ██ ██   ██ 
// ██████  ███████  █████      ███████ █████      ██        ██    ██ ██████  
// ██           ██ ██               ██ ██         ██        ██    ██ ██     
// ██      ███████ ███████     ███████ ███████    ██         ██████  ██ 
//-------------------------------------------------Set Up PS2 Receiver-----------------------------------------------------------//
void ps2_setup() {
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);  //GamePad(clock(CLK), command(CMD), attention(CS), data(DAT), Pressures?, Rumble?)

  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  }

  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }
}
//----------------------------------------------------------------------------------------------------------------------------//









// ██████  ███████ ███████ ██ ███    ██ ███████      ██████  █████  ███    ██     ██████  ██    ██ ███████ 
// ██   ██ ██      ██      ██ ████   ██ ██          ██      ██   ██ ████   ██     ██   ██ ██    ██ ██      
// ██   ██ █████   █████   ██ ██ ██  ██ █████       ██      ███████ ██ ██  ██     ██████  ██    ██ ███████ 
// ██   ██ ██      ██      ██ ██  ██ ██ ██          ██      ██   ██ ██  ██ ██     ██   ██ ██    ██      ██ 
// ██████  ███████ ██      ██ ██   ████ ███████      ██████ ██   ██ ██   ████     ██████   ██████  ███████ 
//-------------------------------------------------Define Can Bus Related Stuff-----------------------------------------------------------//
#include <SPI.h>
#include <mcp2515.h>
struct can_frame canMsgIn;            // CAN frame for incoming messages
struct can_frame canMsgOut;           // CAN frame for outgoing messages
MCP2515 mcp2515(53);                  // MCP2515 CAN bus controller instance
const int SPI_CS_PIN = 53;  // Chip Select pin for MCP2515
const int CAN_INT_PIN = 2;  // Interrupt pin for MCP2515
int FL_motor_power = 0;               // Motor power for front left
int FR_motor_power = 0;               // Motor power for front right
int BL_motor_power = 0;               // Motor power for back left
int BR_motor_power = 0;               // Motor power for back right
int motor_range = 100;                // Range of motor power
int motors_speed[] = { 0, 0, 0, 0 };  // Array to store motor speeds
int print_delay = 0;                  // Delay for serial printing
float speed_adjust = 0.2;
//---------------------------------------------------------------------------------------------------------------------------------------//
// ███    ███  ██████ ██████  ██████  ███████  ██ ███████     ███    ███  ██████  ██████  ██    ██ ██      ███████     ███████ ███████ ████████     ██    ██ ██████ 
// ████  ████ ██      ██   ██      ██ ██      ███ ██          ████  ████ ██    ██ ██   ██ ██    ██ ██      ██          ██      ██         ██        ██    ██ ██   ██ 
// ██ ████ ██ ██      ██████   █████  ███████  ██ ███████     ██ ████ ██ ██    ██ ██   ██ ██    ██ ██      █████       ███████ █████      ██        ██    ██ ██████  
// ██  ██  ██ ██      ██      ██           ██  ██      ██     ██  ██  ██ ██    ██ ██   ██ ██    ██ ██      ██               ██ ██         ██        ██    ██ ██     
// ██      ██  ██████ ██      ███████ ███████  ██ ███████     ██      ██  ██████  ██████   ██████  ███████ ███████     ███████ ███████    ██         ██████  ██ 
//-------------------------------------------------Set Up MCP2515 Module-----------------------------------------------------------//
void can_mcp2515_setup() {
  mcp2515.reset();                             // Reset MCP2515 controller
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);  // Set CAN bus bitrate and clock frequency
  mcp2515.setNormalMode();                     // Set MCP2515 to normal mode
}
//--------------------------------------------------------------------------------------------------------------------------------//
//  ██████  █████  ███    ██         ██     ██ ██████  ██ ████████ ███████  ██     ██ 
// ██      ██   ██ ████   ██         ██     ██ ██   ██ ██    ██    ██      ██       ██ 
// ██      ███████ ██ ██  ██         ██  █  ██ ██████  ██    ██    █████   ██       ██ 
// ██      ██   ██ ██  ██ ██         ██ ███ ██ ██   ██ ██    ██    ██      ██       ██ 
//  ██████ ██   ██ ██   ████ ███████  ███ ███  ██   ██ ██    ██    ███████  ██     ██  
//-------------------------------------------------Write the Motor Power to a Specfic Motor through Can_bus Using MCP2515 Module-----------------------------------------------------------//
void can_write(String motor, int power) {
  int motor_write_address = 0;

  // Determine motor write address based on the motor name
  if (motor == "FL") {
    motor_write_address = 0;
  } else if (motor == "FR") {
    motor_write_address = 2;
  } else if (motor == "BL") {
    motor_write_address = 4;
  } else if (motor == "BR") {
    motor_write_address = 6;
  }

  canMsgOut.can_id = 0x200;  // Set CAN message ID
  canMsgOut.can_dlc = 8;     // Set CAN message length

  // Set motor power values in the CAN message data
  canMsgOut.data[motor_write_address] = (power >> 8) & 0xFF;
  canMsgOut.data[motor_write_address + 1] = power & 0xFF;

  mcp2515.sendMessage(&canMsgOut);  // Send CAN message
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//  ██████  █████  ███    ██         ██████  ███████  █████  ██████          ███████ ██████  ███████ ███████ ██████   ██     ██ 
// ██      ██   ██ ████   ██         ██   ██ ██      ██   ██ ██   ██         ██      ██   ██ ██      ██      ██   ██ ██       ██ 
// ██      ███████ ██ ██  ██         ██████  █████   ███████ ██   ██         ███████ ██████  █████   █████   ██   ██ ██       ██ 
// ██      ██   ██ ██  ██ ██         ██   ██ ██      ██   ██ ██   ██              ██ ██      ██      ██      ██   ██ ██       ██ 
//  ██████ ██   ██ ██   ████ ███████ ██   ██ ███████ ██   ██ ██████  ███████ ███████ ██      ███████ ███████ ██████   ██     ██  
//-------------------------------------------------Read All Motors Current Speed through Can_bus Using MCP2515 Module-----------------------------------------------------------//
void can_read_speed() {
  int single_motor_speed = 0;

  if (mcp2515.readMessage(&canMsgIn) == MCP2515::ERROR_OK) {
    // Read speed from CAN message and map it to the motor speed range
    if (canMsgIn.can_id == 0x201) {
      single_motor_speed = (canMsgIn.data[2] << 8) + canMsgIn.data[3];
      if (single_motor_speed < -175) single_motor_speed += 175;
      motors_speed[0] = single_motor_speed;
    }
    if (canMsgIn.can_id == 0x202) {
      single_motor_speed = (canMsgIn.data[2] << 8) + canMsgIn.data[3];
      if (single_motor_speed < -175) single_motor_speed += 175;
      motors_speed[1] = single_motor_speed;
    }
    if (canMsgIn.can_id == 0x203) {
      single_motor_speed = (canMsgIn.data[2] << 8) + canMsgIn.data[3];
      if (single_motor_speed < -175) single_motor_speed += 175;
      motors_speed[2] = single_motor_speed;
    }
    if (canMsgIn.can_id == 0x204) {
      single_motor_speed = (canMsgIn.data[2] << 8) + canMsgIn.data[3];
      if (single_motor_speed < -175) single_motor_speed += 175;
      motors_speed[3] = single_motor_speed;
    }
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------//








// ██████  ███████ ███████ ██ ███    ██ ███████     ██████  ██ ██████ 
// ██   ██ ██      ██      ██ ████   ██ ██          ██   ██ ██ ██   ██ 
// ██   ██ █████   █████   ██ ██ ██  ██ █████       ██████  ██ ██   ██ 
// ██   ██ ██      ██      ██ ██  ██ ██ ██          ██      ██ ██   ██ 
// ██████  ███████ ██      ██ ██   ████ ███████     ██      ██ ██████  
//-------------------------------------------------Define PID Related Stuff-----------------------------------------------------------//
#include <PID_v1.h>
double FL_Setpoint, FL_PID_Input, FL_PID_Output;                               // Variables for front left motor PID control
double FR_Setpoint, FR_PID_Input, FR_PID_Output;                               // Variables for front right motor PID control
double BL_Setpoint, BL_PID_Input, BL_PID_Output;                               // Variables for back left motor PID control
double BR_Setpoint, BR_PID_Input, BR_PID_Output;                               // Variables for back right motor PID control
double Kp = 100, Ki = 0, Kd = 4;                                               // PID tuning parameters
PID FL_pid(&FL_PID_Input, &FL_PID_Output, &FL_Setpoint, Kp, Ki, Kd, DIRECT);   // PID controller for front left motor
PID FR_pid(&FR_PID_Input, &FR_PID_Output, &FR_Setpoint, Kp, Ki, Kd, REVERSE);  // PID controller for front right motor
PID BL_pid(&BL_PID_Input, &BL_PID_Output, &BL_Setpoint, Kp, Ki, Kd, DIRECT);   // PID controller for back left motor
PID BR_pid(&BR_PID_Input, &BR_PID_Output, &BR_Setpoint, Kp, Ki, Kd, REVERSE);  // PID controller for back right motor
//-----------------------------------------------------------------------------------------------------------------------------------//
// ██████  ██ ██████      ███████ ███████ ████████     ██    ██ ██████ 
// ██   ██ ██ ██   ██     ██      ██         ██        ██    ██ ██   ██ 
// ██████  ██ ██   ██     ███████ █████      ██        ██    ██ ██████  
// ██      ██ ██   ██          ██ ██         ██        ██    ██ ██     
// ██      ██ ██████      ███████ ███████    ██         ██████  ██ 
//-------------------------------------------------Set Up PIDs Mode and OutputLimits-----------------------------------------------------------//
void pid_setup() {
  int pid_limit = 4096;
  FL_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for front left motor
  FL_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for front left motor
  FR_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for front right motor
  FR_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for front right motor
  BL_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for back left motor
  BL_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for back left motor
  BR_pid.SetMode(AUTOMATIC);                      // Set PID to automatic mode for back right motor
  BR_pid.SetOutputLimits(-pid_limit, pid_limit);  // Set output limits for back right motor
}
//--------------------------------------------------------------------------------------------------------------------------------------------//
// ██████  ██ ██████           ██████  █████  ██       ██     ██ 
// ██   ██ ██ ██   ██         ██      ██   ██ ██      ██       ██ 
// ██████  ██ ██   ██         ██      ███████ ██      ██       ██ 
// ██      ██ ██   ██         ██      ██   ██ ██      ██       ██ 
// ██      ██ ██████  ███████  ██████ ██   ██ ███████  ██     ██  
//-------------------------------------------------Calculate the Needed Power a Specific Motor Need with PID Algorithum-----------------------------------------------------------//
double pid_cal(int motor_id, double par_Setpoint, double par_PID_INPUT) {
  if (motor_id == 1) {
    FL_Setpoint = par_Setpoint;
    FL_PID_Input = par_PID_INPUT;
    FL_pid.Compute();
    return FL_PID_Output;
  } else if (motor_id == 2) {
    FR_Setpoint = par_Setpoint;
    FR_PID_Input = par_PID_INPUT;
    FR_pid.Compute();
    return FR_PID_Output;
  } else if (motor_id == 3) {
    BL_Setpoint = par_Setpoint;
    BL_PID_Input = par_PID_INPUT;
    BL_pid.Compute();
    return BL_PID_Output;
  } else if (motor_id == 4) {
    BR_Setpoint = par_Setpoint;
    BR_PID_Input = par_PID_INPUT;
    BR_pid.Compute();
    return BR_PID_Output;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//






// ███    ███  ██████  ██    ██ ███████         ███    ███  ██████  ████████  ██████  ██████   ██     ██ 
// ████  ████ ██    ██ ██    ██ ██              ████  ████ ██    ██    ██    ██    ██ ██   ██ ██       ██ 
// ██ ████ ██ ██    ██ ██    ██ █████           ██ ████ ██ ██    ██    ██    ██    ██ ██████  ██       ██ 
// ██  ██  ██ ██    ██  ██  ██  ██              ██  ██  ██ ██    ██    ██    ██    ██ ██   ██ ██       ██ 
// ██      ██  ██████    ████   ███████ ███████ ██      ██  ██████     ██     ██████  ██   ██  ██     ██  
//-------------------------------------------------Move the 4 Motor of a Mecanum Wheel Design-----------------------------------------------------------//
void move_motor(int joy_LX, int joy_LY, int joy_RX) {
  // ███████ ████████ ███████ ██████       ██     ███    ███ ███████  ██████  █████  ███    ██ ██    ██ ███    ███      █████  ██       ██████   ██████  ██████  ██ ████████ ██   ██ ███    ███ 
  // ██         ██    ██      ██   ██     ███     ████  ████ ██      ██      ██   ██ ████   ██ ██    ██ ████  ████     ██   ██ ██      ██       ██    ██ ██   ██ ██    ██    ██   ██ ████  ████ 
  // ███████    ██    █████   ██████       ██     ██ ████ ██ █████   ██      ███████ ██ ██  ██ ██    ██ ██ ████ ██     ███████ ██      ██   ███ ██    ██ ██████  ██    ██    ███████ ██ ████ ██ 
  //      ██    ██    ██      ██           ██     ██  ██  ██ ██      ██      ██   ██ ██  ██ ██ ██    ██ ██  ██  ██     ██   ██ ██      ██    ██ ██    ██ ██   ██ ██    ██    ██   ██ ██  ██  ██ 
  // ███████    ██    ███████ ██           ██     ██      ██ ███████  ██████ ██   ██ ██   ████  ██████  ██      ██     ██   ██ ███████  ██████   ██████  ██   ██ ██    ██    ██   ██ ██      ██ 
  //Step 1: Mecanum algorithm for motion control
  int joy_deadzone = 10;
  int speed_deadzone = 2;
  int forward = map(joy_LY, 0, 255, motor_range, -motor_range);
  int sideway = map(joy_LX, 0, 255, -motor_range, motor_range);
  int turn = map(joy_RX, 0, 255, motor_range, -motor_range)*0.75;

  // Apply deadzones to joystick values
  if (forward <= joy_deadzone && forward >= -joy_deadzone) {
    forward = 0;
  }
  if (sideway <= joy_deadzone && sideway >= -joy_deadzone) {
    sideway = 0;
  }
  if (turn <= joy_deadzone && turn >= -joy_deadzone) {
    turn = 0;
  }
  // Calculate motor speeds based on joystick input
  int FL_speed = constrain((forward + sideway - turn), -motor_range, motor_range);
  int FR_speed = constrain((forward - sideway + turn), -motor_range, motor_range);
  int BL_speed = constrain((forward - sideway - turn), -motor_range, motor_range);
  int BR_speed = constrain((forward + sideway + turn), -motor_range, motor_range);

  if (ps2x.ButtonPressed(PSB_L1) && speed_adjust > 0.1) {
    speed_adjust -= 0.2;
  }
  if (ps2x.ButtonPressed(PSB_R1) && speed_adjust < 1) {
    speed_adjust += 0.2;
  }

  FL_speed *= speed_adjust;
  FR_speed *= speed_adjust;
  BL_speed *= speed_adjust;
  BR_speed *= speed_adjust;
  // analogWrite(wheel_speed_led_pin, int(128 * speed_adjust));

  // Apply deadzones to motor speeds
  if (FL_speed <= speed_deadzone && FL_speed >= -speed_deadzone) {
    FL_speed = 0;
  }
  if (FR_speed <= speed_deadzone && FR_speed >= -speed_deadzone) {
    FR_speed = 0;
  }
  if (BL_speed <= speed_deadzone && BL_speed >= -speed_deadzone) {
    BL_speed = 0;
  }
  if (BR_speed <= speed_deadzone && BR_speed >= -speed_deadzone) {
    BR_speed = 0;
  }

  // ███████ ████████ ███████ ██████      ██████      ██████  ███████  █████  ██████       ██████ ██    ██ ██████  ██████  ███████ ███    ██ ████████     ███    ███  ██████  ████████  ██████  ██████      ███████ ██████  ███████ ███████ ██████  ███████ 
  // ██         ██    ██      ██   ██          ██     ██   ██ ██      ██   ██ ██   ██     ██      ██    ██ ██   ██ ██   ██ ██      ████   ██    ██        ████  ████ ██    ██    ██    ██    ██ ██   ██     ██      ██   ██ ██      ██      ██   ██ ██      
  // ███████    ██    █████   ██████       █████      ██████  █████   ███████ ██   ██     ██      ██    ██ ██████  ██████  █████   ██ ██  ██    ██        ██ ████ ██ ██    ██    ██    ██    ██ ██████      ███████ ██████  █████   █████   ██   ██ ███████ 
  //      ██    ██    ██      ██          ██          ██   ██ ██      ██   ██ ██   ██     ██      ██    ██ ██   ██ ██   ██ ██      ██  ██ ██    ██        ██  ██  ██ ██    ██    ██    ██    ██ ██   ██          ██ ██      ██      ██      ██   ██      ██ 
  // ███████    ██    ███████ ██          ███████     ██   ██ ███████ ██   ██ ██████       ██████  ██████  ██   ██ ██   ██ ███████ ██   ████    ██        ██      ██  ██████     ██     ██████  ██   ██     ███████ ██      ███████ ███████ ██████  ███████ 
  // Step 2: Read current motor speeds
  can_read_speed();
  int FL_current_speed = map(motors_speed[0], -10000, 10000, -motor_range, motor_range);
  int FR_current_speed = map(motors_speed[1], -10000, 10000, -motor_range, motor_range);
  int BL_current_speed = map(motors_speed[2], -10000, 10000, -motor_range, motor_range);
  int BR_current_speed = map(motors_speed[3], -10000, 10000, -motor_range, motor_range);


  // ███████ ████████ ███████ ██████      ██████      ██████  ███████ ██████  ███████  ██████  ██████  ███    ███     ██████  ██ ██████       ██████  ██████  ███    ██ ████████ ██████   ██████  ██          ███████  ██████  ██████      ███████  █████   ██████ ██   ██     ███    ███  ██████  ████████  ██████  ██████ 
  // ██         ██    ██      ██   ██          ██     ██   ██ ██      ██   ██ ██      ██    ██ ██   ██ ████  ████     ██   ██ ██ ██   ██     ██      ██    ██ ████   ██    ██    ██   ██ ██    ██ ██          ██      ██    ██ ██   ██     ██      ██   ██ ██      ██   ██     ████  ████ ██    ██    ██    ██    ██ ██   ██ 
  // ███████    ██    █████   ██████       █████      ██████  █████   ██████  █████   ██    ██ ██████  ██ ████ ██     ██████  ██ ██   ██     ██      ██    ██ ██ ██  ██    ██    ██████  ██    ██ ██          █████   ██    ██ ██████      █████   ███████ ██      ███████     ██ ████ ██ ██    ██    ██    ██    ██ ██████  
  //      ██    ██    ██      ██               ██     ██      ██      ██   ██ ██      ██    ██ ██   ██ ██  ██  ██     ██      ██ ██   ██     ██      ██    ██ ██  ██ ██    ██    ██   ██ ██    ██ ██          ██      ██    ██ ██   ██     ██      ██   ██ ██      ██   ██     ██  ██  ██ ██    ██    ██    ██    ██ ██   ██ 
  // ███████    ██    ███████ ██          ██████      ██      ███████ ██   ██ ██       ██████  ██   ██ ██      ██     ██      ██ ██████       ██████  ██████  ██   ████    ██    ██   ██  ██████  ███████     ██       ██████  ██   ██     ███████ ██   ██  ██████ ██   ██     ██      ██  ██████     ██     ██████  ██   ██ 
  // Step 3: Perform PID control for each motor
  double FL_Power = pid_cal(1, FL_speed, FL_current_speed);
  double FR_Power = pid_cal(2, FR_speed, -FR_current_speed);
  double BL_Power = pid_cal(3, BL_speed, BL_current_speed);
  double BR_Power = pid_cal(4, BR_speed, -BR_current_speed);

  // Print motor speeds, desired speeds, and calculated powers for debugging
  print_delay += 1;
  if (print_delay == 10) {
    Serial.print("\nSpeed level: ");
    Serial.print(int(speed_adjust * 10));
    Serial.print("\nFL_current_speed(mapped): ");
    Serial.print(FL_current_speed);
    Serial.print("    FL_current_speed(unmapped): ");
    Serial.print(motors_speed[0]);
    Serial.print("    FL_speed: ");
    Serial.print(FL_speed);
    Serial.print("    FL_Power: ");
    Serial.print(FL_Power);

    Serial.print("\nFR_current_speed(mapped): ");
    Serial.print(FR_current_speed);
    Serial.print("    FR_current_speed(unmapped): ");
    Serial.print(motors_speed[1]);
    Serial.print("    FR_speed: ");
    Serial.print(FR_speed);
    Serial.print("    FR_Power: ");
    Serial.print(FR_Power);

    Serial.print("\nBL_current_speed(mapped): ");
    Serial.print(BL_current_speed);
    Serial.print("    BL_current_speed(unmapped): ");
    Serial.print(motors_speed[2]);
    Serial.print("    BL_speed: ");
    Serial.print(BL_speed);
    Serial.print("    BL_Power: ");
    Serial.print(BL_Power);

    Serial.print("\nBR_current_speed(mapped): ");
    Serial.print(BR_current_speed);
    Serial.print("    BR_current_speed(unmapped): ");
    Serial.print(motors_speed[3]);
    Serial.print("    BR_speed: ");
    Serial.print(BR_speed);
    Serial.print("    BR_Power: ");
    Serial.print(BR_Power);

    Serial.print("\n\n");
    print_delay = 0;
  }


  // ███████ ████████ ███████ ██████      ██   ██     ██     ██ ██████  ██ ████████ ███████     ███    ███  ██████  ████████  ██████  ██████      ██████   ██████  ██     ██ ███████ ██████  ███████     ████████  ██████      ████████ ██   ██ ███████      ██████  █████  ███    ██     ██████  ██    ██ ███████ 
  // ██         ██    ██      ██   ██     ██   ██     ██     ██ ██   ██ ██    ██    ██          ████  ████ ██    ██    ██    ██    ██ ██   ██     ██   ██ ██    ██ ██     ██ ██      ██   ██ ██             ██    ██    ██        ██    ██   ██ ██          ██      ██   ██ ████   ██     ██   ██ ██    ██ ██      
  // ███████    ██    █████   ██████      ███████     ██  █  ██ ██████  ██    ██    █████       ██ ████ ██ ██    ██    ██    ██    ██ ██████      ██████  ██    ██ ██  █  ██ █████   ██████  ███████        ██    ██    ██        ██    ███████ █████       ██      ███████ ██ ██  ██     ██████  ██    ██ ███████ 
  //      ██    ██    ██      ██               ██     ██ ███ ██ ██   ██ ██    ██    ██          ██  ██  ██ ██    ██    ██    ██    ██ ██   ██     ██      ██    ██ ██ ███ ██ ██      ██   ██      ██        ██    ██    ██        ██    ██   ██ ██          ██      ██   ██ ██  ██ ██     ██   ██ ██    ██      ██ 
  // ███████    ██    ███████ ██               ██      ███ ███  ██   ██ ██    ██    ███████     ██      ██  ██████     ██     ██████  ██   ██     ██       ██████   ███ ███  ███████ ██   ██ ███████        ██     ██████         ██    ██   ██ ███████      ██████ ██   ██ ██   ████     ██████   ██████  ███████ 
  // Step 4: Write motor powers to the CAN bus
  can_write("FL", int(FL_Power));
  can_write("FR", int(FR_Power));
  can_write("BL", int(BL_Power));
  can_write("BR", int(BR_Power));
}






// ██    ██  ██████  ██ ██████      ███████ ███████ ████████ ██    ██ ██████       ██     ██ 
// ██    ██ ██    ██ ██ ██   ██     ██      ██         ██    ██    ██ ██   ██     ██       ██ 
// ██    ██ ██    ██ ██ ██   ██     ███████ █████      ██    ██    ██ ██████      ██       ██ 
//  ██  ██  ██    ██ ██ ██   ██          ██ ██         ██    ██    ██ ██          ██       ██ 
//   ████    ██████  ██ ██████      ███████ ███████    ██     ██████  ██           ██     ██  
void setup() {

  while (!Serial) {
    continue;  // Wait for serial connection to be established
  }
  Serial.begin(115200);  // Initialize serial communication

  ps2_setup();          // Setup PS2 controller
  can_mcp2515_setup();  // Setup MCP2515 CAN bus controller
  pid_setup();          // Setup PID controllers

}
//---------------------------------------------------------------------------------------------------------------



// ██    ██  ██████  ██ ██████      ██       ██████   ██████  ██████       ██     ██ 
// ██    ██ ██    ██ ██ ██   ██     ██      ██    ██ ██    ██ ██   ██     ██       ██ 
// ██    ██ ██    ██ ██ ██   ██     ██      ██    ██ ██    ██ ██████      ██       ██ 
//  ██  ██  ██    ██ ██ ██   ██     ██      ██    ██ ██    ██ ██          ██       ██ 
//   ████    ██████  ██ ██████      ███████  ██████   ██████  ██           ██     ██  
void loop() {
  ps2x.read_gamepad(false, 1);                                                // Read input from PS2 controller
  move_motor(ps2x.Analog(PSS_LX), ps2x.Analog(PSS_LY), ps2x.Analog(PSS_RX));  // Move the motor based on joystick input

  delay(10);  // Small delay between iterations
} 
