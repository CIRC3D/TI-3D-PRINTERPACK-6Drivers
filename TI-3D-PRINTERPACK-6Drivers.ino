// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

/* Modified by CIRC3D for Texas Instruments/UTD
        Original project is Tonokip, repository: https://github.com/tonokip/Tonokip-Firmware
	Works with Energia for MSP430F5529 now, not Arduino
	June 2015
        This code is licensed as GPLv2
   For use with the CIRC3D BoosterPack 
*/

#include <msp430.h>
#include <Energia.h>
#include <msp430f5529.h>

#include "configuration.h"
#include "pins.h"
#include "ThermistorTable.h"
#include "custom.h"


//Implemented Codes
//-------------------
// G0 -> G1
// G1  - Coordinated Movement X Y Z E Q B
// G4  - Dwell S<seconds> or P<milliseconds>
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set target temp
// M105 - Read current temp
// M109 - Wait for current temp to reach target temp.

//Custom M Codes
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E,Q and B codes absolute (default)
// M83  - Set E, Q and B codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M86  - If Endstop is Not Activated then Abort Print. Specify X and/or Y
// M92  - Set axis_steps_per_unit - same syntax as G92

//Stepper Movement Variables
bool home_all_axis = true;

// The axis order in all axis related arrays is X, Y, Z, E, Q, B
const int NUM_AXIS = 6;
char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E', 'Q', 'B'};
bool direction_x, direction_y, direction_z, direction_e, direction_q, direction_b;
unsigned long previous_micros=0, previous_micros_x=0, previous_micros_y=0, previous_micros_z=0,
previous_micros_e=0, previous_micros_q=0, previous_micros_b=0, previous_millis_heater;
unsigned long x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, q_steps_to_take, b_steps_to_take;
float destination_x =0.0, destination_y = 0.0, destination_z = 0.0, destination_e = 0.0, destination_q = 0.0, destination_b = 0.0;
float current_x = 0.0, current_y = 0.0, current_z = 0.0, current_e = 0.0, current_q = 0.0, current_b = 0.0;

// for speed delay
float x_interval, y_interval, z_interval, e_interval, q_interval, b_interval;
float feedrate = 1500, next_feedrate, saved_feedrate;
float time_for_move;
long gcode_N, gcode_LastN;

//Determine Absolute or Relative Coordinates
bool relative_mode = false;

//Determine Absolute or Relative E, Q, B Codes while in Absolute Coordinates mode. E, Q, B is always relative in Relative Coordinates mode.
bool relative_mode_e = false;
bool relative_mode_q = false;
bool relative_mode_b = false;

// comm variables
#define MAX_CMD_SIZE 256
char cmdbuffer[MAX_CMD_SIZE];
char serial_char;
int serial_count = 0;
boolean comment_mode = false;

// just a pointer to find chars in the cmd string like X, Y, Z, E, Q, B
char *strchr_pointer;

//manage heater variables
int target_raw_0 = 0, target_raw_1 = 0, target_raw_2 = 0;
int current_raw_0, current_raw_1, current_raw_2;
int temp_raw;

//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;

// ==========================================  TOOLHEAD SPECIFICATIONS  ==========================================
const int NUMBER_OF_TOOLS = 2; // change this to the number of tools the machine has hooked up
// do not change first number (tool reference), syntax is tool #, X offset from tool 0, Y offset from tool 0, Z offset from tool 0 
const float TOOLCHANGER[4][4] = {{0, 0, 0, 0}, // this is the tool that all other tools are referenced from
                                 {1, 0, 0, 0}, // optional tool 1, x, y, and z offset from tool 0
                                 {2, 0, 0, 0}, // optional tool 2, x, y, and z offset from tool 0
                                 {3, 0, 0, 0}}; // optional tool 3, x, y, and z offset from tool 0
float x_0, y_0, z_0;
int current_tool = 0; // default is tool 0
int current_tool_previous;
float toolhead_clearance = 0.5; // distance to move other tools away from active tool (for milling deep into parts, make this larger to keep other toolheads out of the way.  
//This can also be changed with T14 command)
// ================================================================================================================

void setup()
{ 
  //Initialize Step Pins
  if(X_STEP_PIN > -1) pinMode(X_STEP_PIN,OUTPUT);
  if(Y_STEP_PIN > -1) pinMode(Y_STEP_PIN,OUTPUT);
  if(Z_STEP_PIN > -1) pinMode(Z_STEP_PIN,OUTPUT);
  if(E_STEP_PIN > -1) pinMode(E_STEP_PIN,OUTPUT);
  if(Q_STEP_PIN > -1) pinMode(Q_STEP_PIN,OUTPUT);
  if(B_STEP_PIN > -1) pinMode(B_STEP_PIN,OUTPUT);
  
  //Initialize Dir Pins
  if(X_DIR_PIN > -1) pinMode(X_DIR_PIN,OUTPUT);
  if(Y_DIR_PIN > -1) pinMode(Y_DIR_PIN,OUTPUT);
  if(Z_DIR_PIN > -1) pinMode(Z_DIR_PIN,OUTPUT);
  if(E_DIR_PIN > -1) pinMode(E_DIR_PIN,OUTPUT);
  if(Q_DIR_PIN > -1) pinMode(Q_DIR_PIN,OUTPUT);
  if(B_DIR_PIN > -1) pinMode(B_DIR_PIN,OUTPUT);

  //Steppers default to disabled.
  if(X_ENABLE_PIN > -1) if(!X_ENABLE_ON) digitalWrite(X_ENABLE_PIN,HIGH);
  if(Y_ENABLE_PIN > -1) if(!Y_ENABLE_ON) digitalWrite(Y_ENABLE_PIN,HIGH);
  if(Z_ENABLE_PIN > -1) if(!Z_ENABLE_ON) digitalWrite(Z_ENABLE_PIN,HIGH);
  if(E_ENABLE_PIN > -1) if(!E_ENABLE_ON) digitalWrite(E_ENABLE_PIN,HIGH);
  if(Q_ENABLE_PIN > -1) if(!Q_ENABLE_ON) digitalWrite(Q_ENABLE_PIN,HIGH);
  if(B_ENABLE_PIN > -1) if(!B_ENABLE_ON) digitalWrite(B_ENABLE_PIN,HIGH);
  
  //Initialize Enable Pins
  if(X_ENABLE_PIN > -1) pinMode(X_ENABLE_PIN,OUTPUT);
  if(Y_ENABLE_PIN > -1) pinMode(Y_ENABLE_PIN,OUTPUT);
  if(Z_ENABLE_PIN > -1) pinMode(Z_ENABLE_PIN,OUTPUT);
  if(E_ENABLE_PIN > -1) pinMode(E_ENABLE_PIN,OUTPUT);
  if(Q_ENABLE_PIN > -1) pinMode(Q_ENABLE_PIN,OUTPUT);
  if(B_ENABLE_PIN > -1) pinMode(B_ENABLE_PIN,OUTPUT);
  
  //endstops and pullups
  if(ENDSTOPPULLUPS){
    if(X_MIN_PIN > -1) pinMode(X_MIN_PIN, INPUT_PULLUP);
    if(X_MAX_PIN > -1) pinMode(X_MAX_PIN, INPUT_PULLUP);
    if(Y_MIN_PIN > -1) pinMode(Y_MIN_PIN, INPUT_PULLUP);
    if(Y_MAX_PIN > -1) pinMode(Y_MAX_PIN, INPUT_PULLUP);
    if(Z_MIN_PIN > -1) pinMode(Z_MIN_PIN, INPUT_PULLUP);
    if(Z_MAX_PIN > -1) pinMode(Z_MAX_PIN, INPUT_PULLUP);}
  else{
    if(X_MIN_PIN > -1) pinMode(X_MIN_PIN, INPUT);
    if(X_MAX_PIN > -1) pinMode(X_MAX_PIN, INPUT);
    if(Y_MIN_PIN > -1) pinMode(Y_MIN_PIN, INPUT);
    if(Y_MAX_PIN > -1) pinMode(Y_MAX_PIN, INPUT);
    if(Z_MIN_PIN > -1) pinMode(Z_MIN_PIN, INPUT);
    if(Z_MAX_PIN > -1) pinMode(Z_MAX_PIN, INPUT);}

  if(HEATER_0_PIN > -1) pinMode(HEATER_0_PIN,OUTPUT);
  if(HEATER_1_PIN > -1) pinMode(HEATER_1_PIN,OUTPUT);
  if(HEATER_2_PIN > -1) pinMode(HEATER_2_PIN,OUTPUT);
  
  
  Serial.begin(BAUDRATE);
  Serial.println("start");
}


void loop()
{
  get_command();
  manage_heater();
  
  //shutdown if not receiving any new commands
  manage_inactivity(1);
}

inline void get_command() 
{
  if( Serial.available() > 0 ) {
    serial_char = Serial.read();
    if(serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_count >= (MAX_CMD_SIZE - 1) ) 
    {
      //if empty line,
      if(!serial_count) return;
      //terminate string
      cmdbuffer[serial_count] = 0;
      Serial.print("Echo:");
      Serial.println(&cmdbuffer[0]);
      
      process_commands();
      
      //for new command
      comment_mode = false;
      //clear buffer
      serial_count = 0;
      //Serial.println("ok"); 
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[serial_count++] = serial_char; 
    }
  }  
}


//#define code_num (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL))
//inline void code_search(char code) { strchr_pointer = strchr(cmdbuffer, code); }
inline float code_value() { return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL)); }
inline long code_value_long() { return (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10)); }
//Return True if the string was found
inline bool code_seen(char code_string[]) { return (strstr(cmdbuffer, code_string) != NULL); }

inline bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer, code);
  //Return True if a character was found
  return (strchr_pointer != NULL);
}



inline void process_commands()
{
  //throw away variable
  unsigned long codenum;
  
  if(code_seen('N'))
  {
    gcode_N = code_value_long();
    if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer, "M110") == NULL) ) {
    //if(gcode_N != gcode_LastN+1 && !code_seen("M110") ) {
      //Hmm, compile size is different between using this vs the line above even though it should be the same thing. Keeping old method.
      Serial.print("Serial Error: Line Number is not Last Line Number+1, Last Line:");
      Serial.println(gcode_LastN);
      FlushSerialRequestResend();
      return;
    }
    
    if(code_seen('*'))
    {
      byte checksum = 0;
      byte count=0;
      while(cmdbuffer[count] != '*') checksum = checksum^cmdbuffer[count++];
     
      if( (int)code_value() != checksum) {
        Serial.print("Error: checksum mismatch, Last Line:");
        Serial.println(gcode_LastN);
        FlushSerialRequestResend();
        return;
      }
      //if no errors, continue parsing
    }
    else 
    {
      Serial.print("Error: No Checksum with line number, Last Line:");
      Serial.println(gcode_LastN);
      FlushSerialRequestResend();
      return;
    }
    
    gcode_LastN = gcode_N;
    //if no errors, continue parsing
  }
  // if we don't receive 'N' but still see '*'
  else
  {
    if(code_seen('*'))
    {
      Serial.print("Error: No Line Number with checksum, Last Line:");
      Serial.println(gcode_LastN);
      return;
    }
  }

  //continues parsing only if we don't receive any 'N' or '*' or no errors if we do. :)
  
  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        get_coordinates(); // For X Y Z E Q B F
        
        if(destination_x > current_x) x_steps_to_take = (destination_x - current_x)*x_steps_per_unit;
        else x_steps_to_take = (current_x - destination_x)*x_steps_per_unit;
        if(destination_y > current_y) y_steps_to_take = (destination_y - current_y)*y_steps_per_unit;
        else y_steps_to_take = (current_y - destination_y)*y_steps_per_unit;
        if(destination_z > current_z) z_steps_to_take = (destination_z - current_z)*z_steps_per_unit;
        else z_steps_to_take = (current_z - destination_z)*z_steps_per_unit;
        if(destination_e > current_e) e_steps_to_take = (destination_e - current_e)*e_steps_per_unit;
        else e_steps_to_take = (current_e - destination_e)*e_steps_per_unit;
        if(destination_q > current_q) q_steps_to_take = (destination_q - current_q)*q_steps_per_unit;
        else q_steps_to_take = (current_q - destination_q)*q_steps_per_unit;
        if(destination_b > current_b) b_steps_to_take = (destination_b - current_b)*b_steps_per_unit;
        else b_steps_to_take = (current_b - destination_b)*b_steps_per_unit;

        #define X_TIME_FOR_MOVE ((float)x_steps_to_take / (x_steps_per_unit*feedrate/60000000))
        #define Y_TIME_FOR_MOVE ((float)y_steps_to_take / (y_steps_per_unit*feedrate/60000000))
        #define Z_TIME_FOR_MOVE ((float)z_steps_to_take / (z_steps_per_unit*feedrate/60000000))
        #define E_TIME_FOR_MOVE ((float)e_steps_to_take / (e_steps_per_unit*feedrate/60000000))
        #define Q_TIME_FOR_MOVE ((float)q_steps_to_take / (q_steps_per_unit*feedrate/60000000))
        #define B_TIME_FOR_MOVE ((float)b_steps_to_take / (b_steps_per_unit*feedrate/60000000))
        
        time_for_move = max(X_TIME_FOR_MOVE,Y_TIME_FOR_MOVE);
        time_for_move = max(time_for_move,Z_TIME_FOR_MOVE);
        time_for_move = max(time_for_move,E_TIME_FOR_MOVE);
        time_for_move = max(time_for_move,Q_TIME_FOR_MOVE);
        time_for_move = max(time_for_move,B_TIME_FOR_MOVE);

        if(x_steps_to_take) x_interval = time_for_move/x_steps_to_take;
        if(y_steps_to_take) y_interval = time_for_move/y_steps_to_take;
        if(z_steps_to_take) z_interval = time_for_move/z_steps_to_take;
        if(e_steps_to_take) e_interval = time_for_move/e_steps_to_take;
        if(q_steps_to_take) q_interval = time_for_move/q_steps_to_take;
        if(b_steps_to_take) b_interval = time_for_move/b_steps_to_take;
        
        #define DEBUGGING true
        if(DEBUGGING) {
          Serial.print("destination_x: "); Serial.println(destination_x); 
          Serial.print("current_x: "); Serial.println(current_x); 
          Serial.print("x_steps_to_take: "); Serial.println(x_steps_to_take); 
          Serial.print("X_TIME_FOR_MVE: "); Serial.println(X_TIME_FOR_MOVE); 
          Serial.print("x_interval: "); Serial.println(x_interval); 
          Serial.println("");
          Serial.print("destination_y: "); Serial.println(destination_y); 
          Serial.print("current_y: "); Serial.println(current_y); 
          Serial.print("y_steps_to_take: "); Serial.println(y_steps_to_take); 
          Serial.print("Y_TIME_FOR_MVE: "); Serial.println(Y_TIME_FOR_MOVE); 
          Serial.print("y_interval: "); Serial.println(y_interval); 
          Serial.println("");
          Serial.print("destination_z: "); Serial.println(destination_z); 
          Serial.print("current_z: "); Serial.println(current_z); 
          Serial.print("z_steps_to_take: "); Serial.println(z_steps_to_take); 
          Serial.print("Z_TIME_FOR_MVE: "); Serial.println(Z_TIME_FOR_MOVE); 
          Serial.print("z_interval: "); Serial.println(z_interval); 
          Serial.println("");
          Serial.print("destination_e: "); Serial.println(destination_e); 
          Serial.print("current_e: "); Serial.println(current_e); 
          Serial.print("e_steps_to_take: "); Serial.println(e_steps_to_take); 
          Serial.print("E_TIME_FOR_MVE: "); Serial.println(E_TIME_FOR_MOVE); 
          Serial.print("e_interval: "); Serial.println(e_interval); 
          Serial.println("");
          Serial.print("destination_q: "); Serial.println(destination_q); 
          Serial.print("current_q: "); Serial.println(current_q); 
          Serial.print("q_steps_to_take: "); Serial.println(q_steps_to_take); 
          Serial.print("Q_TIME_FOR_MVE: "); Serial.println(Q_TIME_FOR_MOVE); 
          Serial.print("q_interval: "); Serial.println(q_interval); 
          Serial.println("");
          Serial.print("destination_b: "); Serial.println(destination_b); 
          Serial.print("current_b: "); Serial.println(current_b); 
          Serial.print("b_steps_to_take: "); Serial.println(b_steps_to_take); 
          Serial.print("B_TIME_FOR_MVE: "); Serial.println(B_TIME_FOR_MOVE); 
          Serial.print("b_interval: "); Serial.println(b_interval); 
          Serial.println("");
        }
        // make the move
        //linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, q_steps_to_take, b_steps_to_take);
        linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, q_steps_to_take, b_steps_to_take);
        ClearToSend();
        return;
      case 4: // G4 dwell
        codenum = 0;
        // milliseconds to wait
        if(code_seen('P')) codenum = code_value();
        // seconds to wait
        if(code_seen('S')) codenum = code_value()*1000;
        // keep track of when we started waiting
        previous_millis_heater = millis();
        //manage heater until time is up
        while((millis() - previous_millis_heater) < codenum ) manage_heater();
        Serial.println("ok");
        break;
      
      case 28: // G28 Homing
        saved_feedrate = feedrate;
        home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));
        if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) 
        {
          if ((X_MIN_PIN > -1 && X_HOME_DIR==-1) || (X_MAX_PIN > -1 && X_HOME_DIR==1))
          {
            destination_x = 0;
            current_x = -1.5*X_MAX_LENGTH*X_HOME_DIR;
            direction_x = 0;
            x_steps_to_take = abs(destination_x - current_x)*x_steps_per_unit;
            y_steps_to_take = 0; z_steps_to_take = 0; e_steps_to_take = 0; q_steps_to_take = 0; b_steps_to_take = 0;
            feedrate = homing_feedrate[X_AXIS];
            #define X_TIME_FOR_MOVE ((float)x_steps_to_take / (x_steps_per_unit*feedrate/60000000))
            time_for_move = X_TIME_FOR_MOVE;
            if(x_steps_to_take) x_interval = time_for_move/x_steps_to_take;
            // make the move
            linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, q_steps_to_take, b_steps_to_take);
          }
        }
        //showString(PSTR("HOME X AXIS\r\n"));

        if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) 
        {
          if ((Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (Y_MAX_PIN > -1 && Y_HOME_DIR==1))
          {
            destination_y = 0;
            current_y = -1.5*Y_MAX_LENGTH*Y_HOME_DIR;
            direction_y = 0;
            y_steps_to_take = abs(destination_y - current_y)*y_steps_per_unit;
            x_steps_to_take = 0; z_steps_to_take = 0; e_steps_to_take = 0; q_steps_to_take = 0; b_steps_to_take = 0;
            feedrate = homing_feedrate[Y_AXIS];
            #define Y_TIME_FOR_MOVE ((float)y_steps_to_take / (y_steps_per_unit*feedrate/60000000))
            time_for_move = Y_TIME_FOR_MOVE;
            if(y_steps_to_take) y_interval = time_for_move/y_steps_to_take;
            // make the move
            linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, q_steps_to_take, b_steps_to_take);
          }
        }
        //showString(PSTR("HOME Y AXIS\r\n"));

        if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) 
        {
          if ((Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (Z_MAX_PIN > -1 && Z_HOME_DIR==1))
          {
            destination_z = 0;
            current_z = -1.5*Z_MAX_LENGTH*Z_HOME_DIR;
            direction_z = 0;
            z_steps_to_take = abs(destination_z - current_z)*z_steps_per_unit;
            x_steps_to_take = 0; y_steps_to_take = 0; e_steps_to_take = 0; q_steps_to_take = 0; b_steps_to_take = 0;
            feedrate = homing_feedrate[Z_AXIS];
            #define Z_TIME_FOR_MOVE ((float)z_steps_to_take / (z_steps_per_unit*feedrate/60000000))
            time_for_move = Z_TIME_FOR_MOVE;
            if(z_steps_to_take) z_interval = time_for_move/z_steps_to_take;
            // make the move
            linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, q_steps_to_take, b_steps_to_take);
          }
        }
        //showString(PSTR("HOME Z AXIS\r\n"));
        
        feedrate = saved_feedrate;
        ClearToSend();
        break;   
        
      case 90: // G90
        relative_mode = false;
        Serial.println("ok");
        break;
      case 91: // G91
        relative_mode = true;
        Serial.println("ok");
        break;
      case 92: // G92
        if(code_seen('X')) current_x = code_value();
        if(code_seen('Y')) current_y = code_value();
        if(code_seen('Z')) current_z = code_value();
        if(code_seen('E')&& (current_tool==0)) current_e = code_value();
        if(code_seen('E')&& (current_tool==1)) current_q = code_value();
        if(code_seen('E')&& (current_tool==2)) current_b = code_value();
        Serial.println("ok");
        break;
        
    }
  }

  if(code_seen('M'))
  {
    
    switch((int)code_value()) 
    {
      case 104: // M104 - Set Extruder target temp
        
		//Test 1/////////////////////////////////////////////// - This doesn't work correctly 
		/*
		if (code_seen('S')) 
		{
			temp_raw = temp2analog(code_value());
		if (code_seen('T')) 
			{
			switch((int)code_value())
			case 0: target_raw_0 = temp_raw;
			break;
			case 1: target_raw_1 = temp_raw;
			break;
			}
		}
		
        Serial.println("ok");
        break;*/
		
		// Test 2////////////////////////////////////////////////
		
		//if (code_seen('S')) target_raw_0  = temp2analog(code_value());
		
		if (code_seen('S')) target_raw_1  = temp2analog(code_value()); // This will turn them both on the same time if both are uncommented  
        
		Serial.println("ok");
        break;
		
      case 105: // M105 - Read Temp
        //Serial.print("ok T0:"); 
        //Serial.print(analog2temp(analogRead(TEMP_0_PIN))); //Extruder 1
		Serial.print("ok T1:");
        Serial.print(analog2temp(analogRead(TEMP_1_PIN)));   //Extruder 2
        Serial.print(" B:");
        Serial.println(analog2temp(analogRead(TEMP_2_PIN))); //Heating Bed
        if(!code_seen('N')) return;  // If M105 is sent from generated gcode, then it needs a response.
        break;
		
      case 109: // M109 - Wait for extruder heater to reach target.
        
		//Test 1////////////////////////////////////////////
		/*
		if (code_seen('S')) {
			temp_raw = temp2analog(code_value());
			if (code_seen('T')) {
			switch((int)code_value()){
			
			case 0: 
			target_raw_0 = temp_raw;
			previous_millis_heater = millis();
			while(current_raw_0 < target_raw_0) {
			if( (millis()-previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
			{
				Serial.print("T:"); // this needs to be T, I tried T1 and the Temp wasn't reading for some reason. 
				Serial.println( analog2temp(analogRead(TEMP_0_PIN)) ); 
				previous_millis_heater = millis(); 
			}
			manage_heater();
			}
			Serial.println("ok");
			break;
			
			case 1: 
			target_raw_1 = temp_raw;
			previous_millis_heater = millis();
			while(current_raw_1 < target_raw_1) {
			if( (millis()-previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
			{
				Serial.print("T:"); // this needs to be T, I tried T1 and the Temp wasn't reading for some reason. 
				Serial.println( analog2temp(analogRead(TEMP_1_PIN)) ); 
				previous_millis_heater = millis(); 
			}
			manage_heater();
			}
			Serial.println("ok");
			break;
			
			}
		}
		
		}
		*/
		
		
		// Test 2/////////////////////////////////////
		/*
		if (code_seen('S')) {
          int val = code_value();
          target_raw_0 = temp2analog(val);
          target_raw_1 = temp2analog(val);
        }
        previous_millis_heater = millis(); 
        while((current_raw_0 < target_raw_0) && (current_raw_1 < target_raw_1)) {
          if( (millis()-previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            Serial.print("T0:");
            Serial.println( analog2temp(analogRead(TEMP_0_PIN)) ); 
            //TODO: have no idea how software will react for additional temperature reading
            Serial.print("T1:");
            Serial.println( analog2temp(analogRead(TEMP_1_PIN)) ); 
            previous_millis_heater = millis(); 
          }
          manage_heater();
        }
        Serial.println("ok");
        break;
		*/
		
		//Test 3/////////////////////////////////////// - Just rewrote the original if statement for both
		/*
		if (code_seen('S')) target_raw_0 = temp2analog(code_value()); // Extruder 1 
        previous_millis_heater = millis(); 
        while(current_raw_0 < target_raw_0) {
          if( (millis()-previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            Serial.print("T0:");
            Serial.println( analog2temp(analogRead(TEMP_0_PIN)) ); 
            previous_millis_heater = millis(); 
          }
          manage_heater();
        }
        Serial.println("ok");
        break; */
		
		if (code_seen('S')) target_raw_1 = temp2analog(code_value()); // Extruder 2
        previous_millis_heater = millis(); 
        while(current_raw_1 < target_raw_1) {
          if( (millis()-previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            Serial.print("T1:");
            Serial.println( analog2temp(analogRead(TEMP_1_PIN)) ); 
            previous_millis_heater = millis(); 
          }
          manage_heater();
        }
        Serial.println("ok");
        break;  

      case 140:  //M140 - Set bed temp
        if (code_seen('S')) target_raw_2 = temp2analog(code_value()); // I changed it to target_raw_2 from 1 
        Serial.println("ok");
        break;
		
      case 190:  //M190 - Wait for bed heater to reach target
        if (code_seen('S')) target_raw_2 = temp2analog(code_value()); // I changed it to target_raw_2 from 1
        previous_millis_heater = millis(); 
        while(current_raw_2 < target_raw_2) { // again with 2
          if( (millis()-previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            Serial.print("B:");
            Serial.println( analog2temp(analogRead(TEMP_2_PIN)) );  // 2
            previous_millis_heater = millis(); 
          }
          manage_heater();
        }
        Serial.println("ok");
        break;
		
      case 80: 
        if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,OUTPUT); //GND
        Serial.println("ok");
        break;
		
      case 81: 
        if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT); //Floating
        Serial.println("ok");
        break;
		
      case 82: 
        relative_mode_e = false;
        relative_mode_q = false;
        relative_mode_b = false;
        Serial.println("ok");
        break;
		
      case 83: 
        relative_mode_e = true;
        relative_mode_q = true;
        relative_mode_b = true;
        Serial.println("ok");
        break;
		
      case 84:
        disable_x();
        disable_y();
        disable_z();
        disable_e();
        disable_q();
        disable_b();
        Serial.println("ok");
        break;
		
      case 85: 
        code_seen('S');
        max_inactive_time = code_value()*1000; 
        Serial.println("ok");
        break;
		
      case 86: 
        if(code_seen('X')) if( digitalRead(X_MIN_PIN) == ENDSTOPS_INVERTING ) kill(3);
        if(code_seen('Y')) if( digitalRead(Y_MIN_PIN) == ENDSTOPS_INVERTING ) kill(4);
        Serial.println("ok");
        break;
		
      case 92: 
        if(code_seen('X')) x_steps_per_unit = (float)code_value();
        if(code_seen('Y')) y_steps_per_unit = (float)code_value();
        if(code_seen('Z')) z_steps_per_unit = (float)code_value();
        if(code_seen('E')&& (current_tool==0)) e_steps_per_unit = (float)code_value();
        if(code_seen('E')&& (current_tool==1)) q_steps_per_unit = (float)code_value();
        if(code_seen('E')&& (current_tool==2)) b_steps_per_unit = (float)code_value();
        Serial.println("ok");
        break;
    }
    
  }
  
  if (code_seen('T')) // T code
    {
      switch((int)code_value())
      {
        // codes for managing multiple toolheads
        
        case 0: // T0, use tool 0
          switch_tool(0);
          break;
        case 1: // T1, use tool 1
          switch_tool(1);
          break;
        case 2: // T2, use tool 2
          switch_tool(2);
          break;
        case 3: // T3, use tool 3
          switch_tool(3);
          break;
        case 10: // T10, home all z axes
          zero_all_z();
          break;
        case 11: // T11, level all z axes
          level_all_tools();
          break;
        case 14: // T14, set desired active toolhead clearance
          // P parameter contains desired clearance
          
          if (code_seen('P')) // We found a P value
          {
            toolhead_clearance = (float)code_value();
          }
          break;
        case 15: // T15, move all non active toolheads to achieve appropriate active toolhead clearance
          set_toolhead_clearance();
          break;
        case 18: // T18, Z-axis jog move
          current_tool_previous = current_tool;
          
          if (code_seen('P')) // We found a P value
          {
            current_tool = (int)code_value();
            if (current_tool >= 0 && current_tool < NUMBER_OF_TOOLS) { // valid tool choice
              
              if (code_seen('Z')) // We found a Z value
              {
                destination_z = current_z + (float)code_value();
                
                get_feedrate();
                
                // move to the coordinates, don't worry about direction, decide that later
                x_steps_to_take = 0;
                y_steps_to_take = 0;
                z_steps_to_take = abs(destination_z - current_z)*z_steps_per_unit;
                e_steps_to_take = 0;
                
                // find time for move by moving largest distance at defined feedrate
                time_for_move = (float)z_steps_to_take*(1/(z_steps_per_unit*feedrate/60000000)); // time in microseconds
                
                z_interval = time_for_move/z_steps_to_take;
                
                linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, 0, 0); // make the move
              }
            }
          }
          
          current_tool = current_tool_previous;
          
          break;
        case 20: // T20, move all non active toolheads to achieve appropriate active toolhead clearance
          startup();
          break;
        
      } 
    }
    
  ClearToSend();
}

inline void FlushSerialRequestResend()
{
  char cmdbuffer[100]="Resend:";
  ltoa(gcode_LastN+1, cmdbuffer+7, 10);
  Serial.flush();
  Serial.println(cmdbuffer);
  ClearToSend();
}

inline void ClearToSend()
{
  previous_millis_cmd = millis();
  Serial.println("ok"); 
}

inline void get_coordinates()
{
  Serial.println("get_coor");
  Serial.println(code_seen('X'));
  if(code_seen('X')) destination_x = (float)code_value() + relative_mode*current_x;
  else destination_x = current_x; //Are these else lines really needed?
  if(code_seen('Y')) destination_y = (float)code_value() + relative_mode*current_y;
  else destination_y = current_y;
  if(code_seen('Z')) destination_z = (float)code_value() + relative_mode*current_z;
  else destination_z = current_z;
  if(code_seen('E')&& (current_tool==0)) destination_e = (float)code_value() + (relative_mode_e || relative_mode)*current_e;
  else destination_e = current_e;
  if(code_seen('E')&& (current_tool==1)) destination_q = (float)code_value() + (relative_mode_q || relative_mode)*current_q;
  else destination_q = current_q;
  if(code_seen('E')&& (current_tool==2)) destination_b = (float)code_value() + (relative_mode_b || relative_mode)*current_b;
  else destination_b = current_b;
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
 
  Serial.print("destination_x:"); 
  Serial.println(destination_x);
    Serial.print("destination_y:"); 
  Serial.println(destination_y);
    Serial.print("destination_z:"); 
  Serial.println(destination_z);
    Serial.print("destination_e:"); 
  Serial.println(destination_e);
    Serial.print("destination_q:"); 
  Serial.println(destination_q);
      Serial.print("destination_b:"); 
  Serial.println(destination_b);
  //Find direction
  if(destination_x >= current_x) direction_x=1;
  else direction_x=0;
  if(destination_y >= current_y) direction_y=1;
  else direction_y=0;
  if(destination_z >= current_z) direction_z=1;
  else direction_z=0;
  if(destination_e >= current_e) direction_e=1;
  else direction_e=0;
  if(destination_q >= current_q) direction_q=1;
  else direction_q=0;
  if(destination_b >= current_b) direction_b=1;
  else direction_b=0;
  
  
  if (min_software_endstops) {
    if (destination_x < 0) destination_x = 0.0;
    if (destination_y < 0) destination_y = 0.0;
    if (destination_z < 0) destination_z = 0.0;
  }

  if (max_software_endstops) {
    if (destination_x > X_MAX_LENGTH) destination_x = X_MAX_LENGTH;
    if (destination_y > Y_MAX_LENGTH) destination_y = Y_MAX_LENGTH;
    if (destination_z > Z_MAX_LENGTH) destination_z = Z_MAX_LENGTH;
  }
  
  if(code_seen('Z') && code_seen('F') && feedrate > z_max_feedrate) feedrate = z_max_feedrate; 
  else if(feedrate > max_feedrate) feedrate = max_feedrate;
}

// make linear move with preset speeds and destinations, see G0 and G1
void linear_move(unsigned long x_steps_remaining, unsigned long y_steps_remaining, unsigned long z_steps_remaining,
unsigned long e_steps_remaining, unsigned long q_steps_remaining, unsigned long b_steps_remaining)
{
  //Determine direction of movement
  if (destination_x > current_x) {digitalWrite(X_DIR_PIN,!INVERT_X_DIR); current_x = current_x + x_steps_remaining/x_steps_per_unit;}
  else {digitalWrite(X_DIR_PIN,INVERT_X_DIR); current_x = current_x - x_steps_remaining/x_steps_per_unit;}
  
  if (destination_y > current_y) {digitalWrite(Y_DIR_PIN,!INVERT_Y_DIR); current_y = current_y + y_steps_remaining/y_steps_per_unit;}
  else {digitalWrite(Y_DIR_PIN,INVERT_Y_DIR); current_y = current_y - y_steps_remaining/y_steps_per_unit;}
  
  if (destination_z > current_z) {digitalWrite(Z_DIR_PIN,!INVERT_Z_DIR); current_z = current_z + z_steps_remaining/z_steps_per_unit;}
  else {digitalWrite(Z_DIR_PIN,INVERT_Z_DIR); current_z = current_z - z_steps_remaining/z_steps_per_unit;}
  
  if (destination_e > current_e) {digitalWrite(E_DIR_PIN,!INVERT_E_DIR); current_e = current_e + (float)e_steps_remaining/e_steps_per_unit;}
  else {digitalWrite(E_DIR_PIN,INVERT_E_DIR); current_e = current_e - (float)e_steps_remaining/e_steps_per_unit;}
  
  if (destination_q > current_q) {digitalWrite(Q_DIR_PIN,!INVERT_Q_DIR); current_q = current_q + (float)q_steps_remaining/q_steps_per_unit;}
  else {digitalWrite(Q_DIR_PIN,INVERT_Q_DIR); current_q = current_q - (float)q_steps_remaining/q_steps_per_unit;}
  
  if (destination_b > current_b) {digitalWrite(B_DIR_PIN,!INVERT_B_DIR); current_b = current_b + (float)b_steps_remaining/b_steps_per_unit;}
  else {digitalWrite(B_DIR_PIN,INVERT_B_DIR); current_b = current_b - (float)b_steps_remaining/b_steps_per_unit;}
  
  //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
  if(x_steps_remaining) enable_x();
  if(y_steps_remaining) enable_y();
  if(z_steps_remaining) enable_z();
  if(e_steps_remaining) enable_e();
  if(q_steps_remaining) enable_q();
  if(b_steps_remaining) enable_b();

  if(X_MIN_PIN > -1) if(!direction_x) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) x_steps_remaining=0;
  if(Y_MIN_PIN > -1) if(!direction_y) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) y_steps_remaining=0;
  if(Z_MIN_PIN > -1) if(!direction_z) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) z_steps_remaining=0;
  
  previous_millis_heater = millis();

  // move until no more steps remain
  //while(x_steps_remaining > 0 || y_steps_remaining > 0 || z_steps_remaining > 0 || e_steps_remaining > 0 || q_steps_remaining > 0 || b_steps_remaining > 0)
  // move until no more steps remain
  while(x_steps_remaining + y_steps_remaining + z_steps_remaining + e_steps_remaining + q_steps_remaining + b_steps_remaining > 0)
  { 
    if(x_steps_remaining) {
      if ((micros()-previous_micros_x) >= x_interval) { do_x_step(); x_steps_remaining--; }
      if(X_MIN_PIN > -1) if(!direction_x) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) x_steps_remaining=0;
    }
    
    if(y_steps_remaining) {
      if ((micros()-previous_micros_y) >= y_interval) { do_y_step(); y_steps_remaining--; }
      if(Y_MIN_PIN > -1) if(!direction_y) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) y_steps_remaining=0;
    }
    
    if(z_steps_remaining) {
      if ((micros()-previous_micros_z) >= z_interval) { do_z_step(); z_steps_remaining--; }
      if(Z_MIN_PIN > -1) if(!direction_z) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) z_steps_remaining=0;
    }    
    
    if(e_steps_remaining) if ((micros()-previous_micros_e) >= e_interval) { do_e_step(); e_steps_remaining--; }
    if(q_steps_remaining) if ((micros()-previous_micros_q) >= q_interval) { do_q_step(); q_steps_remaining--; }
    if(b_steps_remaining) if ((micros()-previous_micros_b) >= b_interval) { do_b_step(); b_steps_remaining--; }
    
    if( (millis() - previous_millis_heater) >= 500 ) {
      manage_heater();
      previous_millis_heater = millis();
      
      manage_inactivity(2);
    }
  }
  
  if(DISABLE_X) disable_x();
  if(DISABLE_Y) disable_y();
  if(DISABLE_Z) disable_z();
  if(DISABLE_E) disable_e();
  if(DISABLE_Q) disable_q();
  if(DISABLE_B) disable_b();
}


inline void do_x_step()
{
  digitalWrite(X_STEP_PIN, HIGH);
  previous_micros_x = micros();
  //delayMicroseconds(3);
  digitalWrite(X_STEP_PIN, LOW);
}

inline void do_y_step()
{
  digitalWrite(Y_STEP_PIN, HIGH);
  previous_micros_y = micros();
  //delayMicroseconds(3);
  digitalWrite(Y_STEP_PIN, LOW);
}

inline void do_z_step()
{
  digitalWrite(Z_STEP_PIN, HIGH);
  previous_micros_z = micros();
  //delayMicroseconds(3);
  digitalWrite(Z_STEP_PIN, LOW);
}

inline void do_e_step()
{
  digitalWrite(E_STEP_PIN, HIGH);
  previous_micros_e = micros();
  //delayMicroseconds(3);
  digitalWrite(E_STEP_PIN, LOW);
}

inline void do_q_step()
{
  digitalWrite(Q_STEP_PIN, HIGH);
  previous_micros_q = micros();
  //delayMicroseconds(3);
  digitalWrite(Q_STEP_PIN, LOW);
}

inline void do_b_step()
{
  digitalWrite(B_STEP_PIN, HIGH);
  previous_micros_b = micros();
  //delayMicroseconds(3);
  digitalWrite(B_STEP_PIN, LOW);
}

inline void disable_x() { if(X_ENABLE_PIN > -1) digitalWrite(X_ENABLE_PIN,!X_ENABLE_ON); }
inline void disable_y() { if(Y_ENABLE_PIN > -1) digitalWrite(Y_ENABLE_PIN,!Y_ENABLE_ON); }
inline void disable_z() { if(Z_ENABLE_PIN > -1) digitalWrite(Z_ENABLE_PIN,!Z_ENABLE_ON); }
inline void disable_e() { if(E_ENABLE_PIN > -1) digitalWrite(E_ENABLE_PIN,!E_ENABLE_ON); }
inline void disable_q() { if(Q_ENABLE_PIN > -1) digitalWrite(Q_ENABLE_PIN,!Q_ENABLE_ON); }
inline void disable_b() { if(B_ENABLE_PIN > -1) digitalWrite(B_ENABLE_PIN,!B_ENABLE_ON); }

inline void  enable_x() { if(X_ENABLE_PIN > -1) digitalWrite(X_ENABLE_PIN, X_ENABLE_ON); }
inline void  enable_y() { if(Y_ENABLE_PIN > -1) digitalWrite(Y_ENABLE_PIN, Y_ENABLE_ON); }
inline void  enable_z() { if(Z_ENABLE_PIN > -1) digitalWrite(Z_ENABLE_PIN, Z_ENABLE_ON); }
inline void  enable_e() { if(E_ENABLE_PIN > -1) digitalWrite(E_ENABLE_PIN, E_ENABLE_ON); }
inline void  enable_q() { if(Q_ENABLE_PIN > -1) digitalWrite(Q_ENABLE_PIN, Q_ENABLE_ON); }
inline void  enable_b() { if(B_ENABLE_PIN > -1) digitalWrite(B_ENABLE_PIN, B_ENABLE_ON); }

inline void manage_heater()
{
  // If using thermistor, when the heater is colder than target temp, we get a higher analog reading than target, 
  current_raw_0 = analogRead(TEMP_0_PIN);
  current_raw_1 = analogRead(TEMP_1_PIN);
  current_raw_2 = analogRead(TEMP_2_PIN);
  if(USE_THERMISTOR){
    // this switches it up so that the reading appears lower than target for the control logic.
    current_raw_0 = max_adc_val - current_raw_0;
    current_raw_1 = max_adc_val - current_raw_1;
	current_raw_2 = max_adc_val - current_raw_2;
	}
  
  if(current_raw_0 >= target_raw_0) digitalWrite(HEATER_0_PIN,LOW);
  else digitalWrite(HEATER_0_PIN,HIGH);
  if(current_raw_1 >= target_raw_1) digitalWrite(HEATER_1_PIN,LOW);
  else digitalWrite(HEATER_1_PIN,HIGH);
  if(current_raw_2 >= target_raw_2) digitalWrite(HEATER_2_PIN,LOW);
  else digitalWrite(HEATER_2_PIN,HIGH);
}

// Takes temperature value as input and returns corresponding analog value from RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given analog value, not a temp value.
// This function is derived from inversing the logic from a portion of getTemperature() in FiveD RepRap firmware.
float temp2analog(int celsius) {
  if(USE_THERMISTOR) {
    int raw = 0;
    byte i;
    
    for (i=1; i<NUMTEMPS; i++)
    {
      if (temptable[i][1] < celsius)
      {
        raw = temptable[i-1][0] + 
          (celsius - temptable[i-1][1]) * 
          (temptable[i][0] - temptable[i-1][0]) /
          (temptable[i][1] - temptable[i-1][1]);
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == NUMTEMPS) raw = temptable[i-1][0];

    return max_adc_val - raw;
  } else {
    return celsius * (1024.0/(5.0*100.0));
  }
}

// Derived from RepRap FiveD extruder::getTemperature()
float analog2temp(int raw) {
  if(USE_THERMISTOR) {
    int celsius = 0;
    byte i;

    for (i=1; i<NUMTEMPS; i++)
    {
      if (temptable[i][0] > raw)
      {
        celsius  = temptable[i-1][1] + 
          (raw - temptable[i-1][0]) * 
          (temptable[i][1] - temptable[i-1][1]) /
          (temptable[i][0] - temptable[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == NUMTEMPS) celsius = temptable[i-1][1];

    return celsius;
    
  } else {
    return raw * ((5.0*100.0)/1024.0);
  }
}

inline void kill(byte debug)
{
  if(HEATER_0_PIN > -1) digitalWrite(HEATER_0_PIN,LOW);
  if(HEATER_1_PIN > -1) digitalWrite(HEATER_1_PIN,LOW);
  if(HEATER_2_PIN > -1) digitalWrite(HEATER_2_PIN,LOW);
  
  disable_x();
  disable_y();
  disable_z();
  disable_e();
  disable_q();
  disable_b();
  
  if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
  
  while(1)
  {
    switch(debug)
    {
      case 1: Serial.print("Inactivity Shutdown, Last Line: "); break;
      case 2: Serial.print("Linear Move Abort, Last Line: "); break;
      case 3: Serial.print("Homing X Min Stop Fail, Last Line: "); break;
      case 4: Serial.print("Homing Y Min Stop Fail, Last Line: "); break;
    } 
    Serial.println(gcode_LastN);
    delay(5000); // 5 Second delay
  }
}

inline void manage_inactivity(byte debug) { if( (millis()-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill(debug);}

void switch_tool(int new_tool) {
  int z_steps_to_take_temp;
  float destination_z_temp;
  
  x_0 = current_x; // storage of original positions
  y_0 = current_y;
  z_0 = current_z;
  
  destination_x = current_x - (TOOLCHANGER[new_tool][1] - TOOLCHANGER[current_tool][1]);
  destination_y = current_y - (TOOLCHANGER[new_tool][2] - TOOLCHANGER[current_tool][2]);
  destination_z = current_z;
  destination_z_temp = current_z + (TOOLCHANGER[current_tool][3] - TOOLCHANGER[new_tool][3]);
  
   // move to the coordinates, don't worry about direction, decide that later
  x_steps_to_take = abs(destination_x - current_x)*x_steps_per_unit;
  y_steps_to_take = abs(destination_y - current_y)*y_steps_per_unit;
  z_steps_to_take = 0;
  z_steps_to_take_temp = abs(destination_z_temp - current_z)*z_steps_per_unit;
  e_steps_to_take = 0;
  
  feedrate = max_feedrate;
  
  // find time for move by moving largest distance at max speed
  if (x_steps_to_take >= y_steps_to_take && x_steps_to_take >= z_steps_to_take) {
    time_for_move = (float)x_steps_to_take*(1/(x_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  else if (y_steps_to_take >= x_steps_to_take && y_steps_to_take >= z_steps_to_take) {
    time_for_move = (float)y_steps_to_take*(1/(y_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  else {
    time_for_move = (float)z_steps_to_take*(1/(z_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  x_interval = time_for_move/x_steps_to_take;
  y_interval = time_for_move/y_steps_to_take;
  z_interval = time_for_move/z_steps_to_take; // this should probably have it's own max, different from XY
  
  current_tool = new_tool;
  
  linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, 0, 0); // make the move
  
  current_x = x_0; // restore original positioning
  current_y = y_0;
  
  
  destination_x = current_x;
  destination_y = current_y;
  destination_z = destination_z_temp;
  
   // move to the coordinates, don't worry about direction, decide that later
  x_steps_to_take = 0;
  y_steps_to_take = 0;
  z_steps_to_take = z_steps_to_take_temp;
  e_steps_to_take = 0;
  
  feedrate = max_feedrate;
  
  // find time for move by moving largest distance at max speed
  if (x_steps_to_take >= y_steps_to_take && x_steps_to_take >= z_steps_to_take) {
    time_for_move = (float)x_steps_to_take*(1/(x_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  else if (y_steps_to_take >= x_steps_to_take && y_steps_to_take >= z_steps_to_take) {
    time_for_move = (float)y_steps_to_take*(1/(y_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  else {
    time_for_move = (float)z_steps_to_take*(1/(z_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  x_interval = time_for_move/x_steps_to_take;
  y_interval = time_for_move/y_steps_to_take;
  z_interval = time_for_move/z_steps_to_take;
  
  linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, 0, 0); // make the move
  
  current_z = z_0; 
}

void zero_all_z() {
  
  current_tool_previous = current_tool;
  
  for (int n = 0; n < NUMBER_OF_TOOLS; n++) {
    
    current_tool = n;
    
    destination_x = current_x; // don't move
    destination_y = current_y; // don't move
    destination_z = Z_MAX_LENGTH*2; // times 2 just to make sure we go far enough
    
    x_steps_to_take = 0;
    y_steps_to_take = 0;
    z_steps_to_take = abs(destination_z - current_z)*z_steps_per_unit;
    e_steps_to_take = 0;
    
    feedrate = 1500;
    
    time_for_move = (float)z_steps_to_take*(1/(z_steps_per_unit*feedrate/60000000)); // time in microseconds
    z_interval = time_for_move/z_steps_to_take;
    
    linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, 0, 0); // make the move
    
  }
  
  current_tool = current_tool_previous;
  current_z = 0.0; // zero at top
  
}

void level_all_tools() {
  // compensate for z tool lengths, at end all tools should be at same z height as current tool
  
  current_tool_previous = current_tool;
  
  for (int n = 0; n < NUMBER_OF_TOOLS; n++) {
    if (n != current_tool_previous) { // we want to move all other tools up to this level
      current_tool = n;
      
      destination_x = current_x; // don't move
      destination_y = current_y; // don't move
      destination_z = current_z + (TOOLCHANGER[current_tool_previous][3] - TOOLCHANGER[current_tool][3]);
      
      x_steps_to_take = 0;
      y_steps_to_take = 0;
      z_steps_to_take = abs(destination_z - current_z)*z_steps_per_unit;
      e_steps_to_take = 0;
      
      feedrate = 1500;
      
      time_for_move = (float)z_steps_to_take*(1/(z_steps_per_unit*feedrate/60000000)); // time in microseconds
      z_interval = time_for_move/z_steps_to_take;
      
      linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, 0, 0); // make the move
    }
  }
  
  current_tool = current_tool_previous; // restore active tool
  
}

void set_toolhead_clearance() {
  // move all toolheads set distance above active tool
  
  current_tool_previous = current_tool;
  
  for (int n = 0; n < NUMBER_OF_TOOLS; n++) {
    if (n != current_tool_previous) {
      
      current_tool = n;
      
      destination_x = current_x; // don't move
      destination_y = current_y; // don't move
      destination_z = current_z + (TOOLCHANGER[current_tool_previous][3] - TOOLCHANGER[current_tool][3]) + toolhead_clearance;
      
      x_steps_to_take = 0;
      y_steps_to_take = 0;
      z_steps_to_take = abs(destination_z - current_z)*z_steps_per_unit;
      e_steps_to_take = 0;
      
      feedrate = 1500;
      
      time_for_move = (float)z_steps_to_take*(1/(z_steps_per_unit*feedrate/60000000)); // time in microseconds
      z_interval = time_for_move/z_steps_to_take;
      
      linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, 0, 0); // make the move
    }
  }
  
  current_tool = current_tool_previous; // restore active tool
  
}

void get_feedrate()
{

  if (code_seen('F')) // We found an F value
  {
    feedrate = (float)code_value();

    if (feedrate > max_feedrate) { // if it wants us to go too fast
      feedrate = max_feedrate;
    }
  }
  else {
    if (feedrate == -1.0) { // we haven't set the feedrate yet and we are trying to do a linear move
      feedrate = 1500;
    }
    // else the feedrate has already been set and we will keep the previous value
  }
  
}

void zero_xy() {
  // zero xy axes
  destination_x = -X_MAX_LENGTH*2; // times 2 just to make sure we go far enough
  destination_y = -Y_MAX_LENGTH*2; // times 2 just to make sure we go far enough
  destination_z = current_z;
  
  x_steps_to_take = abs(destination_x - current_x)*x_steps_per_unit;
  y_steps_to_take = abs(destination_y - current_y)*y_steps_per_unit;
  z_steps_to_take = 0;
  e_steps_to_take = 0;
  
  feedrate = 1500;
  
  // find time for move by moving largest distance at max speed
  if (x_steps_to_take >= y_steps_to_take) {
    time_for_move = (float)x_steps_to_take*(1/(x_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  else {
    time_for_move = (float)y_steps_to_take*(1/(y_steps_per_unit*feedrate/60000000)); // time in microseconds
  }
  x_interval = time_for_move/x_steps_to_take;
  y_interval = time_for_move/y_steps_to_take;
  
  linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take, 0, 0); // make the move
  
  current_x = 0.0;
  current_y = 0.0;
}

void startup() {
  zero_xy();
  zero_all_z();
  level_all_tools();
  set_toolhead_clearance();
}
