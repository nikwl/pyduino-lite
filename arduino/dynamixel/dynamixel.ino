/* 
  Copyright 2020, Nikolas Lamb. 
*/
#include <Arduino.h>
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define Serial soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define Serial SerialUSB
#endif

#define DXL_PROTOCOL_VERSION 2.0
#define DEBUG_SERIAL_PORT 115200
#define COMM_SERIAL_PORT 1000000

#include "protocol.h"

/*
  arduino_controller

  When changing protocol, update the following functions:
    read_command     - Should be identical to python write_command
    write_response   - Should be identical to python read_response

    validate_command - New Orders need to pass this test
    execute_command  - Here's where you implement any new Order

  When adding a new Order, update the following functions:
    validate_command - New Orders need to pass this test
    execute_command  - Here's where you implement any new Order

  When adding a new physical device, update the following functions:
    setup            - Do any device setup here (obvoiusly)
    loop             - Some devices, like steppers, need to be constantly updated

    validate_command - You'll probably need to verify the new device id
    execute_command  - Here's where you implement the mechanics

  After reading an incoming command, the command is always stored in the 
    variable 'inc_command' which can be indexed using CommandIndexer. Use 
    this to grab any values from the incoming command that you need, for  
    instance within the execute_command function. Similarly, place any return 
    values in the variable 'out_resp' which can be indexed using 
    ResponseIndexer. Any data in this array will be written via serial after
    the current loop completes.
*/

// Total number of devices
const uint8_t num_devices = 4;

// Dynamixel Motors
DynamixelShield dxl;
const uint8_t   dxl_ids[num_devices] = {1, 2, 3, 4}; // I've indexed these in kind of a dumb way, so they require a mapping
uint8_t         dxl_active[num_devices] = {0, 0, 0, 0};

// Connection variables
bool      is_connected = false;

// Incoming message variables
int16_t   inc_command[5];
uint8_t   inc_len = 5;

// Outgoing message variables
int16_t   out_resp[3];
uint8_t   out_len = 3;

void setup()
{ 
  // Use UART port of DYNAMIXEL Shield to debug.
  Serial.begin(DEBUG_SERIAL_PORT);

  // Ping all dynamixel motors. If the motor is found, flash its led. Record
  // all motors found in 'dxl_active' variable.
  dxl.begin(COMM_SERIAL_PORT);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  for (uint8_t i = 0; i < num_devices; i++) {
    uint8_t id = dxl_ids[i];
    
    // Motor found
    if (dxl.ping(id)) {

      // Set servos to operate in joint mode (not wheel mode)
      if ((id == 1) || (id == 2)) {
        dxl.writeControlTableItem(TORQUE_ENABLE, id, 0);
        dxl.writeControlTableItem(OPERATING_MODE, id, 3);
        //dxl.writeControlTableItem(TORQUE_ENABLE, id, 1);
      } else if ((id == 3) || (id == 4)) {
        dxl.writeControlTableItem(TORQUE_ENABLE, id, 0);
        dxl.writeControlTableItem(CONTROL_MODE, id, 2);
        //dxl.writeControlTableItem(TORQUE_ENABLE, id, 1);
      }

      // Flash the LED
      dxl.ledOn(id);
      delay(50);
      dxl.ledOff(id);

      dxl_active[i] = 1;
    } else {
      dxl_active[i] = 0;
    }
  }
}

void read_command() 
{
  /* 
    Update this with your custom protocol for incoming commands (excepting) the
      HELLO handshake. This function should be called only if the serial buffer
      is not empty, ie if there is an imcoming command. 

    Once the array inc_command is populated, you can access any of its elements 
      in the body of your code. 
    
    Note that the last element of an incoming command should always be the 
      checksum. 
    
    Make sure this corresponds to the write_command function on the host PC. 
      For example this is configured to read a command of the form:
        [order, device_id, motor_velocity, motor_position]
  */

  // First byte is always the order
  inc_command[CMD_ORDER] = read_order();

  // HELLO is the only command that doesn't have additional data appended
  if (inc_command[0] == HELLO) {
    return;
  }

  // Read the rest of the command
  inc_command[CMD_INDEX] = read_i8();
  inc_command[CMD_VEL] = read_i16();
  inc_command[CMD_POS] = read_i16();

  // Checksum will be automatically appended and handled
  inc_command[CMD_CHECKSUM] = read_i16();
}

void write_response()
{
  /*
    Update this with your custom protocol for dxl_idsoutgoing messages.

    Note that the last element of an outgoing command should always be the 
      checksum. 

    Make sure this corresponds to the read_response function on the host PC.
      For example this is configured to write a response of the form:
        [order, data]
      where order usually indicates SUCCESS or ERROR.
  */

  write_i8(out_resp[RESP_ORDER]);
  write_i16(out_resp[RESP_DATA]);

  // Checksum will be automatically appended and handled
  write_i16(out_resp[RESP_CHECKSUM]);
}

bool validate_command() {
  /* 
    Validates an incoming command
  */

  // Make sure this motor is actually connected
  if (inc_command[CMD_ORDER] == POSITION_MOVE || inc_command[CMD_ORDER] == VELOCITY_MOVE || inc_command[CMD_ORDER] == QUERY_DEVICE) {
    if ((inc_command[CMD_INDEX] >= 0) && (inc_command[CMD_INDEX] < num_devices)){
      if (dxl_active[inc_command[CMD_INDEX]]){
        return true;
      }
    }
  }
  
  return false;
}

void execute_command() {
  /* 
    Executes a valid command
  */

  switch (inc_command[CMD_ORDER]) {
    case POSITION_MOVE: {
      // Position control mode is implemented using velocity control with a default velocity
      uint8_t device_index = dxl_ids[inc_command[CMD_INDEX]];
      uint16_t motor_velocity = 480;
      uint16_t motor_position = inc_command[CMD_POS];

      // XL-430s: velocity mode = PROFILE_VELOCITY
      if ((device_index == 1) || (device_index == 2)) {
        dxl.writeControlTableItem(PROFILE_VELOCITY, device_index, motor_velocity);
      
      // XL-320s: velocity mode = MOVING_SPEED
      } else if ((device_index == 3) || (device_index == 4)) {
        dxl.writeControlTableItem(MOVING_SPEED, device_index, motor_velocity);
      }
      
      dxl.writeControlTableItem(GOAL_POSITION, device_index, motor_position);

      // I could return some DATA here, but I don't really feel like it. 
      break;
    } case VELOCITY_MOVE: {
      uint8_t device_index = dxl_ids[inc_command[CMD_INDEX]];
      uint16_t motor_velocity = inc_command[CMD_VEL];
      uint16_t motor_position = inc_command[CMD_POS];

      // XL-430s: velocity mode = PROFILE_VELOCITY
      if ((device_index == 1) || (device_index == 2)) {
        dxl.writeControlTableItem(PROFILE_VELOCITY, device_index, motor_velocity);
      
      // XL-320s: velocity mode = MOVING_SPEED
      } else if ((device_index == 3) || (device_index == 4)) {
        dxl.writeControlTableItem(MOVING_SPEED, device_index, motor_velocity);
      }
      
      dxl.writeControlTableItem(GOAL_POSITION, device_index, motor_position);

      // I could return some DATA here, but I don't really feel like it. 
      break;
    } case QUERY_DEVICE: {
      uint8_t device_index = dxl_ids[inc_command[CMD_INDEX]];
      out_resp[RESP_DATA] = (int16_t)dxl.readControlTableItem(PRESENT_POSITION, device_index);
      break;
    } default : {
      out_resp[RESP_ORDER] = ERROR;
      out_resp[RESP_DATA] = INVALID_ORDER;
      break;
    }
  }
}

void loop()
{
  // Check to see if there are new messages
  get_messages_from_serial();
}

void get_messages_from_serial()
{
  if(Serial.available() > 0) { 
  
    // Get the next command
    read_command();

    // HELLO is a special case
    if(inc_command[CMD_ORDER] == HELLO) {

      // Don't say hello twice
      if(!is_connected) {
        is_connected = true;
        write_order(HELLO);
      } else {
        write_order(ALREADY_CONNECTED);
      }
      return;

    } else {
     // A default data value. Overwrite this with whatever you want in execute_command
      out_resp[RESP_DATA] = 0;

      // First check for message corruption
      if (!validate_checksum()) {
        out_resp[RESP_ORDER] = ERROR;
        out_resp[RESP_DATA] = CORRUPTION;
      } else {
        // Then do error checking
        if (!validate_command()) {
          out_resp[RESP_ORDER] = ERROR;
          out_resp[RESP_DATA] = INVALID_COMMAND;
        } else {
          // Return a default SUCCESS value. This can be changed SO LONG AS
          // you don't return something equal to ERROR. 
          out_resp[RESP_ORDER] = SUCCESS; 
          execute_command();
        }
      }

      // This updates the last element of the array with the checksum
      // MAKE SURE TO DO THIS BEFORE WRITING
      generate_checksum();
      // After all the response elements are set, write to serial
      write_response();
    }
  }
}

void generate_checksum() {
  /*
    Generates the checksum
  */

  int16_t sum = 0;
  for (uint8_t i = 0; i < out_len-1; i++)
    sum += out_resp[i];
  out_resp[RESP_CHECKSUM] = ~sum;
}

bool validate_checksum() {
  /*
    Checks for corruption by verifying the checksum
  */

  int16_t sum = 0;
  for (uint8_t i = 0; i < inc_len; i++)
    sum += inc_command[i];
  if (~sum == 0)
    return true;
  return false;
}

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
	unsigned long startTime = micros();
	//Wait for incoming bytes or exit if timeout
	while ((Serial.available() < num_bytes) && (micros() - startTime < timeout)){}
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t* buffer, size_t n)
{
	size_t i = 0;
	int c;
	while (i < n)
	{
		c = Serial.read();
		if (c < 0) break;
		*buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
		i++;
	}
}

Order read_order()
{
	return (Order) read_i8();
}

int8_t read_i8()
{
	wait_for_bytes(1, 500); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

int16_t read_i16()
{
  int8_t buffer[2];
	wait_for_bytes(2, 500); // Wait for 2 bytes with a timeout of 100 ms
	read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int32_t read_i32()
{
  int8_t buffer[4];
	wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
	read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

void write_order(enum Order o)
{
	uint8_t* Order = (uint8_t*) &o;
  Serial.write(Order, sizeof(uint8_t));
}

void write_i8(int8_t num)
{
  Serial.write(num);
}

void write_i16(int16_t num)
{
	int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

void write_i32(int32_t num)
{
	int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}
