#include <Arduino.h>
#include <AccelStepper.h>

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

  After reading an incoming command, the command is always stored in the 
    variable 'inc_command' which can be indexed using IncCommandIndexer. Use 
    this to grab any values from the incoming command that you need, for  
    instance within the execute_command function. Similarly, place any return 
    values in the variable 'out_resp' which can be indexed using 
    OutCommandIndexer. Any data in this array will be written via serial after
    the current loop completes.
*/

// Total number of devices
uint8_t num_devices = 2;

// Stepper will be device 0
AccelStepper  steppers[1] = {AccelStepper(1, 4, 3)};  // pin 4 = step, pin 3 = direction
const uint8_t num_steppers = 1;
uint8_t       stepper_iterator;

// Encoder will be device 1
const uint8_t pinA = 5;
const uint8_t pinB = 6;
int16_t pinALast, aVal, bVal;
int16_t tick_count = 0;

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
  Serial.begin(DEBUG_SERIAL_PORT);

  // Setup any devices
  for (uint8_t i = 0; i < num_steppers; i++){
    steppers[i].setMaxSpeed(4000);
    steppers[i].setCurrentPosition(0);
  }
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinALast = digitalRead(pinA);
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
  inc_command[INC_ORDER] = read_order();

  // HELLO is the only command that doesn't have additional data appended
  if (inc_command[0] == HELLO) {
    return;
  }

  // Read the rest of the command
  inc_command[INC_INDEX] = read_i8();
  inc_command[INC_VEL] = read_i16();
  inc_command[INC_POS] = read_i16();

  // Checksum will be automatically appended and handled
  inc_command[INC_CHECKSUM] = read_i16();
}

void write_response()
{
  /*
    Update this with your custom protocol for outgoing messages.

    Note that the last element of an outgoing command should always be the 
      checksum. 

    Make sure this corresponds to the read_response function on the host PC.
      For example this is configured to write a response of the form:
        [order, data]
      where order usually indicates SUCCESS or ERROR.
  */

  // This updates the last element of the array with the checksum
  generate_checksum();

  write_i8(out_resp[OUT_ORDER]);
  write_i16(out_resp[OUT_DATA]);

  // Checksum will be automatically appended and handled
  write_i16(out_resp[OUT_CHECKSUM]);
}

void loop()
{
  // Check to see if there are new messages
  get_messages_from_serial();

  // Call any code that needs to be updated frquently here, like stepper code
  for (uint8_t i = 0; i < num_steppers; i++){
    steppers[i].runSpeedToPosition();
  }

  aVal = digitalRead(pinA);
  bVal = digitalRead(pinB);

  if ((pinALast == HIGH) && (aVal == LOW)){
      if (bVal == LOW){
          tick_count++;
      } else {
          tick_count--;
      }
  }
  pinALast = aVal;
}

bool validate_command() {
  /* 
    Validates an incoming command
  */

  if (inc_command[INC_ORDER] == MOVE_OR_QUERY) {
    if ((inc_command[INC_INDEX] >= 0) && (inc_command[INC_INDEX] < num_devices)){
      return true;
    }
  }
  return false;
}

void execute_command() {
  /* 
    Executes a valid command
  */

  if (inc_command[INC_ORDER] == MOVE_OR_QUERY) {
    switch(inc_command[INC_INDEX]) {
      case 0: {
        steppers[inc_command[INC_INDEX]].moveTo(inc_command[INC_POS]);
        steppers[inc_command[INC_INDEX]].setSpeed(inc_command[INC_VEL]);
        break;
      } 
      case 1: {
        out_resp[OUT_DATA] = tick_count;
        break;
      }
      default:
        out_resp[OUT_ORDER] = ERROR;
        out_resp[OUT_DATA] = INVALID_DEVICE;
        break;
    }
  } else {
    out_resp[OUT_ORDER] = ERROR;
    out_resp[OUT_DATA] = INVALID_ORDER;
  }
}

void get_messages_from_serial()
{
  if(Serial.available() > 0) { 
  
    // Get the next command
    read_command();

    // HELLO is a special case
    if(inc_command[INC_ORDER] == HELLO) {

      // Don't say hello twice
      if(!is_connected) {
        is_connected = true;
        write_order(HELLO);
      } else {
        write_order(ALREADY_CONNECTED);
      }
      return;

    } else {
      out_resp[OUT_DATA] = 0; // Make sure to return something 

      // First check for message corruption
      if (!validate_checksum()) {
        out_resp[OUT_ORDER] = ERROR;
        out_resp[OUT_DATA] = CORRUPTION;
      } else {
        // Then do error checking
        if (!validate_command()) {
          out_resp[OUT_ORDER] = ERROR;
          out_resp[OUT_DATA] = INVALID_COMMAND;
        } else {
          out_resp[OUT_ORDER] = SUCCESS;
          execute_command();
        }
      }

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
  out_resp[OUT_CHECKSUM] = ~sum;
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
