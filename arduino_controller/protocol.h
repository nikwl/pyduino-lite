#ifndef PROTOCOL_H
#define PROTOCOL_H

#define DEBUG_SERIAL_PORT 115200

// Define the orders that can be sent and received
enum Order {
  HELLO = 0,
  ALREADY_CONNECTED = 1,
  ERROR = 2,
  SUCCESS = 3,
  MOVE_OR_QUERY = 4,
};

// Define possible errors
enum Error {
  CORRUPTION = 0,
  INVALID_COMMAND = 1,
  INVALID_ORDER = 2,
  INVALID_DEVICE = 3,
};

// Makes indexing easier
enum IncCommandIndexer {
  INC_ORDER = 0,
  INC_INDEX = 1,
  INC_VEL = 2,
  INC_POS = 3,
  INC_CHECKSUM = 4,
};

// Makes indexing easier
enum OutCommandIndexer {
  OUT_ORDER = 0,
  OUT_DATA = 1,
  OUT_CHECKSUM = 2,
};

#endif
