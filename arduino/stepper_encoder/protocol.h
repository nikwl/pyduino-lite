#ifndef PROTOCOL_H
#define PROTOCOL_H

/* NOTE: All enums here should match those defined in protocol.py */

// Define the orders that can be sent and received
enum Order {
  HELLO = 0,
  ALREADY_CONNECTED = 1,
  ERROR = 2,
  SUCCESS = 3,
  POSITION_MOVE = 4,
  VELOCITY_MOVE = 5,
  QUERY_DEVICE = 6,
};

// Define possible errors
enum Error {
  CORRUPTION = 0,
  INVALID_COMMAND = 1,
  INVALID_ORDER = 2,
  INVALID_DEVICE = 3,
};

// Makes indexing easier
enum CommandIndexer {
  CMD_ORDER = 0,
  CMD_INDEX = 1,
  CMD_VEL = 2,
  CMD_POS = 3,
  CMD_CHECKSUM = 4,
};

// Makes indexing easier
enum ResponseIndexer {
  RESP_ORDER = 0,
  RESP_DATA = 1,
  RESP_CHECKSUM = 2,
};

#endif