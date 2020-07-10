from enum import Enum
import struct

class Order(Enum):
    HELLO = 0
    ALREADY_CONNECTED = 1
    ERROR = 2
    SUCCESS = 3
    MOVE_OR_QUERY = 4

class Error(Enum):
    CORRUPTION = 0
    INVALID_COMMAND = 1
    INVALID_ORDER = 2
    INVALID_DEVICE = 3

# Makes indexing easier
class IncCommandIndexer(Enum):
    INC_ORDER = 0
    INC_INDEX = 1
    INC_VEL = 2
    INC_POS = 3
    INC_CHECKSUM = 4

class OutCommandIndexer(Enum):
    OUT_ORDER = 0
    OUT_DATA = 1
    OUT_CHECKSUM = 2

def write_command(serial_file, command):
    '''
        write_command: Given an array, will write all elements to the arduino. 
            Specify the size and type here. For example, this is configured to 
            write a command of the following form: 
                [order, device_id, motor_velocity, motor_position].
        
        Checksum is automatically appended.

        Make sure this corresponds to the read_command function on the 
            arduino. 
    '''
    order, device_index, motor_velocity, motor_position, checksum = command
    write_i8(serial_file, order)
    write_i8(serial_file, device_index)
    write_i16(serial_file, motor_velocity)
    write_i16(serial_file, motor_position)
    write_i16(serial_file, checksum)

def read_response(serial_file):
    '''
        read_response: Will read a response from the arduino. For example this 
            is configured to read a response of the form:
                [order, data]
            where order usually indicates SUCCESS or ERROR.

        Returned checksum should be the last element. 

        Make sure this corresponds to the write_response function on the 
            arduino
    '''
    order = read_i8(serial_file)
    value = read_i16(serial_file)
    checksum = read_i16(serial_file)
    return [order, value, checksum]

def read_order(ser):
    '''
        :param ser: file handler or serial file
        :return: (Order Enum Object)
    '''
    return Order(read_i8(ser))

def read_i8(ser):
    '''
        :param ser: file handler or serial file
        :return: (int8_t)
    '''
    return struct.unpack('<b', bytearray(ser.read(1)))[0]


def read_i16(ser):
    '''
        :param ser: file handler or serial file
        :return: (int16_t)
    '''
    return struct.unpack('<h', bytearray(ser.read(2)))[0]


def read_i32(ser):
    '''
        :param ser: file handler or serial file
        :return: (int32_t)
    '''
    return struct.unpack('<l', bytearray(ser.read(4)))[0]


def write_i8(ser, value):
    '''
        :param ser: file handler or serial file
        :param value: (int8_t)
    '''
    ser.write(struct.pack('<b', value))

def write_order(ser, order):
    '''
        :param ser: file handler or serial file
        :param order: (Order Enum Object)
    '''
    write_i8(ser, order.value)


def write_i16(ser, value):
    '''
        :param ser: file handler or serial file
        :param value: (int16_t)
    '''
    ser.write(struct.pack('<h', value))


def write_i32(ser, value):
    '''
        :param ser: file handler or serial file
        :param value: (int32_t)
    '''
    ser.write(struct.pack('<l', value))
