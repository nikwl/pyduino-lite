from enum import Enum
import struct

class Order(Enum):
    '''
        Order: Use this to name the orders, or operations, that the
            arduino can perform.
    '''
    HELLO = 0
    ALREADY_CONNECTED = 1
    ERROR = 2
    SUCCESS = 3
    POSITION_MOVE = 4
    VELOCITY_MOVE = 5
    QUERY_DEVICE = 6

class Error(Enum):
    '''
        Error: Use this to name the possible errors that the arduino 
            can return. 
    '''
    CORRUPTION = 0
    INVALID_COMMAND = 1
    INVALID_ORDER = 2
    INVALID_DEVICE = 3

class CommandIndexer(Enum):
    '''
        CommandIndexer: Use this to name the indices of outgoing commands. 
            For example:
                [INC_ORDER, INC_INDEX, INC_VEL, INC_POS, INC_CHECKSUM] = response
    '''
    CMD_ORDER = 0
    CMD_INDEX = 1
    CMD_VEL = 2
    CMD_POS = 3
    CMD_CHECKSUM = 4

class ResponseIndexer(Enum):
    '''
        ResponseIndexer: Use this to name the indices of incoming responses. 
            For example:
                [OUT_ORDER, OUT_DATA, OUT_CHECKSUM] = response
    '''
    RESP_ORDER = 0
    RESP_DATA = 1
    RESP_CHECKSUM = 2

def write_command(serial_file, command):
    '''
        write_command: Called by PyDuino when sending a command. Must 
            correspond to the read_command function on the arduino. Change this 
            and the arduino read_command function to update the outgoing 
            protocol. For example, the current protocol parses commands of the 
            form:
                [Order (int8), 
                 Device Index (int8), 
                 Motor Velocity (int16), 
                 Motor Position (int16), 
                 Checksum (int16)].
            
            Notes:
                1) The command will be exactly the same as whatever is passed
                    to inteface.send() EXCEPT it will have a checksum appended. 
                    For example:
                        PyDuino.send([1, 2, 3, 4]) 
                    would result in the command array:
                        1, 2, 3, 4, -11 = command
                                     ^
                                  checksum
                2) PyDuino doesn't care how many elements are in the array, 
                    so long as the first element is the Order and the second
                    element is the device index. Any command of the form:
                        [Order, Device Index, ...]
                    is valid, and can be parsed here. 
                3) Data types in this case are important. The following are 
                    availible to choose from:
                        int8, int16, int32
                    and have corresponding read and write functions defined 
                    below.
                4) Assuming that you are not manually handling checksums on the
                    arduino side, the checksum MUST be passed last. 
        Inputs:
            serial_file: serial object
                The arduino serial connection. 
            command: [int]
                A list of integers to send to the arduino. 
    '''

    # Here's a bit of parsing that lets commands of the form [Order, Device Index] slide
    if (len(command) == 3):
        order, device_index, checksum = command
        
        # Note: you can ONLY pad with zeros to maintain integrity of checksum
        motor_velocity, motor_position = 0, 0 
    else:
        order, device_index, motor_velocity, motor_position, checksum = command

    write_i8(serial_file, order)
    write_i8(serial_file, device_index)
    write_i16(serial_file, motor_velocity)
    write_i16(serial_file, motor_position)
    write_i16(serial_file, checksum)

def read_response(serial_file):
    '''
        read_response: Called by PyDuino when getting a response from the
            arduino. Must correspond to the write_command function on the
            arduino. Change this and the arduino write_command function to 
            update the incoming protocol. For example the current protocol 
            parses responses of the form:
                [Order (int8), 
                 Data (int16),
                 Checksum (int16)]
                
            Notes:
                1) PyDuino doesn't care how many elements are in the array, 
                    that is returned so long as it has a length of at least 3
                    (This is required to detect response corruption.) The 
                    response, minus the checksum, will be stored in the 
                    corresponding device object. For example if the response
                    had a length of three:
                        1, 2, -4 = [order, value, checksum]
                    Reading from that device would return:
                        PyDuino.read_data(device=0)
                        >>> [1, 2]
                2) Assuming that you are not manually handling checksums on the
                    python side, the checksum MUST be passed last. 
                3) Though can return almost any list of values from the 
                    arduino, DO NOT return a list with the first value equal to
                    Order.Error. This is a special value used to detect message
                    corruption:
                        if (...
                            resp[OutCommandIndexer.OUT_ORDER.value] == Order.ERROR.value and 
                            resp[OutCommandIndexer.OUT_DATA.value] == Error.CORRUPTION.value)):
                            # Message has been orrupted on arduino side 
        Inputs:
            serial_file: serial object
                The arduino serial connection. 
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
