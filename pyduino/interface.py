import threading
import struct
import serial
import time

from Queue import Queue, Empty, Full

from .protocol import Order, Error, CommandIndexer, ResponseIndexer, read_response, write_command, read_order, write_order
import copy

class Device():
    def __init__(self, queue):
        self._command_start = time.time()
        self._command_backoff = 0

        self._queue = queue

        self._data = None
        self._data_lock = threading.Lock()

    def get_command(self):
        return self._queue.get(block=False)
    
    def put_command(self, cmd):
        # Some weirdness prevents a normal put from being interrupted. This 
        # timeout fixes that.
        try:
            self._queue.put(cmd, timeout=10.0)
            return
        except Full:
            self.put(cmd)

    def put_data(self, data):
        with self._data_lock:
            self._data = data
            
    def read_data(self):
        with self._data_lock:
            return self._data

    def start(self, backoff=0):
        self._command_backoff = backoff
        self._command_start = time.time()

    def is_done(self):
        return (self._command_start + self._command_backoff <= time.time())

class PyDuino(threading.Thread):
    def __init__(self, num_devices, queue_size=2, verbose=False):
        '''
            PyDuino: Interface to communicate with arduino over serial,
                specifically allows handling of multiple time-dependant 
                commands accross multiple devices according to a custom 
                protocol. 
                
                Allows specification of an outgoing and incoming protocol in 
                the protocol.py file. This protocol must be the same on the
                arduino side (obviously).
                
                PyDuino creates a number of device objects corresponding to 
                the number of devices specified. Each device object contains a 
                queue of commands for that device, data returned from that 
                device, and timing information such that the device can be set 
                to operate without interruption for a pre-determined amount of 
                time. This is useful for tasks like running steppers for a 
                specific number of steps. 

                PyDuino is designed to mimic the behavior of grbl, such that 
                for each device, commands are enqueued and then sent as soon as 
                the previous command completes. This should create the 
                illusion of an uninterrupted transition between commands.
            Inputs:
                num_devices: int
                    How many devices are attached to the arduino. Devices are 
                    referenced by their index, which will range from 0 to this 
                    number - 1. 
                queue_size: int
                    How many commands should be queued before blocking the 
                    enqueueing thread. Setting this number too small can cause 
                    jerky movement.                
                verbose: bool
                    If true, will print out commands, responses, and other 
                    identifying information.
        '''
        threading.Thread.__init__(self) 
        self.daemon = True
        self.verbose = verbose

        # Devices arduino is controlling
        self._devices = tuple(Device(Queue(maxsize=queue_size)) for _ in range(num_devices))
        self._num_devices = num_devices

        # Keep track of total messages sent
        self._messages_sent = 0

        # Keeps threads exiting before they should
        self._command_counter = 0
        self._counter_lock = threading.Lock()

    def connect(self, ser_string='/dev/ttyUSB1', baudrate=115200, timeout=1.0):
        '''
            connect: Attempts to connect to the arduino. Will return false if 
                unsuccessful. Useful if you're not sure which port the arduino
                is on and you want to try serveral.
            Inputs:
                ser_string: str
                    arduino address.
                baudrate: int 
                    baudrate such as 9600 or 115200 etc.
                timeout: float
                    how long (seconds) to wait for a resposne.
        '''
        is_connected = False

        if self.verbose:
            print('trying device: {}'.format(ser_string))

        # Dead giveaway that this is the wrong device is a serial exception
        try:
            ser = serial.Serial(ser_string, baudrate, timeout=0.1)
        except serial.serialutil.SerialException as e:
            return False

        ser.flushInput()
        start = time.time()
        while (not is_connected) and (time.time() - start < timeout):
            write_order(ser, Order.HELLO)
            
            # Common result of reading from the wrong device is serial exception here
            try:
                bytes_array = bytearray(ser.read(1))
            except serial.serialutil.SerialException:
                break

            if not bytes_array:
                time.sleep(0.01)
                continue
            byte = bytes_array[0]

            # We should only get one of these two return codes
            if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
                is_connected = True
                break
            else:
                break      

        if is_connected:
            ser.timeout = None
            self._ser = ser

            if (self.verbose):
                print('connected to device: {}'.format(ser_string))
            return True
        else:
            return False

    def read_data(self, device):
        '''
            read_data: Read data from a device. Device data is None by default,
                ie if this function is called on a device that has not yet
                recieved data from the arduino then the result will be none. 
        '''
        if (device >= self._num_devices):
            raise ValueError('Device does not exist')
        return self._devices[device].read_data()
        

    def is_connected(self):
        '''
            is_connected: Checks if the interface is currently connected to an
                Arduino. 
        '''
        return hasattr(self, '_ser')

    def is_waiting(self):
        '''
            is_waiting: Will return true if the interface still has messages 
                waiting to be sent. Will return false if it is safe to close 
                the connection. 
        '''
        with self._counter_lock:
            return self._command_counter != 0

    def send(self, command, backoff=0):
        '''
            send: enqueues the input command into the corresponding device 
                queue. The command should correspond with the protocol defined
                in protocol.py. Optionally a backoff can be specified, which 
                prevents additional commands from being sent to a device before
                the backoff time has elapsed. 

                Will partially parse the command to obtain the device index. 
                For this reason, the protocol should have the following form:
                    [Order, Device Index, ...]

                Additionally, PyDuino doesn't care how many elements are in the 
                array, so long as the first element is the Order and the second
                element is the device index. Any command of the form specified
                above is valid, so long as it is parsed in the write_command
                function in protocol.py.
            Inputs:
                command: [int]
                    Command of the form:
                        [Order, Device Index, ...].
                    Should correspond to the your parsing specified in 
                    protocol.py. 
                backoff: float
                    Time to wait before sending another command to this device.
        '''

        # Copy and force to be ints
        command = [int(c) for c in command]

        # Append checksum
        command.append(self.generate_checksum(command))

        # Commands should be a tuple of (command, backoff) so that they're dequeued correctly
        # In addition, we assume that the second element is the device index
        device_id = command[1]
        self._devices[device_id].put_command((command, backoff))

        # Increment command counter
        with self._counter_lock:
            self._command_counter += 1

    def run(self):
        if (not hasattr(self, '_ser')):
            raise RuntimeError('Start method called before serial connection was initialized.')

        command = None
        while self.is_alive():
            for device_index, device in enumerate(self._devices):
                
                # Don't get the next command if the device is still operating
                if (device.is_done()):
                    try:
                        command, target_time = device.get_command()
                    except Empty:
                        pass

                if (command is not None):
                    while self.is_alive():
                        if self.verbose:
                            print('*** sending command ***')
                            print('  {}'.format(command))

                        # Send the command, start device timer
                        self._ser.flushInput()
                        write_command(self._ser, command)
                        device.start(backoff=target_time)

                        # Get the response
                        resp = read_response(self._ser)
                            
                        # Check if it was corrupted on the send or on the return
                        if (not self.verify_checksum(resp) or
                                (resp[ResponseIndexer.RESP_ORDER.value] == Order.ERROR.value and 
                                resp[ResponseIndexer.RESP_DATA.value] == Error.CORRUPTION.value)):
                            if self.verbose:
                                print('*** got response ***')
                                print(' command corrupted: {}'.format(resp))
                            continue
                        else:
                            if self.verbose:
                                print('*** got response ***')
                                print('  {}'.format(resp))
                            # Record the returned value
                            device.put_data(resp[:-1])
                        break
                    
                    # Decrement the number of messages we're waiting on
                    with self._counter_lock:
                        self._command_counter -= 1
                    self._messages_sent += 1
                    
                    # Reset command
                    command = None

    @staticmethod
    def generate_checksum(command):
        return ~sum(command)

    @staticmethod
    def verify_checksum(data):
        return ~sum(data) == 0

def speed_test_interface(interface, num_comm=100, num_test=5):
    import timeit

    def test_fun():
        stime = 0
        for i in range(num_comm):
            interface.send([Order.MOVE_OR_QUERY.value, 1, 1, 1], stime) # Query sensor 1
    t = 0

    for _ in range(num_test):
        t += timeit.timeit(test_fun, number=1)

    print(t/(num_test*num_comm))

def stream_test_interface(interface):
    while True:
        interface.send([Order.MOVE_OR_QUERY.value, 1, 1, 1]) # Query sensor 1
        print(interface.read_data(device=1))

if __name__ == "__main__":
    # Open serial connections
    interface = PyDuino(num_devices=2, verbose=False)
    interface.connect('/dev/ttyACM0', 115200)
    interface.start()

    speed_test_interface(interface)