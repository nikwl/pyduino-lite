# pyduino-lite
## Overview 
This repo implements easily modifiable Python-to-Arduino communication package that enables fast sending and receiving of commands and implements a double-sided checksum to prevent message corruption. Designed with ease of use in mind, it should be very easy to adapt to new projects and augment with new protocols. I've used it in several projects to control multiple Dynamixel servos and stepper motors and read from encoders and pressure sensors simultaneously.

The system has the following advantages:

1) Message passing protocol can be updated quickly by changing the read and write functions on the Python and Arduino sides.
2) Each device has its own independent outgoing command queue. 
3) Commands can be appended with a backoff, which will prevent new commands being sent until that command has completed. 
4) Reading from serial on the arduino side is extremely fast, enabling simultaneous control of many stepper motors or other devices that require looping quickly over the body of the Arduino code.

Correct usage of this packaged depends on your use case.

* If you're executing a sequence of commands then the goal is to fill up the command queues and pass each command with a backoff that's just slightly less than you think it will take for the command to complete. When a device queue is full then the `PyDuino.send()` method will block until there's space in the queue, so there's no risk of overloading a device. This technique will result in a (near) smooth transition between commands. 
* If you're executing real-time commands then the goal is to pick a rate at which to send (velocity) commands and then instantiate PyDuino with a queue size of 1  (commands sent on every iteration should overwrite previous commands, and should be sent as soon as they are enqueued).

## Installation

1) Flash the arduino script in `Arduino/stepper_encoder`.
2) Install required python packages:
    ```bash
    pip install -r requirements.txt
    ```
3) Test the installation:
    ```bash
    python test.py
    ```
    By default the test will stream data from an encoder attached to pins 5 and 6. If no encoder is connected it will just stream the default value which is 0. If you see a stream of 0s then then you're in business. Attach an encoder to those pins and watch the stream of values update!

## Usage
```python
from pyduino import PyDuino, Order, Error

# Open serial connections
interface = PyDuino(num_devices=2, verbose=False)
interface.connect('/dev/ttyACM0', 115200)
interface.start()

# Command to query device 1
command = [Order.QUERY_DEVICE.value, 1]

print('*** streaming sensor data ***')
while True:
    interface.send(command)
    data = interface.read_data(device=1)

    if (data is not None):
        print('{}'.format(data[-1]))
```

## Methods

`PyDuino(num_devices, queue_size=2, verbose=False)`

Starts PyDuino with the given number of devices. The queue size determines the number of commands that can be appended to each device queue. 

`PyDuino.connect(ser_string='/dev/ttyUSB1', baudrate=115200, timeout=1.0)`

Attempts to "ping" the Arduino without throwing an exception (will return True if connection was successful, False otherwise). Useful for testing several serial ports if you're not sure which one the Arduino is connected to. 

`PyDuino.send(command, backoff=0)`

Will enqueue the command to the corresponding device queue. Assumes that the command has the basic form: [Order, Device Index, ...]. Optionally specify a time for PyDuino to "backoff" before sending the next command.

`PyDuino.read_data(device)`

Will return the most recent data received from a device. 

`PyDuino.is_connected()`

Will return True if the PC is connected to the Arduino. 

`PyDuino.is_waiting()`

Will return True if there are still commands that the interface is waiting to send. Call this before exiting your script to ensure you don't close the connection too early.

## Applications
### Dynamixel Shield
I initially developed this package to control motors using the [Dynamixel Shield](http://www.robotis.us/dynamixel-shield/). The Dynamixels are great because they implement velocity and/or position control and can also be queried for their location. I've added an example that controls 4 Dynamixel motors in `Arduino/dynamixel`, using the same protocol as the stepper and encoder example.

If you'd like to use this library to control Dynamixel motors with the Dynamixel Shield but aren't quite sure where to start, I'd recommend checking out my [Dynamixel Shield tutorial](https://github.com/nikwl/dynamixel-shield-toolbox). It will guide you through the process of purchasing the right hardware and setting up the motors with the shield.

## Sources
The low level serialization is based on the great [robust-serial](https://github.com/araffin/arduino-robust-serial) package covered in this [Medium article](https://medium.com/@araffin/simple-and-robust-computer-arduino-serial-communication-f91b95596788). I really liked this implementation but I didn't like how large and cumbersome the package was to adapt to new projects, so I decided to make my own.

The command queuing structure is based on GRBL and the method they use to simulate smooth transition between commands when streaming. See this [script](https://github.com/grbl/grbl/blob/master/doc/script/stream.py) in the GRBL source code for more information.
