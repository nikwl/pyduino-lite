# pyduino-lite
## Overview 
An easily modifiable python-arduino communication package that enables fast sending and recieving of commands and implements a double-sided checksum to prevent message corruption. Designed with ease of use in mind, it should be very easy to adapt to new projects and augment with new protocols. I've used it in several projects to control multiple Dynamixel servos and stepper motors and read from encoders and pressure sensors simultanously.

The system has the following advantages:

1) The protocol that dictates message passing can be updated easily by changing the read and write functions on the python and arduino sides.
2) Commands sent by the host pc can be queued up before being sent, and then sent after a precise and device-independant amount of time using the PyDuino.send function. This allows a single script to issue sequences of distinct commands to many distinct motors, or quickly read from multiple sensors.
3) The Arduino can be "pinged" using the PyDuino.connect function without throwing and exception. This is useful if the calling script is unsure of which port the ardino is connected to.
4) Reading from serial on the arduino side is extremely fast, enabling simultaneous control of many stepper motors or other devices that require looping over the body of the arduino code very quickly.

## Installation
1) Flash the arduino script in `/arduino/stepper_encoder`.
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

## Applications
### Dynamixel Shield
I initially developed this package specifically to control a dynamixel shield because I couldn't find any existing packages or tutorials for the shield. The dynamixels are great because you can control them via velocity and or position commands and also easily query their location. I've added an example that controls 4 dynamixel motors in `arduino/dynamixel`, using the same protocol as the stepper and encoder example.

However the dynamixel motors are a little more difficult to set up because in order to control multiple motors you need to reset all of their ids, and if you're using two different kinds of motor you also need to change their baudrates. See <> for dynamixel setup code.

## Sources
The low level serialization is based on the great [robust-serial](https://github.com/araffin/arduino-robust-serial) package covered in this [Medium article](https://medium.com/@araffin/simple-and-robust-computer-arduino-serial-communication-f91b95596788). I really liked this implementation but I didn't like how large and cumbersome the package was to adapt to new projects, so I decided to make my own.

The command queuing structure is based on GRBL and the method they use to simulate smooth transition between commands when streaming. See this [script](https://github.com/grbl/grbl/blob/master/doc/script/stream.py) in the GRBL source code for more information.
