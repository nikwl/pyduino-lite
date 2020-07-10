from pyduino import PyDuino, Order, Error

# Open serial connections
interface = PyDuino(num_devices=2, verbose=False)
interface.connect('/dev/ttyACM0', 115200)
interface.start()

# Command to query sensor 1, with two 0s as padding
command = [Order.MOVE_OR_QUERY.value, 1, 0, 0]

print('*** streaming sensor data ***')
while True:
    interface.send(command) 
    data = interface.read_data(device=1)

    if (data is not None):
        print('{}'.format(data[-1]))