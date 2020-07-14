from pyduino import PyDuino, Order, Error

# Open serial connections
interface = PyDuino(num_devices=2, verbose=False)

# Scan ACM ports and attempt to connect
for postfix in range(10):
    if (interface.connect('/dev/ttyACM0', 115200)):
        break
if (not interface.is_connected):
    raise RuntimeError('Device connection failed.')
interface.start()

# Command to query device 1
command = [Order.QUERY_DEVICE.value, 1]

print('*** streaming sensor data ***')
while True:
    interface.send(command)
    data = interface.read_data(device=1)

    if (data is not None):
        print('{}'.format(data[-1]))