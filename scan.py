import smbus

bus = smbus.SMBus(1)  # Adjust the bus number if necessary
for device in range(128):
    try:
        bus.read_byte(device)
        print(f"I2C device found at address {hex(device)}")
    except Exception as e:
        print(f"Failed at address {hex(device)}: {e}")