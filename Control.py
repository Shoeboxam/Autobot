import os

# If run on the pi, control the buggy! Otherwise, run the emulator
if os.uname()[4].startswith('arm'):
    import Interface.Autobot as env
else:
    import Interface.Emulator as env

# Primary control loop; left as a stub until environment code is stable
while True:
    serial_data = env.bluetooth_read()
