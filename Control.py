import os
import threading

# If run on the pi, control the buggy! Otherwise, run the emulator
if os.uname()[4].startswith('arm'):
    import Interface.Autobot as Env
else:
    import Interface.Emulator as Env

# Primary control loop; left as a stub until environment code is stable
interval_locate = 1.0
interval_control = 0.1


def locate():
    threading.Timer(interval_locate, locate).start()


    attitude = Env.attitude_poll()


locate()
