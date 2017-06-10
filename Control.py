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
    # Wheel circumference: 21 cm
    # Spokes: 20
    # 1.05 cm traveled per light trigger
    distance_right_cm = Env.light_spokes_right * 1.05
    velocity_right_cm = distance_right_cm / interval_locate

    distance_left_cm = Env.light_spokes_left * 1.05
    velocity_left_cm = distance_left_cm / interval_locate

    Env.light_spokes_left = 0
    Env.light_spokes_left = 0

    attitude = Env.attitude_poll()


locate()
