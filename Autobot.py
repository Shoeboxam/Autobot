import subprocess
import os
import sys
from datetime import datetime
import time

# Raspberian dependent
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

pin = {'button':            27,
       'light_left':        17,
       'light_right':       18,
       'ultra_fr_trigger':  5,
       'ultra_fr_echo':     6,
       'ultra_fl_trigger':  12,
       'ultra_fl_echo':     13,
       'ultra_br_trigger':  16,
       'ultra_br_echo':     19,
       'ultra_bl_trigger':  20,
       'ultra_bl_echo':     21}


# BUTTON
def code_refresh(channel):
    # Stop detection to prevent multiple reads
    GPIO.remove_event_detect(channel)

    # Execute git pull and wait
    process = subprocess.Popen(['git' 'pull', 'master'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    while process.poll() is None:
        pass

    # Store process codes
    # stdout, stderrout = process.communicate()

    # Restart the script with updates
    os.execv(__file__, sys.argv)

GPIO.setup(pin['button'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['button'], GPIO.RISING, callback=code_refresh, bouncetime=300)


# LIGHT SENSOR
class CircularList:
    def __init__(self, length):
        self.cache = [None] * length
        self.length = length
        self.pointer = 0

    def set(self, data):
        self.cache[self.pointer] = data
        self.pointer = (self.pointer + 1) % self.length

light_cache_memory = 2**4
light_cache_left = CircularList(light_cache_memory)
light_cache_right = CircularList(light_cache_memory)
light_bouncetime = 100

GPIO.setup(pin['light_left'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_left'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=lambda x: light_cache_left.set(datetime.now().time()))

GPIO.setup(pin['light_right'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_right'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=lambda x: light_cache_right.set(datetime.now().time()))


# ULTRASONIC SENSOR
ultrasonic_readings = 9
distance_record = {}

keys = ['fl', 'fr', 'bl', 'br']
# Front left, front right, back left, back right

for key in keys:
    GPIO.setup(pin['ultra_' + key + '_trigger'], GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin['ultra_' + key + '_echo'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def ultrasonic_detect(locations=None):
    for loc in locations:
        assert(loc in keys)

    if locations is None:
        locations = keys

    for loc in locations:
        distances = []
        for read in range(ultrasonic_readings):
            GPIO.output(pin['ultra_' + loc + '_trigger'], 1)
            time.sleep(1e-5)
            GPIO.output(pin['ultra_' + loc + '_trigger'], 0)

            distances.append(GPIO.input(pin['ultra_' + loc + '_echo']) * .034 / 2)
        distance_record[loc] = sorted(distances)[int(ultrasonic_readings / 2)]
