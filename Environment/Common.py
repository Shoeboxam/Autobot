import subprocess
import os
import sys


def code_refresh():
    # Execute git pull and wait
    process = subprocess.Popen(['git' 'pull', 'master'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    while process.poll() is None:
        pass

    # Store process codes
    # stdout, stderrout = process.communicate()

    # Restart the script with updates
    os.execv(__file__, sys.argv)


def tag_match(tags, keys):
    if tags is 'all':
        return keys
    else:
        locations = []
        for tag in tags:
            for keyval in keys:
                if tag in keyval and keyval not in locations:
                    locations.append(keyval)
        return locations


def arg_match(args, keys):
    if type(args) is list:
        if len(args) != 1:
            assert(len(args) == len(keys))
        else:
            args *= len(keys)
    else:
        args = [args] * len(keys)
    return args


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


# ULTRASONIC SENSORS
ultrasonic_readings = 9
keys_ultrasonic = ['fl', 'fr', 'bl', 'br']
# Front left, front right, back left, back right


# MOTORS
motor_keys = ['fl', 'fr', 'bl', 'br']
