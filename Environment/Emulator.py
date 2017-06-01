import Environment.Common as Common
import random


def button_press(channel):
    Common.code_refresh()


def ultrasonic_detect(tags='all'):
    distance_record = {}

    locations = Common.tag_match(tags, Common.keys_ultrasonic)

    for loc in locations:
        distance_record[loc] = random.randint(0, 255)
    return distance_record


def set_speed(speed, tags='all', direction='forward'):
    pass
