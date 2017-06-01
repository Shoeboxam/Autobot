import Environment.Common as Common
from datetime import datetime
import time
import atexit

# Raspberian dependent
import picamera
import Adafruit_BMP.BMP085 as BMP085
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import RTIMU
import serial
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

pin = {'button':            13,
       'light_left':        11,
       'light_right':       12,
       'ultra_fr_trigger':  29,
       'ultra_fr_echo':     31,
       'ultra_fl_trigger':  32,
       'ultra_fl_echo':     33,
       'ultra_br_trigger':  36,
       'ultra_br_echo':     35,
       'ultra_bl_trigger':  38,
       'ultra_bl_echo':     40}


# BUTTON
def on_press(channel):
    # Stop detection to prevent multiple reads
    GPIO.remove_event_detect(channel)
    Common.code_refresh()

GPIO.setup(pin['button'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['button'], GPIO.RISING, callback=on_press, bouncetime=300)


# LIGHT SENSOR
light_bouncetime = 100  # Delay to fix fluctuation

GPIO.setup(pin['light_left'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_left'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=lambda: Common.light_cache_left.set(datetime.now().time()))

GPIO.setup(pin['light_right'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_right'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=lambda: Common.light_cache_right.set(datetime.now().time()))


# ULTRASONIC SENSORS
for key in Common.keys_ultrasonic:
    GPIO.setup(pin['ultra_' + key + '_trigger'], GPIO.OUT)
    GPIO.setup(pin['ultra_' + key + '_echo'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def ultrasonic_detect(tags='all'):
    distance_record = {}

    locations = Common.tag_match(tags, Common.keys_ultrasonic)

    for loc in locations:
        distances = []
        for read in range(Common.ultrasonic_readings):
            GPIO.output(pin['ultra_' + loc + '_trigger'], 1)
            time.sleep(1e-5)
            GPIO.output(pin['ultra_' + loc + '_trigger'], 0)

            distances.append(GPIO.input(pin['ultra_' + loc + '_echo']) * .034 / 2)
        distance_record[loc] = sorted(distances)[int(Common.ultrasonic_readings / 2)]
    return distance_record


# GPS
# https://www.raspberrypi.org/forums/viewtopic.php?f=28&t=56023


# CAMERA
camera = picamera.PiCamera()
camera.capture('~/assets/queue/image.jpg')


# BAROMETER
# https://github.com/adafruit/Adafruit_Python_BMP/blob/master/examples/simpletest.py
sensor = BMP085.BMP085()


# ATTITUDE
SETTINGS_FILE = "RTIMULib.ini"
imu = RTIMU.RTIMU(RTIMU.Settings(SETTINGS_FILE))

assert(imu.IMUInit())

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()


# https://github.com/Nick-Currawong/RTIMULib2/tree/master/Linux/python/tests
def attitude_poll():
    if imu.IMURead():
        return imu.getIMUData()


# BLUETOOTH
# http://www.uugear.com/portfolio/bluetooth-communication-between-raspberry-pi-and-arduino/
def bluetooth_read():
    bluetooth_serial = serial.Serial("/dev/rfcomm1", baudrate=9600)
    data = []

    while True:
        data.append(bluetooth_serial.readLine())
        if not data:
            break

    return data


# MOTORS
mh = Adafruit_MotorHAT(addr=0x60)
motors = {'fl': mh.getMotor(0),
          'fr': mh.getMotor(1),
          'bl': mh.getMotor(2),
          'br': mh.getMotor(3)}


def set_speed(speed, tags='all', direction='forward'):
    locations = Common.tag_match(tags, Common.motor_keys)

    speed = Common.arg_match(speed, locations)
    direction = Common.arg_match(direction, locations)

    # Change type of direction from string to directional type
    for count, direct in enumerate(direction):
        if direct in 'back':
            direction[count] = Adafruit_MotorHAT.BACKWARD
        else:
            direction[count] = Adafruit_MotorHAT.FORWARD

    # Set the motors!
    for count, loc in enumerate(locations):
        if speed[count] == 0:
            motors[loc].run(Adafruit_MotorHAT.RELEASE)
        else:
            motors[loc].setSpeed(speed[count])
            motors[loc].run(direction[count])

atexit.register(set_speed, speed=0)