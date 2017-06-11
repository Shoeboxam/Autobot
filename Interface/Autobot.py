from Interface.Common import *
import time
import datetime
import threading
import atexit

# Raspberian dependent
import picamera
import Adafruit_BMP.BMP085 as BMP085
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import serial
import pynmea2
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
    code_refresh()


# Simple demo for running motors
def on_press_m(channel):
    set_speed(255, 'all', 'forward')


def on_depress_m(channel):
    set_speed(0, 'all')

GPIO.setup(pin['button'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['button'], GPIO.RISING, callback=on_press_m, bouncetime=300)

# LIGHT SENSOR
light_bouncetime = 100  # Delay to fix fluctuation

light_spokes_left = 0
light_spokes_right = 0


def light_spokes_left_increment(channel):
    global light_spokes_left
    light_spokes_left += 1
GPIO.setup(pin['light_left'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_left'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=light_spokes_left_increment)


def light_spokes_right_increment(channel):
    global light_spokes_right
    light_spokes_right += 1
GPIO.setup(pin['light_right'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_right'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=light_spokes_right_increment)


# ULTRASONIC SENSORS
for key in keys_ultrasonic:
    GPIO.setup(pin['ultra_' + key + '_trigger'], GPIO.OUT)
    GPIO.setup(pin['ultra_' + key + '_echo'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def ultrasonic_detect(tags='all'):
    distance_record = {}

    locations = tag_match(tags, keys_ultrasonic)

    for loc in locations:
        distances = []
        for read in range(ultrasonic_readings):
            GPIO.output(pin['ultra_' + loc + '_trigger'], 1)
            time.sleep(1e-5)
            GPIO.output(pin['ultra_' + loc + '_trigger'], 0)

            distances.append(GPIO.input(pin['ultra_' + loc + '_echo']) * .034 / 2)
        distance_record[loc] = sorted(distances)[int(ultrasonic_readings / 2)]
    return distance_record


# GPS
ser_gps = serial.Serial('COM6', 9600, timeout=0)
pynmea2.parse(ser_gps.readline().decode("utf-8"))

gps = {}


def get_geolocation():
    gps_temp = gps
    gps.clear()
    return gps_temp


# Update gps records once per second
def thread_geolocation():
    threading.Timer(1.0, thread_geolocation).start()
    utc = 0

    try:
        while True:
            data = ser_gps.readline().decode('UTF-8')

            # Minimum recommended data
            if data.startswith('$GPRMC'):
                sentence = pynmea2.parse(data)

                combined_time = datetime.datetime.combine(sentence['datestamp'], sentence['timestamp'])
                utc = (combined_time - datetime.datetime.utcfromtimestamp(0)).total_seconds()
                gps[utc] = {}

            # Velocity made good
            if data.startswith('$GPVTG'):
                sentence = pynmea2.parse(data)
                gps[utc]['velocity'] = sentence['spd_over_grnd_kmph']
                gps[utc]['heading'] = sentence['true_track']  # True north

            # Fix data
            geoidal_separation = None
            if data.startswith('$GPGGA'):
                sentence = pynmea2.parse(data)

                # pass when inadequate fixation
                gps[utc]['num_sats'] = sentence['num_sats']

                # Conditionals correct for hemisphere
                gps[utc]['latitude'] = sentence['lat']
                if sentence['lat_dir'] == 'S':
                    gps[utc]['latitude'] *= -1

                gps[utc]['longitude'] = sentence['lon']
                if sentence['lon_dir'] == 'W':
                    gps['longitude'] *= -1

                gps[utc]['altitude'] = sentence['altitude']

                geoidal_separation = sentence['geo_sep']

            # Satellite reception data for precision estimate
            if data.startswith('$GPGSA'):
                sentence = pynmea2.parse(data)
                gps[utc]['precision'] = sentence['PDOP'] * geoidal_separation

                # Filter entry without reception data
                if gps[utc]['num_sats'] == 0:
                    del gps[utc]

    except pynmea2.ParseError:
        pass

thread_geolocation()

# CAMERA
camera = picamera.PiCamera()
# camera.capture('~/assets/queue/image.jpg')


# BAROMETER
# https://github.com/adafruit/Adafruit_Python_BMP/blob/master/examples/simpletest.py
sensor = BMP085.BMP085()


# ATTITUDE
# Calibration file generated by RTIMULibCal
SETTINGS_FILE = "RTIMULib.ini"
# imu = RTIMU.RTIMU(RTIMU.Settings(SETTINGS_FILE))

# assert(imu.IMUInit())

# imu.setSlerpPower(0.02)
# imu.setGyroEnable(True)
# imu.setAccelEnable(True)
# imu.setCompassEnable(True)

# poll_interval = imu.IMUGetPollInterval()


# def attitude_poll():
#     if imu.IMURead():
#         # https://github.com/RPi-Distro/RTIMULib/blob/master/Linux/python/PyRTIMU_RTIMU.cpp#L189
#         return imu.getIMUData()


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
motors = {'fl': mh.getMotor(1),
          'bl': mh.getMotor(2),
          'fr': mh.getMotor(3),
          'br': mh.getMotor(4)}


def set_speed(speed, tags='all', direction='forward'):
    locations = tag_match(tags, motor_keys)

    speed_list = arg_match(speed, locations)
    direction = arg_match(direction, locations)
    for index, key in enumerate(locations):
        if key is 'fl' or key is 'fr':
            if direction[index] is 'forward':
                direction[index] = 'backward'
            else:
                direction[index] = 'forward'

    print(direction)
    # Change type of direction from string to directional type
    for count, direct in enumerate(direction):
        if 'back' in direct:
            direction[count] = Adafruit_MotorHAT.BACKWARD
        else:
            direction[count] = Adafruit_MotorHAT.FORWARD

    # Set the motors!
    for count, loc in enumerate(locations):
        if speed_list[count] == 0:
            motors[loc].run(Adafruit_MotorHAT.RELEASE)
        else:
            motors[loc].run(direction[count])
            motors[loc].setSpeed(speed_list[count])

atexit.register(set_speed, speed=0)
