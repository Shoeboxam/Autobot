from Interface.Common import *
import time
import datetime
import threading
import serial
import pynmea2
import atexit

# Raspberian dependent
import picamera
# import RTIMU
from Adafruit_BMP import BMP085
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
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

stationary = True


# BUTTON
def on_press(channel):

    # Toggle wheels on/off
    global stationary
    if stationary:
        set_speed(255, 'all', 'forward')
    else:
        set_speed(0)
    stationary = not stationary

    # Stop detection to prevent multiple reads
    # GPIO.remove_event_detect(channel)
    # code_refresh()

GPIO.setup(pin['button'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['button'], GPIO.RISING, callback=on_press, bouncetime=300)


# LIGHT SENSOR
light_bouncetime = 100  # Delay to fix fluctuation


class CircularList:
    def __init__(self, length):
        self.cache = [None] * length
        self.length = length
        self.pointer = 0

    def set(self, data):
        self.cache[self.pointer] = data
        self.pointer = (self.pointer + 1) % self.length

light_cache_memory = 2**4
light_left = CircularList(light_cache_memory)
light_right = CircularList(light_cache_memory)


# CM / sec
def get_velocity():
    init_time = time.time()

    left_impulses = 0
    for trigger in light_left.cache:
        if init_time - trigger < 1.0:
            left_impulses += 1

    right_impulses = 0
    for trigger in light_right.cache:
        if init_time - trigger < 1.0:
            right_impulses += 1

    # Wheel circumference: 21 cm
    # Spokes: 20
    # 1.05 cm traveled per light trigger
    return [left_impulses * 1.05, right_impulses * 1.05]


def light_spoke_left(channel):
    global light_left
    light_left.set(time.time())
GPIO.setup(pin['light_left'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_left'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=light_spoke_left)


def light_spoke_right(channel):
    global light_right
    light_left.set(time.time())
GPIO.setup(pin['light_right'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(pin['light_right'], GPIO.RISING, bouncetime=light_bouncetime,
                      callback=light_spoke_right)


# ULTRASONIC SENSORS
for key in keys_ultrasonic:
    GPIO.setup(pin['ultra_' + key + '_trigger'], GPIO.OUT)
    GPIO.setup(pin['ultra_' + key + '_echo'], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def get_ultra_depth(tags='all'):
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
gps = {}
if os.uname()[4].startswith('arm'):
    ser_gps = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
else:
    ser_gps = serial.Serial('COM6', 9600, timeout=0)


def get_geolocation():
    gps_temp = gps
    gps.clear()
    return gps_temp


# Update gps records once per second
def thread_geolocation():
    utc = 0

    try:
        while True:
            data = ser_gps.readline().decode('UTF-8')

            # Minimum recommended data
            if data.startswith('$GPRMC'):
                sentence = pynmea2.parse(data)

                combined_time = datetime.datetime.combine(sentence.datestamp, sentence.timestamp)
                utc = (combined_time - datetime.datetime.utcfromtimestamp(0)).total_seconds()
                print(gps)
                gps[utc] = {}

            # Velocity made good
            if data.startswith('$GPVTG'):
                sentence = pynmea2.parse(data)
                gps[utc]['velocity'] = sentence.spd_over_grnd_kmph
                gps[utc]['heading'] = sentence.true_track # True north

            # Fix data
            geoidal_separation = None
            if data.startswith('$GPGGA'):
                sentence = pynmea2.parse(data)
                
                gps[utc]['num_sats'] = float(sentence.num_sats)

                # Conditionals correct for hemisphere
                gps[utc]['latitude'] = sentence.lat
                if sentence.lat_dir == 'S':
                    gps[utc]['latitude'] *= -1

                gps[utc]['longitude'] = sentence.lon
                if sentence.lon_dir == 'W':
                    gps['longitude'] *= -1

                gps[utc]['altitude'] = sentence.altitude

                try:
                    geoidal_separation = float(sentence.geo_sep)
                except ValueError:
                    pass

            # Satellite reception data for precision estimate
            if data.startswith('$GPGSA'):
                sentence = pynmea2.parse(data)
                try:
                    gps[utc]['precision'] = float(sentence.pdop) * geoidal_separation
                except TypeError:
                    pass
                        

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
bmp_sensor = BMP085(0x77, 2)  # High resolution mode


def get_temperature():
    return bmp_sensor.readTemperature()


def get_pressure():
    return bmp_sensor.readPressure()


def get_altitude():
    return bmp_sensor.readAltitude()


# ATTITUDE
# Calibration file generated by RTIMULibCal
##SETTINGS_FILE = "RTIMULib.ini"
##imu = RTIMU.RTIMU(RTIMU.Settings(SETTINGS_FILE))
##
##assert(imu.IMUInit())
##
##imu.setSlerpPower(0.02)
##imu.setGyroEnable(True)
##imu.setAccelEnable(True)
##imu.setCompassEnable(True)
##
##poll_interval = imu.IMUGetPollInterval()
##attitude_data = None


def thread_attitude():
    global attitude_data
    threading.Timer(poll_interval, thread_attitude)
    if imu.IMURead():
        attitude_data = imu.getIMUData()

thread_attitude()


def get_acceleration():
    return attitude_data['accel']


def get_orientation():
    return attitude_data['compass']


def get_rotation():
    return attitude_data['gyro']


# BLUETOOTH
if os.uname()[4].startswith('arm'):
    bluetooth_serial = serial.Serial('dev/ttyAMA0', baudrate=57600)
else:
    bluetooth_serial = serial.Serial("COM8", baudrate=57600)


def bluetooth_control():
    while True:
        data = bluetooth_serial.readline()
        data_decoded = memoryview(data).cast('B')

        if len(data_decoded) < 2:
            continue

        vel_x = (+data_decoded[1] - 128) / 2
        vel_y = (-data_decoded[0] + 128) * 2

        def clamp(n, smallest, largest):
            return max(smallest, min(n, largest))

        speed_l = clamp(int(vel_y + vel_x), -255, 255)
        speed_r = clamp(int(vel_y - vel_x), -255, 255)

        set_speed(speed_l, 'l')
        set_speed(speed_r, 'r')


# MOTORS
mh = Adafruit_MotorHAT(addr=0x60)
motors = {'fl': mh.getMotor(1),
          'bl': mh.getMotor(2),
          'fr': mh.getMotor(3),
          'br': mh.getMotor(4)}


def set_speed(speed, tags='all'):
    locations = tag_match(tags, motor_keys)
    speed_list = arg_match(speed, locations)

    # Convert signed speeds into directions
    direction = []
    for ind, speed_ind in enumerate(speed_list):
        speed_list[ind] = abs(speed_list[ind])
        if speed_ind == 0:
            direction.append('release')
        elif speed_ind < 0:
            direction.append('backward')
        else:
            direction.append('forward')

    # Fix reorientated motor directions
    for index, key in enumerate(locations):
        if key is 'fl' or key is 'fr':
            if direction[index] is 'forward':
                direction[index] = 'backward'
            else:
                direction[index] = 'forward'

    # Change type of direction from string to directional type
    for count, direct in enumerate(direction):
        if 'back' in direct:
            direction[count] = Adafruit_MotorHAT.BACKWARD
        else:
            direction[count] = Adafruit_MotorHAT.FORWARD

    # Set the motors!
    for count, loc in enumerate(locations):
        if direction[count] is 'release':
            motors[loc].run(Adafruit_MotorHAT.RELEASE)
        else:
            motors[loc].run(direction[count])
            motors[loc].setSpeed(speed_list[count])

atexit.register(set_speed, speed=0)
