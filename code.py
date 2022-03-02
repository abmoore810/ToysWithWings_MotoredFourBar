"""
----- CHANGE BELOW NUMBER TO CHANGE AMOUNT OF TIME PROGRAM WILL RUN -----
"""
sample_time = 5    # In seconds
CCW = True        # Set to True for CCW movement or False for CW movement

"""
---------- DO NOT CHANGE ANYTHING BELOW THIS LINE ----------
"""

import time
from math import atan2, degrees
import pwmio
import board
import busio
import adafruit_mpu6050
import digitalio
from adafruit_motor import servo

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

i2c = busio.I2C(scl=board.GP1, sda=board.GP0)
sensor = adafruit_mpu6050.MPU6050(i2c)
pwm = pwmio.PWMOut(board.GP2, frequency=50)
s = servo.ContinuousServo(pwm)

# callibrate initial readings
init_xa = []
init_ya = []
init_za = []
samples = 1000
print("Callibrating Sensor...")
led.value = True
for i in range(0, samples, 1):
    xi, yi, zi = sensor.acceleration
    init_xa.append(xi)
    init_ya.append(yi)
    init_za.append(zi)

xi = sum(init_xa)/samples
yi = sum(init_ya)/samples
zi = sum(init_za)/samples

led.value = False
print("Measuring...")
time.sleep(3)

# Given a point (x, y) return the angle of that point relative to x axis.
# Returns: angle in degrees
def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 90:
        angle += 360
    return angle

# Get initial angles
angle_xzi = vector_2_degrees(xi,zi)
angle_yzi = vector_2_degrees(yi,zi)

# Given an accelerometer sensor object return the inclination angles of X/Z and Y/Z
# Returns: tuple containing the two angles in degrees
def get_inclination(_sensor):
    x, y, z = _sensor.acceleration
    return vector_2_degrees(x, z), vector_2_degrees(y, z)

startTime = int(round(time.time() * 1000))
currTime = int(round(time.time() * 1000))

# Sets default direction to Counter-Clockwise
direction = 1
if not CCW:
    direction = -1

while (currTime - startTime) < sample_time*1000:
    currTime = int(round(time.time() * 1000))
    angle_xz, angle_yz = get_inclination(sensor)
    xj, yj, zj = sensor.gyro
    xj = xj * (-1)
    delta_angle_xz = angle_xz - angle_xzi
    delta_angle_yz = angle_yz - angle_yzi
print("Link Angle = {:6.2f}deg   gyro: x = {:6.2f}deg/s  y = {:6.2f}deg/s  z = {:6.2f}deg/s".format(delta_angle_yz, xj, yj, zj))
    s.throttle = 0.5 * direction
    time.sleep(0.2)

s.throttle = 0.0 * direction
print("\nProgram Complete!")
