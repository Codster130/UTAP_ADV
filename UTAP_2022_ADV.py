import threading
import time
import math
import board
import busio

#copied
#We tried several IMU sensors - may go back to this one
#import adafruit_lsm303dlh_mag
#import adafruit_lsm303_accel
#import adafruit_l3gd20
import adafruit_fxos8700
import adafruit_fxas21002c

#OLED screen
import adafruit_ssd1306

#PWM Board
import adafruit_pca9685

#Temp, Humidity, Pressure Sensor
import adafruit_bme280

#For OLED screen support
from PIL import Image, ImageDraw, ImageFont

#Error handling
import subprocess

#For access to operating system and array types
#Joystick support
import os, sys, struct, array

#Input output control
#Joystick support
from fcntl import ioctl

#RPi general purpose input/output pins
import RPi.GPIO as GPIO

#I2C address for the PWM driver board retrieved automatically
i2c_pwm = board.I2C()
pwm = adafruit_pca9685.PCA9685(i2c_pwm)

pwm.frequency = 1600

# i2c_lsm = board.I2C()
# mag_sensor = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c_lsm)
# accel_sensor = adafruit_lsm303_accel.LSM303_Accel(i2c_lsm)
# gyro_sensor = adafruit_l3gd20.L3GD20_I2C(i2c_lsm,rng=1,address=0x6B)
i2c_nxp = board.I2C()
mag_accel_sensor = adafruit_fxos8700.FXOS8700(i2c_nxp)
gyro_sensor = adafruit_fxas21002c.FXAS21002C(i2c_nxp)

#I2C address for the OLED screen is 0x3c
i2c_oled = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(128,64, i2c_oled,addr=0x3c,reset=[])

#I2C address for the temp, humidity and pressure sensor is 0x76
i2c_bme = board.I2C()
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c_bme,0x76)

def sensor_read(arg1):
    while True:
        global yy
        global xx

        global yaw
        global tilt_yaw
        global pitch
        global roll
        global yawDeg
        global pitchDeg
        global rollDeg

        global mag_x
        global mag_y
        global mag_z

        global accel_x
        global accel_y
        global accel_z

        global gyro_x
        global gyro_y
        global gyro_z

        #take 10 readings per second
        time.sleep(0.1)

        try:

            #Previous code for reading differnt IMU sensor
            #mag_x, mag_y, mag_z = mag_sensor.magnetic
            #accel_x, accel_y, accel_z = accel_sensor.acceleration

            #Read IMU
            mag_x, mag_y, mag_z = mag_accel_sensor.magnetometer
            accel_x, accel_y, accel_z = mag_accel_sensor.accelerometer
            gyro_x, gyro_y, gyro_z = gyro_sensor.gyroscope

            #mag calibration offsets for a SPECIFIC device - yours will be different!!
            #X: -46.20, Y:  -83.05, Z: -107.25
            #X: -38.30, Y:  -67.60, Z: -100.35

            mag_cal_x = -5.4499999
            mag_cal_y = -42.05
            mag_cal_z = -29.3

            mag_x = mag_x-mag_cal_x
            mag_y = mag_y-mag_cal_y
            mag_z = mag_z-mag_cal_z
            #mag_x = mag_cal_x
            #mag_y = mag_cal_y
            #mag_z = mag_cal_z
            yaw = math.atan2(mag_y,mag_x)

            #We use print statements for debugging - comment out to spee execution
            #print("mag: {},{},{}".format(mag_x, mag_y, mag_z))
            #print("accel: {}".format(mag_accel_sensor.accelerometer))

            pitch = math.atan2(accel_x,math.sqrt(accel_y**2+accel_z**2))
            roll = math.atan2(accel_y,math.sqrt(accel_x**2+accel_z**2))
            #yaw = 57
            #pitch = 20
            #roll = 4
            #if yaw < 0:
	        #yaw=yaw+2*math.pi

            #tilt compensation
            x_h = mag_x*math.cos(pitch) + mag_z*math.sin(pitch)
            y_h = mag_x*math.sin(roll)*math.sin(pitch)+mag_y*math.cos(roll)-mag_z*math.sin(roll)*math.cos(pitch)

            tilt_yaw = math.atan2(y_h,x_h)

            if tilt_yaw < 0:
                tilt_yaw=tilt_yaw+2*math.pi

        except:

            subprocess.call(['i2cdetect', '-y', '1'])
            continue

        #convert radians to degrees
        rollDeg = roll*57.2958
        pitchDeg = pitch*57.2958
        yawDeg = yaw*57.2958
        yawTilt = tilt_yaw*57.2958

        #print("yawDeg: {}".format(yawDeg))
        #print("yawTilt: {}".format(yawTilt))

        #line (radius) in compass is 23 pixels (compass is 46 pixels wide)
        #offset by pi to orient the display so top is north
        xx = round(math.cos(yaw-math.pi/2)*23)
        yy = round(math.sin(yaw-math.pi/2)*23)

        image = Image.new("1",(128,64))
        draw = ImageDraw.Draw(image)

        font = ImageFont.load_default()

        draw.ellipse((41,17,87,63),outline=255, fill=0)#left,top,right,bottom

        draw.line((xx+64,yy+41,64,41),fill=255) #[left (beginning), top] head, [right (end), bottom] tail - tail is always at center

        draw.text((0,-2),"Yaw: {}".format(round(yawDeg)),font=font,fill=255)
        draw.text((64,-2),"Pitch: {}".format(round(pitchDeg)),font=font,fill=255)
        draw.text((0,6),"Roll: {}".format(round(rollDeg)),font=font,fill=255)

        #Display information from Temp, Heading, Pressure sensor - you may wish to do something based on this information
        draw.text((0,14),"T: {:0.1f}C".format((bme280.temperature)),font=font,fill=255)
        draw.text((0,22),"H: {}%".format(round(bme280.humidity)),font=font,fill=255)
        draw.text((64,6),"P: {}mbar".format(round(bme280.pressure)),font=font,fill=255)

        #Update OLED screen to show new data and orientation (attitude) information
        oled.image(image)
        oled.show()

# Setup OLED screen - get parameters
width = oled.width
height = oled.height
image = Image.new('1', (width, height))

font = ImageFont.load_default()

#### CONFIGURE THE RPI TO INTERFACE WITH CONTROL BOARD ####

#Make it easier to remember which pins control which motors
GR1 = 21
GR2 = 26
BL1 = 13
OR1 = 20
BR1 = 27

#Do the same for the corresponding PWM signals
GR1_PWM = 3
GR2_PWM = 4
BL1_PWM = 2
OR1_PWM = 0
BR1_PWM = 1

######## OUR MOTOR PLACEMENTS ########
#       GREEN 1 :   PLACED ON RIGHT FOR FORWARD/BACKWARD
#       GREEN 2 :   PLACED ON FRONT AS UP/DOWN
#       BLUE 1  :   PLACED ON LEFT AS FORWARD/BACKWARD
#       ORANGE 1:   PLACED ON LEFT AS UP/DOWN
#       BROWN 1 :   PLACED ON RIGHT AS UP/DOWN

#Use the numbering scheme for the Broadcom chip, not the RPi pin numbers
GPIO.setmode(GPIO.BCM)

#Turn off warnings about pins being already configured
GPIO.setwarnings(False)

#Setup pins to control direction on the motor driver chip (MAXIM's MAX14870)
GPIO.setup(GR1,GPIO.OUT)#Green 1
GPIO.setup(GR2,GPIO.OUT)#Green 2
GPIO.setup(BL1,GPIO.OUT)#Blue 1
GPIO.setup(OR1,GPIO.OUT)#Orange 1
GPIO.setup(BR1,GPIO.OUT)#Brown 1

#status LEDs
GPIO.setup(6,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)

# Based on code released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

# Find the joystick device(s)
print('Available devices:')

#need to check to make sure a joystick has been connected before we proceed
#if not, we'll just wait here until someone connects a joystick

#this is usually called a flag and is used to check a condition
#when the desired condition is met, we change the value of the flag
joy_not_found = 1

while joy_not_found:
    for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('  /dev/input/%s' % (fn))
            joy_not_found = 0


# We'll store the states of the axes and buttons
axis_states = {}
button_states = {}

# These constants were borrowed and modified from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'rx',
    0x03 : 'x2',
    0x04 : 'y2',
    0x05 : 'ry',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
}

button_names = {
    0x130 : 'a',
    0x131 : 'b',
    0x133 : 'x',
    0x134 : 'y',
    0x136 : 'LB',
    0x137 : 'RB',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

####print(('%d axes found: %s' % (num_axes, ', '.join(axis_map))))
####print(('%d buttons found: %s' % (num_buttons, ', '.join(button_map))))

#Declare variables for use later
#These will be the values from the right joystick
intValx2 = 0
intValy2 = 0

#These will be the values from the left joystick
intValx = 0
intValy = 0

#These will be the values from the two triggers in the front of the joystick
intValrx = 0
intValry = 0

i=0
timer = [1,2,3,4,5,6,7,8,9,10]
direction = 0
for i in timer:
        evbuf = jsdev.read(8)
        if evbuf:
            tyme, value, type, number = struct.unpack('IhBB', evbuf)
            #print (str(struct.unpack('IhBB',evbuf)))
            #Use for debugging
            #if type & 0x80:
                #print("(initial)",end=""),

            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    #Use "PRINT" for debugging - comment out to speed program execution
                    if value:
                        print("%s pressed" % (button))
                    else:
                        print("%s released" % (button))
        i = i+1

def motor_loop(arg1):
    # Main event loop
    while True:
        global direction

        global intValrx
        global intValx
        global intValx2
        global intValry
        global intValy
        global intValy2

        start_time = time.time()

        if direction == 1:
            wantYaw = 0
            while yawDeg - wantYaw >=20:
                GPIO.output(BL1,GPIO.HIGH)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.LOW)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            while yawDeg - wantYaw <= -20:
                GPIO.output(BL1,GPIO.LOW)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.HIGH)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            pwm.channels[BL1_PWM].duty_cycle = 0
            pwm.channels[GR1_PWM].duty_cycle = 0
        elif direction == 2:
            wantYaw = 90
            while yawDeg - wantYaw >=20:
                GPIO.output(BL1,GPIO.HIGH)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.LOW)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            while yawDeg - wantYaw <= -20:
                GPIO.output(BL1,GPIO.LOW)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.HIGH)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            pwm.channels[BL1_PWM].duty_cycle = 0
            pwm.channels[GR1_PWM].duty_cycle = 0
        elif direction == 3:
            wantYaw = 180
            while yawDeg - wantYaw >=20:
                GPIO.output(BL1,GPIO.HIGH)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.LOW)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            while yawDeg - wantYaw <= -20:
                GPIO.output(BL1,GPIO.LOW)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.HIGH)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            pwm.channels[BL1_PWM].duty_cycle = 0
            pwm.channels[GR1_PWM].duty_cycle = 0
        elif direction == 4:
            wantYaw = -90
            while yawDeg - wantYaw >=20:
                GPIO.output(BL1,GPIO.HIGH)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.LOW)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            while yawDeg - wantYaw <= -20:
                GPIO.output(BL1,GPIO.LOW)
                pwm.channels[BL1_PWM].duty_cycle = abs(0xF000)
                GPIO.output(GR1,GPIO.HIGH)
                pwm.channels[GR1_PWM].duty_cycle = abs(0xF000)
            pwm.channels[BL1_PWM].duty_cycle = 0
            pwm.channels[GR1_PWM].duty_cycle = 0

        #evbuf = jsdev.read(8)
        if evbuf:
            tyme, value, type, number = struct.unpack('IhBB', evbuf)
            #print (str(struct.unpack('IhBB',evbuf)))
            #Use for debugging
            #if type & 0x80:
                #print("(initial)",end=""),

            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    #Use "PRINT" for debugging - comment out to speed program execution
                    if value:
                        print("%s pressed" % (button))
                    else:
                        print("%s released" % (button))

                    if button == "y":
                        direction = 1

                    elif button == "b":
                        direction = 2

                    elif button == "a":
                        direction= 3

                    elif button == "x":
                        direction = 4

                    elif button == "RB":
                        direction = 0

                    else:
                        pass

            if type & 0x02:
                axis = axis_map[number]
                #right joystick fwd/rev
                if axis=="y2":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValy2 = int(fvalue)*2+1
                    #Use "PRINT" for debugging, comment out to speed program
                    #print("%d" % (intValy2))

                #right joystick left/right
                if axis=="x2":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValx2 = int(fvalue)*2+1

                #left joystick fwd/rev
                if axis=="y":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValy = int(fvalue)*2+1
                #left joystick left/right
                if axis=="x":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValx = int(fvalue)*2+1

                #front right trigger fwd (vehicle ascend)
                if axis=="ry":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValry = int(fvalue)*2+1
                #front left trigger rev (vehicle descend)

                if axis=="rx":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValrx = int(fvalue)*2+1

                if axis=="hat0x":
                    #print("%s released" % (button))
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValy2 = int(fvalue)*2+1
                    print("%d" % (intValy2))

                #There's a nice tutorial for single joysick control at http://home.kendra.com/mauser/Joystick.html
                #if intValrx+intValry >= 0xFFFF:
                    #t = threading.Thread(target=sensor_read,args=(1,), daemon=True).start() #testing what putting threading in our for loop does

                if intValy2<-100:
                    #green motor
                    y2 = intValy2

                elif intValy2>100:
                    #green motor
                    y2 = intValy2

                else:
                    #green motor
                    y2 = 0

                if intValy>100:
                    #x motor
                    y = intValy

                elif intValy<-100:
                    #x motor
                    y = intValy

                else:
                    #x motor
                    y = 0

                if intValx>100:
                    #x motor
                    x = intValx

                elif intValx<-100:
                    #x motor
                    x = intValx

                else:
                    #x motor
                    x = 0

                if intValx2>100:
                    #x motor
                    x2 = intValx2

                elif intValx2<-100:
                    #x motor
                    x2 = intValx2

                else:
                    #x motor
                    x2 = 0

                LeftStick = x+y
                RightStick = x2+y2
                LeftStickOpp = y-x
                RightStickOpp = y2-x2

                if LeftStick > 0xFFFF:
                    LeftStick = 0xFFFF
                elif LeftStick < -0xFFFF:
                    LeftStick = -0xFFFF

                if RightStick > 0xFFFF:
                    RightStick = 0xFFFF
                elif RightStick < -0xFFFF:
                    RightStick = -0xFFFF

                if LeftStickOpp > 0xFFFF:
                    LeftStickOpp = 0xFFFF
                elif LeftStickOpp < -0xFFFF:
                    LeftStickOpp = -0xFFFF

                if RightStickOpp > 0xFFFF:
                    RightStickOpp = 0xFFFF
                elif RightStickOpp < -0xFFFF:
                    RightStickOpp = 0xFFFF

                #green2 motor left up/down
                if y2 > 0:
                    GPIO.output(GR2,GPIO.LOW)
                else:
                    GPIO.output(GR2,GPIO.HIGH)
                pwm.channels[GR2_PWM].duty_cycle = abs(y2)

                #blue motor left side
                if LeftStickOpp > 0:
                    GPIO.output(BL1,GPIO.HIGH)
                else:
                    GPIO.output(BL1,GPIO.LOW)
                pwm.channels[BL1_PWM].duty_cycle = abs(LeftStickOpp)

                #green1 motor right side
                if LeftStick > 0:
                    GPIO.output(GR1,GPIO.HIGH)
                else:
                    GPIO.output(GR1,GPIO.LOW)
                pwm.channels[GR1_PWM].duty_cycle = abs(LeftStick)

                #orange motor left side up/down
                if RightStickOpp > 0:
                    GPIO.output(OR1,GPIO.HIGH)
                else:
                    GPIO.output(OR1,GPIO.LOW)
                pwm.channels[OR1_PWM].duty_cycle = abs(RightStickOpp)

                #brown motor right side up/down
                if RightStick > 0:
                    GPIO.output(BR1,GPIO.HIGH)
                else:
                    GPIO.output(BR1,GPIO.LOW)
                pwm.channels[BR1_PWM].duty_cycle = abs(RightStick)

                # if intValrx > 0:
                #     GPIO.output(BR1,GPIO.HIGH)
                #     GPIO.output(OR1,GPIO.HIGH)
                #     GPIO.output(GR2,GPIO.HIGH)
                # else:
                #     GPIO.output(BR1,GPIO.LOW)
                #     GPIO.output(OR1,GPIO.LOW)
                #     GPIO.output(GR2,GPIO.LOW)
                # pwm.channels[BR1_PWM].duty_cycle = abs(intValrx)
                # pwm.channels[BR1_PWM].duty_cycle = abs(intValrx)
                # pwm.channels[BR1_PWM].duty_cycle = abs(intValrx)

        end_time = time.time()
        print (direction)
        print("loop time:",end_time - start_time)

def control_loop(arg1):
    global evbuf
    while True:
        evbuf = jsdev.read(8)

threading.Thread(target=motor_loop,args=(1,), daemon=True).start()
threading.Thread(target=sensor_read,args=(1,), daemon=True).start()
#threading.Thread(target=control_loop,args=(1,), daemon=True).start()
while True:
    evbuf = jsdev.read(8)