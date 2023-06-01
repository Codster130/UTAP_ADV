import RPi.GPIO as GPIO
add = 0
try:
    while True:
        add = add + 1
except (KeyboardInterrupt,SystemExit):
    GPIO.cleanup()