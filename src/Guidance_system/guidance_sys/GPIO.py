import RPi.GPIO as GPIO
from time import sleep     # this lets us have a time delay (see line 12)

# for GPIO numbering, choose BCM
GPIO.setmode(GPIO.BCM)
# or, for pin numbering, choose BOARD
#GPIO.setmode(GPIO.BOARD)
GPIO.setup(25, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)    # set GPIO 25 as input
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, GPIO.HIGH)
try:
    while True:            # this will carry on until you hit CTRL+C
        #if GPIO.input(25): # if port 25 == 1
        print(GPIO.input(25))
        sleep(0.1)         # wait 0.1 seconds

except KeyboardInterrupt:
    GPIO.cleanup()         # clean up after yourself
#https://learn.sparkfun.com/tutorials/raspberry-gpio/python-rpigpio-api