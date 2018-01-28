"""
Some magic

NB: [setTarget(target)] takes quarter microseconds, NOT MICROSECONDS.
"""
import maestro
import time
import sys

import serial



if __name__ == "__main__":
	servo = maestro.Controller(ttyStr="/dev/ttyACM0")
	for i in range(24):
		servo.setTarget(0, 4 * 500)
		time.sleep(1)
		servo.setTarget(0, 4000)
		time.sleep(1)
	servo.close
	print("end")



