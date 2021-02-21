#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import time
import keyboard 


def switch_pressed_callback(channel):
	global LED_on
	LED_on ^= True
	if ~LED_on:
		GPIO.output(LED_pin, GPIO.HIGH)


if __name__ == '__main__':
	LED_pin = 12
	SW_pin = 38
	LED_state = GPIO.HIGH
	LED_on = False

	print('Name: Amritpal Sidhu')
	print('SID: xxxxx3518')
	
	GPIO.setmode(GPIO.BOARD)
	
	GPIO.setwarnings(False)
	
	# pin 12 on the 40 pin header will be used for the LED (active low)
	GPIO.setup(LED_pin, GPIO.OUT, initial=GPIO.HIGH)

	# pin 38 will be used for the switch (active high)
	GPIO.setup(SW_pin, GPIO.IN)
	
	GPIO.add_event_detect(SW_pin, GPIO.RISING, callback=switch_pressed_callback)

	while True:
		# exit loop when q is pressed
		if keyboard.is_pressed('q'):
			print()
			break
		# toggle led at 2Hz if switch pressed
		if LED_on:
			LED_state ^= GPIO.HIGH
			GPIO.output(LED_pin, LED_state)
			time.sleep(0.5)

	print('Calling GPIO.cleanup()')
	GPIO.cleanup()
	print('Exiting Program')
