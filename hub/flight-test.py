import socket
import threading
#import struct
import time

from ultralytics import YOLO

MOTORS_DISARMED = 0
MOTORS_ARMED = 255

ARM_CLOSED = 0
ARM_OPEN = 255

THROTTLE_MAGNITUDE = 10
THROTTLE_NEUTRAL = 127.5 # adjust to get hover
DOWN = THROTTLE_NEUTRAL - THROTTLE_MAGNITUDE
UP = THROTTLE_NEUTRAL + THROTTLE_MAGNITUDE

MOTION_MAGNITUDE = 10
MOTION_NEUTRAL = 127.5
LEFT, FORWARD = MOTION_NEUTRAL - MOTION_MAGNITUDE
RIGHT, BACKWARD = MOTION_NEUTRAL + MOTION_MAGNITUDE

motor_state = MOTORS_DISARMED
arm_state = ARM_CLOSED
throttle = THROTTLE_NEUTRAL
side_motion = MOTION_NEUTRAL
forward_motion = MOTION_NEUTRAL

#altitude = 0.0

def drone_communication():
	global motor_state, arm_state, throttle, side_motion, forward_motion, altitude
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.connect(('192.168.4.1', 2000))
		while True:
			s.sendall(bytes([motor_state, arm_state, int(throttle), int(side_motion), int(forward_motion)]))
			#altitude = struct.unpack('<f', s.recv(4))[0]

threading.Thread(target=drone_communication).start()

motor_state = MOTORS_ARMED
time.sleep(1)
motor_state = MOTORS_DISARMED