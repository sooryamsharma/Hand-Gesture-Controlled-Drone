#!/usr/bin/python
from dronekit import connect, VehicleMode, LocationGlobalRelative
import pymavlink.mavutil as mavutil
import time


def Connect_Arm():
	vehicle = connect('tcp:127.0.0.1:5760', wait_ready=False)
	print "Arming motors:"
	vehicle.mode    = VehicleMode("AUTO")
	vehicle.armed   = True
	while not vehicle.armed:
        	print "  Waiting for arming to be finished"
		#vehicle.armed   = True
        	time.sleep(1)
	#print "Keeping motors armed for 5s"
	#time.sleep(5)
	#print "Disarming"
	#vehicle.armed   = False
	return vehicle


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration,vehicle):
	"""
	Move vehicle in direction based on specified velocity vectors.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

	# send command to vehicle on 1 Hz cycle
    	for x in range(0,duration):
        	vehicle.send_mavlink(msg)
		print("Send msg to play up");
        	time.sleep(1)

vehicle=Connect_Arm()
SOUTH=5
UP=1  #NOTE: up is negative!
DURATION=30

#Fly south and up.
send_ned_velocity(SOUTH,0,UP,DURATION,vehicle)
