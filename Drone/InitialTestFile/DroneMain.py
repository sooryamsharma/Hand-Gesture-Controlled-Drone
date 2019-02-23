import CDroneKitFly as dkf

#create drone object
DroneObj = dkf.CDroneKitFly()
#set vehicle type true for simulation
DroneObj.SetVehicleType(True)
#Connect to vehicle
DroneObj.ConnectingVehicle()
#Arm Vehicle
DroneObj.ArmingVehicle("GUIDED_NOGPS")
#Take off Vihicle
DroneObj.TakeoffVehicle()
print("Hold position for 3 seconds")
DroneObj.FlyDrone(duration = 3)
print("Move forward")
DroneObj.FlyDrone(pitch_angle = 1, thrust = 0.5, duration = 3.21)
print("Move backward")
DroneObj.FlyDrone(pitch_angle = -1, thrust = 0.5, duration = 3)

DroneObj.CloseVehicle()

# Using ned velocity and north south direction
#create drone object
DroneObj = dkf.CDroneKitFly()
#set vehicle type true for simulation
DroneObj.SetVehicleType(True)
#Connect to vehicle
DroneObj.ConnectingVehicle()
#Arm Vehicle
DroneObj.ArmingVehicle()
SOUTH=-2
UP=0.5   #NOTE: up is negative!
DURATION=5
#Fly south and up.
DroneObj.send_ned_velocity(SOUTH,0,UP,DURATION)

