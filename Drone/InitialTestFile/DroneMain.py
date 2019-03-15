import CDroneKitFly as dkf

#mode = "GUIDED_NOGPS"
mode = "GUIDED"


if mode == "GUIDED_NOGPS":
    # create drone object
    DroneObj = dkf.CDroneKitFly()
    # set vehicle type true for simulation
    DroneObj.SetVehicleType(False)
    # Connect to vehicle
    DroneObj.ConnectingVehicle()
    # Arm Vehicle Guided_NoGPS
    DroneObj.ArmingVehicle(mode)
    # Take off Vihicle
    DroneObj.TakeoffVehicle(dura=3)
    print("Hold position for 3 seconds")
    DroneObj.FlyDrone(duration=10)
    print("Move forward")
    #DroneObj.FlyDrone(pitch_angle=1, thrust=0.5, duration=3.21)
    print("Move backward")
    #DroneObj.FlyDrone(pitch_angle=-1, thrust=0.5, duration=3)
    DroneObj.ChangeVehicletoLanding()
    DroneObj.CloseVehicle()
else:
    # Using ned velocity and north south direction
    # create drone object
     DroneObj = dkf.CDroneKitFly()
    # set vehicle type true for simulation
     DroneObj.SetVehicleType(False)
    # Connect to vehicle
     DroneObj.ConnectingVehicle()
    # Arm Vehicle
     DroneObj.ArmingVehicle()
     SOUTH=0.5
     UP=-0.5   #NOTE: up is negative!
     DURATION=12
     #Fly  up.
     print('takeoff')
     DroneObj.vehicleSimpleTafeoff(3)
     # Fly  hover.
     UP = 0
     print('Hover')
     DroneObj.send_ned_velocity(SOUTH, 0, UP, DURATION)
     DroneObj.ChangeVehicletoLanding()
     DroneObj.CloseVehicle()






