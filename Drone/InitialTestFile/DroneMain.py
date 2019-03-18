import CDroneKitFly as dkf

mode = "GUIDED_NOGPS"
#mode = "GUIDED"


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
    DroneObj.TakeoffVehicle()
    print("Hold position for 3 seconds")
    DroneObj.ChangeMode(duration=30) #default is LOITER to keep it hover
    #DroneObj.FlyDrone(duration=2)
    '''
    print("Move forward")
    DroneObj.ChangeMode(mode=mode,duration=1) #default is LOITER to keep it hover
    DroneObj.FlyDrone(pitch_angle1=-3, thrust=0.5, duration=3)
    DroneObj.ChangeMode(duration=2) #default is LOITER to keep it hover

    print("Move backward")
    DroneObj.ChangeMode(mode=mode,duration=1) #default is LOITER to keep it hover
    DroneObj.FlyDrone(pitch_angle1=15, thrust=0.5, duration=3)
    '''
    DroneObj.ChangeMode(duration=2) #default is LOITER to keep it hover
    DroneObj.ChangeVehicletoLanding()
    DroneObj.CloseVehicle()
else:
    # Using ned velocity and north south direction
    # create drone object
     DroneObj = dkf.CDroneKitFly()
    # set vehicle type true for simulation
     DroneObj.SetVehicleType(True)
    # Connect to vehicle
     DroneObj.ConnectingVehicle()
    # Arm Vehicle
     DroneObj.ArmingVehicle()
     SOUTH=0
     UP=-0.1   #NOTE: up is negative!
     DURATION=2
     #Fly  up.
     DroneObj.send_ned_velocity(SOUTH,0,UP,DURATION)
     # Fly  hover.
     UP = 0
     DroneObj.send_ned_velocity(SOUTH, 0, UP, DURATION)
     DroneObj.ChangeVehicletoLanding()
     DroneObj.CloseVehicle()






