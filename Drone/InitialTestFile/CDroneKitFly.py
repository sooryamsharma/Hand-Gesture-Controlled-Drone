from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

#Need torun on simulation
#import dronekit_sitl


class CDroneKitFly:
    def __init__(self):
        # constant
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        self.DEFAULT_TAKEOFF_THRUST = 0.6
        self.SMOOTH_TAKEOFF_THRUST = 0.6
        #check for simulation
        self.simulation = False
        self.sitl = None
        #store connection string
        self.connection_string = ""
        #keep vehicle object
        self.Vehicle = ''
        self.current_altitude = ''
        self.last_altitude_cache = None
        self.target_altitude = 3
        self.base_roll = 3.11   #best roll n pitc combination roll = 3.11 pitch =-2.8
        #self.base_roll = -2.11
        #self.base_pitch = 1.18
        self.base_pitch = -2.8



    # Get vehicle type according to Actual drone or simulation
    def SetVehicleType(self,vehicleType):
        self.simulation = vehicleType
        if(self.simulation == True):
            self.sitl = dronekit_sitl.start_default()
            self.connection_string = self.sitl.connection_string()
            print("Connecting to simulation, FIle : CDroneKitFly , Method : GetVehicleType")
        else:
            self.connection_string = 'tcp:127.0.0.1:5760'
            print("Connecting to physical drone,FIle : CDroneKitFly , Method : GetVehicleType")
        print('Connecting to vehicle on: %s , FIle : CDroneKitFly , Method : GetVehicleType' % self.connection_string)

    # Connect to Vehicle
    def ConnectingVehicle(self):
        #need to check if wait_ready should be true or False, Experiment with both before deciding keep True for now
        self.Vehicle = connect(self.connection_string, wait_ready=True)
        #self.Vehicle.set_mavlink_callback(self.mavrx_debug_handler)
        self.Vehicle.add_attribute_listener('attitude', self.Attitude_Callback)

    #Arm Vehicle
    def ArmingVehicle(self,mode = "GUIDED"):
        print("Pre Arm Checking, FIle : CDroneKitFly , Method : ArmingVehicle")
        #while not self.Vehicle.is_armable:
            #print(" Waiting for vehicle to initialise...")
            #time.sleep(1)
        print("Arming Motor ,FIle : CDroneKitFly , Method : ArmingVehicle ")
        print(mode)
        #GUIDED_NOGPS, AUTO, GUIDED
        self.Vehicle.mode = VehicleMode(mode)
        self.Vehicle.armed = True

        while not self.Vehicle.armed:
            print(" Waiting for arming...FIle : CDroneKitFly , Method : ArmingVehicle ")
            time.sleep(1)

        print(" Vehicle Armed , FIle : CDroneKitFly , Method : ArmingVehicle ")

    def TakeoffVehicle(self,dura=3):
        print("Taking OFF, FIle : CDroneKitFly , Method : TakeoffVehicle ")
        thrust = self.DEFAULT_TAKEOFF_THRUST
    #self.current_altitude = self.Vehicle.global_relative_frame.alt;
	#print(self.current_altitude)
        self.FlyDrone(thrust = thrust,duration=dura)
    #self.current_altitude = self.Vehicle.global_relative_frame.alt;
	#print(self.current_altitude)

    #fly drone
    #according to Qgroung Controller Roll = -2.0, pitch = 1
    def FlyDrone(self,roll_angle1 = 0, pitch_angle1 = 0, yaw_rate = 0.0, thrust = 0.505, duration = 0):
        """
            Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
            with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
            velocity persists until it is canceled. The code below should work on either version
            (sending the message multiple times does not cause problems).
        """

        """
        The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
        so you must use quaternion to control the pitch and roll for those vehicles.
        """
        roll_angle = self.base_roll + roll_angle1
        pitch_angle = self.base_pitch + pitch_angle1

        msg = self.Vehicle.message_factory.set_attitude_target_encode(0,
                                                                      0,
                                                                        # Target system
                                                                      0,
                                                                        # Target component
                                                                      0b00000000,
                                                                        # Type mask: bit 1 is LSB
                                                                      self.to_quaternion(roll_angle, pitch_angle),
                                                                        # Quaternion
                                                                      0,
                                                                        # Body roll rate in radian
                                                                      0,
                                                                        # Body pitch rate in radian
                                                                      math.radians(yaw_rate),
                                                                        # Body yaw rate in radian
                                                                      thrust)
                                                                        # Thrust
        result = self.Vehicle.send_mavlink(msg)
        print("Mv link msg resulti ",result)

        if duration != 0:
            # Divide the duration into the frational and integer parts
            modf = math.modf(duration)

            # Sleep for the fractional part
            time.sleep(modf[0])

            # Send command to vehicle on 1 Hz cycle
            for x in range(0, int(modf[1])):
                time.sleep(1)
                print("sending msg %d",x)
                self.Vehicle.send_mavlink(msg)


    def to_quaternion(self,roll=0.0, pitch=0.0, yaw=0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        print("W valuw %d",w)
        print("x valuw %d",x)
        print("y valuw %d",y)
        print("z valuw %d",z)
	#default X value in missin planner = -0.0271462
	#default Y value in mission planner = 0.02586136
        return [w, x, y, z]

    def vehicleSimpleTafeoff(self,t_altitude=3):
        self.target_altitude = t_altitude
        print("Taking off to target altitude< ",self.target_altitude)
        self.Vehicle.simple_takeoff(self.target_altitude)
	

    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        # Set up velocity mappings
        # velocity_x > 0 => fly North
        # velocity_x < 0 => fly South
        # velocity_y > 0 => fly East
        # velocity_y < 0 => fly West
        # velocity_z < 0 => ascend
        # velocity_z > 0 => descend
        msg = self.Vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.Vehicle.send_mavlink(msg)
            print("Send msg to play up, File CDroneKitFly, Method : send_ned_velocity")
            time.sleep(1)

    def ChangeVehicletoLanding(self):
        print("Changing Drone Mode to Land, File CDroneKitFly , Method : ChangeVehicletoLanding")
        self.Vehicle.mode = VehicleMode("LAND")
        time.sleep(1)

    def CloseVehicle(self):
        print("Close Drone Object, File CDroneKitFly , Method : CloseVehicle")
        self.FlyDrone()
        self.Vehicle.close()
        if self.sitl is not None:
            self.sitl.stop()

        print("Drone Closed, File CDroneKitFly , Method : CloseVehicle")

    def ChangeMode(self,mode = "LOITER",duration=3):
        self.Vehicle.mode = VehicleMode(mode)
        if duration != 0:
            # Divide the duration into the frational and integer parts
            modf = math.modf(duration)

            # Sleep for the fractional part
            time.sleep(modf[0])

            # Send command to vehicle on 1 Hz cycle
            for x in range(0, int(modf[1])):
                time.sleep(1)
		print("Mode maintain for ",x)

    def Attitude_Callback(self, Vehicle,attribute_name,value):
        if value != self.last_altitude_cache:
            #print(" CALLBACK: Attitude changed to", value)
            #print(" CALLBACK: Attitude changed to", attribute_name)
            self.last_altitude_cache = value

    def mavrx_debug_handler(self,message):
        print ("Received %s",message)

    '''	
	def attitude_callback(self.VehicleMode):
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `self` - the associated vehicle object (used if a callback is different for multiple vehicles)
    # `value` is the updated attribute value.
    #last_attitude_cache
    # Only publish when value changes
	print('attitude')
    if value!=self.last_attitude_cache:
        print(" CALLBACK: Attitude changed to", value)
        self.last_attitude_cache=value
        
    '''
		



