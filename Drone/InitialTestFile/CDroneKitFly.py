from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

#Need torun on simulation
import dronekit_sitl


class CDroneKitFly:
    def __init__(self):
        # constant
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        self.DEFAULT_TAKEOFF_THRUST = 0.7
        self.SMOOTH_TAKEOFF_THRUST = 0.6
        #check for simulation
        self.simulation = True
        self.sitl = ""
        #store connection string
        self.connection_string = ""
        #keep vehicle object
        self.Vehicle = ''



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

    #Arm Vehicle
    def ArmingVehicle(self,mode = "GUIDED"):
        print("Pre Arm Checking, FIle : CDroneKitFly , Method : ArmingVehicle")
        while not self.Vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        print("Arming Motor ,FIle : CDroneKitFly , Method : ArmingVehicle ")
        #GUIDED_NOGPS, AUTO, GUIDED
        self.Vehicle.mode = VehicleMode(mode)
        self.Vehicle.armed = True

        while not self.Vehicle.armed:
            print(" Waiting for arming...FIle : CDroneKitFly , Method : ArmingVehicle ")
            time.sleep(1)

        print(" Vehicle Armed , FIle : CDroneKitFly , Method : ArmingVehicle ")

    def TakeoffVehicle(self):
        print("Taking OFF, FIle : CDroneKitFly , Method : TakeoffVehicle ")
        thrust = self.DEFAULT_TAKEOFF_THRUST
        self.FlyDrone(thrust = thrust)

    #fly drone
    def FlyDrone(self,roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
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
        self.Vehicle.send_mavlink(msg)

        if duration != 0:
            # Divide the duration into the frational and integer parts
            modf = math.modf(duration)

            # Sleep for the fractional part
            time.sleep(modf[0])

            # Send command to vehicle on 1 Hz cycle
            for x in range(0, int(modf[1])):
                time.sleep(1)
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

        return [w, x, y, z]

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
        self.Vehicle.close()
        if self.sitl is not None:
            self.sitl.stop()

        print("Drone Closed, File CDroneKitFly , Method : CloseVehicle")


