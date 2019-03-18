import logging
logging.basicConfig(level=logging.INFO)

import time
import numpy as np
import cv2
import pyrealsense as pyrs
import CDroneKitFly as dkf
import CGestureRecongnition as gr
from pyrealsense.constants import rs_option



class CDroneCamera:
    def __init__(self):
        self.MovementCommand = "Backward"
        self.mode = "GUIDED_NOGPS"
        # create drone object
        self.DroneObj = dkf.CDroneKitFly()
        self.gestureObject = gr.CGestureRecongnition()

    def InitializeDroneObj(self):
        if self.mode == "GUIDED_NOGPS":

            # set vehicle type true for simulation
            self.DroneObj.SetVehicleType(False)
            # Connect to vehicle
            self.DroneObj.ConnectingVehicle()
            # Arm Vehicle Guided_NoGPS
            self.DroneObj.ArmingVehicle(self.mode)


            # DroneObj.FlyDrone(duration=2)
            '''
            print("Move forward")
            DroneObj.ChangeMode(mode=mode,duration=1) #default is LOITER to keep it hover
            DroneObj.FlyDrone(pitch_angle1=-3, thrust=0.5, duration=3)
            DroneObj.ChangeMode(duration=2) #default is LOITER to keep it hover

            print("Move backward")
            DroneObj.ChangeMode(mode=mode,duration=1) #default is LOITER to keep it hover
            DroneObj.FlyDrone(pitch_angle1=15, thrust=0.5, duration=3)
            '''


    def GetCameraFeed(self):
        with pyrs.Service() as serv:
            with serv.Device() as dev:

                dev.apply_ivcam_preset(0)

                try:  # set custom gain/exposure values to obtain good depth image
                    custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                                      (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
                    dev.set_device_options(*zip(*custom_options))
                except pyrs.RealsenseError:
                    pass  # options are not available on all devices

                cnt = 0
                last = time.time()
                smoothing = 0.9
                fps_smooth = 30

                self.DroneObj.TakeoffVehicle()
                print("Hold position for 3 seconds")
                self.DroneObj.ChangeMode(duration=5)  # default is LOITER to keep it hover
                move = 1
                maxloop = 1
                while True:
                    # Take off Vehicle
                    cnt += 1
                    if (cnt % 10) == 0:
                        now = time.time()
                        dt = now - last
                        fps = 10 / dt
                        fps_smooth = (fps_smooth * smoothing) + (fps * (1.0 - smoothing))
                        last = now

                    dev.wait_for_frames()
                    c = dev.color
                    c = cv2.cvtColor(c, cv2.COLOR_RGB2BGR)
                    d = dev.depth * dev.depth_scale * 1000
                    d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)

                    movement = self.gestureObject.getMovement(c,move)

                    if(movement == "Backward"):
                        print("Move backward")
                        self.DroneObj.ChangeMode(mode=self.mode, duration=1)  # default is LOITER to keep it hover
                        self.DroneObj.FlyDrone(pitch_angle1=15, thrust=0.5, duration=3)
                        move += 1

                    if(movement == "Forward"):
                        print("Move forward")
                        self.DroneObj.ChangeMode(mode=self.mode, duration=1)  # default is LOITER to keep it hover
                        self.DroneObj.FlyDrone(pitch_angle1=-3, thrust=0.5, duration=3)
                        
                        move += 1

                    if(movement == "Left"):
                        print("Move Left")
                        move += 1

                    if(movement == "Right"):
                        print("Move Right")
                        move += 1




                    cd = np.concatenate((c, d), axis=1)

                    cv2.putText(cd, str(fps_smooth)[:4], (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0))

                    cv2.imshow('', cd)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                    if move == 5:
                        break

                    maxloop += 1

                    if maxloop == 100:
                        break

                self.DroneObj.ChangeMode(duration=2)  # default is LOITER to keep it hover
                self.DroneObj.ChangeVehicletoLanding()
                self.DroneObj.CloseVehicle()
