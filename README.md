# Hand Gesture Controlled Drone
As the title suggests, we are trying to build a hand-gesture controlled drone, where you don't need to have the transmitter in your hand to fly it.

We have divided our project into four modules:
1. Controlling the flight and movement of the drone.
2. Building Gesture recognition model.
3. Flight simulation.
4. Real world implementation.

## Getting Started

### 1. Controlling the flight and movement of the drone
* Drone: We have used Intel Aero Ready To Fly (RTF) drone which comes with intel aero compute board running a quadcore Intel atom processor, and a Intel RealSense R200 camera with Stereo Vision and Depth Sense.
  * We need to install ubuntu on drone’s micro processor and calibrate its sensor for the flight.
  * Establish a local connection between the drone’s WiFi and our personal laptop.
  * Using mavlink to communicate between the drone and the laptop.
  * Testing and calibrating the drone programmatically using a wide range of values for the yaw, pitch & roll of the drone as well as the       speed of the rotors until we got a suitable set of values with which the drone could hover in a stable manner.
  * For detail explaination please refer: [github.com/BhaskarTrivedi/Intel-Aero-Drone](https://github.com/BhaskarTrivedi/Intel-Aero-Drone)

### 2. Building Gesture recognition model
* Since our drone’s camera has resolution limitations, therefore we decided to implement arm-length gestures. (i.e. it could be detected from far away)
* We needed an algorithm that can both classify as well as localize the gestures, in real-time.
* Therefore, we went for Yolo v3 algorithm.
* Since, the project is in early phase we decided to train our model on only two gestures.
* We created a labelled dataset of 3000 images, 1500 each for one gesture.

### 3. Flight simulation
![Output sample](https://github.com/sooryamsharma/hand-gesture-controlled-drone/blob/master/misc/simulation.gif)
* To test the gesture recognition, we used the software V-REP to simulate a drone.
* The simulation was controlled using RemoteApi offered by the V-REP software through a python script.
* The python script contains method to move the drone to either left or right.
* Once the gesture is detected, the method from the drone control script is called to move the drone in the detected direction.

### 4. Real world implementation
* Since the drone crashed during one of the flights, we were unable to test the gesture control on the drone.
* Our future goal will be to calibrate a swarm of drones to respond to gestures.
* Some of the application of this project would be in real world situations that require hands free drone control.

## References
* Intel Aero Wiki: [github.com/intel-aero/meta-intel-aero/wiki](https://github.com/intel-aero/meta-intel-aero/wiki)
* Drone Api tutorial: [ardupilot.org/dev/docs/droneapi-tutorial.html](http://ardupilot.org/dev/docs/droneapi-tutorial.html)
* Darknet (Windows): [github.com/AlexeyAB/darknet](https://github.com/AlexeyAB/darknet)
* Bounding Box labelling tool: [github.com/NorrisWu/BBox-Label-Tool-master](https://github.com/NorrisWu/BBox-Label-Tool-master)
* Object detection using Yolo: [learnopencv.com/deep-learning-basedobject-detection-using-yolov3-with-opencv-python-c/](https://www.learnopencv.com/deep-learning-basedobject-detection-using-yolov3-with-opencv-python-c/)
* Remote Api Functions Python: [coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)
