# droneController
# drone description:
  type: quadcopter
  configuration: x-con
  sensor info:
  - camera
  - distance/lidar sensor
  - gyroscope
  - accelerometer
  - pressure sensor
// Drone controller for specific drone system
# Things to do
1 - program transmitter to send control data (use dummy data while control still in dev proc)
2 - program receiver to receive control data (---^---)
3 - program controller sensor input (do all the math and have it ready to send proc sensory info)
4 - program motor conrollers (create lift, thrust, yaw, and roll functionalities)
5 - calibrate controller to adjust errors based on sensor inputs
6 - ...more info after progress on this
