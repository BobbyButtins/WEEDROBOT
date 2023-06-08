# WEEDROBOT
**Cal Poly SLO, Mechanical Engineering Senior Project**

September 2022 - June 2023

The project is a large turtle-shaped weed cutting robot. Weeds are cut using a scooper system containing two C-shaped extrusions that can interface together to form a basket-like container. One half has a secured blade that shears the weeds. These weeds are then transported to an opening near the top of the shell, where it will pass through a shredder and the remains will be kept in a storage container. All actions are to be controlled using a PS4 DualShock4 controller via Bluetooth connection. The script was written in Python 3.7 on an NVIDIA Jetson Nano Developer Kit. PWM communication was done with the help of the Adafruit CircuitPython PCA9685 documentation and its related libraries: https://github.com/adafruit/Adafruit_CircuitPython_PCA9685. Controller support and input communication was implemented with reference to the Approxeng.Input library: https://github.com/ApproxEng/approxeng.input.

Weed identification has been attempted using a model from the Jetson Inference Object Detection library: https://github.com/dusty-nv/jetson-inference. With limited success, weeds can be identified with a connected video device (such as a USB Webcam). The model still needs to be trained longer and with more examples for weed identification to be fully functional.

*FUNCTIONS OF THE ROBOT*:

-weed cutting (opening/closing scooper)

-weed transporting (raising/lowering scooper arms)

-full forward and reverse drive with turning

-storage door movement (opening/closing)

[FUTURE IMPLEMENTATION]

-shredding (on/off)

-weed identification

-autonomous movement
