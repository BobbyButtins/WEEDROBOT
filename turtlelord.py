import time
import busio
import board
import adafruit_pca9685
from adafruit_servokit import ServoKit
from approxeng.input.selectbinder import ControllerResource
from approxeng.input.switch import SwitchJoyConLeft
from approxeng.input.dualshock4 import DualShock4
from approxeng.input.controllers import find_matching_controllers, ControllerRequirement

# Set channels to the number of servo channels on the kit
# For the PCA9685 driver, we have 16 channels
kit = ServoKit(channels=16)
# Don't need to setup bus if SCL and SDA are connected to pins 3/5
# which are for bus 1 (default)

# If using bus 0 (pins 26/27), will need to create bus object like this:
# i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
# and include it in the parameters for ServoKit: i2c=i2c_bus0

# Setup bus for DC motor control through PCA9685 driver
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
# Set frequency for board
pca.frequency = 60
# Specify channels for each DC motor
# Channel 4 for left wheel, channel 5 for right wheel, channel 6 for shredder
# More channels to be added later for LED control
l_wheel = pca.channels[4]
l_wheel_r = pca.channels[5]
r_wheel = pca.channels[6]
r_wheel_r = pca.channels[7]
shred = pca.channels[8]
battery_LED = pca.channels[9]

def move_left(angle):
    '''!
    @brief Controls the rotation of the left scooper servo.
    @param angle The desired angle for the left servo.
    '''
    kit.servo[0].angle = angle

def move_right(angle):
    '''!
    @brief Controls the rotation of the right scooper servo.
    @param angle The desired angle for the right servo.
    '''
    kit.servo[1].angle = angle

def move_transport(angle):
    '''!
    @brief Controls the rotation of the lifting servo.
    @param angle The desired angle of the lifting servo.
    '''
    kit.servo[2].angle = angle

def move_shreddoor(angle):
    '''!
    @brief Controls the rotation of the shredder door.
    @param angle The desired angle of the door servo.
    '''
    kit.servo[3].angle = angle


if __name__ == "__main__":
    try:
        print("Initializing motor default positions...")
        # Change PWM range for all the servos to get the full range of motion
        kit.servo[0].set_pulse_width_range(1000, 3000)
        kit.servo[1].set_pulse_width_range(1000, 3000)
        kit.servo[2].set_pulse_width_range(1000, 3000)
        kit.servo[3].set_pulse_width_range(1000, 3000)

        # Set starting positions for the scooper servos
        move_left(50)
        move_right(50)
        scoop = "closed"

        # Set range of motion for DOCYKE servo to 360 degrees (default is 180 degrees)
        kit.servo[2].actuation_range = 360
        move_transport(0)
        lift = "down"

        # Set starting position for shredder door servo
        move_shreddoor(0)
        shreddoor = "closed"
        shred_weed = "off"
        print("Initialized")
        n = 0

        while True:
            with ControllerResource() as joystick:
                print(type(joystick).__name__)
                print("Waiting for input from controller")
                while joystick.connected:   
                    # test for button presses
                    presses = joystick.check_presses()
                    # detect joystick input
                    lx = joystick['lx']
                    ly = joystick['ly']

                    # detect controller input

                    # ROBOT MOVEMENT CONTROL
                    if ly != 0:
                        accel = 1
                        # FULL FORWARD DRIVE
                        while ly > 0.1 and lx == 0:
                            presses = joystick.check_presses()
                            if l_wheel.duty_cycle == 0x0000:
                                # if wheels are idle
                                while accel < 5:
                                    # accelerate wheels over span of 2.5 seconds
                                    l_wheel.duty_cycle = accel*0x2500
                                    l_wheel_r.duty_cycle = 0x0000
                                    r_wheel.duty_cycle = l_wheel.duty_cycle
                                    r_wheel_r.duty_cycle = 0x0000
                                    time.sleep(0.5)
                                    accel += 1
                                    ly = joystick['ly']
                                    print('Forward Drive')
                                    # emergency stop
                                    if presses['home']:
                                        raise KeyboardInterrupt
                            else:
                                # if wheels are running, no change
                                ly = joystick['ly']
                                print('Continue FDrive')
                                if presses['home']:
                                    raise KeyboardInterrupt
                                pass

                        # FULL REVERSE DRIVE
                        while ly < -0.1 and lx == 0:
                            presses = joystick.check_presses()
                            if l_wheel_r.duty_cycle == 0x0000:
                                # accelerate wheels over span of 2.5 seconds
                                while accel < 5:
                                    l_wheel_r.duty_cycle = accel*0x2500
                                    l_wheel.duty_cycle = 0x0000
                                    r_wheel_r.duty_cycle = l_wheel.duty_cycle
                                    r_wheel.duty_cycle = 0x0000
                                    time.sleep(0.5)
                                    accel += 1
                                    ly = joystick['ly']
                                    print('Reverse Drive')
                                    # emergency stop
                                    if presses['home']:
                                        raise KeyboardInterrupt
                            else:
                                # if wheels are running, no change
                                ly = joystick['ly']
                                print('Continue RDrive')
                                if presses['home']:
                                    raise KeyboardInterrupt
                                pass

                        # TURN LEFT FORWARD DRIVE
                        while ly > 0.1 and lx < -0.1:
                            presses = joystick.check_presses()
                            if l_wheel.duty_cycle == 0x0000:
                                # accelerate wheels over span of 2.5 seconds
                                while accel < 5:
                                    r_wheel.duty_cycle = accel*0x2500
                                    r_wheel_r.duty_cycle = 0x0000
                                    l_wheel.duty_cycle = l_wheel.duty_cycle/2
                                    l_wheel_r.duty_cycle = 0x0000
                                    time.sleep(0.5)
                                    accel += 1
                                    ly = joystick['ly']
                                    print('Turn Left')
                                    # emergency stop
                                    if presses['home']:
                                        raise KeyboardInterrupt
                            else:
                                # if wheels are running
                                ly = joystick['ly']
                                print('Left')
                                r_wheel.duty_cycle = accel*0x2500
                                r_wheel_r.duty_cycle = 0x0000
                                l_wheel.duty_cycle = l_wheel.duty_cycle/2
                                l_wheel_r.duty_cycle = 0x0000
                                if presses['home']:
                                    raise KeyboardInterrupt

                        # TURN RIGHT FORWARD DRIVE
                        while ly > 0.1 and lx > -0.1:
                            presses = joystick.check_presses()
                            if l_wheel.duty_cycle == 0x0000:
                                # accelerate wheels over span of 2.5 seconds
                                while accel < 5:
                                    r_wheel.duty_cycle = accel*0x1250
                                    r_wheel_r.duty_cycle = 0x0000
                                    l_wheel.duty_cycle = 2*l_wheel.duty_cycle
                                    l_wheel_r.duty_cycle = 0x0000
                                    time.sleep(0.5)
                                    accel += 1
                                    ly = joystick['ly']
                                    print('Turn Right')
                                    # emergency stop
                                    if presses['home']:
                                        raise KeyboardInterrupt
                            else:
                                # if wheels are running
                                ly = joystick['ly']
                                print('Right')
                                r_wheel.duty_cycle = accel*0x1250
                                r_wheel_r.duty_cycle = 0x0000
                                l_wheel.duty_cycle = 2*l_wheel.duty_cycle
                                l_wheel_r.duty_cycle = 0x0000
                                if presses['home']:
                                    raise KeyboardInterrupt

                        # REVERSE TO LEFT
                        while ly < -0.1 and lx < -0.1:
                            presses = joystick.check_presses()
                            if l_wheel.duty_cycle == 0x0000:
                                # accelerate wheels over span of 2.5 seconds
                                while accel < 5:
                                    r_wheel_r.duty_cycle = accel*0x2500
                                    r_wheel.duty_cycle = 0x0000
                                    l_wheel_r.duty_cycle = l_wheel.duty_cycle/2
                                    l_wheel.duty_cycle = 0x0000
                                    time.sleep(0.5)
                                    accel += 1
                                    ly = joystick['ly']
                                    print('Reverse Left')
                                    # emergency stop
                                    if presses['home']:
                                        raise KeyboardInterrupt
                            else:
                                # if wheels are running
                                ly = joystick['ly']
                                print('RLeft')
                                r_wheel_r.duty_cycle = accel*0x2500
                                r_wheel.duty_cycle = 0x0000
                                l_wheel_r.duty_cycle = l_wheel.duty_cycle/2
                                l_wheel.duty_cycle = 0x0000
                                if presses['home']:
                                    raise KeyboardInterrupt

                        # REVERSE TO RIGHT
                        while ly < -0.1 and lx > 0.1:
                            presses = joystick.check_presses()
                            if l_wheel.duty_cycle == 0x0000:
                                # accelerate wheels over span of 2.5 seconds
                                while accel < 5:
                                    r_wheel_r.duty_cycle = accel*0x1250
                                    r_wheel.duty_cycle = 0x0000
                                    l_wheel_r.duty_cycle = 2*l_wheel.duty_cycle
                                    l_wheel.duty_cycle = 0x0000
                                    time.sleep(0.5)
                                    accel += 1
                                    ly = joystick['ly']
                                    print('Reverse Right')
                                    # emergency stop
                                    if presses['home']:
                                        raise KeyboardInterrupt
                            else:
                                # if wheels are running
                                ly = joystick['ly']
                                print('RRight')
                                r_wheel_r.duty_cycle = accel*0x1250
                                r_wheel.duty_cycle = 0x0000
                                l_wheel_r.duty_cycle = 2*l_wheel.duty_cycle
                                l_wheel.duty_cycle = 0x0000
                                if presses['home']:
                                    raise KeyboardInterrupt

                        print("Stopping wheels")                          
                        l_wheel.duty_cycle = 0x0000
                        r_wheel.duty_cycle = 0x0000
                        accel = 1
                        time.sleep(0.1)
                    
                        # OPEN/CLOSE SCOOPER
                    if presses['circle']:
                        if scoop == "closed":
                            print("Opening scooper")
                            move_left(70)
                            move_right(30)
                            scoop = "open"
                        else:
                            print("Closing scooper")
                            move_left(50)
                            move_right(50)
                            scoop = "closed"

                    # MOVE SCOOPER UP/DOWN
                    elif presses['cross']:
                        if lift == "down":
                            print("Lifting scooper")
                            move_transport(50)
                            lift = "up"
                        else:
                            print("Lowering scooper")
                            move_transport(0)
                            lift = "down"

                    # OPEN/CLOSE SHREDDER DOOR
                    elif presses['triangle']:
                        if shreddoor == "closed":
                            print("Opening shredder door")
                            move_shreddoor(60)
                            shreddoor = "open"
                        else:
                            print("Closing shredder door")
                            move_shreddoor(0)
                            shreddoor = "closed"

                    # TURN ON/OFF SHREDDER
                    elif presses['square']:
                        if shred_weed == "off":
                            print("Turning on shredder")
                            shred.duty_cycle = 0x0100
                            shred_weed = "on"
                        else:
                            print("Turning off shredder")
                            shred.duty_cycle = 0x0000
                            shred_weed = "off"

                    # EMERGENCY STOP
                    elif presses['home']:
                        raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("Operation terminated.")
        l_wheel.duty_cycle = 0x0000
        r_wheel.duty_cycle = 0x0000            




