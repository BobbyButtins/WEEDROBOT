# Cal Poly Mechanical Engineering Senior Project - WEED ROBOT
# September 2022 - June 2023
# written by: Jackie Chen

import time
import busio
import board
import adafruit_pca9685
import digitalio
from adafruit_servokit import ServoKit
from approxeng.input.selectbinder import ControllerResource
from approxeng.input.switch import SwitchJoyConLeft
from approxeng.input.dualshock4 import DualShock4
from approxeng.input.controllers import find_matching_controllers, ControllerRequirement
global u

# Set channels to the number of servo channels on the kit
# For the PCA9685 driver, we have 16 channels
# If using bus 0 (pins 26/27), will need to create bus object like this:
i2c_bus0 = busio.I2C(board.SCL_1, board.SDA_1)
# and include it in the parameters for ServoKit: i2c=i2c_bus0
kit = ServoKit(channels=16, i2c=i2c_bus0)
# Don't need to setup bus if SCL and SDA are connected to pins 3/5
# which are for bus 1 (default)

# Setup bus for DC motor control through PCA9685 driver
i2c = busio.I2C(board.SCL_1, board.SDA_1)
pca = adafruit_pca9685.PCA9685(i2c_bus0)
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
init_LED = pca.channels[9]
# Setup for GPIO pins on the Jetson 40-pin expansion header
# Used to set the enable pins on the DC motor for forward and reverse drive
# Jetson pin 12
l_en = digitalio.DigitalInOut(board.D18)
l_en.direction = digitalio.Direction.OUTPUT
l_en.value = True
# pin 11
lrev_en = digitalio.DigitalInOut(board.D17)
lrev_en.direction = digitalio.Direction.OUTPUT
lrev_en.value = True
# pin 22
r_en = digitalio.DigitalInOut(board.D25)
r_en.direction = digitalio.Direction.OUTPUT
r_en.value = True
# pin 21
rrev_en = digitalio.DigitalInOut(board.D9)
rrev_en.direction = digitalio.Direction.OUTPUT
rrev_en.value = True
# OE pin on PCA9685 for disabling all PWM outputs (Low/False = On, High/True = Off)
OE = digitalio.DigitalInOut(board.D19)
OE.direction = digitalio.Direction.OUTPUT
OE.value = True

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

def ldc(pwm):
    '''!
    @brief Controls the forward duty cycle of the left DC motor.
    @param pwm The desired pwm duty cycle.
    '''
    l_wheel.duty_cycle = pwm

def ldc_r(pwm):
    '''!
    @brief Controls the reverse duty cycle of the left DC motor.
    @param pwm The desired pwm duty cycle.
    '''
    l_wheel_r.duty_cycle = pwm

def rdc(pwm):
    '''!
    @brief Controls the forward duty cycle of the right DC motor.
    @param pwm The desired pwm duty cycle.
    '''
    r_wheel.duty_cycle = pwm

def rdc_r(pwm):
    '''!
    @brief Controls the reverse duty cycle of the right DC motor.
    @param pwm The desired pwm duty cycle.
    '''
    r_wheel_r.duty_cycle = pwm

def estop():
    '''!
    @brief Emergency stop command. Disables the enable pins for the DC motors and stops operation of the servos.
           Purely logical stop that locks out controller input. Does not cut off the power to any of the components.
    '''
    # bring all DC motors to a stop by setting pwm to 0 and disabling enable pins
    ldc(0x0000)
    ldc_r(0x0000)
    rdc(0x0000)
    rdc_r(0x0000)
    l_en.value = False
    lrev_en.value = False
    r_en.value = False
    rrev_en.value = False

    # stop the transportation arms and hold them in place
    global u
    move_transport(u)
    # turn off the "STATUS" LED
    init_LED.duty_cycle = 0x0000
    OE.value = True
    print('EMERGENCY STOP')
    exit()

def new_input():
    '''!
    @brief Detects and reads new inputs from the controller. Changes the speed ratio between the left
           and right motors depending on how far horizontally the left analogue stick is pushed.
    '''
    global lx
    global ly
    global turn_ratio
    global turn_ratio_r
    lx = joystick['lx']
    ly = joystick['ly']
    # turn ratios for skid steering (turns by one side of the wheels moving faster than the other)
    # pwm ratio for forward drive
    turn_ratio = 1 + 2*abs(lx)
    # pwm ratio for reverse drive
    turn_ratio_r = 2 - 0.5*abs(lx)
    
if __name__ == "__main__":
    try:
        print("Initializing motor default positions...")
        # Change PWM range for all the servos to get the full range of motion
        kit.servo[0].set_pulse_width_range(1000, 3000)
        kit.servo[1].set_pulse_width_range(1000, 3000)
        kit.servo[2].set_pulse_width_range(1000, 4000)
        kit.servo[3].set_pulse_width_range(1000, 3000)

        # Set starting positions for the scooper servos
        OE.value = False
        move_left(60)
        move_right(40)
        scoop = "closed"

        # Set range of motion for DOCYKE servo to 360 degrees (default is 180 degrees)
        kit.servo[2].actuation_range = 360
        u = 5
        lift = "down"

        # Set starting position for shredder door servo
        move_shreddoor(120)
        shreddoor = "closed"
        shred_weed = "off"
        print("Initialized")

        while True:
            with ControllerResource() as joystick:
                print(type(joystick).__name__)
                print("Waiting for input from controller")
                # turn the "STATUS" LED on to convey that the robot will start taking controller inputs
                init_LED.duty_cycle = 0xFFFF
                while joystick.connected: 
                    # read controller inputs
                    new_input()

                    # duty cycle calculations for driving
                    # calculation dependent on y position (for angles of 45-135 degrees on the left analogue stick)
                    duty_y = int(round(abs(ly)*100)*0x028F)
                    # same calculation as above but divided by the turn ratio to offset the wheel speeds for turning
                    duty_ty = int(round(abs(ly)*100)*0x028F//turn_ratio)
                    # calculation dependent on the x position (for angles of 0-45 and 135-180 degrees on the left analogue stick)
                    duty_x = int(round(abs(lx)*100)*0x028F)
                    # same calculation as above but divided by the turn ratio to offset the wheel speeds for turning
                    duty_rev = int(round(abs(lx)*100)*0x028F//turn_ratio_r)

                    # ROBOT MOVEMENT CONTROL
                    # forward drive (no turn ratio, full drive)
                    if ly > 0.2:
                        if lx > -0.2 and lx < 0.2:
                            # disable reverse pwm on both sides 
                            # FORWARD AND REVERSE PWM MUST NOT BE ON AT THE SAME TIME OR THE MOTOR DRIVERS WILL BURN OUT
                            ldc_r(0x0000)
                            rdc_r(0x0000)
                            # forward pwms enabled
                            ldc(duty_y)
                            print(duty_y)
                            rdc(duty_y)
                            print(duty_y)
                            # wait for new inputs
                            new_input()
                            print('forward')
                            # check for estop (for future implementation use threading instead to listen to the button press?)
                            presses = joystick.check_presses()
                            if presses['home']:
                                estop()
                                
                        # skid turn left (left wheels at reduced/reverse drive, right wheels at full speed)
                        if lx < -0.2:
                            # if the left stick angle is within 90-135 degrees
                            if ly > abs(lx):
                                # disable reverse pwm on both sides 
                                # FORWARD AND REVERSE PWM MUST NOT BE ON AT THE SAME TIME OR THE MOTOR DRIVERS WILL BURN OUT
                                ldc_r(0x0000)
                                rdc_r(0x0000)
                                # forward pwms enabled
                                ldc(duty_ty)
                                print(-duty_ty)
                                rdc(duty_y)
                                print(duty_y)
                                # wait for new inputs
                                new_input()
                                print('left')
                                # check for estop (for future implementation use threading instead to listen to the button press?)
                                presses = joystick.check_presses()
                                if presses['home']:
                                    estop()
                            # if the left stick is within 135-180 degrees (right wheel will drive forward and left will reverse)
                            elif abs(lx) >= ly:
                                # disable forward pwm on left and reverse pwm on right
                                # FORWARD AND REVERSE PWM MUST NOT BE ON AT THE SAME TIME OR THE MOTOR DRIVERS WILL BURN OUT
                                ldc(0x0000)
                                rdc_r(0x0000)
                                # reverse left pwm and foward right pwm enabled
                                ldc_r(duty_rev)
                                print(-duty_rev)
                                rdc(duty_x)
                                print(duty_x)
                                # wait for new inputs
                                new_input()
                                print('left-rev')
                                # check for estop
                                presses = joystick.check_presses()
                                if presses['home']:
                                    estop()

                        # skid turn right (right wheels at reduced/reverse drive, left wheels at full speed)  
                        elif lx > 0.2:
                            # if the left stick angle is within 45-90 degrees
                            if ly > lx:
                                # disable reverse pwm on both sides 
                                # FORWARD AND REVERSE PWM MUST NOT BE ON AT THE SAME TIME OR THE MOTOR DRIVERS WILL BURN OUT
                                ldc_r(0x0000)
                                rdc_r(0x0000)
                                # forward pwms enabled
                                ldc(duty_y)
                                print(duty_y)
                                rdc(duty_ty)
                                print(duty_ty)
                                # wait for new inputs
                                new_input()
                                print('right')
                                # check for estop
                                presses = joystick.check_presses()
                                if presses['home']:
                                    estop()
                            # if the left stick angle is within 0-45 degrees
                            elif ly <= lx:
                                # disable reverse pwm on left side and forward pwm on right 
                                # FORWARD AND REVERSE PWM MUST NOT BE ON AT THE SAME TIME OR THE MOTOR DRIVERS WILL BURN OUT
                                ldc_r(0x0000)
                                rdc(0x0000)
                                # foward left pwm and reverse right pwm enabled
                                ldc(duty_x)
                                print(duty_x)
                                rdc_r(duty_rev)
                                print(duty_rev)
                                # wait for new inputs
                                new_input()
                                print('right-rev')
                                # check for estop
                                presses = joystick.check_presses()
                                if presses['home']:
                                    estop()
                            
                    # reverse drive (no turn ratio, full reverse drive)
                    # turning in reverse is effectively the same as turning in forward drive
                    # to back up at an angle, turn in forward drive and then go straight reverse drive (not smooth, will have to optimize later)
                    elif ly < -0.2:
                        if lx > -0.2 and lx < 0.2:
                            # disable forward pwm on both sides
                            # FORWARD AND REVERSE PWM MUST NOT BE ON AT THE SAME TIME OR THE MOTOR DRIVERS WILL BURN OUT
                            ldc(0x0000)
                            rdc(0x0000)
                            # reverse pwms enabled
                            ldc_r(duty_y)
                            print(-duty_y)
                            rdc_r(duty_y)
                            # wait for new inputs
                            new_input()
                            print('reverse')
                            # check for estop
                            presses = joystick.check_presses()
                            if presses['home']:
                                estop()
                   
                    # idle state when the robot is not driving
                    # the conditions for x and y are offset from 0 due to input noise from the cheap controller
                    else:
                        while ly > -0.2 and ly < 0.2:
                            # set all pwms to 0 to stop the motors
                            ldc(0x0000)
                            ldc_r(0x0000)
                            rdc(0x0000)
                            rdc_r(0x0000)
                            # continue to wait for new inputs
                            new_input()
                            presses = joystick.check_presses()

                            # OPEN/CLOSE SCOOPER
                            if presses['circle']:
                                # starts closed, will cycle between closed and open on button press depending on the previous state
                                if scoop == "closed":
                                    print("Opening scooper")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # check for estop
                                    if presses['home']:
                                        estop()
                                    move_left(77)
                                    move_right(15)
                                    scoop = "open"
                                else:
                                    print("Closing scooper")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # check for estop
                                    if presses['home']:
                                        estop()
                                    move_left(45)
                                    move_right(47)
                                    scoop = "closed"

                            # MOVE SCOOPER UP/DOWN
                            elif presses['cross']:
                                # starts at down position, will cycle between lowered and raised positions on button press depending on the previous state
                                if lift == "down":
                                    print("Lifting scooper")
                                    u = 30
                                    time.sleep(0.5)
                                    # moves at small intervals to avoid a sudden swing that could damage/injure surroundings and/or the scooper arms
                                    while u <= 345:
                                        u += 15
                                        move_transport(u)
                                        presses = joystick.check_presses()
                                        # check for estop
                                        if presses['home']:
                                            estop()
                                        time.sleep(0.3)
                                    lift = "up"
                                else:
                                    print("Lowering scooper")
                                    u = 345
                                    time.sleep(0.5)
                                    # moves at small intervals to avoid a sudden swing that could damage/injure surroundings and/or the scooper arms
                                    while u > 30:
                                        move_transport(u)
                                        u -= 15
                                        presses = joystick.check_presses()
                                        # check for estop
                                        if presses['home']:
                                            estop()
                                        time.sleep(0.3)
                                    lift = "down"

                            # OPEN/CLOSE SHREDDER DOOR
                            elif presses['triangle']:
                                # starts closed, will cycle between closed and open on button press depending on the previous state
                                if shreddoor == "closed":
                                    print("Opening shredder door")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # check for estop
                                    if presses['home']:
                                        estop()
                                    move_shreddoor(10)
                                    shreddoor = "open"
                                else:
                                    print("Closing shredder door")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # check for estop
                                    if presses['home']:
                                        estop()
                                    move_shreddoor(120)
                                    shreddoor = "closed"

                            # TURN ON/OFF SHREDDER (not implemented)
                            # elif presses['square']:
                            #     if shred_weed == "off":
                            #         print("Turning on shredder")
                            #         shred.duty_cycle = 0x0100
                            #         shred_weed = "on"
                            #     else:
                            #         print("Turning off shredder")
                            #         shred.duty_cycle = 0x0000
                            #         shred_weed = "off"

                            elif presses['home']:
                                estop()
                
                # if controller is disconnected, stop all motors
                # bring all DC motors to a stop by setting pwm to 0 and disabling enable pins
                ldc(0x0000)
                ldc_r(0x0000)
                rdc(0x0000)
                rdc_r(0x0000)
                l_en.value = False
                lrev_en.value = False
                r_en.value = False
                rrev_en.value = False

                # stop the transportation arms and hold them in place
                move_transport(u)
                init_LED.duty_cycle = 0x0000
                OE.value = True
                print('Controller disconnected')
                exit()
                
    # Keyboard Interrupt exception for testing and debugging code
    except KeyboardInterrupt: 
        print("Operation terminated.")
        ldc(0x0000)
        ldc_r(0x0000)
        rdc(0x0000)
        rdc_r(0x0000)
        l_en.value = False
        lrev_en.value = False
        r_en.value = False
        rrev_en.value = False

        # stop the transportation arms and hold them in place
        move_transport(u)
        init_LED.duty_cycle = 0x0000
        OE.value = True
