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
# physical ESTOP (pin 31)
# pulled down (False when switch off, True when switch on)
switch = digitalio.DigitalInOut(board.D6)
switch.direction = digitalio.Direction.OUTPUT
switch.value = False
switch.direction = digitalio.Direction.INPUT
# switch.pull = digitalio.Pull.DOWN
# OE pin on PCA9685 for disabling all PWM outputs (Low = On, High = Off)
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
    @brief Emergency stop command. Disables the enable pins for the DC motors and stops operation of the servos
           after moving them to a safe position. 
    '''
    # bring all DC motors to a stop
    # ldc(0x0000)
    # ldc_r(0x0000)
    # rdc(0x0000)
    # rdc_r(0x0000)
    l_en.value = False
    lrev_en.value = False
    r_en.value = False
    rrev_en.value = False

    # slowly set all servos to safe locations
    move_left(70)
    move_right(30)
    global u
    while u > 30:
        move_transport(u)
        u -= 5
        time.sleep(0.5)
    move_shreddoor(10)

    # keep power on while preventing any further input
    # wait for diagnosis of issue/reset or shutdown
    # switch.direction = digitalio.Direction.OUTPUT
    # switch.value = False
    init_LED.duty_cycle = 0x0000
    # OE.value = True
    print('EMERGENCY STOP')
    exit()

    
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
        move_shreddoor(10)
        shreddoor = "closed"
        shred_weed = "off"
        print("Initialized")
        n = 0

        while True:
            # check if estop is pressed
            print(switch.value)
            if switch.value == True:
                estop()
            with ControllerResource() as joystick:
                print(type(joystick).__name__)
                print("Waiting for input from controller")
                init_LED.duty_cycle = 0xFFFF
                while joystick.connected: 
                    # detect joystick input
                    lx = joystick['lx']
                    ly = joystick['ly']
                    
                    # turn ratio between the left and right wheels
                    turn_ratio = 50*abs(lx)
                    
                    # ROBOT MOVEMENT CONTROL
                    # forward drive
                    if ly > 0.2:
                        if lx > -0.1 and lx < 0.1:
                            ldc_r(0x0000)
                            ldc(int(round(abs(ly)*100)*0x028F))
                            print(-int(round(abs(ly)*100)*0x028F))
                            rdc_r(0x0000)
                            rdc(int(round(abs(ly)*100)*0x028F))
                            print(int(round(abs(ly)*100)*0x028F))
                            lx = joystick['lx']
                            ly = joystick['ly']
                            turn_ratio = 50*abs(lx)
                            print('forward')
                            presses = joystick.check_presses()
                            if presses['home'] or switch.value == True:
                                estop()
                        # skid turn left
                        if lx < -0.1:
                            if ly > abs(lx):
                                ldc_r(0x0000)
                                ldc(int(round(abs(ly)*100)*0x028F//turn_ratio))
                                print(-int(round(abs(ly)*100)*0x028F//turn_ratio))
                                rdc_r(0x0000)
                                rdc(int(round(abs(ly)*100)*0x028F))
                                print(int(round(abs(ly)*100)*0x028F))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('left')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()
                            elif abs(lx) >= ly:
                                ldc_r(0x0000)
                                ldc(int(round(abs(lx)*100)*0x028F//turn_ratio))
                                print(-int(round(abs(lx)*100)*0x028F//turn_ratio))
                                rdc_r(0x0000)
                                rdc(int(round(abs(lx)*100)*0x028F))
                                print(int(round(abs(lx)*100)*0x028F))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('left')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()

                        # skid turn right    
                        elif lx > 0.1:
                            if ly > lx:
                                ldc_r(0x0000)
                                ldc(int(round(abs(ly)*100)*0x028F))
                                print(int(round(abs(ly)*100)*0x028F))
                                rdc_r(0x0000)
                                rdc(int(round(abs(ly)*100)*0x028F//turn_ratio))
                                print(int(round(abs(ly)*100)*0x028F//turn_ratio))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('right')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()

                            elif ly <= lx:
                                ldc_r(0x0000)
                                ldc(int(round(abs(lx)*100)*0x028F))
                                print(int(round(abs(lx)*100)*0x028F))
                                rdc_r(0x0000)
                                rdc(int(round(abs(lx)*100)*0x028F//turn_ratio))
                                print(int(round(abs(lx)*100)*0x028F//turn_ratio))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('right')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()
                            
                    # reverse drive
                    elif ly < -0.2:
                        if lx > -0.1 and lx < 0.1:
                            ldc(0x0000)
                            ldc_r(int(round(abs(ly)*100)*0x028F))
                            print(-int(round(abs(ly)*100)*0x028F))
                            rdc(0x0000)
                            rdc_r(int(round(abs(ly)*100)*0x028F))
                            lx = joystick['lx']
                            ly = joystick['ly']
                            turn_ratio = 50*abs(lx)
                            print('reverse')
                            presses = joystick.check_presses()
                            if presses['home'] or switch.value == True:
                                estop()
                        # skid reverse left
                        if lx < -0.1:
                            if abs(ly) > abs(lx):
                                ldc(0x0000)
                                ldc_r(int(round(abs(ly)*100)*0x028F//turn_ratio))
                                print(-int(round(abs(ly)*100)*0x028F//turn_ratio))
                                rdc(0x0000)
                                rdc_r(int(round(abs(ly)*100)*0x028F))
                                print(-int(round(abs(ly)*100)*0x028F))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('rleft')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()
                            elif abs(ly) <= abs(lx):
                                ldc(0x0000)
                                ldc_r(int(round(abs(lx)*100)*0x028F//turn_ratio))
                                print(-int(round(abs(lx)*100)*0x028F//turn_ratio))
                                rdc(0x0000)
                                rdc_r(int(round(abs(lx)*100)*0x028F))
                                print(-int(round(abs(lx)*100)*0x028F))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('rleft')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()
                            
                        # skid reverse right    
                        elif lx > 0.1:
                            if abs(ly) > lx:
                                ldc(0x0000)
                                ldc_r(int(round(abs(ly)*100)*0x028F))
                                print(int(round(abs(ly)*100)*0x028F))
                                rdc(0x0000)
                                rdc_r(int(round(abs(ly)*100)*0x028F//turn_ratio))
                                print(int(round(abs(ly)*100)*0x028F//turn_ratio))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('rright')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()
                            if abs(ly) <= lx:
                                ldc(0x0000)
                                ldc_r(int(round(abs(lx)*100)*0x028F))
                                print(int(round(abs(lx)*100)*0x028F))
                                rdc(0x0000)
                                rdc_r(int(round(abs(lx)*100)*0x028F//turn_ratio))
                                print(int(round(abs(lx)*100)*0x028F//turn_ratio))
                                lx = joystick['lx']
                                ly = joystick['ly']
                                turn_ratio = 50*abs(lx)
                                print('rright')
                                presses = joystick.check_presses()
                                if presses['home'] or switch.value == True:
                                    estop()
                    # idle state
                    else:
                        while ly > -0.2 and ly < 0.2:
                            ldc(0x0000)
                            ldc_r(0x0000)
                            rdc(0x0000)
                            rdc_r(0x0000)
                            lx = joystick['lx']
                            ly = joystick['ly']
                            presses = joystick.check_presses()

                            # OPEN/CLOSE SCOOPER
                            if presses['circle']:
                                if scoop == "closed":
                                    print("Opening scooper")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # prob not the most efficient method to do this, but will be optimized in future
                                    if presses['home'] or switch.value == True:
                                        estop()
                                    move_left(77)
                                    move_right(27)
                                    scoop = "open"
                                else:
                                    print("Closing scooper")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # prob not the most efficient method to do this, but will be optimized in future
                                    if presses['home'] or switch.value == True:
                                        estop()
                                    move_left(50)
                                    move_right(50)
                                    scoop = "closed"

                            # MOVE SCOOPER UP/DOWN
                            elif presses['cross']:
                                if lift == "down":
                                    print("Lifting scooper")
                                    u = 30
                                    time.sleep(0.5)
                                    while u < 345:
                                        u += 15
                                        move_transport(u)
                                        presses = joystick.check_presses()
                                        if presses['home'] or switch.value == True:
                                            estop()
                                        time.sleep(0.5)
                                    lift = "up"
                                else:
                                    print("Lowering scooper")
                                    u = 345
                                    time.sleep(0.5)
                                    while u > 30:
                                        move_transport(u)
                                        u -= 15
                                        presses = joystick.check_presses()
                                        if presses['home'] or switch.value == True:
                                            estop()
                                        time.sleep(0.5)
                                    lift = "down"

                            # OPEN/CLOSE SHREDDER DOOR
                            elif presses['triangle']:
                                if shreddoor == "closed":
                                    print("Opening shredder door")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # prob not the most efficient method to do this, but will be optimized in future
                                    if presses['home'] or switch.value == True:
                                        estop()
                                    move_shreddoor(90)
                                    shreddoor = "open"
                                else:
                                    print("Closing shredder door")
                                    time.sleep(0.5)
                                    presses = joystick.check_presses()
                                    # prob not the most efficient method to do this, but will be optimized in future
                                    if presses['home'] or switch.value == True:
                                        estop()
                                    move_shreddoor(10)
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

                            # EMERGENCY STOP
                            elif presses['home'] or switch == True:
                                estop()
                
                # if controller is disconnected, stop all motors
                move_left(80) 
                move_right(20)
                move_transport(u)
                move_shreddoor(120) 
                l_en.value = False
                lrev_en.value = False
                r_en.value = False
                rrev_en.value = False
                init_LED = 0x0000
                print('Controller disconnected')
                exit()

    except KeyboardInterrupt: 
        print("Operation terminated.")
        move_left(80)
        move_right(20)
        move_shreddoor(120)
        l_en.value = False
        lrev_en.value = False
        r_en.value = False
        rrev_en.value = False
        init_LED.duty_cycle = 0x0000
