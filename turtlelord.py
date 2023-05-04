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
# Setup for GPIO pins on the Jetson 40-pin expansion header
# Used to set the enable pins on the DC motor for forward and reverse drive
l_en = digitalio.DigitalInOut(board.D18)
l_en.direction = digitalio.Direction.OUTPUT
l_en.value = True
lrev_en = digitalio.DigitalInOut(board.D17)
lrev_en.direction = digitalio.Direction.OUTPUT
lrev_en.value = True
r_en = digitalio.DigitalInOut(board.D23)
r_en.direction = digitalio.Direction.OUTPUT
r_en.value = True
rrev_en = digitalio.DigitalInOut(board.D22)
rrev_en.direction = digitalio.Direction.OUTPUT
rrev_en.value = True

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
                    # detect joystick input
                    lx = joystick['lx']
                    ly = joystick['ly']

                    # turn ratio between the left and right wheels
                    turn_ratio = 1 + abs(lx)
                    
                    # ROBOT MOVEMENT CONTROL
                    # forward drive
                    if ly > 0.2:
                        # skid turn left
                        while lx < -0.1:
                            ldc(int(round(abs(ly)*100)*0x0090//turn_ratio))
                            ldc_r(0x0000)
                            print(-int(round(abs(ly)*100)*0x0090//turn_ratio))
                            rdc(int(round(abs(ly)*100)*0x0090))
                            rdc_r(0x0000)
                            lx = joystick['lx']
                            ly = joystick['ly']
                            turn_ratio = 1 + abs(lx)
                            print('left')
                            presses = joystick.check_presses()
                            if presses['home']:
                                raise KeyboardInterrupt
                            
                        # skid turn right    
                        while lx > 0.1:
                            ldc(int(round(abs(ly)*100)*0x0090))
                            ldc_r(0x0000)
                            print(int(round(abs(ly)*100)*0x0090//turn_ratio))
                            rdc(int(round(abs(ly)*100)*0x0090//turn_ratio))
                            rdc_r(0x0000)
                            lx = joystick['lx']
                            ly = joystick['ly']
                            turn_ratio = 1 + abs(lx)
                            print('right')
                            presses = joystick.check_presses()
                            if presses['home']:
                                raise KeyboardInterrupt
                            
                    # reverse drive
                    elif ly < -0.2:
                        # skid reverse left
                        while lx < -0.1:
                            ldc_r(int(round(abs(ly)*100)*0x0090//turn_ratio))
                            ldc(0x0000)
                            print(-int(round(abs(ly)*100)*0x0090//turn_ratio))
                            rdc_r(int(round(abs(ly)*100)*0x0090))
                            rdc(0x0000)
                            lx = joystick['lx']
                            ly = joystick['ly']
                            turn_ratio = 1 + abs(lx)
                            print('left')
                            presses = joystick.check_presses()
                            if presses['home']:
                                raise KeyboardInterrupt
                            
                        # skid reverse right    
                        while lx > 0.1:
                            ldc_r(int(round(abs(ly)*100)*0x0090))
                            ldc(0x0000)
                            print(int(round(abs(ly)*100)*0x0090//turn_ratio))
                            rdc_r(int(round(abs(ly)*100)*0x0090//turn_ratio))
                            rdc(0x0000)
                            lx = joystick['lx']
                            ly = joystick['ly']
                            turn_ratio = 1 + abs(lx)
                            print('right')
                            presses = joystick.check_presses()
                            if presses['home']:
                                raise KeyboardInterrupt
                            
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
                                    u = 5
                                    while u < 50:
                                        move_transport(u)
                                        u += 5
                                        if presses['home']:
                                            raise KeyboardInterrupt
                                        time.sleep(0.5)
                                    lift = "up"
                                else:
                                    print("Lowering scooper")
                                    d = 45
                                    while d > 0:
                                        move_transport(u)
                                        d -= 5
                                        if presses['home']:
                                            raise KeyboardInterrupt
                                        time.sleep(0.5)
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
        ldc(0x0000)
        ldc_r(0x0000)
        rdc(0x0000)
        rdc_r(0x0000)         




