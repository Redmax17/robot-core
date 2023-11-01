"""

Connect to controller using bluetooth

Use GPIO pins for
12: Left  motor PWM
19: Right motor PWM

PWM Input Pulse (high time)
1-2 ms nominal

PWM Input Rate (period)
2.9 - 100 ms

Backwards: 1.0 ms =  50000
Neutral:   1.5 ms =  75000
Forwards:  2.0 ms = 100000

"""

import sys
import time
from math import dist, atan2, sin, cos, pi
from select import select

# Print date and time
print(time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()))

# time.sleep(30)

print("Importing 3rd party libraries...")

# Append system path so that evdev can be imported
sys.path.append('/home/pi/.local/lib/python3.9/site-packages')

# Import libraries

# Used to read the bluetooth controller
from evdev import list_devices, InputDevice, categorize, ecodes, ff

# Used for PWM to drive the motors
import pigpio

# Used to get input from the tackle sensor
import RPi.GPIO as GPIO



#
# Constants
#

DISABLE_MOTORS = False

ERROR = 0.15

# PWM frequency
f = 50

# PWM ranges
pMin = 1000
pMax = 2000



    

#
# Pin Setup
#

# Function shortcut
pwm = pigpio.pi()

# Right motor
R = 19
pwm.set_mode(R, pigpio.OUTPUT)
# pwm.set_PWM_frequency(R, f)
# pwm.set_servo_pulsewidth(R, 0)

# Left motor
L = 12
pwm.set_mode(L, pigpio.OUTPUT)
# pwm.set_PWM_frequency(L, f)
# pwm.set_servo_pulsewidth(L, 0)

# Track that moves the ball
TRACK = 13
pwm.set_mode(TRACK, pigpio.OUTPUT)

# Motor driver of the laucher
LAUNCHER = 21
pwm.set_mode(LAUNCHER, pigpio.OUTPUT)

# Tackle Sensor
TACKLE = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(TACKLE, GPIO.IN)



#
# Controller Setup
#
"""
XBOX 360

  Y
X   B
  A

A = SOUTH
B = EAST
X = NORTH
Y = WEST
"""

# Acceptable controllers
names = [
    "Xbox Wireless Controller",
    "Wireless Controller",                  # PS4
    "DualSense Wireless Controller",        # PS5 Dual Shock
    "Xbox 360 Wireless Receiver (XBOX)",
]

# Search until a listed device is connected
dev = None
while dev == None:
    # All the conected devices
    for d in list_devices():
        # print(d)
        if InputDevice(d).name in names:
            dev = InputDevice(d)
            del d
            break           
        
# Prints out device info at start
# {('EV_SYN', 0): [('SYN_REPORT', 0), ('SYN_CONFIG', 1), ('SYN_DROPPED', 3), ('?', 21)], ('EV_KEY', 1): [(['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH'], 304), (['BTN_B', 'BTN_EAST'], 305), (['BTN_NORTH', 'BTN_X'], 307), (['BTN_WEST', 'BTN_Y'], 308), ('BTN_TL', 310), ('BTN_TR', 311), ('BTN_TL2', 312), ('BTN_TR2', 313), ('BTN_SELECT', 314), ('BTN_START', 315), ('BTN_MODE', 316), ('BTN_THUMBL', 317), ('BTN_THUMBR', 318)], ('EV_ABS', 3): [(('ABS_X', 0), AbsInfo(value=124, min=0, max=255, fuzz=0, flat=0, resolution=0)), (('ABS_Y', 1), AbsInfo(value=129, min=0, max=255, fuzz=0, flat=0, resolution=0)), (('ABS_Z', 2), AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)), (('ABS_RX', 3), AbsInfo(value=128, min=0, max=255, fuzz=0, flat=0, resolution=0)), (('ABS_RY', 4), AbsInfo(value=127, min=0, max=255, fuzz=0, flat=0, resolution=0)), (('ABS_RZ', 5), AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)), (('ABS_HAT0X', 16), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)), (('ABS_HAT0Y', 17), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0))], ('EV_FF', 21): [(['FF_EFFECT_MIN', 'FF_RUMBLE'], 80), ('FF_PERIODIC', 81), (['FF_SQUARE', 'FF_WAVEFORM_MIN'], 88), ('FF_TRIANGLE', 89), ('FF_SINE', 90), (['FF_GAIN', 'FF_MAX_EFFECTS'], 96)]}
print(dev)
print(dev.capabilities(verbose=True))

# Universal Controller Constants
A     = ecodes.BTN_A
B     = ecodes.BTN_B
X     = ecodes.BTN_X
Y     = ecodes.BTN_Y
LB    = ecodes.BTN_TL
RB    = ecodes.BTN_TR
BACK  = ecodes.BTN_SELECT
START = ecodes.BTN_START
HOME  = ecodes.BTN_MODE

match dev.name:

    case "Xbox Wireless Controller":
        LX    =   0
        LY    =   1
        RX    =   2     # ABS_RZ
        RY    =   5     # ABS_Z
        BACK  = ecodes.KEY_BACK
        MIN = 0         # Joystick min
        MAX = 2**16-1   # Joystick max
    
    case "Wireless Controller" | "DualSense Wireless Controller":
        LX    =   0
        LY    =   1
        RX    =   3
        RY    =   4
        MIN = 0         # Joystick min
        MAX = 2**8-1    # Joystick max
        

#
# Rummble
#

"""
rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
effect_type = ff.EffectType(ff_rumble_effect=rumble)
duration_ms = 1000
effect = ff.Effect(
    ecodes.FF_RUMBLE, -1, 0,
    ff.Trigger(0, 0),
    ff.Replay(duration_ms, 0),
    effect_type
    ecodes.FF_RUMBLE, # type
    
    #-1, # id (set by ioctl)
    # 0,  # direction
    # ff.Trigger(0, 0), # no triggers
    #ff.Replay(duration_ms, 0), # length and delay
    #ff.EffectType(ff_rumble_effect=rumble)
)
repeat_count = 1
effect_id = dev.upload_effect(effect)
dev.write(ecodes.EV_FF, effect_id, repeat_count)
dev.erase_effect(effect_id)
"""



#
# Control Vars
#

# Reverse
rev = 1

# Speed scale
scale = 0.3

# Initial event
event = None

# Initial joystick position
rightX = rightY = MAX/2

# Initial velocity
vL = vR = 0

# Initial acceleration
aR = aL = 0



#
# Event handler
#

def read_event(event):
    """ Event handler function called for every event """
    
    global rightX, rightY # Only used outside of the fuction for the console display
    global aL, aR
    
    # print(categorize(event))
    #print(event)
    #read stick axis movement
    #elif event.type == ecodes.EV_ABS:

    # Left button reverses
    if LB in dev.active_keys():
        rev = -1
    else:
        rev = 1

    # Home Button Pressed
    if HOME in dev.active_keys():
        raise Exception('Home Button Pressed')
        
    # Right Joystick Moved
    elif event.code == RX or event.code == RY:

        # Shift range of joystick to [-1,1]
        if event.code == RX:
            rightX = event.value / MAX * 2 - 1
        else:
            rightY = event.value / MAX * 2 - 1
            
        if (abs(rightY) < ERROR): rightY = 0
        if (abs(rightX) < ERROR): rightX = 0
        
        # Angle of joystick
        theta = atan2(rightX, -rightY*rev)

        # Radius of joystick from center
        r = dist((0, 0), (rightX, rightY))
        
        # Clamp radius [0, 1] because the device is not physically perfect
        r = min(r, 1)
        
        
        
        #
        # Acceleration
        # Equal to the 
        #
        
        # Calculate Acceleration
        aL = -sin(theta) + cos(theta)
        aR =  sin(theta) + cos(theta) 
    
        # Clamp values [-1,1]
        aL = min(max(aL, -1), 1)
        aR = min(max(aR, -1), 1)

        # Scale by radius
        aL *= r
        aR *= r
        
        # Round to edge values using error
        aL = 0 if abs(aL) < ERROR else aL
        aR = 0 if abs(aR) < ERROR else aR

        # ~ aL = int(255 * aL) / 255.0
        # ~ aR = round(aR, 2)



#
# Main Control Loop
#

try:
    while True: 
    
        # Read from the device or timeout
        rlist, wlist, xlist = select([dev.fd], [], [], 0.0001)

        # If there is something to be read from dev
        if rlist:
            for event in dev.read():
                read_event(event)
    
    
    
        #
        # Velocity (vL, vR)
        # Equal to the 
        #

        # Initial velocity is equal to acceleration        
        vL = aL
        vR = aR
        
        # Clamp velocity [-1,1]
        vL = min(max(vL, -1), 1)
        vR = min(max(vR, -1), 1)
        
        scale_min = 0.3
        
        # Change in velocity's scale over time
        delta = 0.0001
        
        # Increase / decrease speed scale if joystick in far enough from center
        if abs(aL) > 0.8 or abs(aR) > 0.8:
            scale += delta
        else:
            scale -= delta * 100
            
        # Clamp scale [0.3, 1]
        scale = min(max(scale, scale_min), 1)
        
        # Round to edge values using error
        # ~ vL = 0 if abs(vL) < 0.0 else vL
        # ~ vR = 0 if abs(vR) < ERROR else vR



        #
        # Output PWM (pL, pR)
        # The signals sent to the motor drivers
        #
        
        # Initially Start with the velocity
        pL = vL * -scale
        pR = vR *  scale
    
        # Adjust to PWM values        
        pDif = pMax - pMin
        pAmp = pDif // 2
        
        pL = int(pMin + pAmp + pAmp * pL)
        pR = int(pMin + pAmp + pAmp * pR)

        # Round in base 10 to prevent jittering
        pL = pL // 10 * 10
        pR = pR // 10 * 10



        #
        # Football Launcher
        #
        
        if LB in dev.active_keys():
            pwm.set_servo_pulsewidth(TRACK, 2000)
            pwm.set_servo_pulsewidth(LAUNCHER, 1500)
        elif RB in dev.active_keys():
            pwm.set_servo_pulsewidth(TRACK, pMax)
            pwm.set_servo_pulsewidth(LAUNCHER, pMax)
        else:
            pwm.set_servo_pulsewidth(TRACK, 1500)
            pwm.set_servo_pulsewidth(LAUNCHER, 1500)
            

        
        #
        # Debug Text
        #
        
        # text  = ""
        # text += f"Input {rightX: 1.3f} {rightY: 1.3f}"
        # text += f" | A {aL: 1.4f} {aR: 1.4f}"
        # text += f" | V {vL: 1.4f} {vR: 1.4f}"
        # text += f" | P {pL} {pR}"
        # print(text)
        
        
        
        #
        # Update motor drivers
        #
        
        # Update driver if there is no tackle
        if GPIO.input(TACKLE):
            if not DISABLE_MOTORS:
                #pwm.hardware_PWM(L, f, pL)
                #pwm.hardware_PWM(R, f, pR)
             
                """
                if B in dev.active_keys():
                    pwm.set_servo_pulsewidth(L, 0)
                    pwm.set_servo_pulsewidth(R, 0)
                    print("0")
                    time.sleep(1)
                    pwm.set_servo_pulsewidth(L, pMax)
                    pwm.set_servo_pulsewidth(R, pMax)
                    print("Max")
                    time.sleep(1)
                    pwm.set_servo_pulsewidth(L, pMin)
                    pwm.set_servo_pulsewidth(R, pMin)
                    print("Min")
                    time.sleep(1)
                
                else:
                """
                    
                pwm.set_servo_pulsewidth(L, pL)
                pwm.set_servo_pulsewidth(R, pR)
        
        # Otherwise, full stop
        else:
            if not DISABLE_MOTORS:
                pwm.hardware_PWM(L, f, 75000)
                pwm.hardware_PWM(R, f, 75000)
            print("HIT")
            
            # Vibrate controller
            rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
            effect_type = ff.EffectType(ff_rumble_effect=rumble)
            duration_ms = 1000
            effect = ff.Effect(
                ecodes.FF_RUMBLE, -1, 0,
                ff.Trigger(0, 0),
                ff.Replay(duration_ms, 0),
                effect_type
            )
            repeat_count = 1
            effect_id = dev.upload_effect(effect)
            dev.write(ecodes.EV_FF, effect_id, repeat_count)
            dev.erase_effect(effect_id)
                    
        
        
# Clean up the pins
finally:
    print(time.strftime("%a, %d %b %Y %H:%M:%S", time.localtime()))
    print("Cleaning...")
    pwm.stop()
    print("Done")
 
    
 
#
# Terminal Commands
# Only need to use once for setup
#
 
"""
#!/bin/bash
pip install evdev
sudo systemctl enable pigpiod
"""

# Boot on start 

"""
cd Desktop
nano launcher.sh

# #!/bin/sh
# launcher.sh
# navigate to home directory, then to this directory, then execute python script, then back home
cd /
cd home/pi/Desktop
sudo python main.py
cd /

chmod 755 launcher.sh
cd
sudo crontab -e

@reboot sh /home/pi/Desktop/launcher.sh >/home/pi/Desktop/cronlog 2>&1

sudo reboot
"""

"""
sudo python3 /home/pi/Desktop/main.py >> /home/pi/Desktop/log.txt 2>&1 &

sudo killall python3
"""
