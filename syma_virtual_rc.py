#
# Python virtual remote control for Syma S107. This program sends data to the serial port 
# when it receives an acknowledgement for more data from the arduino. The UI simulates
# (rather crudely) a throttle stick, and a yaw/pitch stick, along with a trim control.
#
# This script also includes a simple form of flow control. We're rendering the controls at
# 20fps, which means that we would effectively be sending 20 packets a second to the arduino.
# Unfortunately, the arduino can only consume 10 packets every second. Therefore, it is prudent
# to send data only when the arduino needs it. Whenever the arduino needs data, it will send a 
# byte through the serial channel that lets the python script know that it can send data. Until
# then, the script will queue up data into a local buffer (pretty much a queue).
#
# Author: Vivin S. Paliath
# http://vivin.net
#
import pygame
import math
import os
import serial
import struct

from collections import deque
 
# Define some colors
black    = (   0,   0,   0)
white    = ( 255, 255, 255)
blue     = (  50,  50, 255)
green    = (   0, 255,   0)
dkgreen  = (   0, 100,   0)
red      = ( 255,   0,   0)
purple   = (0xBF,0x0F,0xB5)
brown    = (0x55,0x33,0x00)

# Define some constants

ARDUINO_SERIAL_PORT      = "/dev/ttyACM0"
ARDUINO_SERIAL_BAUD_RATE = 9600

CONTROL_RADIUS  = 83
JOYSTICK_RADIUS = 20

CONTROL_X = 550
CONTROL_Y = 250

WINDOW_X = 400
WINDOW_Y = 200

THROTTLE_X = 175
THROTTLE_Y = 188

THROTTLE_WIDTH  = 34
THROTTLE_HEIGHT = 129

THROTTLE_MAX = 127
THROTTLE_MIN = 0

TRIM_X = 280
TRIM_Y = 320

TRIM_MAX = 126
TRIM_MIN = 0

TRIM_HEIGHT = 34
TRIM_WIDTH = 129

TRIM_MARKER_HEIGHT = 44
TRIM_MARKER_WIDTH = 4

TRANSLATION_FACTOR = 63

TITLE_X = 400
TITLE_Y = 10

VALUES_X = 400
VALUES_Y = 30

THROTTLE_UP_KEY   = pygame.K_w
THROTTLE_DOWN_KEY = pygame.K_s

TRIM_LEFT_KEY  = pygame.K_a
TRIM_RIGHT_KEY = pygame.K_d

YAW      = 0
PITCH    = 1
THROTTLE = 2
TRIM     = 3

ZERO_YAW      = 63
ZERO_PITCH    = 63
ZERO_THROTTLE = 0
ZERO_TRIM     = 63

# Function to draw the background
def draw_background(screen):
    # Set the screen background
    screen.fill(white)

# Function to calculate the distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

# Function to limit the x and y coordinate of the joystick. We call this method if the distance between the current x and y coordinate of the mouse
# and the center of the control is greater than the difference between CONTROL_RADIUS and JOYSTICK_RADIUS. The limited x and y coordinate is the
# intersection of the line between the current x and y coordinate and the center of the control and the circle with radius CONTROL_RADIUS - JOYSTICK_RADIUS
def limit(x, y):
    rise = y - CONTROL_Y
    run = x - CONTROL_X

    sign_x = 1
    if CONTROL_X < 0:
       sign_x = -1

    sign_y = 1
    if CONTROL_Y < 0:
       sign_y = -1

    limit_x = sign_x * ((CONTROL_RADIUS - JOYSTICK_RADIUS) * run) / math.sqrt(math.pow(rise, 2) + math.pow(run, 2))
    limit_y = sign_y * ((CONTROL_RADIUS - JOYSTICK_RADIUS) * rise) / math.sqrt(math.pow(rise, 2) + math.pow(run, 2)) 

    return (limit_x + CONTROL_X, limit_y + CONTROL_Y)

# Open a serial connection to arduino in non-blocking mode
connection = serial.Serial(ARDUINO_SERIAL_PORT, ARDUINO_SERIAL_BAUD_RATE, timeout = 0);

# Position the window
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (WINDOW_X, WINDOW_Y)
 
pygame.init()
 
# Set the height and width of the screen
size = [800, 600]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Syma S107 Virtual Remote Control")
 
#Loop until the user clicks the close button.
done = False
 
# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Set the initial position of the mouse to be on top of our joystick
pygame.mouse.set_pos(CONTROL_X, CONTROL_Y)

# A boolean that states whether the joystick is active. This happens when the user clicks the mousebutton while on the joystick
control_active = False

# These two variables keep track of the position of the joystick. Let's initialize these to be at the center of the control circle
currX = CONTROL_X
currY = CONTROL_Y

# Keeps track of the current throttle value
throttle = 0

# Keeps track of the current trim value
trim = 63

# Keep track of the previous values for these attributes. We want to send data only if something has changed
previousYaw = 0
previousPitch = 0
previousThrottle = 0
previousTrim = 0

# The python script sends 20 packets every second (because we're rendering 20 frames a second). However, the arduino is configured
# to send data to the helicopter at 10 packets every second. Essentially we're sending data twice as fast as the arduino can consume
# it. To prevent loss of data, we do two things. First, we only send data if any of the values (yaw, pitch, throttle, trim) have 
# changed, and second, we only send data if the arduino asks for it. When the arduino is ready to accept data, it sends a byte
# (129) through the serial channel. This lets the python script know that it can send data.
READY_TO_ACCEPT_ACK = 129

# Since we're sending data to the arduino almost twice as fast as it can consume it, let's buffer our data until we actually need
# to send it. To do this, we'll use a queue.
commandValues = deque([]);
 
while done == False:

    draw_background(screen)

    # Get mouse-button states
    (button1, button2, button3) = pygame.mouse.get_pressed();

    # Get keyboard-button states
    states = pygame.key.get_pressed();

    if states[THROTTLE_UP_KEY]:
       throttle += 1
       if throttle > THROTTLE_MAX:
          throttle = THROTTLE_MAX 
   
    if states[THROTTLE_DOWN_KEY]:
       throttle -= 1
       if throttle < THROTTLE_MIN:
          throttle = THROTTLE_MIN

    if states[TRIM_LEFT_KEY]:
       trim += 1
       if trim > TRIM_MAX:
          trim = TRIM_MAX

    if states[TRIM_RIGHT_KEY]:
       trim -= 1
       if trim < TRIM_MIN:
          trim = TRIM_MIN
     
    # Get the current mouse position. This returns the position
    # as a list of two numbers.
    pos = pygame.mouse.get_pos()
     
    # Fetch the x and y out of the list,
    x = pos[0]
    y = pos[1]

    # Calculate the distance between current mouse coordinates and the center of the control circle
    dist = distance(x, y, CONTROL_X, CONTROL_Y)

    # If the control isn't already active and we're clicking the left mouse-button, it means that we
    # might be clicking the joystick
    if not control_active and button1:
       
       # The distance is lesser than the joystick radius, which means that clicking on the joystick
       if dist < JOYSTICK_RADIUS:
           control_active = True

    # Otherwise, it means that the left mouse-button is not clicked, so let's reset the position of the
    # joystick to the center of the control
    elif not button1:
        control_active = False
        (currX, currY) = (CONTROL_X, CONTROL_Y)

    # If the control is active, it means that we are pressing the left mouse-button and moving the joystick.
    if control_active:

        # If the distance between the current mouse coordinates is greater than CONTROL_RADIUS - JOYSTICK_RADIUS
        # it means that we're trying to push the joystick out of the control circle. So let's limit the x and y
        # coordinate
        if dist > (CONTROL_RADIUS - JOYSTICK_RADIUS):
           (currX, currY) = limit(x, y)
        else:
           (currX, currY) = (x, y)

    # Draw control and joystick
    pygame.draw.circle(screen, black, [CONTROL_X, CONTROL_Y], CONTROL_RADIUS, 1)
    pygame.draw.circle(screen, black, [int(round(currX)), int(round(currY))], JOYSTICK_RADIUS, 0)

    # Draw throttle control with throttle level
    pygame.draw.rect(screen, green, [THROTTLE_X + 1, (THROTTLE_Y + THROTTLE_HEIGHT) - throttle - 1, THROTTLE_WIDTH - 2, throttle], 0) 
    pygame.draw.rect(screen, black, [THROTTLE_X, THROTTLE_Y, THROTTLE_WIDTH, THROTTLE_HEIGHT], 1)

    # Draw trim control
    pygame.draw.rect(screen, black, [TRIM_X, TRIM_Y, TRIM_WIDTH, TRIM_HEIGHT], 1)
    pygame.draw.rect(screen, black, [TRIM_X + (TRIM_WIDTH / 2) - (TRIM_MARKER_WIDTH / 2) - trim + TRANSLATION_FACTOR, TRIM_Y - ((TRIM_MARKER_HEIGHT - TRIM_HEIGHT) / 2), TRIM_MARKER_WIDTH, TRIM_MARKER_HEIGHT], 0)

    # Calculate yaw and pitch values
    yaw = int(round(CONTROL_X - currX)) + TRANSLATION_FACTOR
    pitch = TRANSLATION_FACTOR - int(round(CONTROL_Y - currY))

    # Create a font
    font = pygame.font.Font(None, 22)

    # Render the title 
    title = font.render("Syma S107 Virtual Remote Control" , True, black)

    titleRect = title.get_rect();
    titleRect.centerx = TITLE_X 
    titleRect.centery = TITLE_Y

    # Render the yaw, pitch, trottle, and trim values
    values = font.render("Yaw: " + str(yaw) + " Pitch: " + str(pitch) + " Throttle: " + str(throttle) + " Trim: " + str(trim), True, blue);

    valuesRect = values.get_rect();
    valuesRect.centerx = VALUES_X
    valuesRect.centery = VALUES_Y

    # Blit the text
    screen.blit(title, titleRect)
    screen.blit(values, valuesRect)
 
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Write values to arduino only if they have changed
    if(yaw != previousYaw or pitch != previousPitch or throttle != previousThrottle or trim != previousTrim):

        # Queue the data that we have to send
        commandValues.append([yaw, pitch, throttle, trim])
        previousYaw = yaw
        previousPitch = pitch
        previousThrottle = throttle
        previousTrim = trim
 
    print "Pending packets: " + str(len(commandValues))
   
    data = connection.read(1)

    if data: 
       data = struct.unpack('B', data)[0]

    if data == READY_TO_ACCEPT_ACK and len(commandValues) > 0:

        # The arduino is ready to accept our data so let's dequeue and send it
        print "Received ACK. Sending data. Pending packets: " + str(len(commandValues))
        commandValue = commandValues.popleft();
        connection.write(chr(commandValue[YAW]));
        connection.write(chr(commandValue[PITCH]));
        connection.write(chr(commandValue[THROTTLE]));
        connection.write(chr(commandValue[TRIM]));

    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close let's send data to shut down the helicopter
            connection.write(chr(ZERO_YAW));
            connection.write(chr(ZERO_PITCH));
            connection.write(chr(ZERO_THROTTLE));
            connection.write(chr(ZERO_TRIM));
            done = True # Flag that we are done so we exit this loop

    # Limit to 20 frames per second
    clock.tick(20)
 
pygame.quit ()


