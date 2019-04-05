import sensor, image, time
from pyb import Pin, Timer, UART

MAX_ERROR = 60
ERROR_DEAD_BAND = 5
Kp = 1.0
Kd = 0.05
speed = 0
spdiff = 0.20
servoW = 0
D_samples = [0,0,0,0]
# Tracks a black line. Use [(128, 255)] for a tracking a white line.

GRAYSCALE_THRESHOLD = [(75, 255)]



# Each roi is (x, y, w, h). The line detection algorithm will try to find the

# centroid of the largest blob in each roi. The x position of the centroids

# will then be averaged with different weights where the most weight is assigned

# to the roi near the bottom of the image and less to the next roi and so on.

ROIS = [ # [ROI, weight]

        (0, 110, 160, 10),

        (0,  30, 160, 10),

       ]


# Camera setup...

sensor.reset() # Initialize the camera sensor.

sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.

sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.

sensor.skip_frames(time = 2000) # Let new settings take affect.

sensor.set_auto_gain(False) # must be turned off for color tracking

sensor.set_auto_whitebal(False) # must be turned off for color tracking

clock = time.clock() # Tracks FPS.
tim = Timer(4, freq=50) # Frequency in 50Hz
tim2 = Timer(2, freq = 200)
ch1 = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width = 1305)
ch2 = tim2.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent = int(speed))
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)
pin9 = Pin('P9', Pin.OUT_PP, Pin.PULL_NONE)
pin3= Pin('P3', Pin.OUT_PP, Pin.PULL_NONE)
pin8 = Pin('P8', Pin.OUT_PP, Pin.PULL_NONE)
pin1.value(1)
pin9.value(0)
pin3.value(1)
pin8.value(1)
uart = UART(3, 115200)

while(True):

    clock.tick() # Track elapsed milliseconds between snapshots().
    if uart.any() > 0:
        char = uart.read(1)

        if char == b's':
            speed = 0

        elif char == b'r':
            if uart.any() > 1:
                temp = uart.read(2)
                speed = int(temp.decode("utf-8"))
                pin9.value(0)
            uart.write(str(speed))
            uart.write('\n')

        elif char == b'p':
            if uart.any() > 1:
                temp = uart.read(5)
                Kp = float(temp.decode("utf-8"))
            uart.write(str(Kp))
            uart.write('\n')

        elif char == b'd':
            if uart.any() > 1:
                temp = uart.read(5)
                Kd = float(temp.decode("utf-8"))
            uart.write(str(Kd))
            uart.write('\n')

        elif char == b'q':
            if uart.any() > 1:
                temp = uart.read(5)
                spdiff = float(temp.decode("utf-8"))
            uart.write(str(spdiff))
            uart.write('\n')

        if char == b'f':
            uart.write(str(clock.fps()))
            uart.write('\n')




    img = sensor.snapshot() # Take a picture and return the image.


    error = [0, 0, 0]
    Cp = 0
    Dp = 0
    i = 0
    #D_samples[0] = D_samples[1]
    for r in ROIS:

        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.


        if blobs:

            # looking for stop sign
            if len(blobs) == 3:
                print(blobs[0].w(),' ',blobs[1].w(),' ',blobs[2].w())
                if blobs[1].w() > blobs[0].w() - 4 and blobs[1].w() < blobs[0].w() + 4 and blobs[1].w() > blobs[2].w() - 4 and blobs[1].w() < blobs[2].w() + 4:
                    if abs(blobs[2].cx() + blobs[0].cx() - 2 * blobs[1].cx()) < 4:
                        pin9.value(1)
                        speed = 0

            largest_blob = max(blobs, key=lambda b: b.pixels())

            #if it saw a cross, keep error the same as last frame, else update error
            if i == 0 and largest_blob.w() > 25:
                error[i] = D_samples[3]
            else:
                error[i] = largest_blob.cx() - 80

            if i == 0:
                seeTrack = True
            elif i == 1:
                if abs(error[1]) > 35 and largest_blob.w() < 50:
                    seeTurn = True
                else:
                    seeTurn = False
        else:
            if i == 0:
                seeTrack = False
            elif i == 1:
                seeTurn = False
            error[i] = 0
        i = i + 1
    # end for

    # store and shift D_samples
    D_samples[0] = D_samples[1]
    D_samples[1] = D_samples[2]
    D_samples[2] = D_samples[3]
    D_samples[3] = error[0]

    # calculate P term
    if Kp * error[0] >= MAX_ERROR:
        Cp = MAX_ERROR - 10
    elif Kp * error[0] >= ERROR_DEAD_BAND:
        Cp = Kp * error[0] - ERROR_DEAD_BAND
    elif Kp * error[0] <= -MAX_ERROR:
        Cp = -(MAX_ERROR - 10)
    elif Kp * error[0] <= -ERROR_DEAD_BAND:
        Cp = Kp * error[0] + ERROR_DEAD_BAND

    # Calculate D term
    T = 1 / clock.fps()
    Dp = Kp * Kd / (4 * T) * (D_samples[3] + D_samples[2]  - D_samples[1] - D_samples[0])
    if seeTrack == False:
        Dp = 0
    elif abs(Kp * error[0] >= MAX_ERROR):
        Dp = 0

    # If lost track, keep the servo pwm unchange, else update pwm
    if seeTrack == False:
        servoW = servoW
    else:
        servoW = 1305 - (Cp + Dp) * 8

    # shrink servo pwm if out of bound
    if servoW > 1700:
        servoW = 1700
    elif servoW < 910:
        servoW = 910

    # motor speed control
    if seeTrack == False:
        motorSpeed = speed/2
    elif abs(servoW -1305) < 200:
        if seeTurn:
            motorSpeed = speed * 3 / 4
        else:
            motorSpeed = speed
    else:
        motorSpeed = speed - spdiff * speed * (abs(servoW - 1305) - 200) /  80

    if motorSpeed < 0:
        motorSpeed = 0

    # Update servo and motor
    ch1.pulse_width(int(servoW))
    ch2.pulse_width_percent(int(motorSpeed))
    print(clock.fps())
