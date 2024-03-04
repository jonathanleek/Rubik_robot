#!/usr/bin/python
# -*- coding: iso-8859-1 -*-

HOME = "/home/pi/"

from picamera import PiCamera

import RPi.GPIO as GPIO
import os, math

import time

import random

from threading import Thread

# _______________Konstanten________________

GPIO.setmode(GPIO.BOARD)

RIGHT_WRIST = 37  # 26
RIGHT_GRIP = 35  # 19
LEFT_WRIST = 33  # 13
LEFT_GRIP = 31  # 6

C180 = 0  # 1 = 180° turn of gripper; else = 90° (standard)

GRIPPER_MAX = 65
GRIPPER_MIN = 0
TURN_MAX = 270  # max turn angle of servo - limit for setup
TURN_MIN = 0

SLEEP_GRIP = 0.3
SLEEP_LONG_FACTOR = 2

OVERSHOOT = 5  # overshoot at turning

fPWM = 50  # servo PWM frequency; 20 ms duration

TURN_MAX_left_grip = 180
SERVO_PWM_left_grip = 10  # difference of servotiming in percentage of duration
SERVO_OFFSET_left_grip = 2  # lower servo position in percentage of duration

TURN_MAX_left_turn = 180
SERVO_PWM_left_turn = 10  # difference of servotiming in percentage of duration
SERVO_OFFSET_left_turn = 2  # lower servo position in percentage of duration

TURN_MAX_right_grip = 180
SERVO_PWM_right_grip = 10  # difference of servotiming in percentage of duration
SERVO_OFFSET_right_grip = 2  # lower servo position in percentage of duration

TURN_MAX_right_turn = 180
SERVO_PWM_right_turn = 10  # difference of servotiming in percentage of duration
SERVO_OFFSET_right_turn = 2  # lower servo position in percentage of duration

SCRAMBLE_MAX = 20


IMG_WIDTH = 1080
IMG_HEIGHT = 1080

top_row_pxl = 250  # values for cube detection
mid_row_pxl = 500
bot_row_pxl = 750
lft_col_pxl = 200
mid_col_pxl = 450
rgt_col_pxl = 700

wb_row_pxl = 980  # area for white balance
wb_col_pxl = 890

pxl_locs = [
    [
        (lft_col_pxl, top_row_pxl),
        (mid_col_pxl, top_row_pxl),
        (rgt_col_pxl, top_row_pxl),
    ],
    [
        (lft_col_pxl, mid_row_pxl),
        (mid_col_pxl, mid_row_pxl),
        (rgt_col_pxl, mid_row_pxl),
    ],
    [
        (lft_col_pxl, bot_row_pxl),
        (mid_col_pxl, bot_row_pxl),
        (rgt_col_pxl, bot_row_pxl),
    ],
]


TARGET_STANDARD = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB"


# ____________________globale Variablen___________________________________

state_machine = 0  # 0 = Anzeige Start / Solve
# 1 = Anzeige Setup
# 2 = Anzeige Muster
# 3 = Display Scramble
# 10 = left grip einstellen
# 20 = left wrist einstellen
# 30 = right grip einstellen
# 40 = right wrist einstellen
# 50 = Load einstellen
# 60 = SLEEP (delay) einstellen
# 65 = regrip (prior to move a layer)
# 70 = 74 Pattern
# 75 = eigenes Muster
# 76 = eigenes Muster starten
# 77 = eigenes Muster scannen
# 80 = scramble
# 85 = Training   mit Trainindex
# 97 = Display during Training
# 98 = Display during scambling
# 99 = running

timer_time = 0  # actual time

l_pos = 90  # position of left Servo
r_pos = 90

left_grip_tune = 0  # adapted in setup
left_wrist_tune = 0
right_grip_tune = 0
right_wrist_tune = 0

LOAD = 30  # Load position; adapted in setup (cube is released)
SLEEP = 0.5  # Delay for servos; adapted in setup

masterstring = ""  # long string of all servo moves
solve_string = ""  # result from solver
solve_array = []  # array of solver_string moves

Target_string = TARGET_STANDARD
Own_pattern = TARGET_STANDARD

regrip_stat = 1

scramble_count = 0

message = ""

# ____________________________________________________timer__________________


def my_timer():
    global timer_time
    i = 1
    while i == 1:
        timer_time += 0.1
        time.sleep(0.1)


# __________________________________________________________Servos________________________________________________


def setup_servos():
    global left_wrist_servo
    global left_grip_servo
    global right_wrist_servo
    global right_grip_servo

    GPIO.setup(LEFT_WRIST, GPIO.OUT)
    GPIO.setup(LEFT_GRIP, GPIO.OUT)
    GPIO.setup(RIGHT_WRIST, GPIO.OUT)
    GPIO.setup(RIGHT_GRIP, GPIO.OUT)

    left_wrist_servo = GPIO.PWM(LEFT_WRIST, fPWM)
    left_grip_servo = GPIO.PWM(LEFT_GRIP, fPWM)
    right_wrist_servo = GPIO.PWM(RIGHT_WRIST, fPWM)
    right_grip_servo = GPIO.PWM(RIGHT_GRIP, fPWM)

    left_wrist_servo.start(7)
    left_grip_servo.start(5)
    time.sleep(SLEEP)
    right_wrist_servo.start(7)
    right_grip_servo.start(5)
    time.sleep(SLEEP)


def home_servos():
    global l_pos
    global r_pos
    setDirection_left_turn(90 + left_wrist_tune, 1)
    setDirection_right_turn(90 + right_wrist_tune, 1)
    time.sleep(SLEEP)
    regrip()
    setDirection_left_grip(LOAD + left_grip_tune)
    setDirection_right_grip(LOAD + right_grip_tune)
    time.sleep(SLEEP)
    l_pos = 90
    r_pos = 90


def regrip():
    duty = (
        SERVO_PWM_left_grip / TURN_MAX_left_grip * (LOAD + left_grip_tune)
        + SERVO_OFFSET_left_grip
    )
    left_grip_servo.ChangeDutyCycle(duty)
    duty = (
        SERVO_PWM_right_grip / TURN_MAX_right_grip * (LOAD + right_grip_tune)
        + SERVO_OFFSET_right_grip
    )
    right_grip_servo.ChangeDutyCycle(duty)
    time.sleep(SLEEP / 2)
    duty = (
        SERVO_PWM_left_grip / TURN_MAX_left_grip * left_grip_tune
        + SERVO_OFFSET_left_grip
    )
    left_grip_servo.ChangeDutyCycle(duty)
    duty = (
        SERVO_PWM_right_grip / TURN_MAX_right_grip * right_grip_tune
        + SERVO_OFFSET_right_grip
    )
    right_grip_servo.ChangeDutyCycle(duty)
    time.sleep(SLEEP / 2)


def setDirection_left_turn(direction, factor):
    duty = (
        SERVO_PWM_left_turn / TURN_MAX_left_turn * (direction + OVERSHOOT)
        + SERVO_OFFSET_left_turn
    )
    left_wrist_servo.ChangeDutyCycle(duty)
    time.sleep(SLEEP * factor)  # allow to settle


def setDirection_left_grip(direction):
    duty = SERVO_PWM_left_grip / TURN_MAX_left_grip * direction + SERVO_OFFSET_left_grip
    left_grip_servo.ChangeDutyCycle(duty)
    time.sleep(SLEEP_GRIP)  # allow to settle


def setDirection_right_turn(direction, factor):
    duty = (
        SERVO_PWM_right_turn / TURN_MAX_right_turn * (direction + OVERSHOOT)
        + SERVO_OFFSET_right_turn
    )
    right_wrist_servo.ChangeDutyCycle(duty)
    time.sleep(SLEEP * factor)  # allow to settle


def setDirection_right_grip(direction):
    duty = (
        SERVO_PWM_right_grip / TURN_MAX_right_grip * direction + SERVO_OFFSET_right_grip
    )
    right_grip_servo.ChangeDutyCycle(duty)
    time.sleep(SLEEP_GRIP)  # allow to settle


# L: left
# R: right
# M: level wrist
# T: cube wrist
# p: plus (clockwise)
# m: minus

# A: left gripper zu
# a: left gripper auf
# B: right gripper zu
# b: right gripper auf
# M: left 0
# N: left 90
# O: left 180
# X: right 0
# Y: right 90
# Z: right 180
# R: Regrip
# t: Zähler count down

# convert macro moves into single action moves of servos


def LMp():  # wristt level (move)
    if C180 == 1:
        return "OtaNA"
    else:
        return "aMANt"


def LMm():
    return "MtaNA"


def LMpp():
    if C180 == 1:
        return "aMAOtaNA"
    else:
        return "aMANaMANt"


def LTp():
    if C180 == 1:
        return "bOBaNA"
    else:
        return "aMAbNB"


def LTm():
    return "bMBaNA"


def LTpp():
    if C180 == 1:
        return "aMAbOBaNA"
    else:
        return "aMAbNBaMAbNB"


def RMp():
    if C180 == 1:
        return "ZtbYB"
    else:
        return "bXBYt"


def RMm():
    return "XtbYB"


def RMpp():
    if C180 == 1:
        return "bXBZtbYB"
    else:
        return "bXBYbXBYt"


def RTp():
    if C180 == 1:
        return "aZAbYB"
    else:
        return "bXBaYA"


def RTm():
    return "aXAbYB"


def RTpp():
    if C180 == 1:
        return "bXBaZAbYB"
    else:
        return "bXBaYAbXBaYA"


# _______________________________________________________solve_moves______________________________________________


def single_action(action):
    global moves
    global l_pos
    global r_pos
    global regrip_stat
    if action == "A":
        setDirection_left_grip(left_grip_tune)

    elif action == "a":
        setDirection_left_grip(GRIPPER_MAX)

    elif action == "B":
        setDirection_right_grip(right_grip_tune)

    elif action == "b":
        setDirection_right_grip(GRIPPER_MAX)

    elif action == "M":
        if l_pos != 0:
            if l_pos == 90:
                sleep = 1
            else:
                sleep = SLEEP_LONG_factor
            setDirection_left_turn(0 + left_wrist_tune, sleep)
            l_pos = 0

    elif action == "N":
        if l_pos != 90:
            setDirection_left_turn(90 + left_wrist_tune, 1)
            l_pos = 90

    elif action == "O":
        if l_pos != 180:
            if l_pos == 90:
                sleep = 1
            else:
                sleep = SLEEP_LONG_factor
            setDirection_left_turn(180 + left_wrist_tune, sleep)
            l_pos = 180

    elif action == "X":
        if r_pos != 0:
            if r_pos == 90:
                sleep = 1
            else:
                sleep = SLEEP_LONG_factor
            setDirection_right_turn(0 + right_wrist_tune, sleep)
            r_pos = 0

    elif action == "Y":
        if r_pos != 90:
            setDirection_right_turn(90 + right_wrist_tune, 1)
            r_pos = 90

    elif action == "Z":
        if r_pos != 180:
            if r_pos == 90:
                sleep = 1
            else:
                sleep = SLEEP_LONG_factor
            setDirection_right_turn(180 + right_wrist_tune, sleep)
            r_pos = 180

    elif action == "R":
        if regrip_stat == 1:
            regrip()

    elif action == "t":
        moves -= 1


def move_cube(action):
    if action == "U":
        a1 = RTpp()
        a2 = LMp()
        correct_right()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "U2":
        a1 = RTpp()
        a2 = LMpp()
        correct_right()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "U'":
        a1 = RTpp()
        a2 = LMm()
        correct_right()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "R":
        a1 = RMp()
        dummy = "R" + a1
    elif action == "R2":
        a1 = RMpp()
        dummy = "R" + a1
    elif action == "R'":
        a1 = RMm()
        dummy = "R" + a1
    elif action == "L":
        a1 = LTpp()
        a2 = RMp()
        correct_left()
        dummy = a1 + "R" + a2
    elif action == "L2":
        a1 = LTpp()
        a2 = RMpp()
        correct_left()
        dummy = a1 + "R" + a2
    elif action == "L'":
        a1 = LTpp()
        a2 = RMm()
        correct_left()
        dummy = a1 + "R" + a2
    elif action == "F":
        a1 = RTm()
        a2 = LMp()
        correct_right()
        correct_right()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "F2":
        a1 = RTm()
        a2 = LMpp()
        correct_right()
        correct_right()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "F'":
        a1 = RTm()
        a2 = LMm()
        correct_right()
        correct_right()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "B":
        a1 = RTp()
        a2 = LMp()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "B2":
        a1 = RTp()
        a2 = LMpp()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "B'":
        a1 = RTp()
        a2 = LMm()
        correct_right()
        dummy = a1 + "R" + a2
    elif action == "D":
        a1 = LMp()
        dummy = "R" + a1
    elif action == "D2":
        a1 = LMpp()
        dummy = "R" + a1
    elif action == "D'":
        a1 = LMm()
        dummy = "R" + a1
    else:
        print("finished")
        dummy = "finished"
    return dummy


# __________________________________________________________________Wxrfel einlesen_____________________________________


# 0 = U
# 1 = R
# 2 = F
# 3 = D
# 4 = L
# 5 = B


def get_cube():
    regrip()

    camera.capture(HOME + "Cube/face1.jpg")

    #    RTp()

    single_action("b")
    single_action("X")
    single_action("B")
    single_action("a")
    single_action("Y")
    single_action("A")

    regrip()

    camera.capture(HOME + "Cube/face2.jpg")

    #    RTp()
    single_action("b")
    single_action("X")
    single_action("B")
    single_action("a")
    single_action("Y")
    single_action("A")

    regrip()

    camera.capture(HOME + "Cube/face4.jpg")

    #    RTp()
    single_action("b")
    single_action("X")
    single_action("B")
    single_action("a")
    single_action("Y")
    single_action("A")

    regrip()

    camera.capture(HOME + "Cube/face5.jpg")

    #    LTm()

    single_action("b")
    single_action("M")
    single_action("B")
    single_action("a")
    single_action("N")
    single_action("A")

    #    RTp()

    regrip()

    single_action("b")
    single_action("X")
    single_action("B")
    single_action("a")
    single_action("Y")
    single_action("A")

    regrip()

    camera.capture(HOME + "Cube/face3.jpg")

    #    RTpp()

    if C180 == 1:
        single_action("b")
        single_action("X")
        single_action("B")
        single_action("a")
        single_action("Z")
        single_action("A")
        single_action("b")
        single_action("Y")
        single_action("B")

    else:
        single_action("b")
        single_action("X")
        single_action("B")
        single_action("a")
        single_action("Y")
        single_action("A")

        single_action("b")
        single_action("X")
        single_action("B")
        single_action("a")
        single_action("Y")
        single_action("A")

    regrip()

    camera.capture(HOME + "Cube/face0.jpg")


# _______________________________________________________________Image_autokorrektur___________________________________


def pix_average(im, x, y):
    r, g, b = 0, 0, 0
    for i in range(0, 10):
        for j in range(0, 10):
            r1, g1, b1 = im.getpixel((x - 5 + i, y - 5 + j))
            r += r1
            g += g1
            b += b1
    r = r / 100
    g = g / 100
    b = b / 100

    return r, g, b


# ________________________________________________________________Farben_ermitteln________________________________________________________


# 0 = U
# 1 = R
# 2 = F
# 3 = D
# 4 = L
# 5 = B


def get_sticker():
    global solve_string
    col_sticker = []
    for i in range(54):
        col_sticker.append("")

    # read center piece and set as color of the side

    im = Image.open(HOME + "Cube/face1.jpg")
    im = im.convert("RGB")
    base_R_r, base_R_g, base_R_b = pix_average(im, pxl_locs[1][1][0], pxl_locs[1][1][1])
    wb_r, wb_g, wb_b = pix_average(
        im, wb_col_pxl, wb_row_pxl
    )  # manual white ballance correction
    print("%6.2f %6.2f %6.2f   " % (wb_r, wb_g, wb_b) + " R")
    base_R_r = base_R_r / wb_r * 255
    base_R_g = base_R_g / wb_g * 255
    base_R_b = base_R_b / wb_b * 255

    im = Image.open(HOME + "Cube/face5.jpg")
    im = im.convert("RGB")
    base_B_r, base_B_g, base_B_b = pix_average(im, pxl_locs[1][1][0], pxl_locs[1][1][1])
    wb_r, wb_g, wb_b = pix_average(im, wb_col_pxl, wb_row_pxl)
    print("%6.2f %6.2f %6.2f   " % (wb_r, wb_g, wb_b) + " B")
    base_B_r = base_B_r / wb_r * 255
    base_B_g = base_B_g / wb_g * 255
    base_B_b = base_B_b / wb_b * 255

    im = Image.open(HOME + "Cube/face0.jpg")
    im = im.convert("RGB")
    base_U_r, base_U_g, base_U_b = pix_average(im, pxl_locs[1][1][0], pxl_locs[1][1][1])
    wb_r, wb_g, wb_b = pix_average(im, wb_col_pxl, wb_row_pxl)
    print("%6.2f %6.2f %6.2f   " % (wb_r, wb_g, wb_b) + " U")
    base_U_r = base_U_r / wb_r * 255
    base_U_g = base_U_g / wb_g * 255
    base_U_b = base_U_b / wb_b * 255

    im = Image.open(HOME + "Cube/face3.jpg")
    im = im.convert("RGB")
    base_D_r, base_D_g, base_D_b = pix_average(im, pxl_locs[1][1][0], pxl_locs[1][1][1])
    wb_r, wb_g, wb_b = pix_average(im, wb_col_pxl, wb_row_pxl)
    print("%6.2f %6.2f %6.2f   " % (wb_r, wb_g, wb_b) + " D")
    base_D_r = base_D_r / wb_r * 255
    base_D_g = base_D_g / wb_g * 255
    base_D_b = base_D_b / wb_b * 255

    im = Image.open(HOME + "Cube/face2.jpg")
    im = im.convert("RGB")
    base_F_r, base_F_g, base_F_b = pix_average(im, pxl_locs[1][1][0], pxl_locs[1][1][1])
    wb_r, wb_g, wb_b = pix_average(im, wb_col_pxl, wb_row_pxl)
    print("%6.2f %6.2f %6.2f   " % (wb_r, wb_g, wb_b) + " F")
    base_F_r = base_F_r / wb_r * 255
    base_F_g = base_F_g / wb_g * 255
    base_F_b = base_F_b / wb_b * 255

    im = Image.open(HOME + "Cube/face4.jpg")
    im = im.convert("RGB")
    base_L_r, base_L_g, base_L_b = pix_average(im, pxl_locs[1][1][0], pxl_locs[1][1][1])
    wb_r, wb_g, wb_b = pix_average(im, wb_col_pxl, wb_row_pxl)
    print("%6.2f %6.2f %6.2f   " % (wb_r, wb_g, wb_b) + " L")
    base_L_r = base_L_r / wb_r * 255
    base_L_g = base_L_g / wb_g * 255
    base_L_b = base_L_b / wb_b * 255

    masterstring = ""
    for img_iter in range(0, 6):
        img_path = HOME + "Cube/face" + str(img_iter) + ".jpg"
        im = Image.open(img_path)
        im = im.convert("RGB")
        wb_r, wb_g, wb_b = pix_average(im, wb_col_pxl, wb_row_pxl)
        for x_iter in range(0, 3):  # iterate over all nine color locations on each face
            for y_iter in range(0, 3):
                r, g, b = pix_average(
                    im, pxl_locs[y_iter][x_iter][0], pxl_locs[y_iter][x_iter][1]
                )  # get pixel value
                # find euclidian distances
                r = r / wb_r * 255
                g = g / wb_g * 255
                b = b / wb_b * 255

                dist_R = (
                    math.pow(r - base_R_r, 2)
                    + math.pow(g - base_R_g, 2)
                    + math.pow(b - base_R_b, 2)
                )
                dist_B = (
                    math.pow(r - base_B_r, 2)
                    + math.pow(g - base_B_g, 2)
                    + math.pow(b - base_B_b, 2)
                )
                dist_U = (
                    math.pow(r - base_U_r, 2)
                    + math.pow(g - base_U_g, 2)
                    + math.pow(b - base_U_b, 2)
                )
                dist_D = (
                    math.pow(r - base_D_r, 2)
                    + math.pow(g - base_D_g, 2)
                    + math.pow(b - base_D_b, 2)
                )
                dist_F = (
                    math.pow(r - base_F_r, 2)
                    + math.pow(g - base_F_g, 2)
                    + math.pow(b - base_F_b, 2)
                )
                dist_L = (
                    math.pow(r - base_L_r, 2)
                    + math.pow(g - base_L_g, 2)
                    + math.pow(b - base_L_b, 2)
                )
                dist_min = min(
                    dist_R, dist_B, dist_U, dist_D, dist_F, dist_L
                )  # find minimum distance value

                # figure out which color has that minimum value

                if dist_min == dist_R:
                    color = "R"
                elif dist_min == dist_B:
                    color = "B"
                elif dist_min == dist_U:
                    color = "U"
                elif dist_min == dist_D:
                    color = "D"
                elif dist_min == dist_F:
                    color = "F"
                else:
                    color = "L"
                print(
                    "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f  "
                    % (dist_R, dist_B, dist_U, dist_D, dist_F, dist_L)
                    + color
                )
                # set cubie face as that color

                col_sticker[img_iter * 9 + 3 * y_iter + x_iter] = color

    # Korrektur oben      //sticker are not in correct order due to movements at reading the cube
    dummy_1 = col_sticker[0]
    dummy_2 = col_sticker[1]
    col_sticker[0] = col_sticker[6]
    col_sticker[1] = col_sticker[3]
    col_sticker[6] = col_sticker[8]
    col_sticker[3] = col_sticker[7]
    col_sticker[8] = col_sticker[2]
    col_sticker[7] = col_sticker[5]
    col_sticker[2] = dummy_1
    col_sticker[5] = dummy_2

    # Korrektur unten
    dummy_1 = col_sticker[27]
    dummy_2 = col_sticker[28]
    col_sticker[27] = col_sticker[33]
    col_sticker[28] = col_sticker[30]
    col_sticker[33] = col_sticker[35]
    col_sticker[30] = col_sticker[34]
    col_sticker[35] = col_sticker[29]
    col_sticker[34] = col_sticker[32]
    col_sticker[29] = dummy_1
    col_sticker[32] = dummy_2

    print(masterstring)
    for i in range(54):
        masterstring = masterstring + col_sticker[i]

    print(masterstring)
    return masterstring


# ____________________________________________________________________Main_________________________________________________


def setup():
    global left_grip_tune
    global left_wrist_tune
    global right_grip_tune
    global right_wrist_tune
    global state_machine
    global LOAD
    global SLEEP
    global Own_pattern
    global regrip_stat

    setup_servos()

    if not os.path.exists(HOME + "Cube"):
        os.makedirs(HOME + "Cube")
        os.chmod(HOME + "Cube", 0o777)

    if os.path.exists(HOME + "Own_pattern.txt"):
        f = open(HOME + "Own_pattern.txt", "r")
        dummy = f.readline()
        Own_pattern = dummy
        f.close()
    print(Own_pattern)

    if os.path.exists(HOME + "tune_values.txt"):
        f = open(HOME + "tune_values.txt", "r")
        print("tune values found")
        dummy = f.readline()
        left_grip_tune = int(dummy)
        dummy = f.readline()
        left_wrist_tune = int(dummy)
        dummy = f.readline()
        right_grip_tune = int(dummy)
        dummy = f.readline()
        right_wrist_tune = int(dummy)
        dummy = f.readline()
        LOAD = int(dummy)
        dummy = f.readline()
        SLEEP = float(dummy)
        dummy = f.readline()
        regrip_stat = int(dummy)
        f.close()
        return True
    else:
        left_grip_tune = 0
        left_wrist_tune = 0
        right_grip_tune = 0
        right_wrist_tune = 0
        LOAD = 30
        f = open(HOME + "tune_values.txt", "w+")
        f.write(str(left_grip_tune) + "\n")
        f.write(str(left_wrist_tune) + "\n")
        f.write(str(right_grip_tune) + "\n")
        f.write(str(right_wrist_tune) + "\n")
        f.write(str(LOAD) + "\n")
        f.write(str(SLEEP) + "\n")
        f.write(str(regrip_stat) + "\n")
        f.close()
        print("Default gesichert")
        return False  # beim ersten Durchgang werden alle Servos auf 0 gestellt damit die Arme richtig montiert werden kxnnen


# _________________________________________Scan_cube_Own_Pattern________________________________


def scan_cube():
    global message
    message = "Scan"

    get_cube()

    message = "Analysis"

    solution = get_sticker()

    try:
        solve_string = kociemba.solve(
            solution
        )  # only to test if scanned pattern is valid
        f = open(HOME + "Own_pattern.txt", "w+")
        f.write(solution + "\n")
        f.close()
        return solution
    except:
        raise


# ____________________________________________scramble______________________________________________


def scramble():
    global moves
    global solve_array
    global solve_sequenze
    global message
    global now
    #    pos_moves = ["U","U2","U'","F","F2","F'","R","R2","R'","L","L2","L'","B","B2","B'","D","D2","D'"]
    pos_moves = [
        ["U", "U2", "U'"],
        ["F", "F2", "F'"],
        ["R", "R2", "R'"],
        ["L", "L2", "L'"],
        ["B", "B2", "B'"],
        ["D", "D2", "D'"],
    ]
    scramble_string = ""
    layer = 0
    last_layer = 0
    for x in range(scramble_count):
        while layer == last_layer:
            layer = random.randint(0, 5)
        last_layer = layer
        scramble_string = scramble_string + " " + pos_moves[layer][random.randint(0, 2)]

    solve_array = scramble_string.split(" ")

    moves = len(solve_array) - 1

    create_master_string()

    regrip()

    for x in solve_sequenze:
        single_action(x)
        message = "Rest: " + str(moves)
        if state_machine == 0:  # DOUBLECLICK auf rightbutton
            break
    message = "Start"
    now = 0
    home_servos()


# ___________________________________________Training________________________________________
def training():
    global moves
    global solve_array
    global solve_sequenze
    global message
    global now
    global Trainindex

    Trainvalue = 1

    if Trainindex == 0:
        Trainvalue = random.randint(1, train.Trainingliste)
    else:
        Trainvalue = Trainindex

    solve_array = train.Training_pattern[int((Trainvalue - 1) * 2) + 1].split(" ")
    print(train.Training_pattern[int((Trainvalue - 1) * 2)])
    print(solve_array)

    if (
        solve_array[2] == "F" or solve_array[2] == "F'"
    ):  # index 2 wegen Probleme beim Entfernen der unnötigen Würfelwristungen in Trainer.py
        correct_right()
        correct_right()
        correct_right()
    elif solve_array[2] == "U" or solve_array[2] == "U'":
        correct_right()
        correct_right()
    elif solve_array[2] == "B" or solve_array[2] == "B'":
        correct_right()

    elif solve_array[2] == "L" or solve_array[2] == "L'":
        correct_left()

    moves = len(solve_array) - 1

    create_master_string()

    regrip()

    for x in solve_sequenze:
        single_action(x)
        message = "Rest: " + str(moves)
        if state_machine == 0:  # DOUBLECLICK auf rightbutton
            break
    message = "Start"
    now = 0
    home_servos()


# __________________________________________Main_______________________________________________

camera = PiCamera()
camera.resolution = (IMG_WIDTH, IMG_HEIGHT)
camera.exposure_mode = "auto"
camera.start_preview()
time.sleep(2)

setup()

now = 0
move_count = 0

t = Thread(target=Anzeige)
t.start()

s = Thread(target=my_timer)
s.start()

#
isRunning = False


timer_time = 0
message = "Start "

state_machine = 0
endless = 1

# dummy = ""

while endless == 1:
    home_servos()

    solve_sequenze = ""

    while state_machine != 99:  # warten bis "start"
        i = 1

    setDirection_left_grip(left_grip_tune)
    setDirection_right_grip(right_grip_tune)

    now = 0
    isRunning = True
    timer_time = 0
    message = "Scan"

    get_cube()

    message = "Analysis"

    solution = get_sticker()

    #    else:
    try:
        if Target_string == TARGET_STANDARD:
            solve_string = kociemba.solve(solution)
        else:
            solve_string = kociemba.solve(solution, Target_string)
    except:
        message = "Scan Error!"
        state_machine = 0
        print(solve_string)
        continue

    print(Target_string)
    print(solve_string)
    solve_array = solve_string.split(" ")

    correct_left()

    moves = len(solve_array)

    create_master_string()

    for x in solve_sequenze:
        single_action(x)
        message = "Rest: " + str(moves)
        if state_machine == 0:  # DOUBLECLICK auf rightbutton
            message = "Start"
            now = 0
            break

    isRunning = False
    state_machine = 0
    message = "Moves: " + str(len(solve_array))


home_servos()
GPIO.cleanup()
camera.close()
print("all done")
