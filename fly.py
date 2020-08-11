import time
import timeit
from signal import signal, SIGINT

import serial
from sys import exit
from tensorflow.keras.models import load_model
import numpy as np
import joblib

from dji_tello import Tello


ser = serial.Serial("/dev/cu.usbserial-AD01TCNO", baudrate=115200, timeout=1)
time.sleep(1)
ser.reset_input_buffer()
ser.reset_output_buffer()
time.sleep(1)
model = load_model("./models/model_full_6_points_more_data.h5")
scaler = joblib.load('./scalers/scaler.gz')
tello = Tello()
IS_FLYING = False
SLOW_MODE = True

def wait_for_input():
    print("Enter command: ")
    command = input()
    return command
    

def run_command(command):
    print(command)

    if command.find('delay') != -1:
        sec = float(command.partition('delay')[2])
        print('delay %s' % sec)
        time.sleep(sec)
        return None
    else:
        tello.send_command(command)

def normalize_data(x):
    x_min = 800
    x_max = 1700
    print(np.min(x), np.max(x))
    x_scaled = (x - x_min) / (x_max - x_min)
    return x_scaled

def get_serial():
    data = ser.readline()
    data = str(data).replace("b'", "").replace("'", "")
    data = data.rstrip("\\r\\n")
    data = data.split(",")

    try:
        if len(data) == 6:
            return [int(data[0]), float(data[1]), float(data[2]), int(data[3]), int(data[4]), int(data[5])]
        else:
            return data
    except ValueError:
        return data

def is_spike_up(threshold_up, threshold_down, x_window):
    spike = [None, None]

    for i in range(len(x_window) - 1):
        if x_window[i + 1] > x_window[i] * threshold_up:
            spike[0] = i
            
    if spike[0] != None:
        for i in range(spike[0], len(x_window) - 1):
            if x_window[i + 1] < x_window[i] * threshold_down:
                spike[1] = i
            
    return spike
            
def is_spike_down(threshold_up, threshold_down, x_window):
    spike = [None, None]

    for i in range(len(x_window) - 1):
        if x_window[i + 1] < x_window[i] * threshold_down:
            spike[0] = i
            
    if spike[0] != None:
        for i in range(spike[0], len(x_window) - 1):
            if x_window[i + 1] > x_window[i] * threshold_up:
                spike[1] = i
            
    return spike
        
def get_is_spike(x, x_window):
    threshold_up = 50
    threshold_down = 0.02
    is_spike = False
    spike = [None, None]
    
    spike_up = is_spike_up(threshold_up, threshold_down, x_window)
    spike_down = is_spike_down(threshold_up, threshold_down, x_window)
    
    if spike_up[0] != None and spike_up[1] != None:
        is_spike = True
        spike = spike_up
    elif spike_down[0] != None and spike_down[1] != None:
        is_spike = True
        spike = spike_down
        
    return (is_spike, spike)


def get_filterd_data(x):
    x = np.array(x)
    for i in range(len(x)):
        if i > 1 and i < len(x) - 2:
            for j in range(len(x[i])):
                x_window = x[i - 2 : i + 3][:, j] # need to be odd
                is_spike, _ = get_is_spike(x[i][j], x_window)
                if is_spike:
                    x[i][j] = np.median(x_window)
    return x


def extract_array_by_sensors(x):
    to_return = []
    for i in range(x.shape[2]):
        acc_x = x[:, :, i]
   
        to_return.append(acc_x)

    return np.array(to_return)  

def predict(input_data):
    prediction = model.predict([input_data[0], input_data[1], input_data[2], input_data[3], input_data[4], input_data[5]])
    return (prediction > 0.7, prediction)

def listen_for_glove():
    global IS_FLYING
    last_data = []
    start_zero_data_collection = None
    start_125_data_collection = None
    
    while True:
        data = get_serial()

        if len(last_data) == 0:
            start_zero_data_collection = timeit.default_timer()

        if isinstance(data[0], int) and isinstance(data[1], float) and isinstance(data[2], float):
            last_data.append(data)

        if len(last_data) >= 150:
            stop_data_collection = timeit.default_timer()
            if start_zero_data_collection:
                print("Time full: ", stop_data_collection - start_zero_data_collection)
                start_zero_data_collection = None

            if start_125_data_collection:
                print("Time 125: ", stop_data_collection - start_125_data_collection)
                start_125_data_collection = None                

            if len(last_data) > 150:
                last_data = last_data[:150]

            flex_data = np.array(last_data)
            flex_data = get_filterd_data(flex_data)
            flex_data_scaled = scaler.transform(flex_data)

            flex_data_scaled = flex_data_scaled.reshape(1, flex_data_scaled.shape[0], flex_data_scaled.shape[1])
            flex_data_scaled = extract_array_by_sensors(flex_data_scaled)
            flex_data_scaled = flex_data_scaled.reshape(flex_data_scaled.shape[0], flex_data_scaled.shape[1], flex_data_scaled.shape[2], 1)

            pred, _ = predict(flex_data_scaled)
            action = 0

            if len(np.nonzero(pred[0])[0]) > 0:
                action = np.nonzero(pred[0])[0][0]

            print("Action: ", action)

            if action == 0:
                last_data = last_data[-125:]
                start_125_data_collection = timeit.default_timer()

            elif action == 1:
                last_data = []
                if IS_FLYING:
                    land()
                else: 
                    takeoff()

            elif action == 2:
                last_data = []
                print("FLIPPING LEFT")
                flip_left()

            elif action == 3:
                last_data = []
                print("FLIPPING RIGHT")
                flip_right()

        else:
            if IS_FLYING:
                fly(data)

def fly(data):
    print("fly")
    if isinstance(data[0], int) and isinstance(data[1], float) and isinstance(data[2], float):
        if SLOW_MODE:
            up = int((data[0] - 1000) / 3)
            left = int(data[1])
            front = int(data[2])
        else:
            up = int((data[0] - 1200))
            left = int(data[1])
            front = int(data[2])
        print(str(up), str(left), str(front))
        tello.send_rc_control(-left, front, up, 0)

def land():
    global IS_FLYING
    try:
        tello.send_rc_control(0, 0, 0, 0)
        tello.send_command_with_return("land")
        tello.set_flying(False)
        IS_FLYING = False
    except ValueError:
        print("Error Landing")

def takeoff():
    global IS_FLYING

    try:
        tello.send_command_with_return("takeoff")
        tello.set_flying(True)
        IS_FLYING = True
    except ValueError:
        print("Error Taking off")

def flip_left():
    tello.send_rc_control(0, 0, 0, 0)
    print("Flipping Left !")
    
    try:
        tello.send_command_with_return("flip l")
    except ValueError:
        print(ValueError)
        print("Flip error")

def flip_right():
    tello.send_rc_control(0, 0, 0, 0)
    print("Flipping Right !")
    
    try:
        tello.send_command_with_return("flip r")
    except ValueError:
        print(ValueError)
        print("Flip error")

def exit_and_land(signal_received, frame):
    global IS_FLYING
    tello.send_rc_control(0, 0, 0, 0)
    tello.send_command_with_return("land")
    tello.set_flying(False)
    IS_FLYING = False
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

def start():
    signal(SIGINT, exit_and_land) # ctrl + c

    tello.send_command_with_return("command")
    tello.get_battery()
    time.sleep(1)
    listen_for_glove()

start()
