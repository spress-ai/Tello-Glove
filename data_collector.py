from os import path
import time
import timeit

import serial
import pandas as pd

ser = serial.Serial("/dev/cu.usbserial-AD01TCNO", baudrate = 115200, timeout = 1)
time.sleep(1)
ser.flushInput()
ser.flushOutput()
time.sleep(1)
data = []
dataset = None
DATASET_FILE = "left_circle_6_points.csv"

if path.exists(DATASET_FILE):
    dataset = pd.read_csv(DATASET_FILE)
    print(dataset.head())

def get_serial():
    ser_data = ser.readline()
    ser_data = str(ser_data).replace("b'", "").replace("'", "")
    ser_data = ser_data.rstrip("\\r\\n")
    ser_data = ser_data.split(",")

    try:
        if len(ser_data) == 6:
            return [int(ser_data[0]), float(ser_data[1]), float(ser_data[2]), int(ser_data[3]), int(ser_data[4]), int(ser_data[5])]
        else:
            return ser_data
    except ValueError:
        return ser_data


start_data_collection = timeit.default_timer()

while len(data) < 150:
    serial_data = get_serial()
    print(serial_data)

    if len(serial_data) == 6:
        if isinstance(serial_data[0], int) and isinstance(serial_data[1], float) and isinstance(serial_data[2], float):
            print(serial_data)
            data.append(serial_data)
    
stop_data_collection = timeit.default_timer()
print("Time: ", stop_data_collection - start_data_collection)


approved = input("Approve ? Enter (Yes) / C / S")

if approved == "":
    if path.exists(DATASET_FILE):
        print(dataset.head())
        df = pd.DataFrame()
        df["0"] = pd.Series(data)
        dataset = pd.concat([dataset, df], join='inner', axis=1)
        dataset.columns = range(len(dataset.columns))
        dataset = dataset.drop(columns=[0], axis=1)
    else:
        dataset = pd.DataFrame()
        dataset["0"] = pd.Series(data)
    print(dataset.head())
    dataset.to_csv(DATASET_FILE)