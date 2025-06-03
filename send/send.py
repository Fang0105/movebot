import serial
import time
import math
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-m",
                    "--method",
                    type=str,
                    help="movebot or vamp")

args = parser.parse_args()

if args.method == "movebot":
    print("Using movebot method")
    method = "movebot"
elif args.method == "vamp":
    print("Using vamp method")
    method = "vamp"
else:
    print("Invalid method specified. Use 'movebot' or 'vamp'.")
    exit(1)

# Open serial connection to Arduino
ser = serial.Serial('/dev/serial0', 9600, timeout=1)
time.sleep(2)  # Wait for connection to establish

tt = 1

def send_a_number(number):
    ser.write((number + '\n').encode())

def send_angles(angles):
    global tt
    print(f"----------{tt}----------")
    tt = tt+1
    for num in angles:
        print(num)
        send_a_number(str(num))


f = open(f"../{method}/output/path.txt", mode="r")

for line in f.readlines():
    line = line.strip()
    line = line[1:-1]
    angles = line.split(", ")

    send_angles(angles)
    time.sleep(0.2)

print("send 300!!")
send_a_number('300')