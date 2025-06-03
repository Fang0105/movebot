# MoveBot vs VAMP

## Requirement
* ARM cpu which supports NEON SIMD

## Install
VAMP requires not only simd but also SIMD Library for Evaluating Elementary Functions (SLEEF)
```bash
sudo apt install libsleef-dev
```

## Setup Arduino
```bash
cd ~
echo 'export PATH=$PATH:$HOME/bin' >> ~/.bashrc
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
source .bashrc
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install Servo
```

## Compile
There is a makefile for each method. Just do
```bash
make
```
and compile.

## RUN
There is a shell script `run.sh` for each method and there are some parameters you can modify.
* `input_dir` : the directory of the input files.
* `output_dir` : the directory of the output files.
* `arm_description_file` : the file that describe the robotic arm
* `start_end_file` : the file include starting configuration and goal configuration
* `path_file` : the file that will record that path
* `obstacles_file` : the file that describe all obstacles
* `max_iterations` : the maximum number of iterations for RRT
* `reach_threshold` : the threshold of goal configuration
* `ExecutionTime_file` : the file that record the execution time
```bash
./run.sh
```

## Input File Format
### arm_description.txt
```
#joints #rods

joint 1 axis minimum_angle maximum_angle
joint 2 axis minimum_angle maximum_angle
joint 3 axis minimum_angle maximum_angle
.
.
.
joint n axis minimum_angle maximum_angle

rod 1 length width height
rod 2 length width height
rod 3 length width height
.
.
.
rod m length width height
```
### movebot/obstacles.txt
```
#obstalces
length width height x y z
length width height x y z
length width height x y z
length width height x y z
.
.
.
```
### vamp/obstacles.txt
```
#obstalces
radius x y z
radius x y z
radius x y z
radius x y z
.
.
.
```
### start_end.txt
```
start_angle_1 start_angle_2 start_angle_3 start_angle_4 start_angle_5 ...
goal_angle_1 goal_angle_2 goal_angle_3 goal_angle_4 goal_angle_5 ...
```
## Receive Path
```bash
cd receive
```
### Compile
```bash
arduino-cli compile --fqbn arduino:avr:nano receive.ino
```
### Upload to board
```bash
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old receive.ino
```



## Send Path
There is a python program, `send/send.py`, helps you send the path to arduino.
To send the path from movebot,
```bash
cd send
python send.py --method=movebot
```
To send the path from vamp
```bash
cd send
python send.py --method=vamp
```


