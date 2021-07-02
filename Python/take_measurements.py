import serial
import math

# Carlos Murillo
# 400197550
# murillc
# Python 3.7

port = "COM6"

s = serial.Serial(port, 115200)

print("Opening: " + s.name)

# Tells the micro-controller that PC is ready to receive data

s.write(b'1')  # or 0x31

x = s.read()

# While the ToF is being initialized
while x != b'\x1b':
    print(x.decode(), end="")
    x = s.read()

# Take input from micro-controller and turn it into int arrays for use later
data_array = []

# room.xyz was used for actual data acquisition
# changed to prevent accidental overwriting
f = open("room_data.xyz", "w")

# Length between steps in mm
step_length = 200  # mm
# Amount of measurements sensor will take
motor_steps = 64
# Amount of y-z slices sensor will take
steps = 14

step = 0

print("Ready to receive data!")

# Loop through y-z slices
for j in range(steps):
    z_coord = step*step_length
    print("Step: " + str(step))
    print("Step Length: " + str(step_length))
    print("Displacement from start: " + str(z_coord))

    step += 1
    angle = math.pi/2 # start at 90 degrees, for me it's easier to tell when sensor is aligned

    # While the system is collecting data
    for i in range(motor_steps):
        first_char = s.read()

        # If micro-controller says pause measurements:
        if first_char == b'\x06':
            add_first_char = False

            print("Measurements paused by user!")

            x = s.read()
            while x != b'\x1b':
                x = s.read()  # Don't do anything until button is pushed again

            print("Measurements resumed by user!")
        else:
            add_first_char = True

        input_string = ""

        # Read the distance inputted from micro-controller
        x = s.read_until()

        if add_first_char:
            input_string = first_char.decode() + x.decode()
        else:
            input_string = x.decode()

        # print(input_string, end="")

        # Trigonometry
        distance = int(input_string.strip("\n"))
        x_coord = distance * math.cos(angle)
        y_coord = distance * math.sin(angle)
        angle += 2*math.pi/motor_steps

        # Print results to console
        print(str(i) + ": " + str(x_coord) + " " + str(y_coord) + " " + str(z_coord))
        # Write results to file
        f.write(str(x_coord) + " " + str(y_coord) + " " + str(z_coord) + "\n")

f.close()

print("Closing: " + s.name)
s.close()
