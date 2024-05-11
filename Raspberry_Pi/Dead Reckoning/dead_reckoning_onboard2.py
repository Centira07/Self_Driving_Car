import matplotlib.pyplot as plt
import numpy as np
import math
import os
import serial

# Function to handle key press event
def on_key_press(event):
    if event.key:  # If any key is pressed
        save_and_close()  # Call the function to save plot and text file, then close

def save_and_close():
    plt.savefig(plot_image_file_path)
    plt.close(fig)
    output_file.close()
    ser.close()
    print("Output saved, program terminated, and serial port closed.")
    exit()

# Initialize the serial port connection to the Arduino
serial_port = 'COM5'  # Adjust this to your Arduino's serial port
baud_rate = 9600
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Directory path setup
print(f"Current working directory: {os.getcwd()}")
directory_path = '/home/pi/Projects/ECE_Capstone'
if not os.path.exists(directory_path):
    os.makedirs(directory_path)
output_text_file_path = os.path.join(directory_path, 'output.txt')
plot_image_file_path = os.path.join(directory_path, 'trajectory_plot.png')

# File setup
output_file = open(output_text_file_path, 'w')
def print_and_save(text):
    print(text)
    output_file.write(text + '\n')

# Plotting setup
plt.ion()
fig, ax = plt.subplots()
fig.canvas.mpl_connect('key_press_event', on_key_press)
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.plot(0, 0, 'ko')
ax.set_xlabel('X (feet)')
ax.set_ylabel('Y (feet)')
ax.grid(True)
x_data, y_data = [0], [0]

# Vehicle parameters
wheel_radius = 7.28/12
wheel_distance = 2
ticks_per_revolution = 32
circumference = 2 * math.pi * wheel_radius

# Global variables
v_magnitude = 0
v_angle = 0
post_left_counter = post_right_counter = 0
prev_time = 0
init_left_counter = init_right_counter = None

def update_position(left_counter, right_counter, time_diff):
    global x_data, y_data, post_left_counter, post_right_counter, v_magnitude, v_angle, prev_time, init_left_counter, init_right_counter
    
    if init_left_counter is None or init_right_counter is None:
        init_left_counter, init_right_counter = left_counter, right_counter
    
    adjusted_left_counter = left_counter - init_left_counter
    adjusted_right_counter = right_counter - init_right_counter
    
    left_distance = ((adjusted_left_counter - post_left_counter) / ticks_per_revolution) * circumference
    right_distance = ((adjusted_right_counter - post_right_counter) / ticks_per_revolution) * circumference
    turning_angle = (left_distance - right_distance) / wheel_distance
    if turning_angle != 0:
        distance = 2 * ((right_distance / turning_angle) + wheel_distance / 2) * np.sin(turning_angle / 2)
    else:
        distance = left_distance

    v_angle += turning_angle / (2 * np.pi)
    v_angle_for_print = math.degrees(v_angle + np.pi/2)

    post_left_counter = adjusted_left_counter
    post_right_counter = adjusted_right_counter

    v_magnitude = distance / time_diff
    
    x_hat = distance * np.sin(turning_angle)
    y_hat = distance * np.cos(turning_angle)

    current_x = x_data[-1] + x_hat * np.cos(v_angle) - y_hat * np.sin(v_angle)
    current_y = y_data[-1] + y_hat * np.cos(v_angle) + x_hat * np.sin(v_angle)
    x_data.append(current_x)
    y_data.append(current_y)

    ax.clear()
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.plot(0, 0, 'ko')
    ax.plot(x_data, y_data, '-b')
    ax.plot(current_x, current_y, 'ro')
    ax.set_xlabel('X (feet)')
    ax.set_ylabel('Y (feet)')
    ax.grid(True)
    plt.draw()
    plt.pause(0.1)

    output_text = f"Current Position: ({current_x:.2f}, {current_y:.2f}) feet\n" \
                  f"Speed Magnitude: {v_magnitude:.2f} feet/s\n" \
                  f"Velocity Direction: {v_angle_for_print:.2f}° relative to the positive X-axis\n"
    print_and_save(output_text)

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith("Data:"):
            parts = line.replace("Data:", "").strip().split()
            if len(parts) == 3:
                left_counter, right_counter, time_microseconds = int(parts[0]), int(parts[1]), int(parts[2])
                time_seconds = time_microseconds / 1_000_000
                
                if prev_time == 0:  # Initialize prev_time on the first valid data reading
                    prev_time = time_seconds
                    continue
                
                time_diff = time_seconds - prev_time
                update_position(left_counter, right_counter, time_diff)
                prev_time = time_seconds
            else:
                print("Invalid data received:", line)
except KeyboardInterrupt:
    save_and_close()
