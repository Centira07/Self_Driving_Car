import matplotlib.pyplot as plt
import numpy as np
import math
import os

# Print the current working directory
print(f"Current working directory: {os.getcwd()}")

directory_path = 'c:/Users/10275/Projects/ECE_Capstone'
if not os.path.exists(directory_path):
    os.makedirs(directory_path)
output_text_file_path = os.path.join(directory_path, 'output.txt')
plot_image_file_path = os.path.join(directory_path, 'trajectory_plot.png')

# Open a text file for writing the output
output_file = open(output_text_file_path, 'w')

def print_and_save(text):
    print(text)
    output_file.write(text + '\n')

# Initialize plotting parameters and global variables
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.plot(0, 0, 'ko')  # Plot a black dot at the origin
ax.set_xlabel('X (feet)')
ax.set_ylabel('Y (feet)')
ax.grid(True)  # Enable grid
x_data, y_data = [0], [0]  # Initial position

# Car parameters
wheel_radius = 7.28/12  # Wheel radius in feet
wheel_distance = 2  # Distance between wheels in feet
ticks_per_revolution = 32  # Counter ticks per wheel revolution
circumference = 2 * math.pi * wheel_radius  # Wheel circumference

# Global variables
v_magnitude = 0  # Speed magnitude
v_angle = 0  # Velocity direction angle with respect to the positive X-axis, starts facing positive Y-axis
post_left_counter = 0  # Previous left wheel counter value
post_right_counter = 0  # Previous right wheel counter value
prev_time = 0  # Previous time in seconds

def update_position(left_counter, right_counter, time_diff):
    global x_data, y_data, post_left_counter, post_right_counter, v_magnitude, v_angle
    
    # Calculate the distance each wheel has traveled
    left_distance = ((left_counter - post_left_counter) / ticks_per_revolution) * circumference
    right_distance = ((right_counter - post_right_counter) / ticks_per_revolution) * circumference
    #distance = (left_distance + right_distance) / 2  # Average distance traveled
    turning_angle = (left_distance - right_distance) / wheel_distance
    if turning_angle != 0:
        distance = 2*((right_distance/turning_angle)+wheel_distance/2)*np.sin(turning_angle/2)
    else:
        distance = left_distance 

    # Update angle
    v_angle += turning_angle / (2 * np.pi)  
    v_angle_for_print = math.degrees(v_angle + np.pi/2)
    
    # Update the previous counter values
    post_left_counter = left_counter
    post_right_counter = right_counter
    
    # Update the speed magnitude
    v_magnitude = distance / time_diff  # Use the actual time difference
    
    # Calculate position in current frame
    x_hat = distance * np.sin(turning_angle)
    y_hat = distance * np.cos(turning_angle)
    
    # Update position
    current_x = x_data[-1] + x_hat * np.cos(v_angle) - y_hat * np.sin(v_angle)
    current_y = y_data[-1] + y_hat * np.cos(v_angle) + x_hat * np.sin(v_angle)
    x_data.append(current_x)
    y_data.append(current_y)

    # Plotting
    ax.clear()
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.plot(0, 0, 'ko')  # Origin
    ax.plot(x_data, y_data, '-b')  # Track line in blue
    ax.plot(current_x, current_y, 'ro')  # Current position in red
    ax.set_xlabel('X (feet)')
    ax.set_ylabel('Y (feet)')
    ax.grid(True)  # Enable grid
    plt.draw()
    plt.pause(0.1)
    
    # Print current position, speed, and direction
    output_text = f"Current Position: ({current_x:.2f}, {current_y:.2f}) feet\n" \
                  f"Speed Magnitude: {v_magnitude:.2f} feet/s\n" \
                  f"Velocity Direction: {v_angle_for_print:.2f}° relative to the positive X-axis\n"
    print_and_save(output_text)

# Main program to read from a text file
filename = input("Enter the path to the text file: ")

try:
    with open(filename, 'r') as file:
        first_line = True
        for line in file:
            parts = line.split()
            left_counter, right_counter, time_microseconds = int(parts[0]), int(parts[1]), int(parts[2])
            time_seconds = time_microseconds / 1_000_000  # Convert microseconds to seconds
            
            if first_line:
                prev_time = time_seconds
                first_line = False
                continue  # Skip the first line after setting the initial time
            
            time_diff = time_seconds - prev_time  # Calculate time difference between current and previous reading
            update_position(left_counter, right_counter, time_diff)
            prev_time = time_seconds  # Update previous time for the next iteration
except FileNotFoundError:
    print_and_save(f"File {filename} does not exist. Please check the file path and try again.")
except ValueError:
    print_and_save("Invalid data format in file. Each line should contain two positive integers followed by a time in microseconds.")

# Save the plot
plt.savefig(plot_image_file_path)

# Close the plot window and the output file
plt.show(block=True)
#plt.close()
output_file.close()
