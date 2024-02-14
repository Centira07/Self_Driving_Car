import serial

# Define the serial port and baud rate
ser = serial.Serial("/dev/ttyACM0", 38400)  # You might need to change the port based on your Arduino configuration

try:
    while True:
        # Read data from Arduino
        data = ser.readline().decode().strip()
        
        # Print the received data
        print("Received data from Arduino:", data)

except KeyboardInterrupt:
    # Handle keyboard interrupt
    print("Serial communication terminated.")
    ser.close()