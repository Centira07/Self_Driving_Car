import cv2
import io
import socket
import time
import json
import serial
import time
from PIL import Image



def send_frame(socket, frame):
    pil_img = Image.fromarray(frame)
    buf = io.BytesIO()
    pil_img.save(buf, format='JPEG')
    jpeg_frame = buf.getvalue()

    frame_size = len(jpeg_frame)
    socket.sendall(frame_size.to_bytes(4, 'big'))
    socket.sendall(jpeg_frame)

def receive_detection_data(socket):
    buffer = ""
    while True:
        data = socket.recv(1024).decode()  # Receive data from the socket as a string
        if not data:
            break  # No more data is available
        buffer += data
        while '\n' in buffer:  # Check if there's at least one complete message in the buffer
            message, buffer = buffer.split('\n', 1)  # Split the buffer at the first newline character
            return message  # Return the complete message
    return None

HOST = '10.42.0.151'
PORT = 12345
serial_port = "/dev/ttyACM0"
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate)
time.sleep(1)

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 5)

time.sleep(0.1)

frame_count = 0
start_time = time.time()
try:
    while True:
        ret, frame = camera.read()
        if not ret:
            break
        
        # Adjust color balance by modifying RGB channels
        #frame[:,:,0] = cv2.multiply(frame[:,:,0], 1.2) # Increase blue channel
        #frame[:,:,2] = cv2.multiply(frame[:,:,2], 0.8) # Decrease red channel

        send_frame(client_socket, frame)

        try:
            detection_info = receive_detection_data(client_socket)
            if detection_info:
                print("Detection Info:", detection_info)
                ser.write((detection_info + "\n").encode())  # Send the detection info to the serial port
                with open('detection_info.txt', 'a') as file:
                    file.write(detection_info + '\n')  # Write the string directly
        except Exception as e:
            print(f"Error receiving detection data: {e}")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_count += 1
        elapsed_time = time.time() - start_time
        target_time = frame_count / 5
        sleep_time = target_time - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    cv2.destroyAllWindows()
    camera.release()
    client_socket.close()
