#Author: Tyler Worley
#Date: 4-17-2024
#Version: 1.0.1-RP

import socket
import cv2
import numpy as np
import io
import json
from PIL import Image
import torch
import torchvision.transforms as transforms
from ultralytics import YOLO  # Ensure this import works with your YOLO version
from ultralytics.utils.plotting import Annotator

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def receive_all(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf

 # Function to detect persons in a frame
def detect_persons(frame):
    # Convert frame to suitable format if necessary
    # Perform prediction
    person_count = 0
    results = model.predict(frame, classes = 0)
    for r in results:
        annotator = Annotator(frame)
        boxes = r.boxes
        for box in boxes:
            if box.cls == 0:  # class ID 0 corresponds to 'person'
                person_count += 1
                b = box.xyxy[0]  # Get box coordinates in (left, top, right, bottom) format
                c = box.cls
                annotator.box_label(b, model.names[int(c)]) 
                # Format the detection info for the person
                dataY = float(b[3]) - float(b[1])
                
                m = 2361.98
                par = -1.08911
                Dist = m * (1/dataY) + par
                # If Distance from car is greater than or equal to 8 
                # then send detection_info. 
                if Dist >= 3:
                    detection_info = f"x1: {b[0]}, y1: {b[1]}, x2: {b[2]}, y2: {b[3]}, Distance: {Dist} ft, Count: {person_count}\n"
                else:
                    detection_info = "Stop\n"

                conn.sendall(detection_info.encode('utf-8'))
                print("This is detection info: ", detection_info)

                
                
                # Send the detection info over the socket
                #conn.sendall((json.dumps(detection_info) + '\n').encode('utf-8'))
                #print("This is detection info: ", detection_info, '\n')
                
                
                
                #width = float(b[2]) - float(b[0])
                #height = float(b[3]) - float(b[1])
                #print("This the person's width: " , width , " and height: ", height)


# Server setup
HOST = '0.0.0.0'
PORT = 12345
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print(f"Server listening on port {PORT}...")

# Load the YOLO model
model = YOLO('yolov8n.pt')


conn, addr = server_socket.accept()
print(f"Connected by {addr}")

try:
    while True:
        
        length = receive_all(conn, 4)
        if length is None:
            print("Failed to receive the image size")
            break
        size = int.from_bytes(length, 'big')

        data = receive_all(conn, size)
        if not data:
            print("Failed to receive the image data")
            break

        image_stream = io.BytesIO(data)
        image = Image.open(image_stream)
        frame = np.array(image)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        persons = detect_persons(frame)

        detection_info = json.dumps(persons)
        conn.sendall((json.dumps(detection_info) + '\n').encode('utf-8'))

        
        cv2.imshow('Received Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    conn.close()
    server_socket.close()
    cv2.destroyAllWindows()
    
    
    
# Need to figure out how to lock on the first person that the camera finds. Even if someone comes in
# front of the camera. This will allow us to tell the car to follow the person that the 1 bounding box is bound to. 

  #y = m1/x +b 
  #y = some feet 10 15 20
  #x = height so y1 - y2.
# m = unknown 
#b = y-inc delta y is at 0  


# For data sent back to the pi. We want people and closes persons x1 y1 x2 y2, and foot information.
