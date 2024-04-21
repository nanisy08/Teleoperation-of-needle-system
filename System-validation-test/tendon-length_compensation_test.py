"""
File    : Server_Tendon-length compensation test code.py
Author  : Sooyeon Kim
Date    : April 01, 2024
Update  : April 08, 2024 -- updated to version 2.0 (bending joint needle)
        : April 18, 2024 -- communication problem solved, marker_tracking ver2.1

Description : This script implements motor control with tendon compensation using marker tracking. It serves as a test code to validate the effectiveness of the tendon compensation algorithm.
            : It includes the following features:
            : - Marker tracking version 2.1 for 4 markers
            : - Data logging at 10Hz with timestamp, image processing data, motor commands, and force sensor readings
            : - Tendon compensation algorithm to adjust motor commands based on needle position
Protocol    : TCP/IP communication with FBGs Interrogator(Client 1) and F/T load cell(Client 2)
              Dynamixel motor control for joint movements
              Image processing for marker tracking
"""

import socket
import struct
import threading
import numpy as np
import cv2
from sklearn.cluster import DBSCAN
import pandas as pd
import time
import os
import sys

import math
from datetime import datetime
import csv

######################################################
###################### SETTING #######################
######################################################
### Saving data
image_folder = "240418 test"
image_count = 1

if not os.path.exists(image_folder):
    os.makedirs(image_folder)

csv_file_path = "C:/Users/222BMG13/Desktop/240418 Data_tissue2.csv"



### Dynamixel motor
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * 



######################################################
##################### VARIABLES ######################
######################################################
### FBGs Interrogator
channel_num = [1, 3] # (I4에서 오픈한 채널) 0~3

### Dynamixel
# Motor setting
DXL1_ID                    = 8   # Steering(Flexion)
DXL2_ID                    = 9   # Yawing
DEVICENAME                 = 'COM6' # ex) Windows: "COM1", Linux: "/dev/ttyUSB0", Mac: "/dev/tty.usbserial-*"
BAUDRATE                   = 1000000

MOVING_SPEED               = 100  # 0~1023 x 0.111 rpm (ex-1023:114 rpm)
                                   # AX-12 : No Load Speed	59 [rev/min] (at 12V)
DXL_MOVING_STATUS_THRESHOLD = 3

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

TORQUE_ENABLE              = 1
TORQUE_DISABLE             = 0

PROTOCOL_VERSION           = 1.0

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4
LEN_MX_MOVING              = 1

# Motor initial position (tendon 보정에 의해 update)
DXL1_INIT = 630
DXL2_INIT = 415
dxl1_offset = DXL1_INIT
dxl2_offset = DXL2_INIT

# Safety range
offset_command_limit = 10     # needle 움직이는 거 보고 control input의 범위 정할 것
steering_command_limit = 45

disk_d = 16 #mm

######################################################
### Data Initialize
dxl_goal_position = 0

clnt_socks = []

received_FBGs_data = b''
received_FT_data = b''

# FBGs variables
id_val = [[channel_num[0],0,1],[channel_num[1],0,1]] # [Channel, Sensor1, Sensor2]
FBGs_val = [[0,0],[0,0]] # 채널 2개, 센서 각각 2개씩
force1, force2, force3, force4 = 0, 0, 0, 0

# F/T load cell variables
status, rdt_sequence, ft_sequence = 0, 0, 0
ft_data = [0, 0, 0, 0, 0, 0]
counts_per_force, counts_per_torque = 1000000, 1000000

# Marker tracking variables
origin_position = [0, 0]
tip1_x, tip1_y, tip2_x, tip2_y, shaft1_x, shaft1_y, shaft2_x, shaft2_y = 0,0,0,0,0,0,0,0

tip_angle_deg = 0
R_curvature, Theta = 0, 0
delta_length, dxl_offset, steering_command = 0, 0, 0


######################################################
################ FUNCTION AND THREAD #################
######################################################
exit_flag = False

######################################################
### Thread for TCP communication with FBGs Interrogator
seq = 0
def receive_FBGs_data():
    global received_FBGs_data, exit_flag
    global force1, force2, force3, force4
    global seq

    while not exit_flag:
        try:
            chunk = clnt_socks[0].recv(FBG_PACKET_SIZE)
            if not chunk:
                # Client connection closed
                raise ConnectionError("Connection closed unexpectedly")
        except Exception as e:
            exit_flag = True
            print(f"Error in receive_data: {e}")
            break

        received_FBGs_data = chunk

        if len(received_FBGs_data) >= FBG_PACKET_SIZE:
            id = struct.unpack('BBB', received_FBGs_data[:3])
            FBGs = struct.unpack('d', received_FBGs_data[3:11])[0]

            if id[0] == id_val[0][0]: # 1st channel
                if id[2] == id_val[0][1]: # 1st sensor
                    force1 = FBGs
                elif id[2] == id_val[0][2]: # 2nd sensor
                    force2 = FBGs
            elif id[0] == id_val[1][0]: # 2nd channel
                if id[2] == id_val[1][1]: # 1st sensor
                    force3 = FBGs
                elif id[2] == id_val[1][2]: # 2nd sensor
                    force4 = FBGs

            seq = id[1] # to cheack the sequence
            ## Print received data
            # print(f"{id[1]}:: Sensor ID: {id[2]}, Channel: {id[0]}, FBGs: {FBGs} nm")

    else:
        clnt_socks[0].close()


######################################################
### Thread for TCP communication with F/T load cell
def receive_FT_data():
    global received_FT_data, exit_flag
    global rdt_sequence, ft_sequence, status, ft_data

    while not exit_flag:
        chunk = clnt_socks[1].recv(FT_PACKET_SIZE)
        if not chunk:  # Client connection failed
            print("[!] No data received.")
            break
        received_FT_data = chunk

        # Encoding
        rdt_sequence, ft_sequence, status = struct.unpack('!III', received_FT_data[:12])
        ft_data = struct.unpack('!iiiiii', received_FT_data[12:36])
        ft_data = list(ft_data)

        for i in range(6):
            if i < 3:
                ft_data[i] = ft_data[i] / counts_per_force  # [N]
            else:
                ft_data[i] = ft_data[i] / counts_per_torque  # [Nmm]

        # print(f"Received Data::{rdt_sequence}\t Fx: {ft_data[0]:.5f} Fy: {ft_data[1]:.5f} Fz: {ft_data[2]:.5f} Tx: {ft_data[3]:.5f} Ty: {ft_data[4]:.5f} Tz: {ft_data[5]:.5f}")

        # Update data while holding the log_lock
        # with log_lock:
        #     rdt_sequence, ft_sequence, status = rdt_sequence, ft_sequence, status
        #     ft_data = ft_data

    else:
        clnt_socks[1].close()


######################################################
### Thread for data logging
log_lock = threading.Lock()
log_freq = 10
def log_data(log_lock, csv_file_path):
    # motor data
    global delta_length, steering_command, dxl_goal_position, dxl_offset
    # marker data
    global tip1_x, tip1_y, tip2_x, tip2_y, shaft1_x, shaft1_y, shaft2_x, shaft2_y, tip_angle_deg
    # FBGs data
    global force1, force2, force3, force4
    # F/T loadcell data
    global rdt_sequence, ft_sequence, status, ft_data


    with open(csv_file_path, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['Init pos of Dynamixel(#1, #2): ', DXL1_INIT, DXL2_INIT])
        csv_writer.writerow(['RDT Sample Rate: 20'])
        csv_writer.writerow(['Force Units: N', 'Torque Units: Nmm'])
        
        csv_writer.writerow(['Time', 'Image Processing', '','','','','','','','','','','', 'Dynamixel','','','FBGs','','FT load cell'])
        csv_writer.writerow(['', 'Tip1', '', 'Tip2', '','Shaft1', '', 'Shaft2', '', 'Tip_angle[deg]', 'R_Curvature', 'θ_Curvature', 'Tendon compenstaion(desired)[mm]','Tendon compenstaion(motor)[mm]','Steering command[-]','Motor command[-]','Sensor 1', 'Sensor 2', 'Fx','Fy','Fz','Tx','Ty','Tz'])


    while not exit_flag:

        start_time = time.time()
        time.sleep(max(0, 1/log_freq - (time.time() - start_time)))

        data_to_write = []
        data_to_write.append([datetime.now(),  tip1_x, tip1_y, tip2_x, tip2_y, shaft1_x, shaft1_y, shaft2_x, shaft2_y, tip_angle_deg, R_curvature, Theta, delta_length, (dxl_offset*300/1023*math.pi/180*disk_d), steering_command, dxl_goal_position, force1, force2, ft_data[0],ft_data[1],ft_data[2],ft_data[3],ft_data[4],ft_data[5]])

        with log_lock:
            with open(csv_file_path, mode='a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerows(data_to_write)

    csv_file.close()
    print("Logging stopped.")


######################################################
### Thread for marker tracking
def marker_tracking():
    global image_count
    global tip1_x, tip1_y, tip2_x, tip2_y, shaft1_x, shaft1_y, shaft2_x, shaft2_y, tip_angle_deg

    marker_tot = 6 # 찍은 마커의 최대 개수
    outlier_threshold = 10 # pixel 단위

    markers = {
    'tip1': np.array([[0, 0]]),
    'tip2': np.array([[0, 0]]),
    'shaft1': np.array([[0, 0]]),
    'shaft2': np.array([[0, 0]])
    }
    colors = {
        'tip1': (0, 255, 255),
        'tip2': (0, 255, 255),
        'shaft1': (255, 255, 255),
        'shaft2': (255, 255, 255)
    }

    while not exit_flag:
        # 웹캠 있을 때
        # suc, frame = cap.read()
        # img = frame.copy()

        # image_path = os.path.join(image_folder, f"image{image_count}.jpg") # saving
        # cv2.imwrite(image_path, frame)
        # image_count += 1

        # 웹캠 없을 때 테스트용
        image_path = "C:/Users/222BMG13/Desktop/Image Processing/test_image.jpg" 
        frame = cv2.imread(image_path)
        
        ##########################################################
        ## Marker detection
        red_channel = cv2.subtract(frame[:, :, 2], cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        _, red_channel = cv2.threshold(cv2.medianBlur(red_channel, 3), 10, 255, cv2.THRESH_BINARY) # 안 잡히면 10보다 낮추기
        contours, _ = cv2.findContours(red_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 빨간 점 marker_tot 초과인 경우 한 번 erode
        if len(contours) > marker_tot: 
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            red_channel = cv2.erode(red_channel, kernel)
            contours, _ = cv2.findContours(red_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        red_markers = []

        # 빨간 점 marker_tot 이하인 경우 두 개의 무게중심 계산 
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            
            for cnt in contours:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    red_markers.append([cx, cy])
                else:
                    red_markers.append([0, 0])

            red_markers.sort(key=lambda x: x[0], reverse=True)

            
            for i, (marker_name, marker) in enumerate(markers.items()):
                if i < len(red_markers):
                    marker = np.append(marker, [red_markers[i]], axis=0)
                    if len(marker) > 2 and np.linalg.norm(marker[-1] - marker[-2]) >= outlier_threshold:
                        marker[-1] = marker[-2]
                else:
                    marker = np.append(marker, [[0, 0]], axis=0)
                markers[marker_name] = marker

        else:
            for marker_name in markers.keys():
                markers[marker_name] = np.append(markers[marker_name], [[0, 0]], axis=0)

        
        ## Filtering (0이 아닌 가장 최근 값으로 대체)
        for marker_name, marker in markers.items():
            if np.any(marker[-1] == 0) and np.any(marker != 0):
                prev_indices = np.nonzero(marker[:, 0])[0]
                if len(prev_indices) > 0:
                    prev_index = prev_indices[-1]
                    prev_marker = marker[prev_index]
                    marker[-1] = prev_marker
                else:
                    marker[-1] = 0


        ##########################################################
        # Marker 표시
        for marker_name, marker in markers.items():
            color = colors[marker_name]
            cv2.circle(img, tuple(np.round(marker[-1]).astype(int)), 5, color, 1, cv2.LINE_AA)

        # Draw grid
        grid_color = (100, 100, 100)  # Color of the grid lines
        spacing = 50  # Spacing between grid lines

        # Draw horizontal&vertical lines
        for i in range(0, img.shape[0], spacing):
            cv2.line(img, (0, i), (img.shape[1], i), grid_color, 1, cv2.LINE_AA)
        for j in range(0, img.shape[1], spacing):
            cv2.line(img, (j, 0), (j, img.shape[0]), grid_color, 1, cv2.LINE_AA)

        # 이미지 띄우기
        cv2.imshow('Needle Tip Guidance', img)


        marker_positions = {}
        for marker_name, marker in markers.items():
            marker_positions[marker_name] = marker[-1] - origin_position

        tip1_x, tip1_y = marker_positions["tip1"]
        tip2_x, tip2_y = marker_positions["tip2"]
        shaft1_x, shaft1_y = marker_positions["shaft1"]
        shaft2_x, shaft2_y = marker_positions["shaft2"]

        tip_angle_deg = calculate_tip_angle(tip1_x, tip1_y, tip2_x, tip2_y, shaft1_x, shaft1_y, shaft2_x, shaft2_y)

        cv2.waitKey(30) # ms (30 fps)


######################################################
### Function for mapping
def mapping(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

### Function for TCP/IP communication
def accept_clients(serv_sock):
    clnt_socks = []
    for i in range(NUM):
        clnt_sock, clnt_addr = serv_sock.accept()
        clnt_socks.append(clnt_sock)
        
        print(f"Connection from {clnt_addr}")

        clnt_num = str(i).encode()
        clnt_sock.send(clnt_num)  # 클라이언트 번호 전송

    return clnt_socks

def open_socket():
    serv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    serv_sock.bind(('0.0.0.0', PORT))
    serv_sock.listen(5)

    print("Server is waiting for client connection...")
    
    clnt_socks = accept_clients(serv_sock)
    
    return serv_sock, clnt_socks

### Function for tendon length compensation
def tendon_compensation(shaft_x, shaft_y):
    global R_curvature, Theta
    needle_diameter = 2.2 #mm (needle 1.27mm, lumen가지 2.2mm)
    
    a = shaft_x # bending joint needle에 한해서...
    if (a > 100):
        b = shaft_y
    else:
        a, b = 0, 0

    delta_length = -needle_diameter/2 * math.asin(2*a*b/(a**2+b**2 + 1e-9))
    R_curvature = -(a**2+b**2)/2/(b+ 1e-50)
    Theta = -math.degrees(math.asin(2*a*b/(a**2+b**2 + 1e-50)))

    return delta_length

### Function for tip angle calculation
def calculate_tip_angle(tip1_x, tip1_y, tip2_x, tip2_y, shaft1_x, shaft1_y, shaft2_x, shaft2_y):
    # 240417 assumption 참고함

    # P1, P2, P3
    P1 = np.array([tip1_x, tip1_y])
    P2 = np.array([tip2_x, tip2_y])
    P3 = np.array([shaft1_x, shaft1_y])
    P4 = np.array([shaft2_x, shaft2_y])
    
    # P1P2, P2P3 vectors
    vector_P1P2 = P2 - P1
    vector_P3P4 = P3 - P4

    # Dot product
    dot_product = np.dot(vector_P1P2, vector_P3P4)
    magnitude_P1P2 = np.linalg.norm(vector_P1P2)
    magnitude_P3P4 = np.linalg.norm(vector_P3P4)

    # Handling division by zero
    if magnitude_P1P2 == 0 or magnitude_P3P4 == 0:
        return None

    angle_rad = np.arccos(dot_product / (magnitude_P1P2 * magnitude_P3P4))

    # Cross product to determine sign
    cross_product = np.cross(vector_P1P2, vector_P3P4)

    # Tip angle
    # Check sign of cross product to adjust angle sign
    if cross_product < 0:
        angle_rad = -angle_rad
    
    # Convert to degrees
    angle_deg = np.degrees(angle_rad)

    return angle_deg



######################################################
#################### CONNCETION ######################
######################################################
print("Loading...")

### Image processing (webcam connection) #############
# cap = cv2.VideoCapture(0)
# 웹캠 없을 때 테스트용
image_path = "C:/Users/222BMG13/Desktop/Image Processing/test_image.jpg" 
img = cv2.imread(image_path)


### TCP/IP communication #############################
PORT = 4578
NUM = 2 # 클라이언트 개수
FT_PACKET_SIZE = 36
FBG_PACKET_SIZE = 11

serv_sock, clnt_socks = open_socket()



######################################################
### Dynamixel ########################################
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite/Read instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port of Dynamixel")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Write moving speed to maximize the response
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))




######################################################
################### CALIBRATION ######################
######################################################
## check the initial marker position

### Thread
marker_thread = threading.Thread(target=marker_tracking)
marker_thread.start()
time.sleep(0.5)
print("\n")

print(f">> Origin({int(origin_position[0])},{int(origin_position[1])}):\tMarker1({int(shaft2_x)},{int(shaft2_y)}) \tMarker2({int(shaft1_x)},{int(shaft1_y)})\tMarker3({int(tip2_x)},{int(tip2_y)})\tMarker4({int(tip1_x)},{int(tip1_y)})")


input("\nPress 'Enter' to set the initial position")
origin_position = (shaft1_x, shaft1_y)
time.sleep(0.2)
print(f">> Origin({int(origin_position[0])},{int(origin_position[1])}):\tMarker1({int(shaft2_x)},{int(shaft2_y)}) \tMarker2({int(shaft1_x)},{int(shaft1_y)})\tMarker3({int(tip2_x)},{int(tip2_y)})\tMarker4({int(tip1_x)},{int(tip1_y)})")


input("\nPress 'Enter'")
input()


######################################################
######################## MAIN ########################
######################################################

### Thread
threads = []

threads.append(threading.Thread(target=receive_FBGs_data))
threads.append(threading.Thread(target=receive_FT_data))
threads.append(threading.Thread(target=log_data, args=(log_lock, csv_file_path)))

for thread in threads:
    thread.start()


count = 0
print("\nPress 'esc' to quit")

## Control Loop
while not exit_flag:
    count += 1

    # Tendon length & offset 보정
    delta_length = tendon_compensation(shaft1_x, shaft1_y)
    delta_length_per_command = disk_d * math.radians(300)/1024 # mm/[-]
    dxl_offset = delta_length/delta_length_per_command
    dxl_offset = max(min(dxl_offset, offset_command_limit), -offset_command_limit)

    # Steering motion
    steering_command = 0 # 나중에 user input으로 대체함
    steering_command = max(min(steering_command, steering_command_limit), -steering_command_limit)
    
    dxl_goal_position = int(dxl_offset + steering_command)
    dxl1_goal_position = DXL1_INIT #+ dxl_goal_position
    dxl2_goal_position = DXL2_INIT #+ dxl_goal_position


    # print(f"{count} \t Origin({int(origin_position[0])},{int(origin_position[1])}):\tMarker({int(marker_x)},{int(marker_y)})\tJoint({int(joint_x)},{int(joint_y)})\tTip({int(tip_x)},{int(tip_y)})")
    # print(tip[-1],"\t",joint[-1],"\t",marker[-1])
    print(f"{count} \t Tip angle: {tip_angle_deg:.2f} || R: {R_curvature:.2f}\t theta: {Theta:.2f} deg")


    # Allocate goal position value into byte array
    param_goal_position1 = [ DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)),
                            DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)),
                            DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)),
                            DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position)) ]
    param_goal_position2 = [ DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)),
                            DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)),
                            DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)),
                            DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position)) ]

    # Add Dynamixel#1,2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addParam failed" % DXL1_ID)
        quit()
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position2)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addParam failed" % DXL2_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
        


# Close
serv_sock.close()
clnt_socks.close()


# cap.release()
cv2.destroyAllWindows()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()

print("Quit")
sys.exit(0)
