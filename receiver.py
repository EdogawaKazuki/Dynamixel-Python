import socket

# Define the UDP IP and port to listen on
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 5000  # Change this to the desired port number

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the IP and port
sock.bind((UDP_IP, UDP_PORT))

print("UDP server started. Waiting for messages...")

# Continuously listen for incoming messages
while True:
    # Receive message from client
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes

    # Decode the received data assuming it's in UTF-8 format
    message = data.decode("utf-8")
    joints = message.split(',')
    if len(joints) == 3:
        # Print the received message
        print("Received joints from client:", "joint1: ", joints[0], "joint2: ", joints[1], "joint3: ", joints[2])
    elif len(joints) == 6:
        # Print the received message
        print("Received joints from client:", "joint1: ", joints[0], "joint2: ", joints[1], "joint3: ", joints[2], "joint4: ", joints[3], "joint5: ", joints[4], "joint6: ", joints[5])
    else:
        print("String received from client:", message)
