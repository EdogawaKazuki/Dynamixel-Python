import socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('0.0.0.0', 22000))
print(f"UDP receiver started on {'0.0.0.0'}:{22000}")
data, addr = server_socket.recvfrom(1024)  # Adjust buffer size as needed
while True:
    data, xaddr = server_socket.recvfrom(1024)  # Adjust buffer size as needed
    print(data)