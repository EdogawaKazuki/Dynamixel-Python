import socket
import struct
import threading
import time


def bytes_to_float(byte_array, byte_order='<'):
    # Assuming the byte array represents a 32-bit floating-point number (single precision)
    float_size = 4

    # Ensure the byte array length matches the float size
    if len(byte_array) != float_size:
        raise ValueError(f"Invalid byte array length. Expected {float_size} bytes.")

    # Convert byte array to float
    format_str = f"{byte_order}f"
    return struct.unpack(format_str, byte_array)[0]

class UDPServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_addr = None
        self.is_connected = False
        self.send_thread = None
        self.cmd_callback = None

    def start_server(self):
        self.socket.bind((self.host, self.port))
        print(f"UDP receiver started on {self.host}:{self.port}")
        data, addr = self.socket.recvfrom(1024)  # Adjust buffer size as needed
        while True:
            try:
                data, addr = self.socket.recvfrom(1024)  # Adjust buffer size as needed
                cmd_type = data[0]
                if cmd_type == 0:
                    print("Connect Message: ")
                    cmd_data = data[1]
                    if cmd_data == 1:
                        print("Connect")
                        self.send_thread = threading.Thread(target=self.start_sender)
                        self.send_thread.start()
                        self.is_connected = True
                        self.client_addr = addr
                    else:
                        print("Disconnect")
                        self.is_connected = False
                        self.send_thread.join()
                elif cmd_type == 1:
                    print("Unlock Message: ")
                    cmd_data = data[1]
                    if cmd_data == 0:
                        print("Unlock")
                    else:
                        print("Lock")
                elif cmd_type == 2:
                    print("Angle Command")
                    # print(data)
                    angles = []
                    for i in range(3):
                        # print(data[1 + i * 4: 1 + (i + 1) * 4])
                        angles.append(bytes_to_float(data[1 + i * 4: 1 + (i + 1) * 4], '<'))
                        print(f"Angle {i}: {angles[i]}", end="; ")

                    self.cmd_callback(angles)
                    print()
            except Exception as e:
                print(e)

    def start_sender(self):
        while True:
            if not self.is_connected:
                break
            send_data = b'\x01\x02\x03\x04\x01\x02\x03\x04\x01\x02\x03\x04\x01\x02\x03\x04\x01\x02\x03\x04'
            self.socket.sendto(send_data, self.client_addr)
            time.sleep(0.02)

    def send_data(self, data):
        self.socket.sendto(data, self.client_addr)

    def stop(self):
        self.socket.close()
        print("UDP receiver stopped")


# Usage example
if __name__ == "__main__":
    receiver = UDPServer("localhost", 1234)  # Replace with your desired host and port
    receiver.start_server()
