import socket
import struct
import threading
import time
import RobotArmClient


class UDPServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_addr = None
        self.is_connected = False
        self.send_thread = None

    def start_server(self):
        self.socket.bind((self.host, self.port))
        print(f"UDP receiver started on {self.host}:{self.port}")
        data, addr = self.socket.recvfrom(1024)  # Adjust buffer size as needed
        while True:
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
                for i in range(3):
                    # print(data[1 + i * 4: 1 + (i + 1) * 4])
                    print(f"Angle {i}: {bytes_to_float(data[1 + i * 4: 1 + (i + 1) * 4], '<')}", end="; ")
                print()

    def start_sender(self):
        while True:
            if not self.is_connected:
                break
            send_data = b'\x01\x02\x03\x04\x01\x02\x03\x04\x01\x02\x03\x04\x01\x02\x03\x04\x01\x02\x03\x04'
            self.socket.sendto(send_data, self.client_addr)
            time.sleep(0.02)

    def stop(self):
        self.socket.close()
        print("UDP receiver stopped")


# Usage example
if __name__ == "__main__":
    receiver = UDPServer("localhost", 1234)  # Replace with your desired host and port
    receiver.start_server()
