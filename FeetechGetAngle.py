from scservo_sdk import *

PORT = "/dev/tty.usbmodem5AB01811321"
BAUD_RATE = 1000000

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(PORT)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = sms_sts(portHandler)


# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    # getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUD_RATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    # getch()
    quit()
connected = True
groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 4)

connected_ids = []

for id in range(0, 30):
    data_read, result, error = packetHandler.read1ByteTxRx(id, SMS_STS_ID)
    if result == COMM_SUCCESS:
        connected_ids.append(id)
        packetHandler.unLockEprom(id)
    else:
        print(f"Servo {id} is not connected")

print("Before setting ID:")
print(connected_ids)

for id in connected_ids:
    packetHandler.LockEprom(id)

angles = []
while True:
    for id in connected_ids:
        packetHandler.unLockEprom(id)
        angles.append(packetHandler.ReadPos(id))
        packetHandler.LockEprom(id)
        print(id, angles[-1], end="; ")
    print()
    angles = []
    time.sleep(0.1)

