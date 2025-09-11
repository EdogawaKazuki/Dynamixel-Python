from scservo_sdk import *

PORT = "COM10"
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

for id in range(1, 10):
    data_read, result, error = packetHandler.read1ByteTxRx(id, SMS_STS_ID)
    if result == COMM_SUCCESS:
        connected_ids.append(id)
        packetHandler.unLockEprom(id)
    else:
        print(f"Servo {id} is not connected")

print("Before setting ID:")
print(connected_ids)
connected_ids.reverse()


if len(connected_ids) != 6:
    # not 6 servo, increase the ID by 1
    for id in connected_ids:
        packetHandler.set_servo_id(id, id + 1)
    connected_ids = [id + 1 for id in connected_ids]

# 6 servo, reverse the ID list
if len(connected_ids) == 6:
    confirm_reverse = input("Reverse the ID list? (y/n): ")
    if confirm_reverse == "y":
        for id in connected_ids:
            packetHandler.set_servo_id(id, id + 10)
        connected_ids = [id + 10 for id in connected_ids]
        for id in connected_ids:
            packetHandler.set_servo_id(id, 20 - id - 2)
        connected_ids = [20 - id - 2 for id in connected_ids]
    confirm_calibration = input("Set current position as zero position? (y/n): ")
    if confirm_calibration == "y":
        for id in connected_ids:
            packetHandler.set_current_position_as_2048(id)

for id in connected_ids:
    packetHandler.LockEprom(id)

connected_ids = []
for id in range(1, 10):
    data_read, result, error = packetHandler.read1ByteTxRx(id, SMS_STS_ID)
    if result == COMM_SUCCESS:
        connected_ids.append(id)
    else:
        print(f"Servo {id} is not connected")

print("After setting ID:")
print(connected_ids)
