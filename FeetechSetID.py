from scservo_sdk import *
import subprocess

def change_id(id, new_id):
    packetHandler.unLockEprom(id)
    packetHandler.set_servo_id(id, new_id)
    packetHandler.LockEprom(new_id)

def scan_servo(min_id, max_id):
    connected_ids = []
    for id in range(min_id, max_id):
        data_read, result, error = packetHandler.read1ByteTxRx(id, SMS_STS_ID)
        if result == COMM_SUCCESS:
            connected_ids.append(id)
    return connected_ids

def get_port():
    try:
        # Run ls /dev/tty.usb* and capture output
        result = subprocess.run(['ls /dev/tty.usb*'], 
                              capture_output=True, 
                              text=True, 
                              shell=True)
        
        if result.returncode == 0 and result.stdout.strip():
            # Get the first line of output
            first_port = result.stdout.strip().split('\n')[0]
            print(f"Found USB port: {first_port}")
            return first_port
        else:
            # Fallback to default port if no USB devices found
            print("No USB devices found, using default port")
            exit(-1)
    except Exception as e:
        print(f"Error finding USB port: {e}")
        exit(-1)

PORT = get_port()
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

connected_ids = scan_servo(1, 20)

print("Before setting ID:")
print(connected_ids)
connected_ids.reverse()


if len(connected_ids) != 6 and 1 in connected_ids:
    # not 6 servo, increase the ID by 1
    for id in connected_ids:
        change_id(id, id + 1)
    connected_ids = [id + 1 for id in connected_ids]

# 6 servo, reverse the ID list
if len(connected_ids) == 6:
    confirm_reverse = input("Reverse the ID list? (y/n): ")
    if confirm_reverse == "y":
        for id in connected_ids:
            change_id(id, id + 10)
        connected_ids = [id + 10 for id in connected_ids]
        for id in connected_ids:
            change_id(id, 20 - id - 3)
        connected_ids = [20 - id - 3 for id in connected_ids]
    confirm_calibration = input("Set current position as zero position? (y/n): ")
    if confirm_calibration == "y":
        for id in connected_ids:
            packetHandler.set_current_position_as_2048(id)


ids = scan_servo(1, 20)
print("After setting ID:")
print(ids)
