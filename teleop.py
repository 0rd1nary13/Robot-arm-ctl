import time
from math import pi, sin, cos
from dynamixel_sdk import * 

import lebai_sdk
lebai_sdk.init()

# ----------------------------------------

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

PROTOCOL_VERSION = 2.0

BAUDRATE = 1000000
DEVICENAME = '/dev/cu.usbmodem101'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

DXL_IDS = [1, 2, 3, 4, 5, 6]

LEBAI_IP = "192.168.10.200"

# limits
# servo 1: 0 - 360 deg
# servo 2: 8 - 147 deg
# servo 3: 92 - 280 deg
# servo 4: 69 - 290 deg
# servo 5: -360 - 360 deg
# servo 6: 0 - 93

# ----------------------------------------

def open_port():
    portHandler = PortHandler(DEVICENAME)
    if not portHandler.openPort():
        raise Exception("Failed to open the port")
    if not portHandler.setBaudRate(BAUDRATE):
        raise Exception("Failed to set baudrate")
    return portHandler

def enable_torque(packetHandler, portHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def disable_torque(packetHandler, portHandler, dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

def set_goal_position(packetHandler, portHandler, dxl_id, position):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, position)

def get_present_position(packetHandler, portHandler, dxl_id):
    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    return pos

def pos_to_radians(pos, last_pos, resolution=4096, angle_range=360.0):
    center = (resolution - 1) // 2
    pos = pos-0b11111111111111111111111111111111 if abs(last_pos-pos) > 2147483647 else pos
    return ((pos / center) * (angle_range / 2))*(pi/180)

# ----------------------------------------

def main():
    portHandler = open_port()
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    last_pos = [0]*6
    pos = [0]*6

    lebai = lebai_sdk.connect(LEBAI_IP, False)

    for dxl_id in DXL_IDS:
        disable_torque(packetHandler, portHandler, dxl_id)

    try:
        print("Starting Lebai system...")
        lebai.start_sys()
        time.sleep(2)  # Wait for system to start
        
        print("Ending teach mode (if active)...")
        try:
            lebai.end_teach_mode()
        except:
            pass  # Ignore if not in teach mode
        
        print("Disabling joint limits for teleop...")
        lebai.disable_joint_limits()
        time.sleep(1)
        
        print("Initializing claw...")
        lebai.init_claw()
        
        print("Testing claw full range...")
        # Test full opening
        lebai.set_claw(50, 100)  # Low force, full open
        time.sleep(2)
        # Test full closing  
        lebai.set_claw(50, 0)    # Low force, full close
        time.sleep(2)
        # Set to neutral position
        lebai.set_claw(50, 50)   # Low force, half open
        time.sleep(1)
        
        print("Checking robot state...")
        robot_state = lebai.get_robot_state()
        print(f"Robot state: {robot_state}")
        
        if robot_state != "IDLE":
            print(f"Warning: Robot state is {robot_state}, expected IDLE")
        
        print("Activating robot with initial position...")
        # Move to a neutral position to activate the robot
        initial_pos = [0, -pi/2, pi/2, -pi/2, pi/2, 0]  # Safe neutral position
        lebai.movej(initial_pos, pi, pi/2)  # Low speed for safety
        lebai.wait_move()
        
        print("Starting teleop control loop...")
        print("Move the Dynamixel servos to control the robot arm!")
        print("Press Ctrl+C to stop")

        while True:
            time.sleep(0.05)

            for dxl_id in DXL_IDS:
                pos[dxl_id-1] = pos_to_radians(
                    get_present_position(packetHandler, portHandler, dxl_id),
                    last_pos[dxl_id-1]
                )
                last_pos[dxl_id-1] = pos[dxl_id-1]

            lebai.towardj(
                [ # joint angles (rad)
                    pos[0] - pi, -((3*pi)/4-pos[1]),
                    -(pos[2]-(3*pi)/2), -(pos[3]-pi),
                    pi/2,
                    pos[4]
                ],
                4*pi, # acceleration (rad/s2)
                2*pi # velocity (rad/s)
            )
            
            # Control claw based on servo 6 position with HIGH SENSITIVITY
            # Use only middle 50% of servo range for full claw range (very sensitive)
            servo_normalized = (pos[5] + pi) / (2 * pi)  # Normalize to 0-1
            
            # Map only center 50% of servo (0.25 to 0.75) to full claw range (0-100)
            servo_range = max(0.25, min(1, servo_normalized))  # Use only 25%-75% of servo range
            claw_amplitude = int((1 - (servo_range - 0.25) / 0.5) * 100)  # Map to full 0-100 range
            
            claw_amplitude = max(0, min(100, claw_amplitude))  # Clamp to 0-100 range
            
            # Set claw: force=70 (moderate), amplitude=servo6_position
            lebai.set_claw(100, claw_amplitude)  # (force, amplitude)
            
            # Get claw status for monitoring
            claw_status = lebai.get_claw()
            
            # Debug output showing servo position, claw amplitude, and status
            print(f"Servo6: {pos[5]:.2f}rad -> Claw: {claw_amplitude}% | Force: {claw_status.get('force', 0):.1f} | Hold: {claw_status.get('hold_on', False)}", end="\r")

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        print("Cleaning up...")
        for dxl_id in DXL_IDS:
            disable_torque(packetHandler, portHandler, dxl_id)
        portHandler.closePort()
        
        try:
            lebai.stop_sys()
        except:
            print("Warning: Could not properly shutdown robot")
        print("Cleanup complete.")

if __name__ == "__main__":
    main()