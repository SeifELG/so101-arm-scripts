"""
Minimal SO-101 Calibration Script

Calibrates SO-101 follower or leader arm by recording homing offsets and range of motion.
Replicates exactly what LeRobot's calibrate() does.

Only requires: pip install pyserial feetech-servo-sdk

Usage:
    python minimal_calibrate.py COM9 follower my_follower_arm
    python minimal_calibrate.py COM4 leader my_leader_arm
"""

import json
import os
import sys
import time
import msvcrt  # Windows-specific for detecting key press

try:
    import scservo_sdk as scs
except ImportError:
    print("Error: feetech-servo-sdk not installed!")
    print("Run: pip install feetech-servo-sdk pyserial")
    sys.exit(1)


# === Motor Configuration ===
MOTORS = {
    "shoulder_pan":  {"id": 1},
    "shoulder_lift": {"id": 2},
    "elbow_flex":    {"id": 3},
    "wrist_flex":    {"id": 4},
    "wrist_roll":    {"id": 5},
    "gripper":       {"id": 6},
}

# === Feetech STS3215 Settings ===
BAUD_RATE = 1_000_000
PROTOCOL_VERSION = 0
MODEL_RESOLUTION = 4096  # 12-bit encoder (0-4095)

# Register addresses
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_HOMING_OFFSET = 31
ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11
ADDR_OPERATING_MODE = 33

# Calibration file paths (matches LeRobot structure)
CALIBRATION_BASE_DIR = os.path.expanduser("~/.cache/huggingface/lerobot/calibration")


def get_calibration_path(arm_type, arm_id):
    """Get the calibration file path matching LeRobot's structure."""
    if arm_type == "follower":
        subdir = "robots/so101_follower"
    else:
        subdir = "teleoperators/so101_leader"
    
    cal_dir = os.path.join(CALIBRATION_BASE_DIR, subdir)
    os.makedirs(cal_dir, exist_ok=True)
    return os.path.join(cal_dir, f"{arm_id}.json")


def disable_torque_all(port_handler, packet_handler):
    """Disable torque and unlock EPROM on all motors."""
    print("  Disabling torque on all motors...")
    for name, motor in MOTORS.items():
        motor_id = motor["id"]
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_LOCK, 0)
    time.sleep(0.1)


def set_operating_mode_all(port_handler, packet_handler):
    """Set all motors to position mode."""
    print("  Setting all motors to position mode...")
    for name, motor in MOTORS.items():
        motor_id = motor["id"]
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_OPERATING_MODE, 0)
    time.sleep(0.1)


def read_position(port_handler, packet_handler, motor_id):
    """Read current position from a motor."""
    pos, comm, error = packet_handler.read2ByteTxRx(port_handler, motor_id, ADDR_PRESENT_POSITION)
    if comm != scs.COMM_SUCCESS:
        return None
    return pos


def read_all_positions(port_handler, packet_handler):
    """Read positions from all motors."""
    positions = {}
    for name, motor in MOTORS.items():
        pos = read_position(port_handler, packet_handler, motor["id"])
        if pos is not None:
            positions[name] = pos
    return positions


def write_homing_offset(port_handler, packet_handler, motor_id, offset):
    """Write homing offset to a motor (signed, uses sign-magnitude encoding)."""
    # Feetech uses sign-magnitude encoding for Homing_Offset
    # Bit 11 is sign bit, bits 0-10 are magnitude
    if offset < 0:
        encoded = (1 << 11) | (abs(offset) & 0x7FF)
    else:
        encoded = offset & 0x7FF
    
    packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_HOMING_OFFSET, encoded)


def reset_calibration(port_handler, packet_handler):
    """Reset all motors to factory calibration (no homing offset, full range)."""
    print("  Resetting calibration on all motors...")
    max_pos = MODEL_RESOLUTION - 1  # 4095
    
    for name, motor in MOTORS.items():
        motor_id = motor["id"]
        # Homing_Offset = 0
        write_homing_offset(port_handler, packet_handler, motor_id, 0)
        # Min_Position_Limit = 0
        packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_MIN_POSITION_LIMIT, 0)
        # Max_Position_Limit = 4095
        packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_MAX_POSITION_LIMIT, max_pos)
    time.sleep(0.1)


def calculate_homing_offsets(positions):
    """
    Calculate homing offsets so current position becomes the midpoint (2047).
    
    Formula: homing_offset = current_position - half_turn
    After applying: Present_Position = Actual_Position - Homing_Offset = 2047
    """
    half_turn = (MODEL_RESOLUTION - 1) // 2  # 2047
    offsets = {}
    for name, pos in positions.items():
        offsets[name] = pos - half_turn
    return offsets


def write_homing_offsets(port_handler, packet_handler, offsets):
    """Write homing offsets to all motors."""
    print("  Writing homing offsets to motors...")
    for name, offset in offsets.items():
        motor_id = MOTORS[name]["id"]
        write_homing_offset(port_handler, packet_handler, motor_id, offset)
    time.sleep(0.1)


def key_pressed():
    """Check if a key was pressed (Windows-specific)."""
    return msvcrt.kbhit()


def record_range_of_motion(port_handler, packet_handler):
    """
    Record min/max positions as user moves the arm through its range.
    Returns (mins, maxes) dictionaries.
    """
    print("\n  Recording range of motion...")
    print("  Move each joint through its FULL range of motion.")
    print("  Press ENTER when done.\n")
    
    # Initialize with current positions
    positions = read_all_positions(port_handler, packet_handler)
    mins = positions.copy()
    maxes = positions.copy()
    
    # Print header
    print(f"  {'MOTOR':<15} | {'MIN':>6} | {'POS':>6} | {'MAX':>6}")
    print("  " + "-" * 45)
    
    while True:
        # Read current positions
        positions = read_all_positions(port_handler, packet_handler)
        
        # Update mins and maxes
        for name, pos in positions.items():
            if pos < mins[name]:
                mins[name] = pos
            if pos > maxes[name]:
                maxes[name] = pos
        
        # Display current state
        for name in MOTORS.keys():
            print(f"  {name:<15} | {mins[name]:>6} | {positions[name]:>6} | {maxes[name]:>6}")
        
        # Check for Enter key
        if key_pressed():
            key = msvcrt.getch()
            if key == b'\r' or key == b'\n':  # Enter key
                break
        
        # Move cursor up to overwrite
        print(f"\033[{len(MOTORS)}A", end="")
        time.sleep(0.05)
    
    # Print final values
    print("\n  Final range values:")
    for name in MOTORS.keys():
        print(f"    {name}: {mins[name]} - {maxes[name]}")
    
    # Validate - check that min != max for all motors
    for name in MOTORS.keys():
        if mins[name] == maxes[name]:
            print(f"\n  WARNING: {name} has same min and max! Move it through its range.")
    
    return mins, maxes


def write_position_limits(port_handler, packet_handler, mins, maxes):
    """Write min/max position limits to all motors."""
    print("  Writing position limits to motors...")
    for name in MOTORS.keys():
        motor_id = MOTORS[name]["id"]
        packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_MIN_POSITION_LIMIT, mins[name])
        packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_MAX_POSITION_LIMIT, maxes[name])
    time.sleep(0.1)


def save_calibration(filepath, homing_offsets, mins, maxes):
    """Save calibration to JSON file in LeRobot-compatible format."""
    calibration = {}
    for name, motor in MOTORS.items():
        calibration[name] = {
            "id": motor["id"],
            "drive_mode": 0,
            "homing_offset": homing_offsets[name],
            "range_min": mins[name],
            "range_max": maxes[name],
        }
    
    with open(filepath, "w") as f:
        json.dump(calibration, f, indent=4)
    
    print(f"\n  Calibration saved to: {filepath}")


def verify_motors(port_handler, packet_handler):
    """Verify all motors are connected and responding."""
    print("  Verifying motors...")
    missing = []
    for name, motor in MOTORS.items():
        motor_id = motor["id"]
        model, comm, _ = packet_handler.ping(port_handler, motor_id)
        if comm != scs.COMM_SUCCESS:
            missing.append(f"{name} (ID {motor_id})")
        else:
            print(f"    {name} (ID {motor_id}): OK")
    
    if missing:
        print(f"\n  ERROR: Motors not found: {', '.join(missing)}")
        print("  Make sure all motors are connected and have correct IDs.")
        print("  Run minimal_setup_motors.py first if needed.")
        return False
    
    return True


def main():
    if len(sys.argv) < 4:
        print("Usage: python minimal_calibrate.py <COM_PORT> <ARM_TYPE> <ARM_ID>")
        print("")
        print("Arguments:")
        print("  COM_PORT  - Serial port (e.g., COM9, COM4)")
        print("  ARM_TYPE  - 'follower' or 'leader'")
        print("  ARM_ID    - Unique identifier for this arm (e.g., 'my_follower')")
        print("")
        print("Examples:")
        print("  python minimal_calibrate.py COM9 follower my_follower")
        print("  python minimal_calibrate.py COM4 leader my_leader")
        sys.exit(1)
    
    port_name = sys.argv[1]
    arm_type = sys.argv[2].lower()
    arm_id = sys.argv[3]
    
    if arm_type not in ["follower", "leader"]:
        print(f"Error: ARM_TYPE must be 'follower' or 'leader', got '{arm_type}'")
        sys.exit(1)
    
    calibration_path = get_calibration_path(arm_type, arm_id)
    
    print("\n" + "="*60)
    print("   SO-101 Minimal Calibration")
    print("="*60)
    print(f"  Port: {port_name}")
    print(f"  Arm Type: {arm_type.upper()}")
    print(f"  Arm ID: {arm_id}")
    print(f"  Calibration file: {calibration_path}")
    print("="*60)
    
    # Initialize Feetech SDK
    port_handler = scs.PortHandler(port_name)
    packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
    
    # Open port
    if not port_handler.openPort():
        print(f"Error: Could not open port {port_name}")
        sys.exit(1)
    
    port_handler.setBaudRate(BAUD_RATE)
    print(f"\n  Port {port_name} opened at {BAUD_RATE} bps.")
    
    try:
        # Verify all motors connected
        if not verify_motors(port_handler, packet_handler):
            sys.exit(1)
        
        # Step 1: Disable torque
        disable_torque_all(port_handler, packet_handler)
        
        # Step 2: Set operating mode
        set_operating_mode_all(port_handler, packet_handler)
        
        # Step 3: Reset calibration
        reset_calibration(port_handler, packet_handler)
        
        # Step 4: Get homing positions
        print("\n" + "="*60)
        print("  STEP 1: Set Middle Position")
        print("="*60)
        input("\n  Move the arm to the MIDDLE of its range of motion.\n  Press ENTER when ready...")
        
        positions = read_all_positions(port_handler, packet_handler)
        print("\n  Current positions:")
        for name, pos in positions.items():
            print(f"    {name}: {pos}")
        
        # Calculate and write homing offsets
        homing_offsets = calculate_homing_offsets(positions)
        write_homing_offsets(port_handler, packet_handler, homing_offsets)
        
        print("\n  Homing offsets set:")
        for name, offset in homing_offsets.items():
            print(f"    {name}: {offset}")
        
        # Step 5: Record range of motion
        print("\n" + "="*60)
        print("  STEP 2: Record Range of Motion")
        print("="*60)
        
        mins, maxes = record_range_of_motion(port_handler, packet_handler)
        
        # Step 6: Write position limits
        write_position_limits(port_handler, packet_handler, mins, maxes)
        
        # Step 7: Save calibration file
        save_calibration(calibration_path, homing_offsets, mins, maxes)
        
        print("\n" + "="*60)
        print("   Calibration Complete!")
        print("="*60)
        print(f"\n  Calibration file saved to:\n  {calibration_path}")
        print("\n  You can now use the arm with the teleoperate script.")
        
    except KeyboardInterrupt:
        print("\n\nCalibration cancelled by user.")
    finally:
        port_handler.closePort()
        print("\nPort closed.")


if __name__ == "__main__":
    main()

