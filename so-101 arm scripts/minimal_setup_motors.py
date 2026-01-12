"""
Minimal SO-101 Motor Setup Script

Sets motor IDs, baud rates, and all configuration settings for SO-101 arms.
Replicates exactly what LeRobot's setup_motors and configure do.

Only requires: pip install pyserial feetech-servo-sdk

Usage:
    python minimal_setup_motors.py COM9 follower
    python minimal_setup_motors.py COM4 leader
"""

import sys
import time

try:
    import scservo_sdk as scs
except ImportError:
    print("Error: feetech-servo-sdk not installed!")
    print("Run: pip install feetech-servo-sdk pyserial")
    sys.exit(1)


# === Motor Configuration ===
# Same for both follower and leader arms
MOTORS = [
    {"name": "gripper",       "target_id": 6},
    {"name": "wrist_roll",    "target_id": 5},
    {"name": "wrist_flex",    "target_id": 4},
    {"name": "elbow_flex",    "target_id": 3},
    {"name": "shoulder_lift", "target_id": 2},
    {"name": "shoulder_pan",  "target_id": 1},
]

# === Feetech STS3215 Register Addresses ===
# EPROM (persistent, requires torque off and lock off to write)
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_RETURN_DELAY_TIME = 7
ADDR_P_COEFFICIENT = 21
ADDR_D_COEFFICIENT = 22
ADDR_I_COEFFICIENT = 23
ADDR_OPERATING_MODE = 33

# SRAM (volatile, can write anytime)
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_LOCK = 55

# Factory area
ADDR_MAXIMUM_ACCELERATION = 85

# === Baud Rate Settings ===
BAUD_RATES = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 57_600,
    6: 38_400,
    7: 19_200,
}

TARGET_BAUD_VALUE = 0  # 1,000,000 bps
TARGET_BAUD_RATE = BAUD_RATES[TARGET_BAUD_VALUE]

# === Configuration Values ===
# From FeetechMotorsBus.configure_motors() - applies to BOTH arms
CONFIG_RETURN_DELAY_TIME = 0       # Reduce response delay from 500µs to 2µs
CONFIG_MAXIMUM_ACCELERATION = 254  # Faster acceleration/deceleration
CONFIG_ACCELERATION = 254          # Faster movement response
CONFIG_OPERATING_MODE = 0          # Position servo mode

# From SO101Follower.configure() - applies to FOLLOWER ONLY
CONFIG_P_COEFFICIENT = 16          # Lower than default 32 to avoid shakiness
CONFIG_I_COEFFICIENT = 0           # Default
CONFIG_D_COEFFICIENT = 32          # Default


def scan_for_motor(port_handler, packet_handler):
    """Scan all baud rates and IDs to find a single connected motor."""
    print("  Scanning for motor...")
    
    for baud_value, baud_rate in BAUD_RATES.items():
        port_handler.setBaudRate(baud_rate)
        time.sleep(0.05)
        
        # Scan IDs 0-10 (factory default is usually 1)
        for motor_id in range(11):
            model_number, comm_result, error = packet_handler.ping(port_handler, motor_id)
            if comm_result == scs.COMM_SUCCESS:
                print(f"  Found motor: ID={motor_id}, Model={model_number}, Baud={baud_rate}")
                return motor_id, baud_rate
    
    return None, None


def disable_torque_and_unlock(port_handler, packet_handler, motor_id):
    """Disable torque and unlock EPROM for writing."""
    # Disable torque
    packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.02)
    # Unlock EPROM (required for writing to EPROM registers)
    packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_LOCK, 0)
    time.sleep(0.02)


def write_register(port_handler, packet_handler, motor_id, address, value, length=1):
    """Write a value to a motor register with error checking."""
    if length == 1:
        comm, error = packet_handler.write1ByteTxRx(port_handler, motor_id, address, value)
    elif length == 2:
        comm, error = packet_handler.write2ByteTxRx(port_handler, motor_id, address, value)
    else:
        raise ValueError(f"Unsupported length: {length}")
    
    if comm != scs.COMM_SUCCESS:
        print(f"    Warning: Failed to write {value} to address {address}: {packet_handler.getTxRxResult(comm)}")
        return False
    return True


def configure_motor(port_handler, packet_handler, motor_id, is_follower):
    """
    Apply all configuration settings to a motor.
    Replicates FeetechMotorsBus.configure_motors() and SO101Follower/Leader.configure()
    """
    print("  Applying configuration settings...")
    
    # === Settings from FeetechMotorsBus.configure_motors() ===
    # These apply to BOTH follower and leader
    
    # Return_Delay_Time = 0 (reduce response delay from 500µs to 2µs)
    print(f"    Return_Delay_Time = {CONFIG_RETURN_DELAY_TIME}")
    write_register(port_handler, packet_handler, motor_id, ADDR_RETURN_DELAY_TIME, CONFIG_RETURN_DELAY_TIME)
    
    # Maximum_Acceleration = 254 (faster acceleration)
    print(f"    Maximum_Acceleration = {CONFIG_MAXIMUM_ACCELERATION}")
    write_register(port_handler, packet_handler, motor_id, ADDR_MAXIMUM_ACCELERATION, CONFIG_MAXIMUM_ACCELERATION)
    
    # Acceleration = 254 (faster movement)
    print(f"    Acceleration = {CONFIG_ACCELERATION}")
    write_register(port_handler, packet_handler, motor_id, ADDR_ACCELERATION, CONFIG_ACCELERATION)
    
    # === Settings from configure() method ===
    # Operating_Mode = 0 (position servo mode) - applies to BOTH
    print(f"    Operating_Mode = {CONFIG_OPERATING_MODE} (position mode)")
    write_register(port_handler, packet_handler, motor_id, ADDR_OPERATING_MODE, CONFIG_OPERATING_MODE)
    
    # === Follower-only PID settings ===
    if is_follower:
        # P_Coefficient = 16 (lower than default 32 to avoid shakiness)
        print(f"    P_Coefficient = {CONFIG_P_COEFFICIENT} (follower: reduced for smoothness)")
        write_register(port_handler, packet_handler, motor_id, ADDR_P_COEFFICIENT, CONFIG_P_COEFFICIENT)
        
        # I_Coefficient = 0 (default)
        print(f"    I_Coefficient = {CONFIG_I_COEFFICIENT}")
        write_register(port_handler, packet_handler, motor_id, ADDR_I_COEFFICIENT, CONFIG_I_COEFFICIENT)
        
        # D_Coefficient = 32 (default)
        print(f"    D_Coefficient = {CONFIG_D_COEFFICIENT}")
        write_register(port_handler, packet_handler, motor_id, ADDR_D_COEFFICIENT, CONFIG_D_COEFFICIENT)


def setup_motor(port_handler, packet_handler, motor_name, target_id, is_follower):
    """Setup a single motor with the correct ID, baud rate, and configuration."""
    print(f"\n{'='*60}")
    print(f"  Motor: {motor_name}")
    print(f"  Target ID: {target_id}")
    print(f"  Arm Type: {'FOLLOWER' if is_follower else 'LEADER'}")
    print(f"{'='*60}")
    
    input(f"\n  Connect ONLY the '{motor_name}' motor and press Enter...")
    
    # Find the motor
    current_id, current_baud = scan_for_motor(port_handler, packet_handler)
    
    if current_id is None:
        print("  ERROR: No motor found! Check connection and try again.")
        return False
    
    # Switch to the baud rate where we found the motor
    port_handler.setBaudRate(current_baud)
    time.sleep(0.05)
    
    # Disable torque and unlock EPROM
    print("  Disabling torque and unlocking EPROM...")
    disable_torque_and_unlock(port_handler, packet_handler, current_id)
    
    # Set new ID if needed
    if current_id != target_id:
        print(f"  Setting ID: {current_id} -> {target_id}")
        if not write_register(port_handler, packet_handler, current_id, ADDR_ID, target_id):
            return False
        time.sleep(0.05)
    else:
        print(f"  ID already correct: {target_id}")
    
    # Set baud rate (communicate with new ID now)
    print(f"  Setting Baud_Rate = {TARGET_BAUD_VALUE} ({TARGET_BAUD_RATE} bps)")
    if not write_register(port_handler, packet_handler, target_id, ADDR_BAUD_RATE, TARGET_BAUD_VALUE):
        return False
    time.sleep(0.05)
    
    # Switch to target baud rate for further configuration
    port_handler.setBaudRate(TARGET_BAUD_RATE)
    time.sleep(0.05)
    
    # Verify motor responds at new settings
    model, comm, _ = packet_handler.ping(port_handler, target_id)
    if comm != scs.COMM_SUCCESS:
        print(f"  ERROR: Motor not responding after ID/baud change!")
        return False
    
    # Unlock again at new baud rate (just to be safe)
    disable_torque_and_unlock(port_handler, packet_handler, target_id)
    
    # Apply all configuration settings
    configure_motor(port_handler, packet_handler, target_id, is_follower)
    
    print(f"\n  SUCCESS! Motor '{motor_name}' fully configured as ID {target_id}")
    return True


def main():
    if len(sys.argv) < 3:
        print("Usage: python minimal_setup_motors.py <COM_PORT> <ARM_TYPE>")
        print("")
        print("Arguments:")
        print("  COM_PORT  - Serial port (e.g., COM9, COM4)")
        print("  ARM_TYPE  - 'follower' or 'leader'")
        print("")
        print("Examples:")
        print("  python minimal_setup_motors.py COM9 follower")
        print("  python minimal_setup_motors.py COM4 leader")
        sys.exit(1)
    
    port_name = sys.argv[1]
    arm_type = sys.argv[2].lower()
    
    if arm_type not in ["follower", "leader"]:
        print(f"Error: ARM_TYPE must be 'follower' or 'leader', got '{arm_type}'")
        sys.exit(1)
    
    is_follower = (arm_type == "follower")
    
    print("\n" + "="*60)
    print("   SO-101 Minimal Motor Setup")
    print("="*60)
    print(f"  Port: {port_name}")
    print(f"  Arm Type: {arm_type.upper()}")
    print(f"  Target baud rate: {TARGET_BAUD_RATE} bps")
    print("")
    print("  This script will configure 6 motors one at a time.")
    print("  Connect each motor individually when prompted.")
    print("")
    print("  Settings that will be applied:")
    print("    - ID (1-6 for each motor)")
    print("    - Baud_Rate = 1,000,000 bps")
    print("    - Return_Delay_Time = 0 (fast response)")
    print("    - Maximum_Acceleration = 254")
    print("    - Acceleration = 254")
    print("    - Operating_Mode = 0 (position mode)")
    if is_follower:
        print("    - P_Coefficient = 16 (follower only)")
        print("    - I_Coefficient = 0 (follower only)")
        print("    - D_Coefficient = 32 (follower only)")
    print("="*60)
    
    # Initialize Feetech SDK
    port_handler = scs.PortHandler(port_name)
    packet_handler = scs.PacketHandler(0)  # Protocol version 0 for STS series
    
    # Open port
    if not port_handler.openPort():
        print(f"Error: Could not open port {port_name}")
        sys.exit(1)
    
    print(f"\nPort {port_name} opened successfully.")
    
    try:
        success_count = 0
        for motor in MOTORS:
            if setup_motor(port_handler, packet_handler, motor["name"], motor["target_id"], is_follower):
                success_count += 1
        
        print("\n" + "="*60)
        print(f"   Setup Complete: {success_count}/{len(MOTORS)} motors configured")
        print("="*60)
        
        if success_count == len(MOTORS):
            print("\n  All motors configured successfully!")
            print("  You can now connect all motors together on the bus.")
            print("")
            print("  Next step: Run calibration script")
        else:
            print("\n  Some motors failed. Please retry those individually.")
            
    except KeyboardInterrupt:
        print("\n\nSetup cancelled by user.")
    finally:
        port_handler.closePort()
        print("\nPort closed.")


if __name__ == "__main__":
    main()
