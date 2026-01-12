"""
Simple SO-101 Arm Play Script

Move the arm by hand or control it with keyboard commands.
Only requires: pip install pyserial feetech-servo-sdk

Usage:
    python play_arm.py COM9
"""

import math
import sys
import time
import msvcrt

try:
    import scservo_sdk as scs
except ImportError:
    print("Error: feetech-servo-sdk not installed!")
    print("Run: pip install feetech-servo-sdk pyserial")
    sys.exit(1)


# === Motor Configuration ===
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

# === Feetech Settings ===
BAUD_RATE = 1_000_000
PROTOCOL_VERSION = 0

# Register addresses
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_GOAL_POSITION = 42

# Movement settings
STEP_SIZE = 50  # How much to move per keypress
MIN_POS = 0
MAX_POS = 4095

# Interpolation settings
INTERPOLATION_STEPS = 50      # Number of steps between poses
DEFAULT_DURATION = 1.0        # Default seconds to move between poses
MIN_DURATION = 0.1            # Minimum duration (100ms)
MAX_DURATION = 5.0            # Maximum duration (5 seconds)
DURATION_STEP = 0.1           # How much to change per keypress

# Recording settings
RECORD_INTERVAL = 0.05        # Record position every 50ms


# === Easing Modes ===
EASING_MODES = ["SMOOTH", "SNAP", "GENTLE", "LINEAR", "INSTANT"]

EASING_DESCRIPTIONS = {
    "SMOOTH":  "ease-in-out  (slow → fast → slow)",
    "SNAP":    "ease-in      (slow → SNAP!)",
    "GENTLE":  "ease-out     (fast → gentle stop)",
    "LINEAR":  "constant     (robotic)",
    "INSTANT": "no interp    (jump to pose)",
}


def ease_in_out(t):
    """
    Smootherstep: slow start, fast middle, slow end.
    Most natural and elegant motion.
    Formula: 6t⁵ - 15t⁴ + 10t³
    """
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def ease_in(t):
    """
    Ease-in (quartic): slow start, accelerates, snaps into place.
    Dramatic and punchy.
    Formula: t⁴
    """
    t = max(0.0, min(1.0, t))
    return t * t * t * t


def ease_out(t):
    """
    Ease-out (quartic): fast start, decelerates, gentle landing.
    Soft and graceful.
    Formula: 1 - (1-t)⁴
    """
    t = max(0.0, min(1.0, t))
    return 1.0 - (1.0 - t) ** 4


def linear(t):
    """
    Linear: constant speed, no easing.
    Robotic and mechanical.
    """
    return max(0.0, min(1.0, t))


def apply_easing(t, mode):
    """Apply the selected easing function to t."""
    if mode == "SMOOTH":
        return ease_in_out(t)
    elif mode == "SNAP":
        return ease_in(t)
    elif mode == "GENTLE":
        return ease_out(t)
    elif mode == "LINEAR":
        return linear(t)
    else:  # INSTANT
        return t


def interpolate_positions(start_positions, end_positions, t, easing_mode):
    """
    Interpolate between two poses using the specified easing.
    
    Args:
        start_positions: List of starting positions
        end_positions: List of ending positions
        t: Progress (0.0 = start, 1.0 = end)
        easing_mode: One of EASING_MODES
    
    Returns:
        List of interpolated positions
    """
    eased_t = apply_easing(t, easing_mode)
    
    result = []
    for start, end in zip(start_positions, end_positions):
        pos = start + (end - start) * eased_t
        result.append(int(pos))
    return result


class ArmController:
    def __init__(self, port_name):
        self.port_handler = scs.PortHandler(port_name)
        self.packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
        self.torque_enabled = False
        self.current_motor_idx = 0  # Which motor is selected for keyboard control
        self.saved_poses = []
        self.easing_mode_idx = 0  # Index into EASING_MODES
        self.duration = DEFAULT_DURATION  # Seconds between poses
        self.loop_mode = False  # Loop playback
        self.stop_requested = False  # Flag to stop playback
        self.recording = False  # Recording mode active
        self.recorded_motion = []  # List of (timestamp, positions) tuples
        self.record_start_time = 0  # When recording started
        
    @property
    def easing_mode(self):
        return EASING_MODES[self.easing_mode_idx]
    
    def cycle_easing_mode(self):
        """Cycle to the next easing mode."""
        self.easing_mode_idx = (self.easing_mode_idx + 1) % len(EASING_MODES)
        return self.easing_mode
    
    def check_stop_requested(self):
        """Check if any key was pressed to stop playback."""
        if msvcrt.kbhit():
            msvcrt.getch()  # Consume the key
            self.stop_requested = True
            return True
        return False
    
    def start_recording(self):
        """Start recording motion."""
        self.recording = True
        self.recorded_motion = []
        self.record_start_time = time.time()
        self.disable_torque()  # Must be free to move by hand
        print("  RECORDING... Move the arm! Press [E] to stop")
    
    def stop_recording(self):
        """Stop recording motion."""
        self.recording = False
        duration = time.time() - self.record_start_time
        frames = len(self.recorded_motion)
        print(f"  Recorded {frames} frames over {duration:.1f}s")
    
    def record_frame(self):
        """Record current position with timestamp."""
        if self.recording:
            timestamp = time.time() - self.record_start_time
            positions = self.read_positions()
            self.recorded_motion.append((timestamp, positions))
    
    def play_recorded_motion(self):
        """Play back recorded motion with interpolation between frames."""
        if not self.recorded_motion:
            print("  No recorded motion! Press [E] to record first.")
            return
        
        loop_str = " [LOOP]" if self.loop_mode else ""
        total_duration = self.recorded_motion[-1][0]
        mode = self.easing_mode
        print(f"  Playing recorded motion ({total_duration:.1f}s, {len(self.recorded_motion)} frames, {mode}){loop_str}...")
        print("  Press any key to stop")
        
        self.enable_torque()
        time.sleep(0.3)
        self.stop_requested = False
        
        while True:
            start_time = time.time()
            
            while True:
                if self.check_stop_requested():
                    print("  Stopped!")
                    return
                
                elapsed = time.time() - start_time
                
                # Check if we've finished
                if elapsed >= total_duration:
                    # Ensure we end at final position
                    _, final_pos = self.recorded_motion[-1]
                    self.write_all_positions(final_pos)
                    break
                
                # Find the two frames we're between
                frame_idx = 0
                for i in range(len(self.recorded_motion) - 1):
                    if self.recorded_motion[i + 1][0] > elapsed:
                        frame_idx = i
                        break
                else:
                    frame_idx = len(self.recorded_motion) - 2
                
                # Get the two frames to interpolate between
                t1, pos1 = self.recorded_motion[frame_idx]
                t2, pos2 = self.recorded_motion[frame_idx + 1]
                
                # Calculate interpolation factor (0.0 to 1.0 between these two frames)
                if t2 > t1:
                    t = (elapsed - t1) / (t2 - t1)
                else:
                    t = 0.0
                
                # Apply easing and interpolate
                interp_pos = interpolate_positions(pos1, pos2, t, mode)
                self.write_all_positions(interp_pos)
                
                # Small delay for smooth updates
                time.sleep(0.01)  # 100 Hz update rate
            
            if not self.loop_mode:
                break
            print("  Looping...")
        
        print("  Done!")
        
    def connect(self):
        if not self.port_handler.openPort():
            print(f"Error: Could not open port")
            return False
        self.port_handler.setBaudRate(BAUD_RATE)
        print(f"Connected at {BAUD_RATE} bps")
        return True
    
    def disconnect(self):
        self.disable_torque()
        self.port_handler.closePort()
        print("Disconnected.")
    
    def enable_torque(self):
        """Enable torque - motors will hold position and respond to commands."""
        for motor_id in MOTOR_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
        self.torque_enabled = True
        print("  Torque ENABLED - Motors active")
    
    def disable_torque(self):
        """Disable torque - motors are free to move by hand."""
        for motor_id in MOTOR_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_LOCK, 0)
        self.torque_enabled = False
        print("  Torque DISABLED - Move freely by hand")
    
    def read_positions(self):
        """Read current positions of all motors."""
        positions = []
        for motor_id in MOTOR_IDS:
            pos, comm, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_POSITION
            )
            positions.append(pos if comm == scs.COMM_SUCCESS else 0)
        return positions
    
    def write_position(self, motor_idx, position):
        """Write goal position to a single motor."""
        position = max(MIN_POS, min(MAX_POS, position))  # Clamp
        motor_id = MOTOR_IDS[motor_idx]
        self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, ADDR_GOAL_POSITION, position
        )
    
    def write_all_positions(self, positions):
        """Write goal positions to all motors."""
        for i, pos in enumerate(positions):
            self.write_position(i, pos)
    
    def play_poses_instant(self):
        """Play poses with instant jumps (no interpolation)."""
        loop_str = " [LOOP]" if self.loop_mode else ""
        print(f"\n  Playing poses (INSTANT){loop_str}... Press any key to stop")
        self.enable_torque()
        time.sleep(0.3)
        self.stop_requested = False
        
        while True:
            for i, pose in enumerate(self.saved_poses):
                if self.check_stop_requested():
                    print("\n  Stopped!")
                    return
                print(f"  Pose {i+1}/{len(self.saved_poses)}")
                self.write_all_positions(pose)
                # Check for stop during wait
                for _ in range(10):
                    if self.check_stop_requested():
                        print("\n  Stopped!")
                        return
                    time.sleep(0.1)
            
            if not self.loop_mode:
                break
            print("  Looping...")
        
        print("  Done!")
    
    def play_poses_interpolated(self):
        """Play poses with interpolation using current easing mode."""
        mode = self.easing_mode
        desc = EASING_DESCRIPTIONS[mode]
        loop_str = " [LOOP]" if self.loop_mode else ""
        print(f"\n  Playing ({mode}, {self.duration:.1f}s){loop_str}... Press any key to stop")
        self.enable_torque()
        time.sleep(0.3)
        self.stop_requested = False
        
        # Start from current position
        current = self.read_positions()
        
        while True:
            for i, target_pose in enumerate(self.saved_poses):
                if self.check_stop_requested():
                    print("\n  Stopped!")
                    return
                    
                print(f"  Pose {i+1}/{len(self.saved_poses)} - {mode.lower()}...")
                
                # Interpolate from current to target
                start_time = time.time()
                while True:
                    # Check for stop
                    if self.check_stop_requested():
                        print("\n  Stopped!")
                        return
                    
                    elapsed = time.time() - start_time
                    t = elapsed / self.duration
                    
                    if t >= 1.0:
                        # Ensure we end exactly at target
                        self.write_all_positions(target_pose)
                        break
                    
                    # Calculate interpolated position with easing
                    interp_pos = interpolate_positions(current, target_pose, t, mode)
                    self.write_all_positions(interp_pos)
                    
                    # Small delay for smooth motion
                    time.sleep(self.duration / INTERPOLATION_STEPS)
                
                # Update current position for next interpolation
                current = target_pose.copy()
                
                # Small pause at each pose
                time.sleep(0.05)
            
            if not self.loop_mode:
                break
            print("  Looping...")
        
        print("  Done!")
    
    def play_poses(self):
        """Play poses using current easing mode."""
        if not self.saved_poses:
            print("\n  No poses saved!")
            return
        
        if self.easing_mode == "INSTANT":
            self.play_poses_instant()
        else:
            self.play_poses_interpolated()
    
    def print_positions(self, positions):
        """Pretty print positions. Returns number of lines printed."""
        lines = []
        lines.append("  " + "-" * 55)
        lines.append(f"  {'MOTOR':<15} | {'POSITION':>8} | {'SELECTED'}")
        lines.append("  " + "-" * 55)
        for i, (name, pos) in enumerate(zip(MOTORS, positions)):
            selected = " <--" if i == self.current_motor_idx else ""
            lines.append(f"  {name:<15} | {pos:>8} |{selected}")
        lines.append("  " + "-" * 55)
        
        mode = self.easing_mode
        desc = EASING_DESCRIPTIONS[mode]
        loop_str = "ON " if self.loop_mode else "OFF"
        rec_str = "  *** RECORDING ***" if self.recording else ""
        
        lines.append(f"  Torque: {'ON' if self.torque_enabled else 'OFF':3}  |  Speed: {self.duration:.1f}s  |  Loop: {loop_str}  |  Poses: {len(self.saved_poses)}  |  Recorded: {len(self.recorded_motion)}")
        lines.append(f"  Easing: {mode:8} {desc}{rec_str}")
        
        for line in lines:
            print(line)
        
        return len(lines)


def clear_screen():
    print("\033[2J\033[H", end="")


def print_menu():
    print("\n" + "="*60)
    print("  SO-101 Arm Controller")
    print("="*60)
    print("""
  MODES:
    [F] Free Move     - Disable torque, move by hand
    [T] Toggle Torque
    
  KEYBOARD CONTROL (when torque ON):
    [W/S] Move selected joint +/-
    [A/D] Select previous/next joint
    [1-6] Select joint directly
    
  RECORDING:
    [E] Start/Stop recording motion (records while you move arm)
    [M] Play recorded motion
    [N] Clear recorded motion
    
  POSES:
    [Space] Save current pose
    [P] Play all saved poses (press any key to stop)
    [L] Toggle loop mode
    [I] Cycle easing mode (SMOOTH → SNAP → GENTLE → LINEAR → INSTANT)
    [+/-] Speed up / slow down (change duration)
    [C] Clear saved poses
    
  EASING MODES:
    SMOOTH  - slow → fast → slow  (elegant)
    SNAP    - slow → SNAP!        (punchy)
    GENTLE  - fast → gentle stop  (graceful)
    LINEAR  - constant speed      (robotic)
    INSTANT - jump to pose        (no interpolation)
    
  OTHER:
    [R] Refresh display
    [Q] Quit
""")


def main():
    if len(sys.argv) < 2:
        print("Usage: python play_arm.py <COM_PORT>")
        print("Example: python play_arm.py COM9")
        sys.exit(1)
    
    port_name = sys.argv[1]
    arm = ArmController(port_name)
    
    if not arm.connect():
        sys.exit(1)
    
    # Start with torque disabled
    arm.disable_torque()
    
    clear_screen()
    print_menu()
    
    # Track how many lines the status display takes
    status_lines = 0
    
    try:
        while True:
            # Read and display positions
            positions = arm.read_positions()
            
            # Record if in recording mode
            if arm.recording:
                arm.record_frame()
            
            status_lines = arm.print_positions(positions)
            
            # Status line (no leading \n to avoid accumulation)
            if arm.recording:
                elapsed = time.time() - arm.record_start_time
                print(f"  Recording: {elapsed:.1f}s ({len(arm.recorded_motion)} frames) - Press [E] to stop")
            else:
                print("  Press a key (or wait for refresh)...")
            status_lines += 1
            
            # Wait for keypress with timeout (shorter when recording for smoother capture)
            timeout = RECORD_INTERVAL if arm.recording else 0.2
            start = time.time()
            key = None
            while time.time() - start < timeout:
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8', errors='ignore').lower()
                    break
                time.sleep(0.01)
            
            # Move cursor up to overwrite (exact number of lines we printed)
            print(f"\033[{status_lines}A", end="")
            
            if key is None:
                continue
            
            # Handle key presses
            if key == 'q':
                if arm.recording:
                    arm.stop_recording()
                print("\n\n  Quitting...")
                break
            
            elif key == 'e':
                # Toggle recording
                if arm.recording:
                    arm.stop_recording()
                    time.sleep(0.5)
                else:
                    arm.start_recording()
                    time.sleep(0.3)
            
            elif key == 'm':
                # Play recorded motion
                if arm.recording:
                    arm.stop_recording()
                arm.play_recorded_motion()
                time.sleep(0.5)
            
            elif key == 'n':
                # Clear recorded motion
                arm.recorded_motion = []
                print("  Cleared recorded motion")
                time.sleep(0.5)
            
            elif key == 'f':
                arm.disable_torque()
            
            elif key == 't':
                if arm.recording:
                    print("  Can't enable torque while recording!")
                    time.sleep(0.3)
                elif arm.torque_enabled:
                    arm.disable_torque()
                else:
                    arm.enable_torque()
            
            elif key == 'w' and arm.torque_enabled:
                # Move selected joint positive
                pos = positions[arm.current_motor_idx] + STEP_SIZE
                arm.write_position(arm.current_motor_idx, pos)
            
            elif key == 's' and arm.torque_enabled:
                # Move selected joint negative
                pos = positions[arm.current_motor_idx] - STEP_SIZE
                arm.write_position(arm.current_motor_idx, pos)
            
            elif key == 'a':
                arm.current_motor_idx = (arm.current_motor_idx - 1) % len(MOTORS)
            
            elif key == 'd':
                arm.current_motor_idx = (arm.current_motor_idx + 1) % len(MOTORS)
            
            elif key in '123456':
                arm.current_motor_idx = int(key) - 1
            
            elif key == ' ':
                # Save current pose
                arm.saved_poses.append(positions.copy())
                print(f"  Saved pose #{len(arm.saved_poses)}")
                time.sleep(0.5)
            
            elif key == 'p':
                arm.play_poses()
                time.sleep(0.5)
            
            elif key == 'i':
                # Cycle easing mode
                new_mode = arm.cycle_easing_mode()
                desc = EASING_DESCRIPTIONS[new_mode]
                print(f"  Easing: {new_mode} - {desc}")
                time.sleep(0.5)
            
            elif key == 'l':
                # Toggle loop mode
                arm.loop_mode = not arm.loop_mode
                status = "ON - will repeat" if arm.loop_mode else "OFF - play once"
                print(f"  Loop: {status}")
                time.sleep(0.5)
            
            elif key == '-' or key == '_':
                # Speed up (decrease duration)
                arm.duration = max(MIN_DURATION, arm.duration - DURATION_STEP)
                print(f"  Speed: {arm.duration:.1f}s per pose (faster)")
                time.sleep(0.3)
            
            elif key == '=' or key == '+':
                # Slow down (increase duration)
                arm.duration = min(MAX_DURATION, arm.duration + DURATION_STEP)
                print(f"  Speed: {arm.duration:.1f}s per pose (slower)")
                time.sleep(0.3)
            
            elif key == 'c':
                arm.saved_poses = []
                print("  Cleared all saved poses")
                time.sleep(0.5)
            
            elif key == 'r':
                clear_screen()
                print_menu()
    
    except KeyboardInterrupt:
        print("\n\nInterrupted.")
    
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
