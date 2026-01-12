# SO-101 Minimal Scripts

Lightweight Python scripts for setting up and calibrating SO-101 robot arms **without** heavy dependencies like NumPy, PyTorch, or the full LeRobot stack.

## Requirements

Core packages (for motor control):

```bash
pip install pyserial feetech-servo-sdk
```

For talking arm (`talk_arm.py`):

```bash
pip install pyttsx3
```

Works with **any Python 3.x** (including 3.9, 3.14, etc.)


Remember I need to do:
```bash
python -m venv venv
.\venv\Scripts\activate
```

---

## Scripts Overview

| Script | Purpose |
|--------|---------|
| `minimal_setup_motors.py` | Set motor IDs, baud rates, and configuration |
| `minimal_calibrate.py` | Calibrate homing offsets and range of motion |
| `play_arm.py` | Interactive arm control (free move, keyboard, poses) |
| `talk_arm.py` | Make the arm "talk" with jaw sync to speech |

---

## 1. Motor Setup (`minimal_setup_motors.py`)

Sets up each motor with the correct ID, baud rate, and all configuration settings.

### What It Does

For **each motor** (one at a time):
1. Scans all baud rates to find the motor
2. Sets the motor ID (1-6)
3. Sets baud rate to 1,000,000 bps
4. Applies configuration settings:
   - `Return_Delay_Time = 0` (fast response)
   - `Maximum_Acceleration = 254`
   - `Acceleration = 254`
   - `Operating_Mode = 0` (position mode)
   - **Follower only:** PID tuning (P=16, I=0, D=32)

### Usage

```bash
# For follower arm
python minimal_setup_motors.py COM9 follower

# For leader arm  
python minimal_setup_motors.py COM4 leader
```

### Motor ID Mapping

| Motor | ID |
|-------|-----|
| shoulder_pan | 1 |
| shoulder_lift | 2 |
| elbow_flex | 3 |
| wrist_flex | 4 |
| wrist_roll | 5 |
| gripper | 6 |

### Notes

- Connect **one motor at a time** when prompted
- Motors are configured in reverse order (gripper → shoulder_pan)
- New motors typically have factory ID = 1, so configure them individually

---

## 2. Calibration (`minimal_calibrate.py`)

Calibrates the arm by recording homing offsets and range of motion limits.

### What It Does

1. **Homing Offset**: You position the arm at its middle position, and the script calculates offsets so that position becomes the center (2047 on a 4096-step encoder)

2. **Range of Motion**: You move each joint through its full range while the script records min/max positions

3. **Saves to file**: Creates a JSON calibration file compatible with LeRobot

### Usage

```bash
# For follower arm
python minimal_calibrate.py COM9 follower my_follower

# For leader arm
python minimal_calibrate.py COM4 leader my_leader
```

### Arguments

| Argument | Description |
|----------|-------------|
| `COM_PORT` | Serial port (e.g., COM9, COM4) |
| `ARM_TYPE` | `follower` or `leader` |
| `ARM_ID` | Unique name for this arm (used in calibration filename) |

### Calibration Steps

1. **Move to middle**: Position all joints at the middle of their range
2. **Press Enter**: Script records current positions and calculates homing offsets
3. **Move through range**: Move each joint through its full range of motion
4. **Press Enter**: Script saves min/max values

### Calibration File Location

Files are saved to match LeRobot's expected paths:

- **Follower**: `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/{arm_id}.json`
- **Leader**: `~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/{arm_id}.json`

---

## 3. Play with Arm (`play_arm.py`)

Interactive control for a single arm — great for testing!

### Usage

```bash
python play_arm.py COM9
```

### Controls

| Key | Action |
|-----|--------|
| `F` | **Free Move** — Disable torque, move arm by hand |
| `T` | **Toggle Torque** — Switch between free/controlled |
| `W/S` | Move selected joint +/- |
| `A/D` | Select previous/next joint |
| `1-6` | Select joint directly |
| `Space` | Save current pose |
| `P` | Play all saved poses |
| `C` | Clear saved poses |
| `R` | Refresh display |
| `Q` | Quit |

### How Torque Works

- **Torque OFF**: Motors are free — you can move the arm by hand
- **Torque ON**: Motors actively hold position and respond to W/S commands

---

## 4. Talking Arm (`talk_arm.py`)

Make your robot arm talk! Uses text-to-speech and syncs the gripper (jaw) to the audio.

### Extra Requirements

```bash
pip install pyttsx3
```

### Usage

```bash
# Speak text directly
python talk_arm.py COM9 "Hello, I am a robot arm!"

# Play an existing audio file
python talk_arm.py COM9 --file speech.wav

# Interactive mode (type to speak)
python talk_arm.py COM9 --interactive

# Use amplitude mode instead of pulse
python talk_arm.py COM9 "Hello" --mode amplitude
```

### Jaw Sync Modes

| Mode | Behavior | Best For |
|------|----------|----------|
| `pulse` (default) | Snap open/closed on syllables | Puppet-like talking |
| `amplitude` | Smooth follow of volume | Wave-like motion |

### Arguments

| Argument | Description |
|----------|-------------|
| `COM_PORT` | Serial port (e.g., COM9) |
| `text` | Text to speak |
| `--file`, `-f` | WAV file to play instead of TTS |
| `--interactive`, `-i` | Interactive mode |
| `--mode`, `-m` | `pulse` or `amplitude` (default: pulse) |
| `--rate`, `-r` | Speech rate (default: 150) |
| `--jaw-closed` | Servo position for closed jaw |
| `--jaw-open` | Servo position for open jaw |

### Interactive Commands

| Command | Action |
|---------|--------|
| `/mode` | Toggle between pulse/amplitude |
| `/rate <num>` | Set speech rate |
| `/jaw <0-100>` | Test jaw position |
| `/quit` | Exit |

### Tuning (edit `talk_arm.py`)

```python
JAW_CLOSED = 1945     # Adjust for your gripper's closed position
JAW_OPEN = 2600       # Adjust for your gripper's open position
PULSE_THRESHOLD = 0.05  # Lower = more sensitive
PULSE_DURATION = 0.10   # How long jaw stays open (seconds)
```

---

## Typical Workflow

### First Time Setup

1. **Setup follower arm motors** (one at a time):
   ```bash
   python minimal_setup_motors.py COM9 follower
   ```

2. **Setup leader arm motors** (one at a time):
   ```bash
   python minimal_setup_motors.py COM4 leader
   ```

3. **Calibrate follower arm** (all motors connected):
   ```bash
   python minimal_calibrate.py COM9 follower my_follower
   ```

4. **Calibrate leader arm** (all motors connected):
   ```bash
   python minimal_calibrate.py COM4 leader my_leader
   ```

5. **Play with the arm!**
   ```bash
   python play_arm.py COM9
   ```

6. **Make it talk!**
   ```bash
   python talk_arm.py COM9 "Hello, I am your robot assistant!"
   ```

7. **Teleoperate!** (if you have both arms):
   ```bash
   python -m lerobot.teleoperate --robot.type=so101_follower --robot.port=COM9 --teleop.type=so101_leader --teleop.port=COM4
   ```

---

## Troubleshooting

### Motor not found during setup
- Ensure only ONE motor is connected
- Check USB cable connection
- Try different baud rates manually

### All motors not responding during calibration
- Run `minimal_setup_motors.py` first to set IDs
- Ensure all 6 motors are connected to the bus
- Check power supply

### Finding COM port
```bash
# On Windows, check Device Manager → Ports (COM & LPT)
# Or use LeRobot's find_port:
python -m lerobot.find_port
```

---

## Differences from LeRobot

| Feature | LeRobot | Minimal Scripts |
|---------|---------|-----------------|
| Dependencies | ~50+ packages | 2 packages |
| Python version | 3.10+ only | Any Python 3.x |
| Install time | Minutes | Seconds |
| Calibration format | Same | Same (compatible) |

These scripts produce **identical motor configurations** and **compatible calibration files** — you can use them interchangeably with the full LeRobot stack.

