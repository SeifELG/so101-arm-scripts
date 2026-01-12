"""
SO-101 Arm Talking Script

Make your robot arm talk! Generates speech and syncs jaw (gripper) movement.

Dependencies: pip install pyttsx3 feetech-servo-sdk pyserial

Usage:
    python talk_arm.py COM9 "Hello, I am a robot arm"
    python talk_arm.py COM9 --file speech.wav
    python talk_arm.py COM9 --interactive
"""

import argparse
import math
import os
import struct
import sys
import tempfile
import threading
import time
import wave
import winsound

try:
    import pyttsx3
except ImportError:
    print("Error: pyttsx3 not installed!")
    print("Run: pip install pyttsx3")
    sys.exit(1)

try:
    import scservo_sdk as scs
except ImportError:
    print("Error: feetech-servo-sdk not installed!")
    print("Run: pip install feetech-servo-sdk pyserial")
    sys.exit(1)


# === Servo Configuration ===
BAUD_RATE = 1_000_000
PROTOCOL_VERSION = 0
GRIPPER_ID = 6  # Jaw motor

# Register addresses
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

# Jaw position mapping
JAW_CLOSED = 1945  # Adjust based on your gripper's closed position
JAW_OPEN = 2600    # Adjust based on your gripper's open position

# Audio analysis settings
CHUNK_DURATION_MS = 30  # Analyze audio in 30ms chunks
SMOOTHING_FACTOR = 0.3  # For smooth mode (not used in pulse mode)

# Jaw sync modes
MODE_PULSE = "pulse"       # Snap open/closed on syllables (puppet-like)
MODE_AMPLITUDE = "amplitude"  # Smooth follow of volume (wave-like)
DEFAULT_MODE = MODE_PULSE

# Pulse mode settings
PULSE_THRESHOLD = 0.05    # Amplitude threshold to trigger jaw open (0.0-1.0)
PULSE_DURATION = 0.10     # How long jaw stays open (seconds)
PULSE_COOLDOWN = 0.05     # Minimum time between pulses (seconds)


class JawController:
    """Controls the gripper servo as a jaw."""
    
    def __init__(self, port_name, jaw_closed=JAW_CLOSED, jaw_open=JAW_OPEN):
        self.port_handler = scs.PortHandler(port_name)
        self.packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
        self.jaw_closed = jaw_closed
        self.jaw_open = jaw_open
        self.current_position = jaw_closed
        self.connected = False
    
    def connect(self):
        if not self.port_handler.openPort():
            print(f"Error: Could not open port")
            return False
        self.port_handler.setBaudRate(BAUD_RATE)
        print(f"Connected to arm at {BAUD_RATE} bps")
        self.connected = True
        return True
    
    def disconnect(self):
        if self.connected:
            self.set_jaw(0)  # Close jaw
            time.sleep(0.1)
            self.disable_torque()
            self.port_handler.closePort()
            print("Disconnected.")
    
    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1
        )
        print("Jaw torque enabled")
    
    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0
        )
    
    def set_jaw(self, openness, smooth=True):
        """
        Set jaw position.
        
        Args:
            openness: 0.0 (closed) to 1.0 (fully open)
            smooth: Apply smoothing to avoid jitter
        """
        openness = max(0.0, min(1.0, openness))
        target = int(self.jaw_closed + (self.jaw_open - self.jaw_closed) * openness)
        
        if smooth:
            # Smooth movement to reduce jitter
            self.current_position = int(
                self.current_position * SMOOTHING_FACTOR + 
                target * (1 - SMOOTHING_FACTOR)
            )
        else:
            self.current_position = target
        
        self.packet_handler.write2ByteTxRx(
            self.port_handler, GRIPPER_ID, ADDR_GOAL_POSITION, self.current_position
        )


def text_to_wav(text, output_path=None, rate=150):
    """
    Convert text to speech and save as WAV.
    
    Args:
        text: Text to speak
        output_path: Where to save WAV (uses temp file if None)
        rate: Speech rate (words per minute)
    
    Returns:
        Path to the WAV file
    """
    if output_path is None:
        fd, output_path = tempfile.mkstemp(suffix='.wav')
        os.close(fd)
    
    engine = pyttsx3.init()
    engine.setProperty('rate', rate)
    
    # Get available voices and pick one
    voices = engine.getProperty('voices')
    if voices:
        # Try to find a clear voice (you can customize this)
        engine.setProperty('voice', voices[0].id)
    
    engine.save_to_file(text, output_path)
    engine.runAndWait()
    
    print(f"Generated speech: {output_path}")
    return output_path


def extract_amplitude_envelope(wav_path, chunk_ms=CHUNK_DURATION_MS):
    """
    Extract amplitude envelope from a WAV file (pure Python, no numpy).
    
    Args:
        wav_path: Path to WAV file
        chunk_ms: Size of each chunk in milliseconds
    
    Returns:
        (envelope, duration) - envelope is list of (time, amplitude) tuples
    """
    with wave.open(wav_path, 'rb') as wf:
        sample_rate = wf.getframerate()
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        n_frames = wf.getnframes()
        
        # Read all audio data
        raw_data = wf.readframes(n_frames)
    
    # Determine struct format for unpacking
    if sample_width == 1:
        fmt_char = 'B'  # unsigned byte
        max_val = 128
        offset = 128
    elif sample_width == 2:
        fmt_char = 'h'  # signed short
        max_val = 32768
        offset = 0
    elif sample_width == 4:
        fmt_char = 'i'  # signed int
        max_val = 2147483648
        offset = 0
    else:
        raise ValueError(f"Unsupported sample width: {sample_width}")
    
    # Unpack all samples
    num_samples = n_frames * n_channels
    samples = struct.unpack(f'<{num_samples}{fmt_char}', raw_data)
    
    # Convert to mono if stereo (average left and right channels)
    if n_channels == 2:
        mono_samples = []
        for i in range(0, len(samples), 2):
            avg = (samples[i] + samples[i + 1]) / 2
            mono_samples.append(avg)
        samples = mono_samples
    else:
        samples = list(samples)
    
    # Normalize to -1.0 to 1.0
    samples = [(s - offset) / max_val for s in samples]
    
    # Calculate chunk size in samples
    chunk_samples = int(sample_rate * chunk_ms / 1000)
    
    # Extract RMS amplitude for each chunk
    envelope = []
    for i in range(0, len(samples), chunk_samples):
        chunk = samples[i:i + chunk_samples]
        if len(chunk) > 0:
            # RMS amplitude: sqrt(mean(x^2))
            sum_squares = sum(x * x for x in chunk)
            rms = math.sqrt(sum_squares / len(chunk))
            time_sec = i / sample_rate
            envelope.append((time_sec, rms))
    
    # Normalize envelope to 0-1 range
    if envelope:
        max_amp = max(amp for _, amp in envelope)
        if max_amp > 0:
            envelope = [(t, amp / max_amp) for t, amp in envelope]
    
    duration = n_frames / sample_rate
    return envelope, duration


def play_audio(wav_path):
    """
    Start audio playback using winsound (Windows built-in).
    
    Returns:
        Thread object (check is_alive() to see if still playing)
    """
    def play_sound():
        winsound.PlaySound(wav_path, winsound.SND_FILENAME)
    
    thread = threading.Thread(target=play_sound)
    thread.start()
    return thread


def is_playing(play_obj):
    """Check if audio is still playing."""
    return play_obj.is_alive() if play_obj else False


def get_amplitude_at_time(envelope, t):
    """Get interpolated amplitude at time t."""
    if not envelope:
        return 0.0
    
    # Find surrounding samples
    for i in range(len(envelope) - 1):
        t1, a1 = envelope[i]
        t2, a2 = envelope[i + 1]
        if t1 <= t <= t2:
            # Linear interpolation
            if t2 > t1:
                factor = (t - t1) / (t2 - t1)
                return a1 + (a2 - a1) * factor
            return a1
    
    # After end, return last amplitude
    if t >= envelope[-1][0]:
        return envelope[-1][1]
    
    return 0.0


def speak(jaw_controller, text=None, wav_path=None, speech_rate=150, mode=DEFAULT_MODE):
    """
    Make the arm speak!
    
    Args:
        jaw_controller: JawController instance
        text: Text to speak (generates TTS)
        wav_path: Path to existing WAV file (alternative to text)
        speech_rate: TTS speech rate
        mode: "pulse" (puppet-like chomps) or "amplitude" (smooth wave)
    """
    # Generate or use provided audio
    temp_file = None
    if text:
        wav_path = text_to_wav(text, rate=speech_rate)
        temp_file = wav_path
    elif not wav_path:
        print("Error: Provide either text or wav_path")
        return
    
    try:
        # Extract amplitude envelope
        print("Analyzing audio...")
        envelope, duration = extract_amplitude_envelope(wav_path)
        print(f"Audio duration: {duration:.2f}s, {len(envelope)} amplitude samples")
        print(f"Mode: {mode}")
        
        # Enable jaw
        jaw_controller.enable_torque()
        time.sleep(0.1)
        
        # Start playback
        print("Speaking...")
        play_obj = play_audio(wav_path)
        start_time = time.time()
        
        if mode == MODE_PULSE:
            # === PULSE MODE: Snap open/closed on syllables ===
            jaw_open = False
            jaw_open_time = 0
            last_pulse_time = -PULSE_COOLDOWN
            was_above_threshold = False
            
            while is_playing(play_obj):
                elapsed = time.time() - start_time
                now = time.time()
                
                amplitude = get_amplitude_at_time(envelope, elapsed)
                is_above_threshold = amplitude > PULSE_THRESHOLD
                
                # Close jaw if pulse duration has passed
                if jaw_open and (now - jaw_open_time) >= PULSE_DURATION:
                    jaw_controller.set_jaw(0, smooth=False)
                    jaw_open = False
                
                # Trigger pulse on rising edge
                if (is_above_threshold and 
                    not was_above_threshold and 
                    not jaw_open and
                    (now - last_pulse_time) >= PULSE_COOLDOWN):
                    jaw_controller.set_jaw(1.0, smooth=False)
                    jaw_open = True
                    jaw_open_time = now
                    last_pulse_time = now
                
                was_above_threshold = is_above_threshold
                time.sleep(0.01)
        
        else:
            # === AMPLITUDE MODE: Smooth follow of volume ===
            while is_playing(play_obj):
                elapsed = time.time() - start_time
                
                amplitude = get_amplitude_at_time(envelope, elapsed)
                # Boost quiet sounds, compress loud sounds
                amplitude = amplitude ** 0.7
                
                jaw_controller.set_jaw(amplitude, smooth=True)
                time.sleep(0.015)
        
        # Close jaw at end
        jaw_controller.set_jaw(0, smooth=False)
        print("Done speaking!")
        
    finally:
        # Clean up temp file
        if temp_file and os.path.exists(temp_file):
            try:
                os.remove(temp_file)
            except:
                pass


def interactive_mode(jaw_controller, initial_mode=DEFAULT_MODE):
    """Interactive mode - type text and hear it spoken."""
    print("\n" + "=" * 50)
    print("  Interactive Talk Mode")
    print("=" * 50)
    print("Type text and press Enter to speak.")
    print("Commands:")
    print("  /mode        - Toggle between pulse/amplitude")
    print("  /rate <num>  - Set speech rate (default: 150)")
    print("  /jaw <0-100> - Test jaw position")
    print("  /quit        - Exit")
    print("=" * 50 + "\n")
    
    speech_rate = 150
    mode = initial_mode
    print(f"Current mode: {mode}")
    
    while True:
        try:
            text = input("Say: ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        
        if not text:
            continue
        
        if text.startswith('/'):
            parts = text.split()
            cmd = parts[0].lower()
            
            if cmd == '/quit':
                break
            elif cmd == '/mode':
                mode = MODE_AMPLITUDE if mode == MODE_PULSE else MODE_PULSE
                print(f"Mode switched to: {mode}")
            elif cmd == '/rate' and len(parts) > 1:
                try:
                    speech_rate = int(parts[1])
                    print(f"Speech rate set to {speech_rate}")
                except ValueError:
                    print("Invalid rate")
            elif cmd == '/jaw' and len(parts) > 1:
                try:
                    pos = int(parts[1]) / 100.0
                    jaw_controller.set_jaw(pos, smooth=False)
                    print(f"Jaw set to {pos*100:.0f}%")
                except ValueError:
                    print("Invalid position (use 0-100)")
            else:
                print("Unknown command")
        else:
            speak(jaw_controller, text=text, speech_rate=speech_rate, mode=mode)


def main():
    parser = argparse.ArgumentParser(
        description="Make your SO-101 arm talk!",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python talk_arm.py COM9 "Hello, I am a robot!"
  python talk_arm.py COM9 --file speech.wav
  python talk_arm.py COM9 --interactive
  python talk_arm.py COM9 "Fast talking robot" --rate 200
  python talk_arm.py COM9 "Hello" --mode amplitude
        """
    )
    parser.add_argument('port', help='Serial port (e.g., COM9)')
    parser.add_argument('text', nargs='?', help='Text to speak')
    parser.add_argument('--file', '-f', help='WAV file to play instead of TTS')
    parser.add_argument('--interactive', '-i', action='store_true', 
                        help='Interactive mode - type to speak')
    parser.add_argument('--mode', '-m', choices=['pulse', 'amplitude'], default=DEFAULT_MODE,
                        help=f'Jaw sync mode (default: {DEFAULT_MODE})')
    parser.add_argument('--rate', '-r', type=int, default=150,
                        help='Speech rate (default: 150)')
    parser.add_argument('--jaw-closed', type=int, default=JAW_CLOSED,
                        help=f'Jaw closed position (default: {JAW_CLOSED})')
    parser.add_argument('--jaw-open', type=int, default=JAW_OPEN,
                        help=f'Jaw open position (default: {JAW_OPEN})')
    
    args = parser.parse_args()
    
    # Pass jaw positions to controller instead of using globals
    jaw_closed = args.jaw_closed
    jaw_open = args.jaw_open
    
    # Validate arguments
    if not args.interactive and not args.text and not args.file:
        parser.error("Provide text, --file, or --interactive")
    
    # Connect to arm
    jaw = JawController(args.port, jaw_closed=jaw_closed, jaw_open=jaw_open)
    if not jaw.connect():
        sys.exit(1)
    
    try:
        if args.interactive:
            interactive_mode(jaw, initial_mode=args.mode)
        elif args.file:
            speak(jaw, wav_path=args.file, mode=args.mode)
        else:
            speak(jaw, text=args.text, speech_rate=args.rate, mode=args.mode)
    
    except KeyboardInterrupt:
        print("\nInterrupted.")
    
    finally:
        jaw.disconnect()


if __name__ == "__main__":
    main()
