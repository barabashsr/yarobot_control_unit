#!/usr/bin/env python3
"""
Joystick Control Tool for YaRobot Control Unit

Python host-side utility that connects to ESP32 via USB serial and accepts
input from a DualShock 4 gamepad connected via Bluetooth. Provides manual
axis control, emergency stop, brake/enable management, and home position management.

Usage:
    python joystick_control.py [--config CONFIG_FILE] [--port PORT]

Author: YaRobot Team
Date: 2025
"""

import argparse
import os
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import pygame
import serial
import yaml


# =============================================================================
# Constants and Enums
# =============================================================================

class EnableState(Enum):
    UNKNOWN = "UNKNOWN"
    ENABLED = "ENABLED"
    DISABLED = "DISABLED"


class BrakeState(Enum):
    UNKNOWN = "UNKNOWN"
    ENGAGED = "ENGAGED"
    RELEASED = "RELEASED"
    NO_BRAKE = "(no brake)"


class SystemMode(Enum):
    IDLE = "IDLE"
    READY = "READY"
    CONFIG = "CONFIG"
    ESTOP = "ESTOP"
    ERROR = "ERROR"
    UNKNOWN = "UNKNOWN"


# Axis names in order
AXES = ['X', 'Y', 'Z', 'A', 'B', 'C', 'D']
SERVO_AXES = ['X', 'Y', 'Z', 'A', 'B']  # Axes with brakes


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class AxisConfig:
    """Configuration for a single axis."""
    button: int
    speed: float
    has_brake: bool


@dataclass
class AxisState:
    """Runtime state for a single axis."""
    enable: EnableState = EnableState.UNKNOWN
    brake: BrakeState = BrakeState.UNKNOWN
    is_moving: bool = False


@dataclass
class Config:
    """Application configuration loaded from YAML."""
    serial_port: str = "/dev/ttyUSB0"
    serial_baudrate: int = 115200
    serial_timeout: float = 1.0
    direction_modifier: int = 7  # R2
    estop_button: int = 8  # Share
    go_home_button: int = 9  # Options
    save_home_button: int = 13  # Touchpad
    axes: Dict[str, AxisConfig] = field(default_factory=dict)
    home_positions: Dict[str, Optional[float]] = field(default_factory=dict)
    config_path: str = ""


# =============================================================================
# Serial Communication Module
# =============================================================================

class SerialConnection:
    """Handles serial communication with ESP32."""

    def __init__(self, port: str, baudrate: int, timeout: float,
                 on_disconnect: Optional[Callable] = None,
                 log_callback: Optional[Callable[[str, str], None]] = None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.on_disconnect = on_disconnect
        self.log_callback = log_callback
        self.serial: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self._connected = False

    def connect(self) -> bool:
        """Connect to the serial port."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self._connected = True
            self._log("INFO", f"Connected to {self.port}")

            # Clear any pending data and wait for boot messages to finish
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Drain any remaining log messages
            self.serial.timeout = 0.1
            while self.serial.readline():
                pass
            self.serial.timeout = self.timeout

            return True
        except serial.SerialException as e:
            self._log("ERROR", f"Cannot connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from the serial port."""
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass
        self._connected = False

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected and self.serial is not None and self.serial.is_open

    def send_command(self, command: str, wait_response: bool = True) -> Optional[str]:
        """
        Send a command and optionally wait for response.

        Returns the response string or None on error.
        """
        if not self.is_connected():
            return None

        with self.lock:
            try:
                # Send command
                cmd_bytes = f"{command}\r\n".encode('utf-8')
                self.serial.write(cmd_bytes)
                self.serial.flush()
                self._log("TX", command)

                if not wait_response:
                    return "OK"

                # Read response
                response = self._read_response()
                if response:
                    self._log("RX", response)
                return response

            except serial.SerialException as e:
                self._log("ERROR", f"Serial error: {e}")
                self._connected = False
                if self.on_disconnect:
                    self.on_disconnect()
                return None

    def _read_response(self, timeout_ms: int = 500) -> Optional[str]:
        """Read response, skipping ESP-IDF log messages."""
        start_time = time.time()
        timeout_sec = timeout_ms / 1000.0

        while (time.time() - start_time) < timeout_sec:
            try:
                if self.serial.in_waiting == 0:
                    time.sleep(0.01)
                    continue

                line = self.serial.readline().decode('utf-8').strip()
                if not line:
                    continue

                # Skip ESP-IDF log messages: "I (12345) tag: message"
                # These start with I/W/E/D followed by space and parenthesis
                if len(line) > 2 and line[0] in 'IWED' and line[1] == ' ' and '(' in line[:10]:
                    continue

                # Skip empty lines and prompts
                if line in ['', '>', 'esp>']:
                    continue

                # This is the actual response
                return line

            except Exception:
                return None

        return None

    def _log(self, direction: str, message: str):
        """Log a message."""
        if self.log_callback:
            self.log_callback(direction, message)


# =============================================================================
# Joystick Input Module
# =============================================================================

class JoystickController:
    """Handles DualShock 4 joystick input via pygame."""

    def __init__(self, on_disconnect: Optional[Callable] = None):
        self.joystick: Optional[pygame.joystick.JoystickType] = None
        self.on_disconnect = on_disconnect
        self._connected = False

    def init(self) -> bool:
        """Initialize pygame and joystick subsystem."""
        pygame.init()
        pygame.joystick.init()
        return True

    def connect(self) -> bool:
        """Find and connect to a DualShock 4 controller."""
        count = pygame.joystick.get_count()
        if count == 0:
            return False

        # Look for DualShock 4
        for i in range(count):
            js = pygame.joystick.Joystick(i)
            js.init()
            name = js.get_name().lower()
            if 'dualshock' in name or 'ps4' in name or 'wireless controller' in name:
                self.joystick = js
                self._connected = True
                return True

        # Use first available if no DualShock 4 found
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self._connected = True
        return True

    def is_connected(self) -> bool:
        """Check if joystick is connected."""
        return self._connected and self.joystick is not None

    def get_button(self, button_id: int) -> bool:
        """Get button state."""
        if not self.is_connected():
            return False
        try:
            return self.joystick.get_button(button_id)
        except pygame.error:
            self._handle_disconnect()
            return False

    def get_hat(self, hat_id: int = 0) -> Tuple[int, int]:
        """Get hat (D-pad) state as (x, y) tuple."""
        if not self.is_connected():
            return (0, 0)
        try:
            return self.joystick.get_hat(hat_id)
        except pygame.error:
            self._handle_disconnect()
            return (0, 0)

    def process_events(self) -> List[pygame.event.Event]:
        """Process and return pygame events."""
        try:
            pygame.event.pump()
            return pygame.event.get()
        except pygame.error:
            self._handle_disconnect()
            return []

    def _handle_disconnect(self):
        """Handle joystick disconnect."""
        self._connected = False
        if self.on_disconnect:
            self.on_disconnect()

    def quit(self):
        """Clean up pygame."""
        pygame.joystick.quit()
        pygame.quit()


# =============================================================================
# Status Table Display
# =============================================================================

class StatusDisplay:
    """Handles console status table display."""

    def __init__(self):
        self.mode: SystemMode = SystemMode.UNKNOWN
        self.axis_states: Dict[str, AxisState] = {axis: AxisState() for axis in AXES}
        self.log_lines: List[str] = []
        self.max_log_lines = 10

    def set_mode(self, mode: SystemMode):
        """Set system mode."""
        self.mode = mode

    def set_enable_state(self, axis: str, state: EnableState):
        """Set enable state for an axis."""
        if axis in self.axis_states:
            self.axis_states[axis].enable = state

    def set_brake_state(self, axis: str, state: BrakeState):
        """Set brake state for an axis."""
        if axis in self.axis_states:
            self.axis_states[axis].brake = state

    def add_log(self, direction: str, message: str):
        """Add a log line."""
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] [{direction}] {message}"
        self.log_lines.append(line)
        if len(self.log_lines) > self.max_log_lines:
            self.log_lines.pop(0)

    def render(self) -> str:
        """Render the status display as a string."""
        lines = []

        # Status table
        lines.append("")
        lines.append("\033[2J\033[H")  # Clear screen and move cursor to home
        lines.append("=" * 50)
        lines.append("       YAROBOT JOYSTICK CONTROL TOOL")
        lines.append("=" * 50)
        lines.append("")
        lines.append("+---------+-----------+---------------+")
        lines.append("|  Axis   |  Enabled  |     Brake     |")
        lines.append("+---------+-----------+---------------+")

        for axis in AXES:
            state = self.axis_states[axis]
            enable_str = state.enable.value.center(9)
            brake_str = state.brake.value.center(13)
            lines.append(f"|    {axis}    |{enable_str}|{brake_str}|")

        lines.append("+---------+-----------+---------------+")
        lines.append("")
        lines.append(f"[MODE] {self.mode.value}")
        lines.append("")
        lines.append("-" * 50)
        lines.append("Recent Commands:")

        for log_line in self.log_lines[-5:]:
            lines.append(log_line)

        lines.append("-" * 50)
        lines.append("")
        lines.append("Controls: Cross/Circle/Square/Triangle/L1/R1/L2 = Jog axes")
        lines.append("          R2 = Negative direction | Share = E-STOP")
        lines.append("          D-Left + Axis = Toggle Brake | D-Up + Axis = Toggle Enable")
        lines.append("          D-Down = RST + MODE READY | D-Right = Save Home")
        lines.append("          Options = Go Home | Ctrl+C = Exit")

        return "\n".join(lines)

    def print_status(self):
        """Print the status table to console."""
        print(self.render())


# =============================================================================
# Main Application
# =============================================================================

class JoystickControlApp:
    """Main application class."""

    def __init__(self, config_path: str, port_override: Optional[str] = None):
        self.config_path = config_path
        self.config = self._load_config(config_path)
        if port_override:
            self.config.serial_port = port_override

        self.display = StatusDisplay()
        self.serial: Optional[SerialConnection] = None
        self.joystick: Optional[JoystickController] = None
        self.running = False
        self.button_states: Dict[int, bool] = {}  # Track button states
        self._needs_redraw = True

        # Set up signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self._signal_handler)

    def _load_config(self, path: str) -> Config:
        """Load configuration from YAML file."""
        config = Config()
        config.config_path = path

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            if 'serial' in data:
                config.serial_port = data['serial'].get('port', config.serial_port)
                config.serial_baudrate = data['serial'].get('baudrate', config.serial_baudrate)
                config.serial_timeout = data['serial'].get('timeout', config.serial_timeout)

            if 'buttons' in data:
                config.direction_modifier = data['buttons'].get('direction_modifier', 7)
                config.estop_button = data['buttons'].get('estop', 8)
                config.go_home_button = data['buttons'].get('go_home', 9)
                config.save_home_button = data['buttons'].get('save_home', 13)

            if 'axes' in data:
                for axis_name, axis_data in data['axes'].items():
                    config.axes[axis_name] = AxisConfig(
                        button=axis_data.get('button', 0),
                        speed=axis_data.get('speed', 0.001),
                        has_brake=axis_data.get('has_brake', False)
                    )

            if 'home_positions' in data:
                config.home_positions = data['home_positions']

        except Exception as e:
            print(f"[WARN] Failed to load config: {e}, using defaults")

        return config

    def _save_config(self):
        """Save configuration to YAML file."""
        try:
            with open(self.config.config_path, 'r') as f:
                data = yaml.safe_load(f)

            data['home_positions'] = self.config.home_positions

            with open(self.config.config_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            self.display.add_log("INFO", "Home positions saved to config.yaml")
        except Exception as e:
            self.display.add_log("ERROR", f"Failed to save config: {e}")

    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C signal."""
        self.display.add_log("INFO", "Ctrl+C received, shutting down...")
        self._graceful_shutdown()

    def _graceful_shutdown(self):
        """Gracefully shut down, stopping all axes."""
        self.running = False

        # Stop all axes
        if self.serial and self.serial.is_connected():
            self.display.add_log("INFO", "Stopping all axes...")
            self.serial.send_command("STOP", wait_response=False)
            time.sleep(0.1)

        # Disconnect
        if self.serial:
            self.serial.disconnect()
        if self.joystick:
            self.joystick.quit()

        print("\n[EXIT] Joystick control terminated.")
        sys.exit(0)

    def _on_serial_disconnect(self):
        """Handle serial disconnect."""
        self.display.add_log("ERROR", "Serial disconnected!")
        self.display.print_status()
        # Try to stop axes before we lose connection completely
        self.running = False

    def _on_joystick_disconnect(self):
        """Handle joystick disconnect."""
        self.display.add_log("ERROR", "Joystick disconnected!")
        # Stop all axes
        if self.serial and self.serial.is_connected():
            self.serial.send_command("STOP", wait_response=False)
        self.display.print_status()
        self.running = False

    def _log_callback(self, direction: str, message: str):
        """Callback for serial logging."""
        self.display.add_log(direction, message)
        self._needs_redraw = True

    def run(self):
        """Main application loop."""
        print("[INIT] Starting Joystick Control Tool...")

        # Connect to serial
        self.serial = SerialConnection(
            port=self.config.serial_port,
            baudrate=self.config.serial_baudrate,
            timeout=self.config.serial_timeout,
            on_disconnect=self._on_serial_disconnect,
            log_callback=self._log_callback
        )

        if not self.serial.connect():
            print(f"[ERROR] Cannot connect to {self.config.serial_port}")
            return

        # Initialize joystick
        self.joystick = JoystickController(on_disconnect=self._on_joystick_disconnect)
        self.joystick.init()

        print("[INIT] Waiting for DualShock 4 controller...")
        if not self.joystick.connect():
            print("[ERROR] No joystick found")
            self.serial.disconnect()
            return

        print(f"[INIT] Connected to: {self.joystick.joystick.get_name()}")

        # Run startup sequence
        self._startup_sequence()

        # Main loop
        self.running = True
        self._needs_redraw = True

        while self.running:
            if not self.serial.is_connected():
                self.display.add_log("ERROR", "Serial connection lost")
                self._graceful_shutdown()
                break

            if not self.joystick.is_connected():
                self.display.add_log("ERROR", "Joystick connection lost")
                self._graceful_shutdown()
                break

            self._process_input()

            # Only redraw when needed
            if self._needs_redraw:
                self.display.print_status()
                self._needs_redraw = False

            time.sleep(0.02)  # 50 Hz loop

    def _startup_sequence(self):
        """Execute startup sequence."""
        # Query current mode
        response = self.serial.send_command("MODE")
        if response:
            mode = self._parse_mode_response(response)
            self.display.set_mode(mode)

            if mode == SystemMode.IDLE:
                # Set to READY
                response = self.serial.send_command("MODE READY")
                if response and response.startswith("OK"):
                    self.display.set_mode(SystemMode.READY)
                    self.display.add_log("INFO", "Mode set to READY")
                else:
                    self.display.add_log("WARN", "Failed to set READY mode")
            elif mode == SystemMode.READY:
                self.display.add_log("INFO", "Already in READY mode")
            elif mode == SystemMode.ESTOP:
                self.display.add_log("WARN", "System in E-STOP! Clear and send RST command.")
            elif mode == SystemMode.ERROR:
                self.display.add_log("WARN", "System in ERROR state! Resolve and restart.")

        # Query brake status for servo axes
        for axis in SERVO_AXES:
            response = self.serial.send_command(f"BRAKE {axis}")
            if response:
                brake_state = self._parse_brake_response(response)
                self.display.set_brake_state(axis, brake_state)

        # Set no-brake for stepper axes
        for axis in ['C', 'D']:
            self.display.set_brake_state(axis, BrakeState.NO_BRAKE)

        # Initialize enable states as unknown
        for axis in AXES:
            self.display.set_enable_state(axis, EnableState.UNKNOWN)

        self.display.add_log("INFO", "Startup complete. Press Share for E-STOP.")
        self._needs_redraw = True

    def _parse_mode_response(self, response: str) -> SystemMode:
        """Parse MODE command response."""
        if "IDLE" in response:
            return SystemMode.IDLE
        elif "READY" in response:
            return SystemMode.READY
        elif "CONFIG" in response:
            return SystemMode.CONFIG
        elif "ESTOP" in response:
            return SystemMode.ESTOP
        elif "ERROR" in response:
            return SystemMode.ERROR
        return SystemMode.UNKNOWN

    def _parse_brake_response(self, response: str) -> BrakeState:
        """Parse BRAKE query response."""
        if "ENGAGED" in response:
            return BrakeState.ENGAGED
        elif "RELEASED" in response:
            return BrakeState.RELEASED
        return BrakeState.UNKNOWN

    def _parse_position_response(self, response: str) -> Dict[str, float]:
        """Parse POS command response."""
        positions = {}
        # Expected format: "OK X:0.150 Y:1.571 Z:0.000 ..."
        pattern = r'([XYZABCD]):(-?\d+\.?\d*)'
        matches = re.findall(pattern, response)
        for axis, value in matches:
            positions[axis] = float(value)
        return positions

    def _process_input(self):
        """Process joystick input."""
        events = self.joystick.process_events()

        # Get modifier states
        r2_pressed = self.joystick.get_button(self.config.direction_modifier)
        hat = self.joystick.get_hat(0)
        dpad_left = hat[0] == -1
        dpad_up = hat[1] == 1
        dpad_down = hat[1] == -1

        for event in events:
            if event.type == pygame.JOYHATMOTION:
                # D-pad Down alone = RST + MODE READY
                if hat[1] == -1 and hat[0] == 0:
                    self._do_reset_and_ready()
                    continue
                # D-pad Right alone = Save Home
                if hat[0] == 1 and hat[1] == 0:
                    self._do_save_home()
                    continue

            if event.type == pygame.JOYBUTTONDOWN:
                self._handle_button_press(event.button, r2_pressed, dpad_left, dpad_up)
            elif event.type == pygame.JOYBUTTONUP:
                self._handle_button_release(event.button)

    def _handle_button_press(self, button: int, r2_pressed: bool,
                             dpad_left: bool, dpad_up: bool):
        """Handle button press event."""
        self.button_states[button] = True

        # E-STOP - highest priority
        if button == self.config.estop_button:
            self._do_estop()
            return

        # Go Home (Options)
        if button == self.config.go_home_button:
            self._do_go_home()
            return

        # Check if this is an axis button
        axis = self._button_to_axis(button)
        if axis:
            if dpad_left:
                # Brake toggle
                self._do_brake_toggle(axis)
            elif dpad_up:
                # Enable toggle
                self._do_enable_toggle(axis)
            else:
                # Axis jog
                self._do_axis_jog_start(axis, r2_pressed)

    def _handle_button_release(self, button: int):
        """Handle button release event."""
        self.button_states[button] = False

        # Check if this is an axis button
        axis = self._button_to_axis(button)
        if axis:
            self._do_axis_jog_stop(axis)

    def _button_to_axis(self, button: int) -> Optional[str]:
        """Convert button ID to axis name."""
        for axis_name, axis_config in self.config.axes.items():
            if axis_config.button == button:
                return axis_name
        return None

    def _do_estop(self):
        """Execute E-STOP."""
        self.serial.send_command("TEST ESTOP", wait_response=False)
        self.display.add_log("!!!", "E-STOP ACTIVATED")
        self.display.set_mode(SystemMode.ESTOP)

        # Mark all axes as disabled
        for axis in AXES:
            self.display.set_enable_state(axis, EnableState.DISABLED)
        self._needs_redraw = True

    def _do_reset_and_ready(self):
        """Reset system and set to READY mode (D-pad Down)."""
        self.display.add_log("INFO", "Sending RST...")
        response = self.serial.send_command("RST")

        if response and "OK" in response:
            self.display.add_log("INFO", "RST OK, setting MODE READY...")
            time.sleep(0.1)  # Brief delay after reset

            response = self.serial.send_command("MODE READY")
            if response and "OK" in response:
                self.display.set_mode(SystemMode.READY)
                self.display.add_log("INFO", "Mode set to READY")
            else:
                self.display.add_log("WARN", f"MODE READY failed: {response}")
        else:
            self.display.add_log("ERROR", f"RST failed: {response}")

        self._needs_redraw = True

    def _do_save_home(self):
        """Save current positions as home."""
        response = self.serial.send_command("POS")
        if response:
            positions = self._parse_position_response(response)
            if positions:
                self.config.home_positions = positions
                self._save_config()
                self.display.add_log("INFO", f"Home positions saved: {positions}")
            else:
                self.display.add_log("ERROR", "Failed to parse position response")

    def _do_go_home(self):
        """Move all axes to home positions using configured speeds."""
        if not self.config.home_positions or all(v is None for v in self.config.home_positions.values()):
            self.display.add_log("ERROR", "No home positions saved. Press D-Right first.")
            return

        self.display.add_log("INFO", "Moving to home positions...")

        for axis in AXES:
            pos = self.config.home_positions.get(axis)
            if pos is not None:
                # Get speed from axis config
                axis_config = self.config.axes.get(axis)
                speed = axis_config.speed if axis_config else 0.001

                self.display.add_log("HOME", f"Moving {axis} to {pos:.6f} at {speed:.4f} m/s")
                response = self.serial.send_command(f"MOVE {axis} {pos:.6f} {speed:.6f}")
                if not response or not response.startswith("OK"):
                    self.display.add_log("WARN", f"Failed to move {axis}")
                # Brief delay between axes
                time.sleep(0.05)

        self.display.add_log("INFO", "Home sequence complete")

    def _do_axis_jog_start(self, axis: str, negative: bool):
        """Start jogging an axis."""
        axis_config = self.config.axes.get(axis)
        if not axis_config:
            return

        speed = axis_config.speed
        if negative:
            speed = -speed

        self.serial.send_command(f"VEL {axis} {speed:.6f}")
        self.display.axis_states[axis].is_moving = True

    def _do_axis_jog_stop(self, axis: str):
        """Stop jogging an axis."""
        if self.display.axis_states[axis].is_moving:
            self.serial.send_command(f"STOP {axis}")
            self.display.axis_states[axis].is_moving = False

    def _do_brake_toggle(self, axis: str):
        """Toggle brake for an axis."""
        axis_config = self.config.axes.get(axis)
        if not axis_config or not axis_config.has_brake:
            self.display.add_log("WARN", f"Axis {axis} has no brake hardware")
            return

        current_state = self.display.axis_states[axis].brake

        if current_state == BrakeState.ENGAGED:
            # Release brake
            response = self.serial.send_command(f"BRAKE {axis} 1")
            if response and response.startswith("OK"):
                self.display.set_brake_state(axis, BrakeState.RELEASED)
                self._needs_redraw = True
        else:
            # Engage brake
            response = self.serial.send_command(f"BRAKE {axis} 0")
            if response and response.startswith("OK"):
                self.display.set_brake_state(axis, BrakeState.ENGAGED)
                self._needs_redraw = True

    def _do_enable_toggle(self, axis: str):
        """Toggle enable for an axis."""
        current_state = self.display.axis_states[axis].enable

        if current_state == EnableState.ENABLED:
            # Disable
            response = self.serial.send_command(f"EN {axis} 0")
            if response and response.startswith("OK"):
                self.display.set_enable_state(axis, EnableState.DISABLED)
                self._needs_redraw = True
        else:
            # Enable and send POSOK
            response = self.serial.send_command(f"EN {axis} 1")
            if response and response.startswith("OK"):
                self.display.set_enable_state(axis, EnableState.ENABLED)
                # Send POSOK to clear position loss
                self.serial.send_command(f"POSOK {axis}")
                self._needs_redraw = True


# =============================================================================
# Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Joystick Control Tool for YaRobot Control Unit"
    )
    parser.add_argument(
        "--config", "-c",
        default=None,
        help="Path to config.yaml file"
    )
    parser.add_argument(
        "--port", "-p",
        default=None,
        help="Serial port override (e.g., /dev/ttyUSB0)"
    )
    args = parser.parse_args()

    # Find config file
    if args.config:
        config_path = args.config
    else:
        # Look in script directory
        script_dir = Path(__file__).parent
        config_path = script_dir / "config.yaml"

    if not os.path.exists(config_path):
        print(f"[ERROR] Config file not found: {config_path}")
        print("Create config.yaml or specify with --config")
        sys.exit(1)

    # Run application
    app = JoystickControlApp(str(config_path), args.port)
    app.run()


if __name__ == "__main__":
    main()
