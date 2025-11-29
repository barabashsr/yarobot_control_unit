# yarobot_control_unit - Product Requirements Document

**Author:** Sergey
**Date:** 2025-11-29
**Version:** 1.0

---

## Executive Summary

The YaRobot Control Unit transforms how engineers interface with motors and actuators by providing a universal, cost-effective bridge between PC software and diverse motor types. Instead of dealing with expensive industrial controllers or writing complex hardware-specific code, engineers get a simple, unified command interface that abstracts away all the low-level complexity.

This ESP32-S3 based controller manages up to 8 motors simultaneously - from precision servos to simple on/off actuators - through human-readable text commands over USB. It handles all the real-time control, safety monitoring, and peripheral management that typically requires specialized expertise or expensive equipment.

### What Makes This Special

**Universal Motor Abstraction** - One controller handles servos, steppers, and discrete actuators through a unified interface. No more juggling different protocols or libraries for different motor types.

**Radical Cost Reduction** - Custom ESP32-S3 design delivers industrial-grade capabilities at maker-friendly prices. Engineers can prototype and deploy without budget constraints.

**Zero Learning Curve** - Simple text commands like "MOVE X 100" replace pages of motor driver documentation. Works immediately with any serial terminal or programming language.

**Real-Time Performance** - FreeRTOS architecture ensures deterministic motor control with hardware pulse generation, making it suitable for precision applications without expensive motion controllers.

---

## Project Classification

**Technical Type:** iot_embedded
**Domain:** general (automation/robotics)
**Complexity:** medium

This is an embedded system project creating a hardware device with firmware that interfaces between host computers and physical motors/actuators. While it has IoT/embedded characteristics, it's designed as a general-purpose engineering tool rather than a domain-specific solution.

{{#if domain_context_summary}}

### Domain Context

{{domain_context_summary}}
{{/if}}

---

## Success Criteria

**Primary Success: Team Productivity**
- Eliminates weeks of low-level motor control coding per project
- Team members can configure and control motors in minutes instead of days
- No more searching for suitable commercial controllers that don't exist

**Technical Performance Targets**
- Step pulse generation up to ESP32-S3 practical limits (80-100 kHz reliable)
- Command-to-motion latency under 10ms for responsive operation
- Deterministic real-time performance without jitter
- Full ROS2 compatibility for integration with robotics workflows

**Reliability for Internal Use**
- Zero crashes or lockups during extended operation
- Robust handling of communication interruptions
- Clear error reporting when limits or faults occur
- Consistent behavior across power cycles

**Development Efficiency**
- New motor types can be added without firmware expertise
- Configuration through YAML without recompilation
- Simple enough that any team member can use it
- Well-documented for knowledge transfer

---

## Product Scope

### MVP - Minimum Viable Product

**Core Motor Control**
- Control all 8 motors (X,Y,Z,A,B,C,D,E) with position and velocity commands
- 5 servo axes with RMT/MCPWM for precise pulse generation
- Hardware: ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB PSRAM)
- 2 stepper axes with hardware position counting and discrete positioning
- 1 discrete axis (E) for simple on/off actuators
- Real-time motion execution with FreeRTOS task management

**Safety & Reliability**
- Hardware E-stop with immediate motor shutdown
- Limit switch monitoring on all axes (14 switches total)
- Brake control system with multiple strategies (on_disable, on_estop, on_idle)
- Automatic brake engagement on power loss (fail-safe)
- Position loss detection and recovery procedures

**Communication & Interface**
- USB CDC serial interface with human-readable commands
- Low-latency command processing (<10ms response time)
- G-code compatible command set (G00, G01, etc.)
- Comprehensive status reporting and event notifications
- Error handling with clear diagnostic messages

**Configuration**
- YAML-based configuration without recompilation
- Runtime adjustable parameters (ratios, limits, speeds)
- Persistent storage of calibration data
- Per-axis enable/disable control

### Growth Features (Post-MVP)

**Enhanced Interfaces**
- ROS2 hardware interface for ros2_control integration
- Mock ROS2 mode for testing without full ROS stack
- OLED display for field diagnostics and status
- Advanced motion profiles (S-curve acceleration)

**Extended Capabilities**
- Coordinated multi-axis motion
- Homing sequences with Z-signal support
- Servo feedback processing (InPos signals)
- Temperature monitoring for motor protection
- Position synchronization across axes

**Developer Features**
- Python/C++ libraries for host integration
- Simulation mode for offline development
- Expanded event system for complex sequencing
- Performance profiling and optimization tools

### Vision (Future)

**Advanced Control**
- Closed-loop control with encoder feedback
- Force/torque sensing integration
- Predictive maintenance alerts
- Machine learning for motion optimization

**Connectivity Options**
- Ethernet interface for network control
- Modbus RTU for industrial integration
- Wireless control options (WiFi/Bluetooth)
- Multi-controller synchronization

**Never Planned**
- Web interface or browser-based control
- Cloud connectivity or remote management
- Consumer/maker-focused features
- Mobile app control

---

{{#if domain_considerations}}

## Domain-Specific Requirements

{{domain_considerations}}

This section shapes all functional and non-functional requirements below.
{{/if}}

---

## Innovation & Novel Patterns

**Unified Command Interface Across Motor Types**

The key innovation is treating fundamentally different motor types (servos, steppers, discrete actuators) as abstract motion axes with a consistent command interface. Engineers use identical commands regardless of underlying hardware:

```
MOVE X 100   # Servo with feedback
MOVE C 100   # Stepper with counting  
MOVE E 100   # Calculated position actuator
```

This abstraction layer handles:
- Different pulse generation methods (RMT vs MCPWM vs LEDC)
- Position feedback variations (encoders vs counters vs time-based)
- Speed control differences (variable vs fixed)
- State management complexity

**Command Design Philosophy**

- **Human-first**: Commands readable without documentation
- **Protocol-agnostic**: Works with any serial terminal or language
- **Stateless options**: Each command self-contained
- **Progressive disclosure**: Simple commands with optional complexity
- **Event-driven**: Asynchronous feedback without polling

### Validation Approach

**Interface Validation**
- Test identical command sequences across all motor types
- Verify consistent behavior regardless of hardware differences
- Measure abstraction overhead impact on performance
- Validate with real team workflows and use cases

**Performance Validation**
- Confirm ESP32-S3 RMT+DMA achieves required pulse rates
- Verify sub-10ms command latency under load
- Test simultaneous 8-axis motion without interference
- Validate real-time determinism with FreeRTOS

---

{{#if project_type_requirements}}

## IoT/Embedded Specific Requirements

**Hardware Architecture**
- ESP32-S3-DevKitC-1 N16R8 as the core platform
- Generates STEP/DIR signals for external motor drivers
- Does NOT drive motors directly - interfaces with industrial servo and stepper drivers
- 24V logic level compatibility through TPIC6B595N shift registers

**Signal Generation**
- STEP pulses via RMT+DMA, MCPWM, or LEDC peripherals for precise timing
- DIR, ENABLE, and BRAKE signals via 3x TPIC6B595N shift registers (24V open-drain outputs)
- Hardware pulse counting for position feedback
- Native 24V logic level compatibility (no level shifters needed for motor drivers)

**Peripheral Expansion**
- 3x MCP23017 I2C expanders for:
  - 14 limit switch inputs (2 per axis)
  - 8 general purpose digital inputs
  - 8 general purpose digital outputs
  - Servo feedback signals (InPos, alarms)
- I2C bus at 400kHz for responsive I/O scanning
- Optional SSD1306 OLED display for diagnostics

**Physical Interface**
- USB-C for host communication and power
- Terminal blocks or pin headers for:
  - STEP/DIR/ENABLE outputs to motor drivers
  - Limit switch inputs
  - Emergency stop input
  - Auxiliary I/O
- Standard 0.1" pitch for easy prototyping

**Environmental Specifications**
- Indoor lab/office environment only
- Operating temperature: 15-30°C (room temperature)
- No IP rating required
- ESD protection on all external interfaces

**Firmware Architecture**
- ESP-IDF v5.0+ with FreeRTOS
- Modular design for motor type additions
- Hardware abstraction layer for peripherals
- Non-blocking architecture for real-time response

**Update & Maintenance**
- Standard ESP32 USB bootloader (no custom bootloader needed)
- Firmware updates via esptool.py or ESP Flash Tool
- Configuration backup/restore via YAML export
- No over-the-air (OTA) update requirement

{{#if endpoint_specification}}

### API Specification

{{endpoint_specification}}
{{/if}}

{{#if authentication_model}}

### Authentication & Authorization

{{authentication_model}}
{{/if}}

{{#if platform_requirements}}

### Platform Support

{{platform_requirements}}
{{/if}}

{{#if device_features}}

### Device Capabilities

{{device_features}}
{{/if}}

{{#if tenant_model}}

### Multi-Tenancy Architecture

{{tenant_model}}
{{/if}}

{{#if permission_matrix}}

### Permissions & Roles

{{permission_matrix}}
{{/if}}
{{/if}}

---

{{#if ux_principles}}

## User Experience Principles

{{ux_principles}}

### Key Interactions

{{key_interactions}}
{{/if}}

---

## Functional Requirements

**Motor Control Capabilities**

- FR1: System can control up to 8 independent motor axes simultaneously (X,Y,Z,A,B,C,D,E)
- FR2: System can generate STEP pulses up to 80-100 kHz for high-speed motion
- FR3: System can control 5 servo motors through STEP/DIR interfaces with position feedback
- FR4: System can control 2 stepper motors with hardware position counting
- FR5: System can control 1 discrete actuator (E axis) with time-based position calculation
- FR6: Each axis can execute absolute position moves (G00/G01)
- FR7: Each axis can execute relative position moves (G91 mode)
- FR8: Each axis can execute continuous jog movements
- FR9: System can stop any axis motion immediately on command
- FR10: System can enable/disable individual axes independently

**Safety and Limit Management**

- FR11: System can monitor 14 limit switches (2 per axis) in real-time
- FR12: System can execute hardware E-stop to disable all motors immediately
- FR13: System can automatically stop axis motion when limit switches activate
- FR14: System can control motor brakes with configurable strategies
- FR15: System can engage brakes automatically on power loss (fail-safe)
- FR16: System can detect and report position loss conditions
- FR17: System can prevent motion commands that would exceed configured limits
- FR18: Users can configure limit switch polarity and behavior

**Communication and Commands**

- FR19: System accepts commands via USB CDC serial interface
- FR20: System responds to commands within 10ms
- FR21: System accepts G-code compatible motion commands
- FR22: System accepts human-readable text commands (MOVE, STATUS, etc.)
- FR23: System provides command acknowledgment and error responses
- FR24: System generates asynchronous event notifications
- FR25: System reports comprehensive status for all axes on demand
- FR26: System maintains command history for debugging

**Configuration and Calibration**

- FR27: System can load configuration from YAML files via USB
- FR28: System can export current configuration to YAML format
- FR29: Users can adjust motion parameters at runtime (speed, acceleration, limits)
- FR30: System persists calibration data across power cycles
- FR31: Users can clear axis positions without physical movement
- FR32: Users can set custom scaling ratios per axis
- FR33: System can execute homing sequences for each axis
- FR34: Users can name axes with custom aliases for clarity

**I/O and Peripheral Control**

- FR35: System can read 8 general-purpose digital inputs
- FR36: System can control 8 general-purpose digital outputs
- FR37: System can process servo feedback signals (InPos, alarms)
- FR38: System can detect floating switch activation on C axis (picker jaw)
- FR39: System can measure object width using C axis floating switch and report via event
- FR40: System can query last measured object width from C axis
- FR41: System provides configurable input debouncing
- FR42: Users can read/write I/O using pin names or aliases

**Status and Monitoring**

- FR43: System reports real-time position for all axes
- FR44: System reports motion status (idle, moving, error) per axis
- FR45: System tracks and reports cumulative error counts
- FR46: System monitors I2C communication health
- FR47: System can display status on optional OLED screen
- FR48: System generates events for motion completion
- FR49: System generates events for errors and faults
- FR50: Users can query individual axis status or all axes

**Operational Modes**

- FR51: System supports normal operation mode for motion control
- FR52: System supports configuration mode for YAML updates
- FR53: System can switch between operational modes on command
- FR54: System indicates current mode via status responses
- FR55: System prevents motion commands in configuration mode

**Error Handling**

- FR56: System detects and reports I2C communication failures
- FR57: System provides detailed error messages with context
- FR58: System can recover from transient communication errors
- FR59: System logs errors for troubleshooting
- FR60: Users can clear error conditions after resolution
- FR61: System prevents unsafe operations when in error state

---

## Non-Functional Requirements

### Performance

- NFR1: Step pulse generation must maintain timing accuracy within 1% up to 80 kHz
- NFR2: Command response latency must not exceed 10ms under normal load
- NFR3: System must handle simultaneous motion on all 8 axes without degradation
- NFR4: I2C polling cycle must complete within 5ms for responsive limit detection
- NFR5: Motion commands must execute with deterministic timing (no jitter >1ms)
- NFR6: System must maintain real-time performance with 1000Hz motion update rate

### Reliability

- NFR7: System must operate continuously for 24+ hours without crashes or lockups
- NFR8: System must handle USB disconnection/reconnection gracefully
- NFR9: System must recover from I2C bus errors without requiring restart
- NFR10: Configuration must persist correctly across 10,000+ power cycles
- NFR11: System must detect and report hardware faults before they cause damage
- NFR12: Brake engagement must occur within 50ms of trigger condition

### Integration

- NFR13: USB CDC interface must work with standard serial terminal programs
- NFR14: Commands must be parseable by common programming languages (Python, C++, etc.)
- NFR15: System must support future ROS2 hardware_interface integration
- NFR16: YAML configuration must follow standard YAML 1.2 specification
- NFR17: G-code subset must be compatible with common CNC conventions
- NFR18: Event messages must be distinguishable from command responses

### Maintainability

- NFR19: Firmware must be updatable via standard ESP32 tools (no custom programmer)
- NFR20: Error messages must clearly indicate root cause and suggested resolution
- NFR21: New motor types must be addable without modifying core architecture
- NFR22: Configuration changes must not require recompilation
- NFR23: System must provide diagnostic data for troubleshooting
- NFR24: Code must follow ESP-IDF coding standards for team consistency

---

_This PRD captures the essence of YaRobot Control Unit - a universal motor control interface that transforms weeks of low-level programming into minutes of simple configuration._

_Created through collaborative discovery between Sergey and AI facilitator._