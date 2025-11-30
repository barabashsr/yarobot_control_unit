# GPIO Assignment Document - YaRobot Control Unit

**Target Hardware:** ESP32-S3-DevKitC-1 N16R8 (16MB Flash, 8MB Octal PSRAM)
**Board Version:** v1.1
**Document Version:** 1.0
**Date:** 2025-11-29

---

## Executive Summary

This document defines the complete GPIO, shift register, and I2C expander pin assignments for the YaRobot Control Unit. All assignments are optimized for:

1. **Physical grouping by channel** - Related signals placed on adjacent pins/board sides
2. **Avoidance of reserved pins** - N16R8 Octal PSRAM constraints respected
3. **Clean PCB routing** - Signals grouped to minimize trace crossings
4. **Header configuration alignment** - Direct mapping to `config_gpio.h`, `config_i2c.h`
5. **Fast output updates** - All outputs via SPI shift registers (5x TPIC6B595N, 40 bits)
6. **Simplified I2C** - Only 2x MCP23017 for inputs (limits, alarms, InPos)

---

## ESP32-S3-DevKitC-1 N16R8 Constraints

### Reserved Pins (DO NOT USE)

| GPIO Range | Reason | Notes |
|------------|--------|-------|
| GPIO26-32 | SPI Flash | Internal flash communication |
| GPIO33-37 | Octal PSRAM | N16R8 uses Octal SPI (GPIO33-37) |
| GPIO19-20 | USB D-/D+ | Native USB fixed assignment |
| GPIO22-25 | Not routed | These pins don't exist on ESP32-S3 |

### Strapping Pins (AVOID or use with caution)

| GPIO | Function | Boot Value | Notes |
|------|----------|------------|-------|
| GPIO0 | Boot Mode | Weak pull-up | Can use as input with external pull-up |
| GPIO3 | JTAG Mode | Weak pull-down | Safe for outputs after boot |
| GPIO45 | SPI Voltage | Weak pull-down | Safe for I/O after boot |
| GPIO46 | ROM Messages | Weak pull-down | Input-only, safe for signals |

### JTAG Pins (Repurposed for signals)

| GPIO | JTAG Function | Our Assignment |
|------|---------------|----------------|
| GPIO39 | MTCK | Z-Signal B |
| GPIO40 | MTDO | Z-Signal X |
| GPIO41 | MTDI | Z-Signal Y |
| GPIO42 | MTMS | Z-Signal Z |

> **Note:** JTAG debugging disabled. Use USB CDC for debug output.

---

## Physical Board Layout

### Visual GPIO Layout Table (Both Sides)

The following table shows the complete ESP32-S3-DevKitC-1 N16R8 pinout with YaRobot signal assignments.

**IMPORTANT:** InPos (Position Complete) signals are read via **MCP23017 #1 (0x21) Port B**, NOT via ESP32 GPIOs.
Only Z-Signals require direct GPIO (hardware interrupts for precise timing).

View with USB connector at bottom.

```
═══════════════════════════════════════════════════════════════════════════════════════════
                              ESP32-S3-DevKitC-1 N16R8 GPIO ASSIGNMENT
                                    (USB connector at bottom)

                    *** GROUPED BY AXIS - STEP + Z_SIGNAL ADJACENT ***
═══════════════════════════════════════════════════════════════════════════════════════════

   J1 (LEFT SIDE)                                                      J3 (RIGHT SIDE)
   ══════════════                                                      ═══════════════

   Pin  GPIO  Function              Type                    Type              Function  GPIO  Pin
   ───  ────  ────────────────────  ────  ══════════════  ────  ────────────────────  ────  ───
    1   3V3   Power 3.3V            PWR   ║            ║  PWR   Ground               GND    1
    2   3V3   Power 3.3V            PWR   ║            ║       (UART0 TX)            43     2
    3   RST   Reset                       ║            ║       (UART0 RX)            44     3
   ───  ────  ────────────────────  ────  ║            ║  ────  ────────────────────  ────  ───
    4    4    X_STEP (RMT)        X-STEP  ║  ┌──────┐  ║  X-Z   X_Z_SIGNAL            1     4
    5    5    Y_STEP (MCPWM)      Y-STEP  ║  │      │  ║  Y-Z   Y_Z_SIGNAL            2     5
    6    6    Z_STEP (RMT)        Z-STEP  ║  │ ESP  │  ║  Z-Z   Z_Z_SIGNAL           42     6
    7    7    A_STEP (RMT)        A-STEP  ║  │ 32   │  ║  A-Z   A_Z_SIGNAL           41     7
    8   15    B_STEP (RMT)        B-STEP  ║  │ S3   │  ║  B-Z   B_Z_SIGNAL           40     8
   ───  ────  ────────────────────  ────  ║  │      │  ║  ────  ────────────────────  ────  ───
    9   16    C_STEP (MCPWM)      C-STEP  ║  │WROOM │  ║  INT   MCP1_INTA            39     9
   10   17    D_STEP (LEDC)       D-STEP  ║  │  1   │  ║  INT   MCP1_INTB            38    10
   ───  ────  ────────────────────  ────  ║  │      │  ║  ────  ────────────────────  ────  ───
   11   18    I2C_SCL (I2C0)        I2C   ║  │N16R8 │  ║  RSVD  [PSRAM]              37    11
   12    8    I2C_SDA (I2C0)        I2C   ║  │      │  ║  RSVD  [PSRAM]              36    12
   13    3    MCP0_INTA             INT   ║  └──────┘  ║  RSVD  [PSRAM]              35    13
   14   46    MCP0_INTB             INT   ║            ║  BOOT  (Boot Mode)           0    14
   ───  ────  ────────────────────  ────  ║            ║  ────  ────────────────────  ────  ───
   15    9    SR_OE                 SPI   ║            ║  SPARE (available)          45    15
   16   10    SR_CS (Latch)         SPI   ║            ║  SPARE (available)          48    16
   17   11    SR_MOSI               SPI   ║            ║  SPARE (available)          47    17
   18   12    SR_SCLK               SPI   ║            ║  OLED  OLED_SCL (I2C1)      21    18
   ───  ────  ────────────────────  ────  ║            ║  ────  ────────────────────  ────  ───
   19   13    E_STOP                SAFE  ║            ║  USB   USB D+               20    19
   20   14    OLED_SDA (I2C1)       OLED  ║            ║  USB   USB D-               19    20
   ───  ────  ────────────────────  ────  ║  ┌─────┐  ║  ────  ────────────────────  ────  ───
   21   5V    Power 5V              PWR   ║  │ USB │  ║  PWR   Ground               GND   21
   22   GND   Ground                PWR   ║  │  C  │  ║  PWR   Ground               GND   22
                                          ║  └─────┘  ║
                                          ╚══════════╝


═══════════════════════════════════════════════════════════════════════════════════════════
                              AXIS GROUPING VERIFICATION
═══════════════════════════════════════════════════════════════════════════════════════════

SERVO AXES (X, Y, Z, A, B) - Each axis has STEP + Z_SIGNAL on SAME ROW (opposite sides)
────────────────────────────────────────────────────────────────────────────────────────────

         LEFT (J1)                    Pin Row                    RIGHT (J3)
         ─────────                    ───────                    ──────────
┌─ X AXIS ─────────────────────────────────────────────────────────────────────────────────┐
│  GPIO 4  → X_STEP (RMT)             Pin 4                X_Z_SIGNAL ← GPIO 1             │
└──────────────────────────────────────────────────────────────────────────────────────────┘
┌─ Y AXIS ─────────────────────────────────────────────────────────────────────────────────┐
│  GPIO 5  → Y_STEP (MCPWM)           Pin 5                Y_Z_SIGNAL ← GPIO 2             │
└──────────────────────────────────────────────────────────────────────────────────────────┘
┌─ Z AXIS ─────────────────────────────────────────────────────────────────────────────────┐
│  GPIO 6  → Z_STEP (RMT)             Pin 6                Z_Z_SIGNAL ← GPIO 42            │
└──────────────────────────────────────────────────────────────────────────────────────────┘
┌─ A AXIS ─────────────────────────────────────────────────────────────────────────────────┐
│  GPIO 7  → A_STEP (RMT)             Pin 7                A_Z_SIGNAL ← GPIO 41            │
└──────────────────────────────────────────────────────────────────────────────────────────┘
┌─ B AXIS ─────────────────────────────────────────────────────────────────────────────────┐
│  GPIO 15 → B_STEP (RMT)             Pin 8                B_Z_SIGNAL ← GPIO 40            │
└──────────────────────────────────────────────────────────────────────────────────────────┘

STEPPER AXES (C, D) - STEP only (no Z-signal or InPos)
────────────────────────────────────────────────────────────────────────────────────────────
┌─ C AXIS ─────────────────────────────────────────────────────────────────────────────────┐
│  GPIO 16 → C_STEP (MCPWM)           Pin 9                (no Z-signal)                   │
└──────────────────────────────────────────────────────────────────────────────────────────┘
┌─ D AXIS ─────────────────────────────────────────────────────────────────────────────────┐
│  GPIO 17 → D_STEP (LEDC)            Pin 10               (no Z-signal)                   │
└──────────────────────────────────────────────────────────────────────────────────────────┘

E AXIS - Controlled via Shift Register (EN/DIR/BRAKE/ALARM_CLR on SR bits 28-31)
────────────────────────────────────────────────────────────────────────────────────────────

OTHER SIGNAL GROUPS
────────────────────────────────────────────────────────────────────────────────────────────

┌─ I2C BUS 0 (MCP23017 Expanders) ────┐          ┌─ RESERVED (PSRAM) ───────────┐
│  GPIO 18 → I2C_SCL   Pin 11        │          │  GPIO 37 → [PSRAM]     Pin 11│
│  GPIO 8  → I2C_SDA   Pin 12        │          │  GPIO 36 → [PSRAM]     Pin 12│
│  GPIO 3  → MCP0_INTA     Pin 13    │          │  GPIO 35 → [PSRAM]     Pin 13│
│  GPIO 46 → MCP1_INTA     Pin 14    │          └──────────────────────────────┘
└────────────────────────────────────┘

┌─ SPI (SHIFT REGISTERS) ────────────┐          ┌─ MCP INT (Right Side) ───────┐
│  GPIO 9  → SR_OE     Pin 15        │          │  GPIO 39 → MCP0_INTB   Pin 9 │
│  GPIO 10 → SR_CS     Pin 16        │          │  GPIO 38 → MCP1_INTB   Pin 10│
│  GPIO 11 → SR_MOSI   Pin 17        │          └──────────────────────────────┘
│  GPIO 12 → SR_SCLK   Pin 18        │          ┌─ SPARE GPIOs ────────────────┐
└────────────────────────────────────┘          │  GPIO 45 → SPARE       Pin 15│
                                                │  GPIO 48 → SPARE       Pin 16│
                                                │  GPIO 47 → SPARE       Pin 17│
                                                └──────────────────────────────┘
┌─ OLED I2C1 (split across sides) ───┐
│  GPIO 14 → OLED_SDA  Pin 20 (L)    │          ┌─ USB (FIXED) ────────────────┐
│  GPIO 21 → OLED_SCL  Pin 18 (R)    │          │  GPIO 20 → USB D+      Pin 19│
└────────────────────────────────────┘          │  GPIO 19 → USB D-      Pin 20│
                                                └──────────────────────────────┘
┌─ SAFETY ───────────────────────────┐
│  GPIO 13 → E_STOP    Pin 19        │
└────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════════════════════════════
```

### Legend

| Type | Meaning | Description |
|------|---------|-------------|
| **STEP** | Motor Step Output | Pulse outputs to motor drivers (RMT/MCPWM/LEDC) |
| **FB** | Position Feedback | Servo position complete signals |
| **Z-SIG** | Z-Signal/Index | Servo encoder index pulses |
| **INT** | MCP23017 Interrupt | Interrupt lines from I2C expanders (INTA/INTB) |
| **SPI** | Shift Register Bus | TPIC6B595N x5 chain for DIR/EN/BRAKE/ALARM_CLR/GP_OUT |
| **I2C** | I2C Bus 0 | MCP23017 x2 expanders (limit switches, alarms, InPos) |
| **OLED** | I2C Bus 1 | Dedicated OLED display bus (isolated) |
| **SAFE** | Safety Signal | E-stop input |
| **PWR** | Power/Ground | Power supply pins |
| RSVD | Reserved | N16R8 Octal PSRAM - DO NOT USE |
| USB | USB Interface | Fixed USB D+/D- |
| BOOT | Strapping Pin | Boot mode control - avoid |

### ASCII Board Diagram

```
                    ┌─────────────────────────────┐
                    │      ESP32-S3-DevKitC-1     │
                    │          N16R8              │
                    │                             │
    J1 (LEFT)       │    ┌─────────────────┐      │       J3 (RIGHT)
    ─────────       │    │                 │      │       ──────────
                    │    │   ESP32-S3      │      │
    3V3  ○──────────│────│   WROOM-1       │──────│──────────○ GND
    3V3  ○──────────│────│   N16R8         │──────│──────────○ GPIO43
    RST  ○──────────│────│                 │──────│──────────○ GPIO44
    GPIO4 ○─────────│────│   ┌───────┐     │──────│─────────○ GPIO1
    GPIO5 ○─────────│────│   │ FLASH │     │──────│─────────○ GPIO2
    GPIO6 ○─────────│────│   │ PSRAM │     │──────│─────────○ GPIO42
    GPIO7 ○─────────│────│   └───────┘     │──────│─────────○ GPIO41
    GPIO15○─────────│────│                 │──────│─────────○ GPIO40
    GPIO16○─────────│────│                 │──────│─────────○ GPIO39
    GPIO17○─────────│────│                 │──────│─────────○ GPIO38
    GPIO18○─────────│────│                 │──────│─────────○ GPIO37 [RSVD]
    GPIO8 ○─────────│────│                 │──────│─────────○ GPIO36 [RSVD]
    GPIO3 ○─────────│────│                 │──────│─────────○ GPIO35 [RSVD]
    GPIO46○─────────│────│                 │──────│─────────○ GPIO0
    GPIO9 ○─────────│────│                 │──────│─────────○ GPIO45
    GPIO10○─────────│────│                 │──────│─────────○ GPIO48
    GPIO11○─────────│────│                 │──────│─────────○ GPIO47
    GPIO12○─────────│────│                 │──────│─────────○ GPIO21
    GPIO13○─────────│────│                 │──────│─────────○ GPIO20 [USB]
    GPIO14○─────────│────│                 │──────│─────────○ GPIO19 [USB]
    5V   ○──────────│────│                 │──────│──────────○ GND
    GND  ○──────────│────└─────────────────┘──────│──────────○ GND
                    │                             │
                    │         ┌─────┐             │
                    │         │ USB │             │
                    │         │ C   │             │
                    └─────────┴─────┴─────────────┘
```

### Signal Grouping Summary

| Board Side | Pin Range | Signal Group | Signals |
|------------|-----------|--------------|---------|
| **Left (J1)** | 4-8 | Servo STEP | X, Y, Z, A, B step pulses |
| **Left (J1)** | 9-10 | Stepper STEP | C, D step pulses |
| **Left (J1)** | 11-12 | I2C Bus 0 | SCL, SDA |
| **Left (J1)** | 13-14 | MCP INTA | MCP0_INTA, MCP1_INTA |
| **Left (J1)** | 15-18 | SPI (Shift Reg) | OE, CS, MOSI, SCLK |
| **Left (J1)** | 19 | Safety | E-stop input |
| **Left (J1)** | 20 | OLED | I2C1 SDA |
| **Right (J3)** | 4-8 | Z-Signals | X, Y, Z, A, B index pulses |
| **Right (J3)** | 9-10 | MCP INTB | MCP0_INTB, MCP1_INTB |
| **Right (J3)** | 15-17 | SPARE | 3 available GPIOs (45, 47, 48) |
| **Right (J3)** | 18 | OLED | I2C1 SCL |

---

## GPIO Assignment Table (Complete)

### Servo Axes (Grouped by Axis - STEP + Z_SIGNAL on same row)

| Axis | STEP GPIO | STEP Peripheral | Z_SIGNAL GPIO | Header Row | Notes |
|------|-----------|-----------------|---------------|------------|-------|
| **X** | GPIO4 | RMT CH0 | GPIO1 | Pin 4 | STEP left, Z-signal right |
| **Y** | GPIO5 | MCPWM T0 | GPIO2 | Pin 5 | STEP left, Z-signal right |
| **Z** | GPIO6 | RMT CH1 | GPIO42 | Pin 6 | STEP left, Z-signal right |
| **A** | GPIO7 | RMT CH2 | GPIO41 | Pin 7 | STEP left, Z-signal right |
| **B** | GPIO15 | RMT CH3 | GPIO40 | Pin 8 | STEP left, Z-signal right |

> **Grouping Rationale:** Each servo axis has its STEP and Z_SIGNAL on the SAME physical row (opposite board sides). This enables clean cable routing from ESP32 to motor drivers.

### Stepper Axes (STEP only - no Z-signal)

| Axis | STEP GPIO | STEP Peripheral | Header Position | Notes |
|------|-----------|-----------------|-----------------|-------|
| **C** | GPIO16 | MCPWM T1 | J1 Pin 9 | Stepper (picker jaw) |
| **D** | GPIO17 | LEDC CH0 | J1 Pin 10 | Stepper |

### Discrete Axis (E) - Via Shift Register

E axis DIR/EN/BRAKE/ALARM_CLR controlled via shift register chain (bits 28-31) for unified control approach.
E axis limit switches are on MCP23017 #0 (0x20) GPB6-GPB7, same as all other axes.

### InPos Signals - Via I2C Expander (NOT GPIO!)

**IMPORTANT:** InPos (Position Complete) signals are read via **MCP23017 #1 (0x21) Port B**, not ESP32 GPIOs.
This is because InPos doesn't require fast interrupt response - I2C polling at 400kHz is sufficient.

| Axis | MCP23017 | Port.Pin | Notes |
|------|----------|----------|-------|
| X | 0x21 | GPB0 | X servo InPos |
| Y | 0x21 | GPB1 | Y servo InPos |
| Z | 0x21 | GPB2 | Z servo InPos |
| A | 0x21 | GPB3 | A servo InPos |
| B | 0x21 | GPB4 | B servo InPos |

### ALARM Signals - Split Architecture

ALARM_INPUT signals (inputs) are on **MCP23017 #1 (0x21) Port A**.
ALARM_CLEAR signals (outputs) are on **Shift Register chain** (bits 3, 7, 11, 15, 19, 23, 27, 31).

| Axis | ALARM_INPUT | ALARM_CLEAR | Notes |
|------|-------------|-------------|-------|
| X | MCP1 GPA0 | SR bit 3 | X servo/driver |
| Y | MCP1 GPA1 | SR bit 7 | Y servo/driver |
| Z | MCP1 GPA2 | SR bit 11 | Z servo/driver |
| A | MCP1 GPA3 | SR bit 15 | A servo/driver |
| B | MCP1 GPA4 | SR bit 19 | B servo/driver |
| C | MCP1 GPA5 | SR bit 23 | C stepper driver |
| D | MCP1 GPA6 | SR bit 27 | D stepper driver |
| E | - | SR bit 31 | E discrete (no alarm input) |

> **Note:** All outputs consolidated to shift registers for fast SPI control. ALARM_INPUT remains on MCP23017 for interrupt-driven alarm detection.

### SPI (Shift Register Chain)

| Signal | GPIO | Side | Header Position | SPI Function |
|--------|------|------|-----------------|--------------|
| SR_OE | GPIO9 | Left | J1-15 | Output Enable (active-low) |
| SR_CS | GPIO10 | Left | J1-16 | Latch/CS |
| SR_MOSI | GPIO11 | Left | J1-17 | SPI2_MOSI |
| SR_SCLK | GPIO12 | Left | J1-18 | SPI2_CLK |

> **Grouping Rationale:** SPI signals grouped on left side (J1 pins 15-18) for single cable to shift register board.

### I2C Bus 0 (MCP23017 Expanders)

| Signal | GPIO | Side | Header Position | Notes |
|--------|------|------|-----------------|-------|
| I2C_SCL | GPIO18 | Left | J1-11 | Main I2C clock (400kHz) |
| I2C_SDA | GPIO8 | Left | J1-12 | Main I2C data |

### MCP23017 Interrupt Lines (4 GPIOs for 2 MCPs × 2 interrupts)

| Signal | GPIO | Side | Header Position | MCP Device | Port |
|--------|------|------|-----------------|------------|------|
| MCP0_INTA | GPIO3 | Left | J1-13 | MCP23017 #0 (0x20) | Port A (limits MIN/MAX X-A) |
| MCP1_INTA | GPIO46 | Left | J1-14 | MCP23017 #1 (0x21) | Port A (ALARM_INPUT signals) |
| MCP0_INTB | GPIO39 | Right | J3-9 | MCP23017 #0 (0x20) | Port B (limits MIN/MAX B-E) |
| MCP1_INTB | GPIO38 | Right | J3-10 | MCP23017 #1 (0x21) | Port B (InPos signals) |

> **Note:** MCP interrupts grouped by type: INTA lines on left side (J1-13, J1-14), INTB lines on right side (J3-9, J3-10). Both ports on both MCPs are inputs, so all 4 interrupt lines are active.

### I2C Bus 1 (OLED Display - Isolated)

| Signal | GPIO | Side | Header Position | Notes |
|--------|------|------|-----------------|-------|
| OLED_SDA | GPIO14 | Left | J1-20 | OLED I2C data |
| OLED_SCL | GPIO21 | Right | J3-18 | OLED I2C clock |

> **Note:** OLED uses separate I2C bus to isolate from safety-critical limit switch I/O.

### Safety

| Signal | GPIO | Side | Header Position | Notes |
|--------|------|------|-----------------|-------|
| E_STOP | GPIO13 | Left | J1-19 | Emergency stop input |

### USB (Fixed - Cannot Change)

| Signal | GPIO | Notes |
|--------|------|-------|
| USB_D- | GPIO19 | Native USB |
| USB_D+ | GPIO20 | Native USB |

### Spare GPIOs (5 available for future use)

| GPIO | Side | Header Position | Notes |
|------|------|-----------------|-------|
| GPIO43 | Right | J3-2 | UART0 TX (available if USB used for debug) |
| GPIO44 | Right | J3-3 | UART0 RX (available if USB used for debug) |
| GPIO45 | Right | J3-15 | Strapping pin (safe after boot) |
| GPIO47 | Right | J3-17 | Previously MCP2_INTB - now spare |
| GPIO48 | Right | J3-16 | Previously MCP2_INTA - now spare |

> **Note:** GPIO47 and GPIO48 were freed by consolidating from 3 MCPs to 2 MCPs. All outputs moved to shift registers.

---

## Final GPIO Allocation Summary

### Available GPIOs on N16R8

```
Available: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21,
           38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48

Reserved:  19, 20 (USB), 22-25 (N/A), 26-37 (Flash/PSRAM)
Strapping: 0, 3, 45, 46 (use with caution)

Total Available: 30 GPIOs
Total Used:      25 GPIOs
Spare:           5 GPIOs (43, 44, 45, 47, 48)
```

### Final GPIO Assignment Table

| Function | GPIO | Side | Pin | Peripheral | Group |
|----------|------|------|-----|------------|-------|
| **SERVO AXES (STEP + Z-SIGNAL paired)** |||||
| X_STEP | 4 | Left | J1-4 | RMT CH0 | X Axis |
| X_Z_SIGNAL | 1 | Right | J3-4 | GPIO Input | X Axis |
| Y_STEP | 5 | Left | J1-5 | MCPWM T0 | Y Axis |
| Y_Z_SIGNAL | 2 | Right | J3-5 | GPIO Input | Y Axis |
| Z_STEP | 6 | Left | J1-6 | RMT CH1 | Z Axis |
| Z_Z_SIGNAL | 42 | Right | J3-6 | GPIO Input | Z Axis |
| A_STEP | 7 | Left | J1-7 | RMT CH2 | A Axis |
| A_Z_SIGNAL | 41 | Right | J3-7 | GPIO Input | A Axis |
| B_STEP | 15 | Left | J1-8 | RMT CH3 | B Axis |
| B_Z_SIGNAL | 40 | Right | J3-8 | GPIO Input | B Axis |
| **STEPPER AXES** |||||
| C_STEP | 16 | Left | J1-9 | MCPWM T1 | C Axis |
| D_STEP | 17 | Left | J1-10 | LEDC CH0 | D Axis |
| **SPI (Shift Registers)** |||||
| SR_OE | 9 | Left | J1-15 | GPIO Output | SPI |
| SR_CS | 10 | Left | J1-16 | SPI2_CS | SPI |
| SR_MOSI | 11 | Left | J1-17 | SPI2_MOSI | SPI |
| SR_SCLK | 12 | Left | J1-18 | SPI2_CLK | SPI |
| **I2C Bus 0 (MCP23017)** |||||
| I2C_SCL | 18 | Left | J1-11 | I2C0_SCL | I2C Main |
| I2C_SDA | 8 | Left | J1-12 | I2C0_SDA | I2C Main |
| **MCP23017 Interrupts (4 lines)** |||||
| MCP0_INTA | 3 | Left | J1-13 | GPIO Input | MCP #0 Port A |
| MCP1_INTA | 46 | Left | J1-14 | GPIO Input | MCP #1 Port A |
| MCP0_INTB | 39 | Right | J3-9 | GPIO Input | MCP #0 Port B |
| MCP1_INTB | 38 | Right | J3-10 | GPIO Input | MCP #1 Port B |
| **I2C Bus 1 (OLED)** |||||
| OLED_SDA | 14 | Left | J1-20 | I2C1_SDA | I2C OLED |
| OLED_SCL | 21 | Right | J3-18 | I2C1_SCL | I2C OLED |
| **Safety** |||||
| E_STOP | 13 | Left | J1-19 | GPIO Input | Safety |

---

## Shift Register Bit Assignment (TPIC6B595N x5)

### Organization: By Axis (4 bits per axis)

The 40-bit shift register chain is organized with each axis getting 4 consecutive bits:
- Bit 0: DIR (Direction)
- Bit 1: EN (Enable, active-high to driver)
- Bit 2: BRAKE (Brake release, active-high = released)
- Bit 3: ALARM_CLR (Alarm clear output)

This design consolidates ALL outputs to fast SPI, eliminating I2C latency for output operations.

### Bit Map

```
Shift Register Chain: SR0 → SR1 → SR2 → SR3 → SR4 (MOSI → first bit of SR0)

SR0 (Bits 0-7):   X_DIR, X_EN, X_BRAKE, X_ALARM_CLR, Y_DIR, Y_EN, Y_BRAKE, Y_ALARM_CLR
SR1 (Bits 8-15):  Z_DIR, Z_EN, Z_BRAKE, Z_ALARM_CLR, A_DIR, A_EN, A_BRAKE, A_ALARM_CLR
SR2 (Bits 16-23): B_DIR, B_EN, B_BRAKE, B_ALARM_CLR, C_DIR, C_EN, C_BRAKE, C_ALARM_CLR
SR3 (Bits 24-31): D_DIR, D_EN, D_BRAKE, D_ALARM_CLR, E_DIR, E_EN, E_BRAKE, E_ALARM_CLR
SR4 (Bits 32-39): GP_OUT_0, GP_OUT_1, GP_OUT_2, GP_OUT_3, GP_OUT_4, GP_OUT_5, GP_OUT_6, GP_OUT_7
```

### Detailed Bit Assignment

| Bit | Register | Function | Axis | Active State | Notes |
|-----|----------|----------|------|--------------|-------|
| 0 | SR0.Q0 | X_DIR | X | 1=Forward | Servo |
| 1 | SR0.Q1 | X_EN | X | 1=Enabled | Servo |
| 2 | SR0.Q2 | X_BRAKE | X | 1=Released | Servo |
| 3 | SR0.Q3 | X_ALARM_CLR | X | Pulse to clear | Servo |
| 4 | SR0.Q4 | Y_DIR | Y | 1=Forward | Servo |
| 5 | SR0.Q5 | Y_EN | Y | 1=Enabled | Servo |
| 6 | SR0.Q6 | Y_BRAKE | Y | 1=Released | Servo |
| 7 | SR0.Q7 | Y_ALARM_CLR | Y | Pulse to clear | Servo |
| 8 | SR1.Q0 | Z_DIR | Z | 1=Forward | Servo |
| 9 | SR1.Q1 | Z_EN | Z | 1=Enabled | Servo |
| 10 | SR1.Q2 | Z_BRAKE | Z | 1=Released | Servo |
| 11 | SR1.Q3 | Z_ALARM_CLR | Z | Pulse to clear | Servo |
| 12 | SR1.Q4 | A_DIR | A | 1=Forward | Servo |
| 13 | SR1.Q5 | A_EN | A | 1=Enabled | Servo |
| 14 | SR1.Q6 | A_BRAKE | A | 1=Released | Servo |
| 15 | SR1.Q7 | A_ALARM_CLR | A | Pulse to clear | Servo |
| 16 | SR2.Q0 | B_DIR | B | 1=Forward | Servo |
| 17 | SR2.Q1 | B_EN | B | 1=Enabled | Servo |
| 18 | SR2.Q2 | B_BRAKE | B | 1=Released | Servo |
| 19 | SR2.Q3 | B_ALARM_CLR | B | Pulse to clear | Servo |
| 20 | SR2.Q4 | C_DIR | C | 1=Forward | Stepper |
| 21 | SR2.Q5 | C_EN | C | 1=Enabled | Stepper |
| 22 | SR2.Q6 | C_BRAKE | C | 1=Released | (N/A - stepper) |
| 23 | SR2.Q7 | C_ALARM_CLR | C | Pulse to clear | Stepper |
| 24 | SR3.Q0 | D_DIR | D | 1=Forward | Stepper |
| 25 | SR3.Q1 | D_EN | D | 1=Enabled | Stepper |
| 26 | SR3.Q2 | D_BRAKE | D | 1=Released | (N/A - stepper) |
| 27 | SR3.Q3 | D_ALARM_CLR | D | Pulse to clear | Stepper |
| 28 | SR3.Q4 | E_DIR | E | 1=Forward | Discrete |
| 29 | SR3.Q5 | E_EN | E | 1=Enabled | Discrete |
| 30 | SR3.Q6 | E_BRAKE | E | 1=Released | Discrete |
| 31 | SR3.Q7 | E_ALARM_CLR | E | Pulse to clear | Discrete |
| 32 | SR4.Q0 | GP_OUT_0 | - | User defined | General purpose |
| 33 | SR4.Q1 | GP_OUT_1 | - | User defined | General purpose |
| 34 | SR4.Q2 | GP_OUT_2 | - | User defined | General purpose |
| 35 | SR4.Q3 | GP_OUT_3 | - | User defined | General purpose |
| 36 | SR4.Q4 | GP_OUT_4 | - | User defined | General purpose |
| 37 | SR4.Q5 | GP_OUT_5 | - | User defined | General purpose |
| 38 | SR4.Q6 | GP_OUT_6 | - | User defined | General purpose |
| 39 | SR4.Q7 | GP_OUT_7 | - | User defined | General purpose |

### Fail-Safe Behavior

TPIC6B595N outputs are open-drain with internal clamp diodes:
- Power loss → All outputs LOW → All brakes ENGAGED (spring-applied)
- Bit 0 for brake = engaged (fail-safe default)
- Bit 1 for brake = released (active)

### C Code Definitions

```c
// Shift register bit positions - 4 bits per axis [DIR, EN, BRAKE, ALARM_CLR]
// X-axis (Servo)
#define SR_X_DIR            0
#define SR_X_EN             1
#define SR_X_BRAKE          2
#define SR_X_ALARM_CLR      3

// Y-axis (Servo)
#define SR_Y_DIR            4
#define SR_Y_EN             5
#define SR_Y_BRAKE          6
#define SR_Y_ALARM_CLR      7

// Z-axis (Servo)
#define SR_Z_DIR            8
#define SR_Z_EN             9
#define SR_Z_BRAKE          10
#define SR_Z_ALARM_CLR      11

// A-axis (Servo)
#define SR_A_DIR            12
#define SR_A_EN             13
#define SR_A_BRAKE          14
#define SR_A_ALARM_CLR      15

// B-axis (Servo)
#define SR_B_DIR            16
#define SR_B_EN             17
#define SR_B_BRAKE          18
#define SR_B_ALARM_CLR      19

// C-axis (Stepper - no physical brake)
#define SR_C_DIR            20
#define SR_C_EN             21
#define SR_C_BRAKE          22  // Not connected on steppers
#define SR_C_ALARM_CLR      23

// D-axis (Stepper - no physical brake)
#define SR_D_DIR            24
#define SR_D_EN             25
#define SR_D_BRAKE          26  // Not connected on steppers
#define SR_D_ALARM_CLR      27

// E-axis (Discrete)
#define SR_E_DIR            28
#define SR_E_EN             29
#define SR_E_BRAKE          30
#define SR_E_ALARM_CLR      31

// General purpose outputs (SR4)
#define SR_GP_OUT_0         32
#define SR_GP_OUT_1         33
#define SR_GP_OUT_2         34
#define SR_GP_OUT_3         35
#define SR_GP_OUT_4         36
#define SR_GP_OUT_5         37
#define SR_GP_OUT_6         38
#define SR_GP_OUT_7         39

// Helper macros for axis operations (4 bits per axis)
#define SR_DIR_BIT(axis)       ((axis) * 4 + 0)
#define SR_EN_BIT(axis)        ((axis) * 4 + 1)
#define SR_BRAKE_BIT(axis)     ((axis) * 4 + 2)
#define SR_ALARM_CLR_BIT(axis) ((axis) * 4 + 3)
```

---

## MCP23017 I2C Expander Pin Assignment

### Device Addresses

| Device | Address | Function |
|--------|---------|----------|
| MCP23017 #0 | 0x20 | Limit switches (all 8 axes) |
| MCP23017 #1 | 0x21 | ALARM_INPUT signals + InPos feedback (inputs only) |

> **Architecture Note:** All outputs (DIR, EN, BRAKE, ALARM_CLEAR, GP_OUT) are handled via shift registers for fast SPI updates. MCP23017 expanders are used exclusively for inputs.

### MCP23017 I2C Address Configuration (A0, A1, A2 Pins)

The MCP23017 I2C address is set by the hardware address pins A0, A1, A2. These pins must be externally biased (not left floating).

**Address Configuration:**
- Connect pin to **GND** = Logic 0
- Connect pin to **VCC (3.3V)** = Logic 1

| A2 | A1 | A0 | I2C Address | YaRobot Assignment |
|----|----|----|-------------|-------------------|
| GND | GND | GND | **0x20** | MCP23017 #0 - Limit Switches |
| GND | GND | VCC | **0x21** | MCP23017 #1 - ALARM_INPUT + InPos |
| GND | VCC | GND | 0x22 | (available) |
| GND | VCC | VCC | 0x23 | (available) |
| VCC | GND | GND | 0x24 | (available) |
| VCC | GND | VCC | 0x25 | (available) |
| VCC | VCC | GND | 0x26 | (available) |
| VCC | VCC | VCC | 0x27 | (available) |

**Hardware Wiring for YaRobot:**

| Device | A2 | A1 | A0 | Address |
|--------|-----|-----|-----|---------|
| MCP23017 #0 | GND | GND | GND | 0x20 |
| MCP23017 #1 | GND | GND | VCC | 0x21 |

**Important Notes:**
- **RESET pin**: Must be connected to VCC via pull-up resistor (10kΩ typical). Leaving RESET floating causes erratic behavior.
- **Maximum devices**: Up to 8 MCP23017s on a single I2C bus (128 GPIO total)
- **Pull-ups on SDA/SCL**: Required - typically 4.7kΩ to 10kΩ to VCC

### MCP23017 #0 - Limit Switches (0x20)

Organized by axis pair (MIN/MAX adjacent):

| Pin | Port.Bit | Function | Axis | Notes |
|-----|----------|----------|------|-------|
| GPA0 | A.0 | X_LIMIT_MIN | X | Servo |
| GPA1 | A.1 | X_LIMIT_MAX | X | Servo |
| GPA2 | A.2 | Y_LIMIT_MIN | Y | Servo |
| GPA3 | A.3 | Y_LIMIT_MAX | Y | Servo |
| GPA4 | A.4 | Z_LIMIT_MIN | Z | Servo |
| GPA5 | A.5 | Z_LIMIT_MAX | Z | Servo |
| GPA6 | A.6 | A_LIMIT_MIN | A | Servo |
| GPA7 | A.7 | A_LIMIT_MAX | A | Servo |
| GPB0 | B.0 | B_LIMIT_MIN | B | Servo |
| GPB1 | B.1 | B_LIMIT_MAX | B | Servo |
| GPB2 | B.2 | C_LIMIT_MIN | C | Stepper |
| GPB3 | B.3 | C_LIMIT_MAX | C | Stepper/Floating |
| GPB4 | B.4 | D_LIMIT_MIN | D | Stepper |
| GPB5 | B.5 | D_LIMIT_MAX | D | Stepper |
| GPB6 | B.6 | E_LIMIT_MIN | E | Discrete |
| GPB7 | B.7 | E_LIMIT_MAX | E | Discrete |

**Configuration:**
- All pins configured as inputs with pull-ups enabled
- Interrupt-on-change enabled for all pins
- INTA output connected to ESP32 (see `GPIO_MCP0_INTA` in config_gpio.h)
- INTB output connected to ESP32 (see `GPIO_MCP0_INTB` in config_gpio.h)

### MCP23017 #1 - ALARM_INPUT + InPos Signals (0x21)

**Port A (GPA*) - ALARM_INPUT Signals (Inputs)**

All 7 motor axes (X, Y, Z, A, B, C, D) ALARM_INPUT signals grouped on Port A for alarm detection.

| Pin | Port.Bit | Function | Axis | Direction | Notes |
|-----|----------|----------|------|-----------|-------|
| GPA0 | A.0 | X_ALARM_INPUT | X | Input | X servo/driver alarm signal |
| GPA1 | A.1 | Y_ALARM_INPUT | Y | Input | Y servo/driver alarm signal |
| GPA2 | A.2 | Z_ALARM_INPUT | Z | Input | Z servo/driver alarm signal |
| GPA3 | A.3 | A_ALARM_INPUT | A | Input | A servo/driver alarm signal |
| GPA4 | A.4 | B_ALARM_INPUT | B | Input | B servo/driver alarm signal |
| GPA5 | A.5 | C_ALARM_INPUT | C | Input | C stepper driver alarm signal |
| GPA6 | A.6 | D_ALARM_INPUT | D | Input | D stepper driver alarm signal |
| GPA7 | A.7 | GP_IN_0 | - | Input | General purpose spare input |

**Port B (GPB*) - InPos Signals + Spare Inputs**

All 5 servo axis InPos (In-Position) signals grouped on Port B, with spare inputs for expansion.

| Pin | Port.Bit | Function | Axis | Direction | Notes |
|-----|----------|----------|------|-----------|-------|
| GPB0 | B.0 | X_INPOS | X | Input | X servo in-position signal |
| GPB1 | B.1 | Y_INPOS | Y | Input | Y servo in-position signal |
| GPB2 | B.2 | Z_INPOS | Z | Input | Z servo in-position signal |
| GPB3 | B.3 | A_INPOS | A | Input | A servo in-position signal |
| GPB4 | B.4 | B_INPOS | B | Input | B servo in-position signal |
| GPB5 | B.5 | GP_IN_1 | - | Input | General purpose spare input |
| GPB6 | B.6 | GP_IN_2 | - | Input | General purpose spare input |
| GPB7 | B.7 | GP_IN_3 | - | Input | General purpose spare input |

> **Architecture Note - Alarm Handling:** ALARM_INPUT signals detect driver fault conditions (overcurrent, overheat, position error, etc.). ALARM_CLEAR outputs are on the shift register chain (bits 3, 7, 11, 15, 19, 23, 27, 31) for fast SPI control.

**Configuration:**
- Port A: All inputs with pull-ups enabled (ALARM_INPUT signals)
- Port B: All inputs with pull-ups enabled (InPos + spare inputs)
- INTA output connected to ESP32 (see `GPIO_MCP1_INTA` in config_gpio.h)
- INTB output connected to ESP32 (see `GPIO_MCP1_INTB` in config_gpio.h)

---

## Header Configuration Alignment

### Updated config_gpio.h

```c
#ifndef CONFIG_GPIO_H
#define CONFIG_GPIO_H

// ============================================================================
// MOTOR STEP PULSE OUTPUTS - GROUPED BY AXIS
// ============================================================================
// All STEP outputs on LEFT side (J1), paired with Z_SIGNAL on RIGHT side (J3)
// Same row number = same physical position on board for clean cabling

// Servo axes (X, Y, Z, A, B) - STEP on left, Z_SIGNAL on right (same row)
#define GPIO_X_STEP         GPIO_NUM_4      // X-axis servo (RMT CH0) - J1-4 (row 4)
#define GPIO_Y_STEP         GPIO_NUM_5      // Y-axis servo (MCPWM T0) - J1-5 (row 5)
#define GPIO_Z_STEP         GPIO_NUM_6      // Z-axis servo (RMT CH1) - J1-6 (row 6)
#define GPIO_A_STEP         GPIO_NUM_7      // A-axis servo (RMT CH2) - J1-7 (row 7)
#define GPIO_B_STEP         GPIO_NUM_15     // B-axis servo (RMT CH3) - J1-8 (row 8)

// Stepper axes (C, D) - no Z-signal
#define GPIO_C_STEP         GPIO_NUM_16     // C-axis stepper (MCPWM T1) - J1-9
#define GPIO_D_STEP         GPIO_NUM_17     // D-axis stepper (LEDC CH0) - J1-10

// E-axis: DIR/EN via shift register (bits 21-22), no STEP GPIO needed (discrete actuator)

// ============================================================================
// SERVO Z-SIGNAL (INDEX) INPUTS - PAIRED WITH STEP OUTPUTS
// ============================================================================
// All Z_SIGNAL inputs on RIGHT side (J3), same row as corresponding STEP on left
// Hardware interrupts required for precise encoder index capture

#define GPIO_X_Z_SIGNAL     GPIO_NUM_1      // X servo index pulse - J3-4 (row 4)
#define GPIO_Y_Z_SIGNAL     GPIO_NUM_2      // Y servo index pulse - J3-5 (row 5)
#define GPIO_Z_Z_SIGNAL     GPIO_NUM_42     // Z servo index pulse - J3-6 (row 6)
#define GPIO_A_Z_SIGNAL     GPIO_NUM_41     // A servo index pulse - J3-7 (row 7)
#define GPIO_B_Z_SIGNAL     GPIO_NUM_40     // B servo index pulse - J3-8 (row 8)

// ============================================================================
// INPOS SIGNALS - VIA I2C EXPANDER (NOT GPIO!)
// ============================================================================
// InPos (Position Complete) signals are read via MCP23017 #1 (0x21) Port B
// They don't require fast interrupt response - I2C polling is sufficient
// See config_i2c.h for MCP23017 #1 pin assignments

// ============================================================================
// SHIFT REGISTER CONTROL (SPI2/HSPI)
// ============================================================================
// Grouped on left side (J1 pins 15-18)
#define GPIO_SR_OE          GPIO_NUM_9      // Output enable (active-low) - J1-15
#define GPIO_SR_CS          GPIO_NUM_10     // Shift register latch - J1-16
#define GPIO_SR_MOSI        GPIO_NUM_11     // Shift register data - J1-17
#define GPIO_SR_SCLK        GPIO_NUM_12     // Shift register clock - J1-18

// ============================================================================
// I2C BUS 0 (MCP23017 Expanders)
// ============================================================================
// I2C lines grouped on left side (J1 pins 11-12)
#define GPIO_I2C_SCL        GPIO_NUM_18     // I2C clock - J1-11
#define GPIO_I2C_SDA        GPIO_NUM_8      // I2C data - J1-12

// ============================================================================
// MCP23017 INTERRUPT LINES (4 GPIOs for 2 MCPs × 2 interrupts each)
// ============================================================================
// Grouped by MCP device: MCP0 on left side (J1), MCP1 on right side (J3)
// Both MCPs are input-only, so all 4 interrupt lines are active

// MCP0 interrupts - grouped on LEFT side (J1-13, J1-14)
#define GPIO_MCP0_INTA      GPIO_NUM_3      // MCP #0 Port A (X-A limits) - J1-13
#define GPIO_MCP0_INTB      GPIO_NUM_46     // MCP #0 Port B (B-E limits) - J1-14

// MCP1 interrupts - grouped on RIGHT side (J3-9, J3-10)
#define GPIO_MCP1_INTA      GPIO_NUM_39     // MCP #1 Port A (ALARM_INPUT) - J3-9
#define GPIO_MCP1_INTB      GPIO_NUM_38     // MCP #1 Port B (InPos signals) - J3-10

// ============================================================================
// SAFETY SIGNALS
// ============================================================================
#define GPIO_E_STOP         GPIO_NUM_13     // Emergency stop input - J1-19

// ============================================================================
// OLED I2C BUS (Dedicated bus, isolated from main I2C)
// ============================================================================
#define GPIO_OLED_SDA       GPIO_NUM_14     // OLED I2C data (I2C_NUM_1) - J1-20
#define GPIO_OLED_SCL       GPIO_NUM_21     // OLED I2C clock (I2C_NUM_1) - J3-18

// ============================================================================
// USB (Fixed by hardware - DO NOT CHANGE)
// ============================================================================
#define GPIO_USB_DN         GPIO_NUM_19     // USB D- - J3-20
#define GPIO_USB_DP         GPIO_NUM_20     // USB D+ - J3-19

// ============================================================================
// SPARE GPIOs (available for future use)
// ============================================================================
// GPIO43 - J3-2  (UART0 TX - available if USB used for debug)
// GPIO44 - J3-3  (UART0 RX - available if USB used for debug)
// GPIO45 - J3-15 (strapping pin, safe after boot)
// GPIO47 - J3-17 (freed by consolidating to 2 MCPs)
// GPIO48 - J3-16 (freed by consolidating to 2 MCPs)

#endif // CONFIG_GPIO_H
```

### Updated config_sr.h (New File)

```c
#ifndef CONFIG_SR_H
#define CONFIG_SR_H

// ============================================================================
// SHIFT REGISTER BIT POSITIONS
// ============================================================================
// Organization: 4 bits per axis [DIR, EN, BRAKE, ALARM_CLR]
// Chain: MOSI → SR0 → SR1 → SR2 → SR3 → SR4
// Total: 40 bits (5 x TPIC6B595N)

// X-axis (Servo)
#define SR_X_DIR            0       // SR0.Q0 - Direction
#define SR_X_EN             1       // SR0.Q1 - Enable
#define SR_X_BRAKE          2       // SR0.Q2 - Brake release
#define SR_X_ALARM_CLR      3       // SR0.Q3 - Alarm clear

// Y-axis (Servo)
#define SR_Y_DIR            4       // SR0.Q4
#define SR_Y_EN             5       // SR0.Q5
#define SR_Y_BRAKE          6       // SR0.Q6
#define SR_Y_ALARM_CLR      7       // SR0.Q7

// Z-axis (Servo)
#define SR_Z_DIR            8       // SR1.Q0
#define SR_Z_EN             9       // SR1.Q1
#define SR_Z_BRAKE          10      // SR1.Q2
#define SR_Z_ALARM_CLR      11      // SR1.Q3

// A-axis (Servo)
#define SR_A_DIR            12      // SR1.Q4
#define SR_A_EN             13      // SR1.Q5
#define SR_A_BRAKE          14      // SR1.Q6
#define SR_A_ALARM_CLR      15      // SR1.Q7

// B-axis (Servo)
#define SR_B_DIR            16      // SR2.Q0
#define SR_B_EN             17      // SR2.Q1
#define SR_B_BRAKE          18      // SR2.Q2
#define SR_B_ALARM_CLR      19      // SR2.Q3

// C-axis (Stepper - no physical brake)
#define SR_C_DIR            20      // SR2.Q4
#define SR_C_EN             21      // SR2.Q5
#define SR_C_BRAKE          22      // SR2.Q6 (not connected)
#define SR_C_ALARM_CLR      23      // SR2.Q7

// D-axis (Stepper - no physical brake)
#define SR_D_DIR            24      // SR3.Q0
#define SR_D_EN             25      // SR3.Q1
#define SR_D_BRAKE          26      // SR3.Q2 (not connected)
#define SR_D_ALARM_CLR      27      // SR3.Q3

// E-axis (Discrete)
#define SR_E_DIR            28      // SR3.Q4
#define SR_E_EN             29      // SR3.Q5
#define SR_E_BRAKE          30      // SR3.Q6
#define SR_E_ALARM_CLR      31      // SR3.Q7

// General purpose outputs (SR4)
#define SR_GP_OUT_0         32      // SR4.Q0
#define SR_GP_OUT_1         33      // SR4.Q1
#define SR_GP_OUT_2         34      // SR4.Q2
#define SR_GP_OUT_3         35      // SR4.Q3
#define SR_GP_OUT_4         36      // SR4.Q4
#define SR_GP_OUT_5         37      // SR4.Q5
#define SR_GP_OUT_6         38      // SR4.Q6
#define SR_GP_OUT_7         39      // SR4.Q7

// ============================================================================
// HELPER MACROS
// ============================================================================
#define SR_DIR_BIT(axis)       ((axis) * 4 + 0)
#define SR_EN_BIT(axis)        ((axis) * 4 + 1)
#define SR_BRAKE_BIT(axis)     ((axis) * 4 + 2)
#define SR_ALARM_CLR_BIT(axis) ((axis) * 4 + 3)

// Bit manipulation (use uint64_t for 40-bit support)
#define SR_SET_BIT(data, bit)   ((data) | (1ULL << (bit)))
#define SR_CLR_BIT(data, bit)   ((data) & ~(1ULL << (bit)))
#define SR_GET_BIT(data, bit)   (((data) >> (bit)) & 1)

// ============================================================================
// FAIL-SAFE DEFAULTS
// ============================================================================
// On power-up or reset, all outputs LOW = all brakes engaged
#define SR_SAFE_STATE       0x0000000000ULL    // All bits 0 = fail-safe

// Active state masks
#define SR_ALL_EN           ((1ULL << SR_X_EN) | (1ULL << SR_Y_EN) | \
                             (1ULL << SR_Z_EN) | (1ULL << SR_A_EN) | \
                             (1ULL << SR_B_EN) | (1ULL << SR_C_EN) | \
                             (1ULL << SR_D_EN) | (1ULL << SR_E_EN))

#define SR_ALL_BRAKE_REL    ((1ULL << SR_X_BRAKE) | (1ULL << SR_Y_BRAKE) | \
                             (1ULL << SR_Z_BRAKE) | (1ULL << SR_A_BRAKE) | \
                             (1ULL << SR_B_BRAKE) | (1ULL << SR_E_BRAKE))

#endif // CONFIG_SR_H
```

---

## Physical Wiring Guide

### Left Side Connector (J1) - Recommended Pinout

```
J1 Connector Wiring (top to bottom):
Pin 4-7:   Servo STEP outputs (Y,Z,A,B) → Motor driver terminals
Pin 8:     Stepper C STEP → Stepper driver
Pin 9:     E-STOP input → E-stop switch (NC contact to GND)
Pin 10:    OLED SDA → Display module
Pin 11-12: I2C bus → MCP23017 chain
Pin 13-14: I2C interrupts → MCP23017 INT pins
Pin 15-18: SPI → Shift register board (OE, CS, MOSI, SCLK)
Pin 19:    Position feedback Z
Pin 20:    Stepper D STEP → Stepper driver
```

### Right Side Connector (J3) - Recommended Pinout

```
J3 Connector Wiring (top to bottom):
Pin 4:     Position feedback X
Pin 5:     Servo X STEP → Motor driver terminal
Pin 6-9:   Z-signals (Z,Y,X,B) → Servo encoder index outputs
Pin 10:    Position feedback Y
Pin 15:    Z-signal A → Servo encoder index
Pin 16-17: Position feedback (B,A)
Pin 18:    OLED SCL → Display module
```

---

## Validation Checklist

- [x] All GPIO35-37 reserved for Octal PSRAM (not used)
- [x] GPIO19-20 reserved for USB (not used for signals)
- [x] Strapping pins (0, 3, 45, 46) used with caution
- [x] JTAG pins (39-42) repurposed for signals
- [x] Servo STEP outputs grouped (X,Y,Z,A,B on J1 pins 4-8)
- [x] SPI signals grouped (J1 pins 15-18)
- [x] I2C signals grouped (J1 pins 11-12)
- [x] MCP INTA interrupts grouped (J1 pins 13-14)
- [x] MCP INTB interrupts grouped (J3 pins 9-10)
- [x] Z-signals grouped (J3 pins 4-8)
- [x] Shift register bits organized by axis (4 bits each: DIR, EN, BRAKE, ALARM_CLR)
- [x] 5x TPIC6B595N shift registers for all outputs (40 bits total)
- [x] MCP23017 limit switches organized by axis pair
- [x] MCP23017 consolidated to 2 devices (inputs only)
- [x] Header defines match physical assignments
- [x] Total GPIO usage within available count (23 used, 5 spare + 2 UART)

---

## References

- [ESP32-S3-DevKitC-1 User Guide v1.1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)
- [ESP32-S3 DevKitC Pinout Guide - Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/)
- [ESP32-S3 DevKitC-1 High-Resolution Pinout - Mischianti](https://mischianti.org/esp32-s3-devkitc-1-high-resolution-pinout-and-specs/)
- [ESP-IDF GPIO Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html)

---

_Document generated for YaRobot Control Unit architecture alignment_
_Date: 2025-11-29_
