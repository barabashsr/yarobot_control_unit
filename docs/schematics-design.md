# YaRobot Control Unit - Schematics Design

## System Architecture Overview

```
                          ┌─────────────────────────────────────────────────┐
                          │              ESP32-S3 DevKit                      │
                          │                                                   │
                          │  ┌─────────┐    ┌─────────┐    ┌─────────┐      │
                          │  │   RMT    │    │  MCPWM  │    │  LEDC   │      │
                          │  │ +DMA     │    │  +PCNT  │    │         │      │
                          │  └────┬────┘    └────┬────┘    └────┬────┘      │
                          │       │              │              │            │
                          │  ┌────┴────────┬────┴────────┬────┴────┐       │
                          │  │ GPIO OUTPUTS│ GPIO INPUTS │ I2C BUS │       │
                          └──┴──────┬──────┴──────┬──────┴────┬────┴───────┘
                                    │             │            │
                       ┌────────────┴───┐   ┌─────┴─────┐   ┌─┴──────────┐
                       │ Shift Registers│   │  Feedback │   │ I2C Devices │
                       │  (2x 74HC595)  │   │  Signals  │   │ (MCP23017)  │
                       └────────┬───────┘   └───────────┘   └────────────┘
                                │
                    ┌───────────┴─────────────┐
                    │   Motor Driver Signals  │
                    │  (DIR/EN/BRAKE)         │
                    └─────────────────────────┘
```

## Power Supply Architecture

```
AC Input ──┬── 24V PSU ──┬── 24V Bus ──┬── Motor Drivers
110/220V   │             │              ├── Brake Control
           │             │              └── I/O Power
           │             │
           │             └── 5V Buck ────┬── ESP32-S3 (via USB)
           │                 Converter   ├── Logic Level
           │                             └── OLED Display
           │
           └── Optional UPS/Battery Backup
```

## ESP32-S3 Pin Allocation

### Motor Control Pins
```
┌─────────────────────────────────────┐
│ MOTOR STEP OUTPUTS                  │
├─────────────────────────────────────┤
│ GPIO2  → X_STEP (RMT CH0)          │
│ GPIO4  → Y_STEP (MCPWM0_0A)        │
│ GPIO5  → Z_STEP (RMT CH1)          │
│ GPIO6  → A_STEP (RMT CH2)          │
│ GPIO7  → B_STEP (RMT CH3)          │
│ GPIO15 → C_STEP (MCPWM0_1A)        │
│ GPIO14 → D_STEP (LEDC CH0)         │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ SHIFT REGISTER CONTROL (SPI)        │
├─────────────────────────────────────┤
│ GPIO11 → SR_DATA (MOSI)            │
│ GPIO12 → SR_CLOCK (SCLK)           │
│ GPIO10 → SR_LATCH (CS)             │
│ GPIO9  → SR_OE (Output Enable)     │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ SERVO FEEDBACK INPUTS               │
├─────────────────────────────────────┤
│ GPIO40 → X_Z_SIGNAL                │
│ GPIO41 → Y_Z_SIGNAL                │
│ GPIO42 → Z_Z_SIGNAL                │
│ GPIO48 → A_Z_SIGNAL                │
│ GPIO47 → B_Z_SIGNAL                │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ I2C BUS                             │
├─────────────────────────────────────┤
│ GPIO8  → I2C_SDA                   │
│ GPIO18 → I2C_SCL                   │
│ GPIO3  → INT0 (MCP23017 #1)        │
│ GPIO46 → INT1 (MCP23017 #2)        │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│ SAFETY & MISC                       │
├─────────────────────────────────────┤
│ GPIO16 → E_STOP (Direct Input)     │
│ GPIO19 → USB_D-                    │
│ GPIO20 → USB_D+                    │
└─────────────────────────────────────┘
```

## Shift Register Chain (24-bit TPIC6B595N)

```
              TPIC6B595N #1 (DIR/EN)         TPIC6B595N #2 (DIR/EN)         TPIC6B595N #3 (BRAKES)
ESP32 ────────┬─────────────────────────┬──────┬─────────────────────────┬──────┬───────────────────┐
              │  Bit 0: X_DIR (24V)     │      │  Bit 0: B_DIR (24V)     │      │ Bit 0: X_BRAKE_RLY│
MOSI ─────────┤  Bit 1: X_EN  (24V)     ├──────┤  Bit 1: B_EN  (24V)     ├──────┤ Bit 1: Y_BRAKE_RLY│
(3.3V)        │  Bit 2: Y_DIR (24V)     │      │  Bit 2: C_DIR (24V)     │      │ Bit 2: Z_BRAKE_RLY│
              │  Bit 3: Y_EN  (24V)     │      │  Bit 3: C_EN  (24V)     │      │ Bit 3: A_BRAKE_RLY│
SCLK ─────────┤  Bit 4: Z_DIR (24V)     ├──────┤  Bit 4: D_DIR (24V)     ├──────┤ Bit 4: B_BRAKE_RLY│
(3.3V)        │  Bit 5: Z_EN  (24V)     │      │  Bit 5: D_EN  (24V)     │      │ Bit 5: SPARE_RLY1 │
              │  Bit 6: A_DIR (24V)     │      │  Bit 6: SPARE           │      │ Bit 6: SPARE_RLY2 │
LATCH ────────┤  Bit 7: A_EN  (24V)     ├──────┤  Bit 7: SPARE           ├──────┤ Bit 7: SPARE_RLY3 │
(3.3V)        └─────────────────────────┘      └─────────────────────────┘      └───────────────────┘
```

### TPIC6B595N Power Supply - CRITICAL CONFIGURATION

```
┌─────────────────────────────────────────┐
│           TPIC6B595N IC                 │
├─────────────────────────────────────────┤
│                                         │
│  VCC (Pin 16) ←──── 5V Logic Supply    │ ← Powers shift register logic
│                      (3.3V-5V range)    │   (Use 5V for noise immunity)
│                                         │
│  VDD (Pin 8)  ←──── 24V Clamp Supply   │ ← Sets output clamp voltage
│                      (up to 50V)        │   (MUST be ≥ load voltage!)
│                                         │  
│  GND (Pin 9)  ←──── Common Ground      │ ← Logic and power ground
│                                         │
│  DRAIN0-7 ────────> Open Drain Outputs │ → Sink current to GND only
│                      (150mA continuous) │   (Cannot source current!)
└─────────────────────────────────────────┘

⚠️ CRITICAL RULE: VDD MUST be ≥ highest switched voltage!
If VDD < Load Voltage → Internal diodes conduct → IC DESTROYED!

Correct Example:
- VCC = 5V (logic supply from buck converter)
- VDD = 24V (from main 24V bus) ✓
- Load = 24V motor signals ✓

Wrong Example:
- VCC = 5V
- VDD = 5V  ← TOO LOW!
- Load = 24V → PERMANENT DAMAGE!
```

### Complete TPIC6B595N Wiring

```
Power Supply Connections (All 3 TPIC6B595N ICs):
┌──────────────────────────────────────────────┐
│  5V Buck  │  24V Bus  │  ESP32  │  To Loads │
├──────────────────────────────────────────────┤
│           │           │         │            │
│    5V─────┴───────────┴─────┐   │            │
│                             ├───┼─> VCC (16)  │ Logic supply
│                             │   │            │
│           ┌─────────────────┴───┼─> VDD (8)   │ Clamp voltage  
│           │ 24V                │            │ (MUST be ≥24V!)
│           │                     │            │
│    GND────┴─────────────────────┴─> GND (9)   │ Common ground
│                                              │
│                                 ┌─> G̅ (10)    │ Output enable
│                            GND──┘             │ (tie low)
│                                              │
│                            5V───┬─> SRCLR̅(11) │ No clear
│                                 │             │ (tie high)
└──────────────────────────────────────────────┘

Data Chain (3.3V logic from ESP32 works directly!):
- ESP32 MOSI (3.3V) → U1 SER IN
- U1 SER OUT → U2 SER IN
- U2 SER OUT → U3 SER IN
- All SRCK → GPIO12 (3.3V)
- All RCK → GPIO10 (3.3V)

Note: TPIC6B595N has TTL-compatible inputs
      VIH = 2.0V min when VCC = 5V
      So 3.3V signals work perfectly!
```

## I2C Device Architecture

```
I2C Bus (400kHz)
│
├─── MCP23017 #1 (0x20) - Limit Switches
│    ├── GPA0: X_LIMIT_MIN
│    ├── GPA1: X_LIMIT_MAX  
│    ├── GPA2: Y_LIMIT_MIN
│    ├── GPA3: Y_LIMIT_MAX
│    ├── GPA4: Z_LIMIT_MIN
│    ├── GPA5: Z_LIMIT_MAX
│    ├── GPA6: A_LIMIT_MIN
│    ├── GPA7: A_LIMIT_MAX
│    ├── GPB0: B_LIMIT_MIN
│    ├── GPB1: B_LIMIT_MAX
│    ├── GPB2: C_LIMIT_MIN  
│    ├── GPB3: C_LIMIT_MAX (Floating Switch)
│    ├── GPB4: D_LIMIT_MIN
│    ├── GPB5: D_LIMIT_MAX
│    ├── GPB6: SPARE
│    └── GPB7: SPARE
│
├─── MCP23017 #2 (0x21) - General Purpose I/O  
│    ├── GPA0-7: Digital Inputs (Buttons, Sensors)
│    └── GPB0-7: Digital Outputs (LEDs, Relays)
│
├─── MCP23017 #3 (0x22) - Servo Feedback
│    ├── GPA0: X_POS_COMPLETE
│    ├── GPA1: Y_POS_COMPLETE
│    ├── GPA2: Z_POS_COMPLETE  
│    ├── GPA3: A_POS_COMPLETE
│    ├── GPA4: B_POS_COMPLETE
│    ├── GPA5-7: SPARE
│    └── GPB0-7: SPARE/Future Use
│
└─── SSD1306 OLED (0x3C) - 128x64 Display
```

## Motor Driver Interface (Typical)

### Servo Driver Connection (24V Logic)
```
┌─────────────────────────────────────┐      ┌──────────────────────┐
│      ESP32 + TPIC6B595N            │      │  Servo Driver (24V)  │
├─────────────────────────────────────┤      ├──────────────────────┤
│ X_STEP (GPIO2) ──────────────────────────>│ PUL+ ←┐              │
│ GND ──────────────────────────────────────>│ PUL-  │ Opto-       │
│                                      │      │       │ isolated    │
│               24V ←──────────────────┐     │       │ 24V logic   │
│                                      ├────>│ DIR+ ←┘              │
│ TPIC6B595N Bit0 ────┤◄──[10kΩ]──────┘     │                      │
│ (X_DIR) Open Drain  └─────────────────────>│ DIR-                 │
│                                      │      │                      │
│               24V ←──────────────────┐     │                      │
│                                      ├────>│ EN+                  │
│ TPIC6B595N Bit1 ────┤◄──[10kΩ]──────┘     │                      │  
│ (X_EN) Open Drain   └─────────────────────>│ EN-                  │
│                                      │      │                      │
│ X_Z_SIGNAL (GPIO40) <───[Level]────────────│ Z+ (Index)          │
│ GND ──────────────────────────────────────>│ Z-                   │
│                                      │      │                      │
│ MCP23017 GPA0 <─────────[Level]────────────│ POS_COMP+ (24V)     │
│ GND ──────────────────────────────────────>│ POS_COMP-           │
└─────────────────────────────────────┘      └──────────────────────┘

Note: TPIC6B595N inverted logic for active-high drivers:
- Bit = 1 → Output ON (pulls to GND) → Signal = HIGH (24V via pullup)
- Bit = 0 → Output OFF (open) → Signal = LOW (0V)

[Level] = Bidirectional level shifter for feedback signals
```

### Stepper Driver Connection (C/D Axes)
```
┌─────────────────────────────────────┐      ┌──────────────────────┐
│      ESP32 + TPIC6B595N            │      │  Stepper Driver      │
├─────────────────────────────────────┤      ├──────────────────────┤
│ C_STEP (GPIO15) ──────────────────────────>│ PUL+                 │
│ GND ──────────────────────────────────────>│ PUL-                 │
│                                      │      │                      │
│               24V ←──────────────────┐     │                      │
│                                      ├────>│ DIR+                 │
│ TPIC6B595N Bit2 ────┤◄──[10kΩ]──────┘     │                      │
│ (C_DIR)             └─────────────────────>│ DIR-                 │
│                                      │      │                      │
│               24V ←──────────────────┐     │                      │
│                                      ├────>│ EN+                  │
│ TPIC6B595N Bit3 ────┤◄──[10kΩ]──────┘     │                      │
│ (C_EN)              └─────────────────────>│ EN-                  │
└─────────────────────────────────────┘      └──────────────────────┘
```

### Stepper Driver Connection (C/D Axes)
```
┌─────────────────────────────────────┐      ┌──────────────────────┐
│         ESP32 + Peripherals         │      │   Stepper Driver     │
├─────────────────────────────────────┤      ├──────────────────────┤
│ C_STEP (GPIO15) ──────────────────────────>│ PUL+                 │
│ GND ──────────────────────────────────────>│ PUL-                 │
│                                      │      │                      │
│ 74HC595 Bit10 (C_DIR) ───[Level]─────────>│ DIR+                 │
│ GND ──────────────────────────────────────>│ DIR-                 │
│                                      │      │                      │
│ 74HC595 Bit11 (C_EN) ────[Level]─────────>│ EN+                  │
│ GND ──────────────────────────────────────>│ EN-                  │
└─────────────────────────────────────┘      └──────────────────────┘
```

## Brake Control via Relay Modules

```
┌───────────────────────────────────────────────────────────────┐
│                    5V/24V Relay Module                        │
│  ┌─────────────┐                          ┌─────────────┐    │  
│  │ TPIC6B595N  │                          │ Relay Coil  │    │
│  │ Bit 0       ├────> IN ───[Optocoupler]──┤ Driver      │    │
│  │(X_BRAKE_RLY)│                          └──────┬──────┘    │
│  └─────────────┘                                 │           │
│                                                  ▼           │
│  GND ────────────────> GND                 ┌──────────┐     │ 24V Brake
│  5V ─────────────────> VCC                 │  RELAY   │     │    Power
│                                            │          │     │      │
│  Status LED ◄─────────────────────────────┤  COM ◄───┼─────┼──────┤
│                                           │   NO ────┼─────┼────> Brake+
│                                           │   NC     │     │
│                                           └──────────┘     │
│                                                           │
└───────────────────────────────────────────────────────────┘
                                                            │
                                                           GND ───> Brake-

Brake Control Logic (Fail-Safe):
- TPIC6B595N Bit = 0 → Relay OFF → NC Contact → Brake Engaged ✓
- TPIC6B595N Bit = 1 → Relay ON → NO Contact → Brake Released
- Power Loss → Relay OFF → Brake Engaged ✓

Relay Module Features:
- Optocoupler isolation (1000V+)
- LED status indicator
- Built-in flyback protection
- Available in 5V or 24V coil versions
```

## Limit Switch Input Circuit

```
                3.3V
                 │
                [R1]  10kΩ (Internal MCP23017 pullup)
                 │
MCP23017 ────────┼─────────┐
GPA0             │         │
(X_MIN)          │         ○ Limit Switch (N.O.)
                 │         │
                 └─────────┘
                           │
                          GND

Switch States:
- Not triggered: Pin reads HIGH (3.3V)
- Triggered: Pin reads LOW (0V)
```

## Emergency Stop Circuit

```
                3.3V
                 │
                [R1]
                10kΩ
                 │
GPIO16 ──────────┼─────────┐
(E_STOP)         │         │
                 │         ○ E-Stop Button (N.C.)
                 │         │
                 └─────────┘
                           │
                          GND
                          
                 ┌─────────────┐
                 │   Optional   │
                 │ Safety Relay │
                 └─────────────┘
```

## OLED Display Connection

```
┌────────────────┐          ┌─────────────┐
│     ESP32      │          │ SSD1306 OLED│
├────────────────┤          ├─────────────┤
│ GPIO8 (SDA) ───────────────> SDA        │
│ GPIO18 (SCL) ──────────────> SCL        │
│ 3.3V ──────────────────────> VCC        │
│ GND ───────────────────────> GND        │
└────────────────┘          └─────────────┘
                            
Note: 2.2kΩ pullups on SDA/SCL
```

## Level Shifting

### Bidirectional I2C Level Shifter
```
        3.3V Side                    5V Side
           │                            │
          [R1]                         [R2]
          2.2k                         2.2k
           │                            │
    SDA ───┼───┐      ┌──────┐        ┌┼─── SDA
           │   └──────│ BSS138├────────┘│
           │          │  or   │         │
    SCL ───┼───┐      │TXS0102│        ┌┼─── SCL
           │   └──────│      ├────────┘ │
          GND ────────└──────┘         GND
```

### Unidirectional Logic Level Shifter (3.3V → 5V)
```
    3.3V Signal          5V Output
         │                  │
         │               5V │
         │                ┌─┴─┐
         └────[R1]────────│B C├──── 5V Signal
              10kΩ        │ E │
                      ┌───┤   │
                     GND  └───┘
                         2N3904
```

## Typical Component Values

### Passive Components
- **TPIC6B595N Pull-ups**: 10kΩ (24V signals), 4.7kΩ (12V), 2.2kΩ (5V)
- **I2C Pull-ups**: 4.7kΩ (3.3V), 2.2kΩ (5V side if level shifted)
- **Current Limiting**: 330Ω (LEDs)
- **Decoupling Caps**: 
  - 0.1µF ceramic at each IC VCC pin
  - 0.1µF ceramic at each TPIC6B595N VDD pin (CRITICAL!)
  - 10µF tantalum per power rail
- **Bulk Capacitance**: 100µF/35V electrolytic per rail

### Active Components  
- **Shift Registers**: TPIC6B595N (150mA/output, 50V max)
- **I/O Expanders**: MCP23017 (16-bit I2C, 25mA/pin)
- **Relay Modules**: 5V/24V coil, 10A/250VAC contacts
- **Level Shifters**: TXS0108 (8-ch), BSS138 (I2C)
- **Buck Converter**: LM2596 module (24V→5V, 3A)

## PCB Layout Considerations

### Power Distribution
1. **Star Ground**: Single point ground for analog/digital
2. **Power Planes**: Separate 24V and 5V/3.3V planes
3. **Decoupling**: 0.1µF caps close to each IC
4. **Trace Width**: 24V brake traces ≥ 1mm (1A capacity)

### Signal Integrity
1. **Differential Pairs**: Keep USB D+/D- matched length
2. **I2C Routing**: Keep traces short, avoid high-speed signals
3. **Step Signals**: Route away from sensitive analog inputs
4. **Ground Plane**: Solid ground plane under high-speed signals

### EMI Considerations
1. **Isolation**: Separate motor power from logic power
2. **Filtering**: RC filters on switch inputs
3. **Shielding**: Metal enclosure with proper grounding
4. **Cable Management**: Twisted pairs for motor signals

## Test Points

Essential test points for debugging:
```
TP1:  3.3V Rail
TP2:  5V Rail  
TP3:  24V Rail
TP4:  GND
TP5:  I2C SDA
TP6:  I2C SCL
TP7:  SPI MOSI
TP8:  SPI CLK
TP9:  E-Stop Signal
TP10: Brake Power
```

## Connector Pinouts

### Motor Connector (DB15 per axis)
```
Pin 1:  PUL+
Pin 2:  PUL-
Pin 3:  DIR+
Pin 4:  DIR-
Pin 5:  EN+
Pin 6:  EN-
Pin 7:  Z+ (Index)
Pin 8:  Z-
Pin 9:  POS_COMP+
Pin 10: POS_COMP-
Pin 11: Brake+
Pin 12: Brake-
Pin 13: Shield
Pin 14: Reserved
Pin 15: Reserved
```

### Limit Switch Connector (RJ45)
```
Pin 1: MIN Switch
Pin 2: MIN GND
Pin 3: MAX Switch  
Pin 4: MAX GND
Pin 5: Reserved
Pin 6: Reserved
Pin 7: Shield
Pin 8: Shield
```

## Future Expansion

### Reserved Connections
- SPI CS pins for additional TPIC6B595N chains
- UART pins for Modbus RTU
- Analog inputs for current sensing
- PWM outputs for proportional control
- CAN bus transceiver footprint

## TPIC6B595N Design Guidelines

### Critical Design Rules
1. **VDD Connection**: MUST be ≥ highest load voltage
2. **Power Sequencing**: Apply VCC before VDD if possible
3. **Decoupling**: Place 0.1µF cap within 10mm of VDD pin
4. **Ground Plane**: Solid ground under IC for thermal management
5. **Pull-up Sizing**: R = V/I where I = desired off-state current

### Thermal Considerations
```
Power Dissipation per Output:
P = I² × RDS(ON) = (0.15A)² × 2Ω = 45mW

Total Package (8 outputs at 150mA):
P_total = 8 × 45mW = 360mW (well within 1.1W limit)

No heatsink required for normal operation
```

### Common Mistakes to Avoid
1. ❌ Connecting VDD to logic supply (5V) when switching 24V
2. ❌ Forgetting pull-up resistors (outputs won't turn off!)
3. ❌ Using as push-pull outputs (they're open-drain only)
4. ❌ Exceeding 150mA continuous per output
5. ❌ Missing VDD decoupling capacitor

This completes the high-level schematics design for the YaRobot Control Unit.