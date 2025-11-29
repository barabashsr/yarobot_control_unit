# Brainstorming Session Results

**Session Date:** 2025-11-19
**Facilitator:** Strategic Business Analyst Mary
**Participant:** Sergey

## Session Start

**Session Plan:** Project-focused technical architecture brainstorming for yarobot control unit
**Approach:** AI-Recommended techniques based on technical problem-solving context
**Target Outcome:** Structured class architecture with clear separation of concerns and hardware abstraction

## Executive Summary

**Topic:** Motor control architecture design - class hierarchy, hardware abstraction, and initialization patterns for yarobot control unit

**Session Goals:** Design clean, maintainable class architecture for motor control with proper hardware abstraction layers, initialization patterns, and clear separation between transmission mechanics and hardware I/O

**Techniques Used:** First Principles Thinking, Morphological Analysis, Assumption Reversal

**Total Ideas Generated:** 25+ architectural decisions and implementation details

### Key Themes Identified:

- **Hardware-Software Separation:** Clear split between timing-critical (GPIO) and control (I2C) signals
- **Axis-Centric Design:** Control axes, not motors - motor is just hardware interface
- **Transparent Composition:** Explicit object creation at startup, not hidden abstractions
- **Scalable I/O Architecture:** I2C expanders enable multi-motor systems within GPIO limits
- **Safety-First Error Handling:** Master-level stop and restart, cut PUL on I2C failure

## Technique Sessions

### First Principles Thinking Session

**Fundamental Hardware Truths Identified:**
- Interface: PUL/DIR servo drives + stepper motor controllers
- Core data flows: Pulses (step commands), Direction (boolean), Enable (boolean)
- Future requirement: Encoder feedback (parallel reading from servo encoders)
- Constraint: Real-time control requirements (timing TBD)

**Key Insight:** Servo drives consume encoder data in closed loop, but parallel reading needed for software feedback

**Fundamental Questions Emerging:**

**Core Control Reality:**
- Stepper vs Servo: NO software difference except encoder availability
- MVP requirement: Reserve 2 GPIOs per motor for encoder channels
- Position persistence: Encoder tracking possible with dedicated power supply (retain position when main power off)

**Data Flow Chain Identified:**
pulses → encoder data (if available) → angular position → angular speed → linear position (if applicable) → linear speed (if applicable)

**Calibration Requirements:**
- Zero positions: angular, linear, encoder count
- Maximum positions: angular, linear limits
- All set during calibration phase

**Foundation:** Start with ESP-IDF PCNT (Pulse Counter) example

**Additional Control:** Brake control (not yet detailed)

**BREAKTHROUGH INSIGHT:** Control target is AXIS movement, not motor directly
- Need access to ALL data levels: pulses → motor → transmission → axis coordinate
- Brake: Simple engage/disengage (digital output)

**Emerging Architecture Pattern:**
```
Axis (what we control)
├── Motor (hardware interface)
├── Transmission (math transforms)
└── Calibration (limits & zeros)
```

**REFINED CONSTRUCTOR PATTERN:**
`axis(motor, encoder, angular_transmission_ratio, linear_transmission_ratio, end_switch1, end_switch2)`

**Key Design Decisions:**
- Motor level: Skip encoder pins initially (placeholder -1), calculate position from pulse count
- Encoder as separate Sensor class (supports external dedicated ICs)
- Transmission ratios passed directly to Axis constructor (simple approach preferred)
- End switches as separate objects (limit detection)

**SENSOR ARCHITECTURE BREAKTHROUGH:**
- Axis SUBSCRIBES to sensors (not just switches)
- Supports diverse sensor types: end switches, current sensors, effort sensors, position sensors
- Pattern: `axis.subscribeTo(sensor, callback_function)`
- Enables complex behaviors: effort-based limits, current monitoring, multi-sensor safety logic

**STATE TRANSITION CALLBACKS:**
- Need edge detection: ON→OFF and OFF→ON transitions
- Critical for calibration procedures (detecting end switch engagement/disengagement)
- No sensor hierarchy initially (all callbacks treated equally)
- Callback patterns: `onRisingEdge()`, `onFallingEdge()`, `onValueChange()`

### Morphological Analysis Session

**Parameter Matrix - Motor Control Architecture**

### **Motor Interface Type:**
- Stepper motor (PUL/DIR/EN)
- Servo drive (PUL/DIR/EN)

### **Encoder Configuration:**
- None (pulse counting only)
- Direct GPIO (2 pins per motor)
- External dedicated IC (I2C/SPI interface)
- Shared encoder channels (multiplexed)

### **Brake Control:**
- None
- Direct GPIO output
- I2C expander output

### **End Switch Configuration:**
- None
- Single end switch
- Dual end switches (home + limit)
- Current sensing (virtual limits)

### **Communication Interface with Host:**
- Serial/UART
- USB CDC
- Ethernet/WiFi TCP
- CAN bus
- I2C slave
- Modbus RTU/TCP

**OPTIMAL COMBINATION IDENTIFIED:**
- **High-speed signals (GPIO):** PUL generation, encoder reading (if no external IC)
- **Low-speed signals (I2C expander):** EN, DIR, switches, brake
- **Rationale:** Critical timing signals stay on fast GPIO, control signals use scalable expander

**IMPLEMENTATION DETAILS:**
- Direction changes: Rare (few seconds intervals) - I2C latency acceptable
- Hardware: CJMCU-2317 expanders (16-bit I2C)
- Capacity: 6 motors planned, dual expanders for expansion
- Safety: Cut PUL generation if I2C communication fails

### Assumption Reversal Session

**ASSUMPTIONS TESTED & DECISIONS:**
- ❌ Hidden motor creation in Axis: Rejected - want transparent object creation at startup
- ❌ Polling PUL/DIR hardware: Impossible - must push data to hardware
- ❌ Isolate errors per axis: Rejected - stop all and retry from master level

**KEY INSIGHT:** Master-level error handling with full system restart preferred over partial recovery

**ADDITIONAL ASSUMPTIONS TESTED:**
- ❌ Coordinated calibration: Rejected - individual axis calibration is standard
- ❌ Central position manager: Rejected - no coordinated moves needed at this level
- ❌ Lookup tables: Rejected - not common case, real-time calculation preferred
- ✅ Built-in acceleration profiles: ACCEPTED - motors should handle acceleration curves internally

## Idea Categorization

### Immediate Opportunities

_Ready for MVP implementation:_

- **Motor class:** PUL/DIR/EN GPIO interface with pulse counting position
- **Axis constructor:** Direct transmission ratios (angular/linear) 
- **I2C expander integration:** CJMCU-2317 for EN/DIR/brake/switches
- **Sensor subscription system:** Callback patterns for edge detection
- **Calibration procedures:** Individual axis with end switch transitions
- **Built-in acceleration profiles:** Motion smoothness at motor level
- **Safety system:** Cut PUL on I2C failure, master-level error handling

_Ideas ready to implement now_

{{immediate_opportunities}}

### Future Innovations

_Post-MVP development:_

- **Encoder integration:** Direct GPIO + external IC support (PCNT example foundation)
- **Enhanced sensor types:** Current sensors for effort detection  
- **Advanced transmission modes:** Support for complex gear trains
- **Multi-axis coordination:** If needed for future applications

### Moonshots

_Ambitious, transformative concepts_

- **Adaptive motor tuning:** Self-optimizing acceleration profiles based on load feedback
- **Predictive maintenance:** Sensor data analysis for proactive component replacement
- **Hot-swappable axis modules:** Runtime addition/removal of motor axes without restart

### Epic Breakdown

**EPIC 1 - Core Hardware Foundation:**
- Motor class (PUL/DIR/EN GPIO interface)
- I2C expander integration (CJMCU-2317)
- Sensor subscription system (callbacks, edge detection)
- USB communication with host
- LED screen indication

**EPIC 2 - Advanced Control Features:**
- Calibration procedures (end switch transitions)
- Built-in acceleration profiles
- Safety system (I2C failure handling, master error control)

### Insights and Learnings

_Key realizations from the session_

- **Critical Insight:** ESP32 GPIO limitations drove the optimal GPIO/I2C split architecture
- **Technical Breakthrough:** Parallel encoder reading possible while servo drive uses same encoder
- **Design Pattern:** Sensor subscription with edge detection enables flexible calibration procedures
- **Implementation Reality:** Built-in acceleration profiles belong in Motor class, not external controllers
- **Architecture Validation:** First Principles thinking confirmed Axis→Motor→Hardware hierarchy is correct

## Action Planning

### Top 3 Priority Ideas

#### #1 Priority: ESP32-S3 GPIO Mapping & Test Circuit Design

- **Rationale:** Hardware foundation must be planned before software implementation
- **Next steps:** 
  - Research ESP32-S3 pinout and GPIO capabilities
  - Map pins for PUL signals (6 motors = 6 GPIO pins)
  - Reserve encoder pins (12 GPIO pins for future use)
  - Design I2C connections for CJMCU-2317 expanders
  - Create breadboard test circuit for single motor
- **Resources needed:** ESP32-S3 datasheet, GPIO mapping tools, breadboard components
- **Timeline:** 1-2 days for mapping and circuit design

#### #2 Priority: ESP-IDF Examples Collection & Study

- **Rationale:** Need proven patterns for GPIO, I2C, USB, display integration
- **Next steps:**
  - Find ESP-IDF PCNT (Pulse Counter) examples for encoder foundation
  - Locate I2C master examples for expander communication
  - Research USB CDC examples for host communication
  - Find display driver examples for LED screen
  - Study project structure and build system
- **Resources needed:** ESP-IDF documentation, GitHub example repositories
- **Timeline:** 2-3 days for research and understanding

#### #3 Priority: Motor Class Software Architecture Planning

- **Rationale:** Core abstraction that everything else builds upon
- **Next steps:**
  - Define Motor class interface (moveToPosition, getCurrentPosition, etc.)
  - Plan GPIO abstraction for PUL/DIR/EN signals
  - Design I2C expander interface for low-speed signals
  - Create axis constructor signature with transmission ratios
  - Plan sensor callback registration system
- **Resources needed:** C++ design patterns, ESP-IDF GPIO/I2C APIs
- **Timeline:** 2-3 days for architecture design

## Reflection and Follow-up

### What Worked Well

- **First Principles Thinking:** Stripped away assumptions to reveal fundamental hardware realities
- **Morphological Analysis:** Systematic exploration revealed optimal GPIO/I2C split architecture  
- **Assumption Reversal:** Validated design decisions against alternative approaches
- **Technical focus:** Deep dive into specific implementation details rather than abstract concepts

### Areas for Further Exploration

- **Real-time timing analysis:** Precise PUL signal requirements and jitter tolerance
- **Advanced sensor integration:** Current sensing and force feedback implementation
- **Communication protocol design:** Command structure for host-ESP32 interface
- **Error recovery strategies:** Beyond simple stop-and-restart patterns

### Recommended Follow-up Techniques

- **Mind Mapping:** Visual architecture diagrams for class relationships
- **SCAMPER Method:** Optimize each component (substitute I2C alternatives, combine functions, etc.)
- **Resource Constraints:** Design under extreme limitations to find essential features

### Questions That Emerged

- **What ESP-IDF project structure is optimal for modular motor control?**
- **How do popular motor control libraries handle the GPIO/I2C split pattern?**
- **What are typical PUL signal frequencies for stepper/servo applications?**
- **Should sensor callbacks run in interrupt context or task context?**

### Next Session Planning

- **Suggested topics:** GPIO mapping session, ESP-IDF example analysis, class interface design
- **Recommended timeframe:** After initial research phase (1 week)
- **Preparation needed:** ESP32-S3 hardware, basic circuit prototyping, IDF examples collected

---

_Session facilitated using the BMAD CIS brainstorming framework_