# Story 3.1: Shift Register Driver

Status: ready-for-dev

## Story

As a **developer**,
I want **the shift register chain operational**,
so that **I can control direction, enable, brake, and alarm-clear signals for all motor axes**.

## Acceptance Criteria

1. **AC1:** Given SPI2 is initialized per Story 1.6 hardware verification, when `sr_init()` is called, then it returns ESP_OK and the 40-bit shift register chain is ready for operation
2. **AC2:** Given the shift register is initialized, when `sr_set_direction(axis, forward)` is called, then the corresponding SR_x_DIR bit is set/cleared in the shadow register
3. **AC3:** Given the shift register is initialized, when `sr_set_enable(axis, enable)` is called, then the corresponding SR_x_EN bit is set/cleared in the shadow register
4. **AC4:** Given the shift register is initialized, when `sr_set_brake(axis, release)` is called, then the corresponding SR_x_BRAKE bit is set/cleared (axes X-D only, E has no brake)
5. **AC5:** Given the shift register is initialized, when `sr_set_alarm_clear(axis, active)` is called, then the corresponding SR_x_ALARM_CLR bit is set/cleared (axes X-D only)
6. **AC6:** Given the shift register is initialized, when `sr_set_gp_output(pin, level)` is called, then the corresponding SR_GP_OUT_n bit (0-7) is set/cleared
7. **AC7:** Given bit changes have been made, when `sr_update()` is called, then all 40 bits are shifted out via SPI and latched to outputs
8. **AC8:** Given I call `sr_get_state()`, when the function returns, then it provides the current 40-bit shadow register state as uint64_t
9. **AC9:** Given an emergency condition, when `sr_emergency_disable_all()` is called, then all 40 bits are set to 0 (SR_SAFE_STATE), GPIO_SR_OE goes HIGH (outputs tristated), and this completes within 100µs
10. **AC10:** Given multiple tasks call sr_set_* functions concurrently, when updates occur, then thread safety is maintained via mutex protection
11. **AC11:** Given `sr_update()` is called, when SPI transfer completes, then GPIO_SR_LATCH is pulsed to latch data to outputs
12. **AC12:** Given normal operation, when shift register is active, then GPIO_SR_OE remains LOW (outputs enabled)

## Tasks / Subtasks

- [ ] **Task 1: Create component structure** (AC: 1)
  - [ ] Create `firmware/components/drivers/tpic6b595/` directory
  - [ ] Create CMakeLists.txt with REQUIRES for driver, esp_driver_spi, config
  - [ ] Create `include/tpic6b595.h` with public API declarations
  - [ ] Create `tpic6b595.c` with implementation

- [ ] **Task 2: Define configuration in config_sr.h** (AC: 2-6)
  - [ ] Create `firmware/components/config/include/config_sr.h` if not exists
  - [ ] Define bit positions for each axis (SR_X_DIR=0, SR_X_EN=1, SR_X_BRAKE=2, SR_X_ALARM_CLR=3, etc.)
  - [ ] Define SR_GP_OUT_0 through SR_GP_OUT_7 for bits 32-39
  - [ ] Define SR_SAFE_STATE as 0x0000000000 (all bits low)
  - [ ] Define SR_CHAIN_LENGTH as 5 (number of TPIC6B595N chips)
  - [ ] Define SR_BITS_TOTAL as 40

- [ ] **Task 3: Verify/add GPIO definitions in config_gpio.h** (AC: 1, 7, 9, 11, 12)
  - [ ] Verify GPIO_SR_MOSI is defined (SPI2 MOSI)
  - [ ] Verify GPIO_SR_CLK is defined (SPI2 CLK)
  - [ ] Verify GPIO_SR_LATCH is defined (latch/RCLK pin)
  - [ ] Verify GPIO_SR_OE is defined (output enable, active-low)

- [ ] **Task 4: Implement sr_init()** (AC: 1, 12)
  - [ ] Configure SPI2 bus if not already configured (check HAL layer)
  - [ ] Add SPI device with appropriate clock speed (1-10 MHz typical)
  - [ ] Configure GPIO_SR_LATCH as output, initially LOW
  - [ ] Configure GPIO_SR_OE as output, initially LOW (outputs enabled)
  - [ ] Initialize shadow register to SR_SAFE_STATE
  - [ ] Initialize mutex for thread safety
  - [ ] Call sr_update() to latch safe state to hardware

- [ ] **Task 5: Implement shadow register and bit manipulation** (AC: 2-6, 8)
  - [ ] Create static uint64_t shadow_register variable
  - [ ] Implement helper: `set_bit(uint8_t position, bool value)`
  - [ ] Implement helper: `get_bit(uint8_t position)`
  - [ ] Implement sr_set_direction() - validate axis 0-7, set SR_x_DIR bit
  - [ ] Implement sr_set_enable() - validate axis 0-7, set SR_x_EN bit
  - [ ] Implement sr_set_brake() - validate axis 0-6 (no E brake), set SR_x_BRAKE bit
  - [ ] Implement sr_set_alarm_clear() - validate axis 0-6 (no E alarm), set SR_x_ALARM_CLR bit
  - [ ] Implement sr_set_gp_output() - validate pin 0-7, set SR_GP_OUT_n bit
  - [ ] Implement sr_get_state() - return shadow_register

- [ ] **Task 6: Implement sr_update()** (AC: 7, 11)
  - [ ] Acquire mutex
  - [ ] Extract 5 bytes from shadow register (LSB first for daisy chain)
  - [ ] Perform SPI DMA transfer of 5 bytes
  - [ ] Pulse GPIO_SR_LATCH (HIGH then LOW, minimum 20ns pulse)
  - [ ] Release mutex
  - [ ] Return ESP_OK on success

- [ ] **Task 7: Implement sr_emergency_disable_all()** (AC: 9)
  - [ ] This function must be ISR-safe (no mutex, no blocking)
  - [ ] Set GPIO_SR_OE HIGH immediately (tristate outputs)
  - [ ] Set shadow register to SR_SAFE_STATE
  - [ ] Perform direct SPI register write (bypass driver for speed)
  - [ ] Pulse GPIO_SR_LATCH
  - [ ] Leave GPIO_SR_OE HIGH (outputs remain tristated until explicit recovery)
  - [ ] Total execution time must be < 100µs

- [ ] **Task 8: Implement thread safety** (AC: 10)
  - [ ] Create static SemaphoreHandle_t sr_mutex
  - [ ] Initialize mutex in sr_init()
  - [ ] Wrap all sr_set_* functions with mutex take/give
  - [ ] sr_update() already uses mutex (Task 6)
  - [ ] sr_emergency_disable_all() must NOT use mutex (ISR-safe)

- [ ] **Task 9: Create unit tests** (AC: 1-12)
  - [ ] Create `firmware/components/drivers/tpic6b595/test/test_tpic6b595.c`
  - [ ] Test sr_init() returns ESP_OK
  - [ ] Test each sr_set_* function updates correct bit in shadow register
  - [ ] Test sr_get_state() returns expected shadow register value
  - [ ] Test bit positions match config_sr.h definitions
  - [ ] Test invalid axis/pin parameters return ESP_ERR_INVALID_ARG
  - [ ] Test sr_emergency_disable_all() sets all bits to 0
  - [ ] Test thread safety with multiple task simulation (if feasible)

- [ ] **Task 10: Build and hardware verification** (AC: 1-12)
  - [ ] Run `idf.py build` - no errors or warnings
  - [ ] Run unit tests - all pass
  - [ ] Verify SPI signals with logic analyzer (MOSI, CLK, LATCH timing)
  - [ ] Verify output enable behavior (OE LOW normal, HIGH emergency)
  - [ ] Measure sr_emergency_disable_all() execution time < 100µs

## Dev Notes

### Architecture Constraints

> **Header-Only Configuration (from Tech Spec)**
>
> Every configurable value MUST be defined in a header file. No magic numbers in source code. GPIO pins from `config_gpio.h`, shift register bits from `config_sr.h`.
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#MANDATORY-Architecture-Constraints]

> **Shift Register Bit Map (from Tech Spec)**
>
> | Bits | Axis | SR_x_DIR | SR_x_EN | SR_x_BRAKE | SR_x_ALARM_CLR |
> |------|------|----------|---------|------------|----------------|
> | 0-3 | X | 0 | 1 | 2 | 3 |
> | 4-7 | Y | 4 | 5 | 6 | 7 |
> | 8-11 | Z | 8 | 9 | 10 | 11 |
> | 12-15 | A | 12 | 13 | 14 | 15 |
> | 16-19 | B | 16 | 17 | 18 | 19 |
> | 20-23 | C | 20 | 21 | 22 | 23 |
> | 24-27 | D | 24 | 25 | 26 | 27 |
> | 28-31 | E | 28 | 29 | - | - |
> | 32-39 | GP | SR_GP_OUT_0 - SR_GP_OUT_7 |
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Data-Models-and-Contracts]

> **ShiftRegisterController API (from Tech Spec)**
>
> ```c
> esp_err_t sr_init(void);
> esp_err_t sr_set_direction(uint8_t axis, bool forward);
> esp_err_t sr_set_enable(uint8_t axis, bool enable);
> esp_err_t sr_set_brake(uint8_t axis, bool release);
> esp_err_t sr_set_alarm_clear(uint8_t axis, bool active);
> esp_err_t sr_set_gp_output(uint8_t pin, bool level);
> esp_err_t sr_update(void);
> void sr_emergency_disable_all(void);
> uint64_t sr_get_state(void);
> ```
>
> [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs-and-Interfaces]

### Core Concepts

**TPIC6B595N Characteristics:**
- 8-bit serial-in, parallel-out shift register with latched outputs
- Open-drain outputs can sink up to 150mA at 50V
- Daisy-chain via serial data output (SDO → SDI of next chip)
- Latch pulse (RCLK) transfers shift register to output register
- Output enable (OE) controls all outputs (active-low)

**SPI Transfer Sequence:**
1. Prepare 5 bytes (40 bits) from shadow register
2. Send MSB of last chip first (SR4 GP outputs) → LSB of first chip last (SR0 X axis)
3. After all 40 bits shifted, pulse LATCH high then low
4. Outputs update on LATCH falling edge

**Fail-Safe Design:**
- TPIC6B595N outputs are open-drain with external pull-ups to 24V
- When OE is HIGH, outputs are tristated → pull-ups pull signals HIGH
- Motor driver logic: EN active-HIGH to enable, BRAKE active-LOW to release
- Safe state (all bits 0): All motors disabled, all brakes engaged

**Emergency Disable Strategy:**
- sr_emergency_disable_all() called from ISR (E-stop, fault detection)
- Must not block (no mutex, no RTOS calls)
- Direct register access for SPI to minimize latency
- OE goes HIGH immediately to tristate outputs
- Then shift register loaded with zeros as secondary protection

### Project Structure Notes

**New Component:**
- `firmware/components/drivers/tpic6b595/` — Shift register driver
- `firmware/components/drivers/tpic6b595/include/tpic6b595.h` — Public API
- `firmware/components/drivers/tpic6b595/tpic6b595.c` — Implementation
- `firmware/components/drivers/tpic6b595/CMakeLists.txt` — Build configuration
- `firmware/components/drivers/tpic6b595/test/test_tpic6b595.c` — Unit tests

**Config Files:**
- `firmware/components/config/include/config_sr.h` — Bit position definitions (new)
- `firmware/components/config/include/config_gpio.h` — GPIO_SR_* pin definitions (verify/add)

### Learnings from Previous Story

**From Story 2-7 (Status: review)**

- **Event System Available**: EVT_MOTION_COMPLETE and EVT_MOTION_ERROR event types ready for Epic 3 stories
- **No Direct Dependencies**: Story 3-1 does not depend on event system, but subsequent motor stories (3.6+) will use events
- **Component Pattern Established**: Follow same CMakeLists.txt pattern as event_manager component

**Files Referenced from 2-7:**
- `firmware/components/events/event_manager/CMakeLists.txt` — Template for component structure

[Source: docs/sprint-artifacts/2-7-event-system-foundation.md#Completion-Notes-List]

### References

- [Source: docs/epics.md#Story-3.1] — Story definition and API specification
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Services-and-Modules] — ShiftRegisterController module
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Data-Models-and-Contracts] — Shift register bit map
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#APIs-and-Interfaces] — API definition
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Acceptance-Criteria] — AC14, AC15
- [Source: docs/sprint-artifacts/tech-spec-epic-3.md#Test-Strategy-Summary] — Story 3.1 test cases

---

## Dev Agent Record

### Context Reference

- docs/sprint-artifacts/3-1-shift-register-driver.context.xml

### Agent Model Used

{{agent_model_name_version}}

### Debug Log References

### Completion Notes List

### File List

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-04 | SM Agent (Bob) | Initial story draft |
