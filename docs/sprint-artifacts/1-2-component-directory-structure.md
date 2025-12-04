# Story 1.2: Component Directory Structure

Status: review

## Story

As a **developer**,
I want **the component folder structure established per architecture spec**,
so that **all subsequent code has a defined home and dependencies are clear**.

## Acceptance Criteria

1. **AC1:** Given the project is initialized, when I examine the `firmware/components/` directory, then the following structure exists with all subdirectories created
2. **AC2:** Each component directory contains a valid `CMakeLists.txt` with `idf_component_register()`
3. **AC3:** Each component has an `include/` directory for public headers
4. **AC4:** The project builds successfully with `cd firmware && idf.py build` (empty components don't break build)
5. **AC5:** Component dependency declarations use `REQUIRES` and `PRIV_REQUIRES` appropriately (even if empty for now)

## Tasks / Subtasks

- [x] **Task 1: Create HAL layer components** (AC: 1, 2, 3)
  - [x] Create `firmware/components/hal/gpio_hal/` with CMakeLists.txt and include/
  - [x] Create `firmware/components/hal/i2c_hal/` with CMakeLists.txt and include/
  - [x] Create `firmware/components/hal/spi_hal/` with CMakeLists.txt and include/

- [x] **Task 2: Create driver components** (AC: 1, 2, 3)
  - [x] Create `firmware/components/drivers/tpic6b595/` with CMakeLists.txt and include/
  - [x] Create `firmware/components/drivers/oled/` with CMakeLists.txt and include/
  - [x] Note: MCP23017 uses espressif/mcp23017 from Component Registry (managed_components/) - no custom component needed

- [x] **Task 3: Create pulse generation component** (AC: 1, 2, 3)
  - [x] Create `firmware/components/pulse_gen/` with CMakeLists.txt and include/

- [x] **Task 4: Create position tracking component** (AC: 1, 2, 3)
  - [x] Create `firmware/components/position/` with CMakeLists.txt and include/

- [x] **Task 5: Create motor abstraction component** (AC: 1, 2, 3)
  - [x] Create `firmware/components/motor/` with CMakeLists.txt and include/

- [x] **Task 6: Create control layer components** (AC: 1, 2, 3)
  - [x] Create `firmware/components/control/motion_controller/` with CMakeLists.txt and include/
  - [x] Create `firmware/components/control/safety_monitor/` with CMakeLists.txt and include/
  - [x] Create `firmware/components/control/command_executor/` with CMakeLists.txt and include/

- [x] **Task 7: Create interface layer components** (AC: 1, 2, 3)
  - [x] Create `firmware/components/interface/usb_cdc/` with CMakeLists.txt and include/
  - [x] Create `firmware/components/interface/command_parser/` with CMakeLists.txt and include/

- [x] **Task 8: Create configuration components** (AC: 1, 2, 3)
  - [x] Create `firmware/components/config/nvs_manager/` with CMakeLists.txt and include/
  - [x] Create `firmware/components/config/yaml_parser/` with CMakeLists.txt and include/

- [x] **Task 9: Create event system component** (AC: 1, 2, 3)
  - [x] Create `firmware/components/events/event_manager/` with CMakeLists.txt and include/

- [x] **Task 10: Verify build** (AC: 4, 5)
  - [x] Run `cd firmware && idf.py build` to verify all components are recognized
  - [x] Confirm no build errors from empty components

## Dev Notes

### Architecture Constraints

> **MANDATORY: ESP-IDF Component Structure**
>
> Each component MUST follow ESP-IDF component conventions:
> - `CMakeLists.txt` at component root with `idf_component_register()`
> - `include/` directory for public headers
> - Source files (.c/.cpp) at component root or in `src/`

> **MANDATORY: No Implementation Code**
>
> This story creates structure only. All CMakeLists.txt files should be minimal stubs.
> Implementation code belongs in subsequent stories.

### CMakeLists.txt Template

For empty components (no source files yet), use this minimal template:

```cmake
idf_component_register(
    INCLUDE_DIRS "include"
)
```

For components that will have dependencies (declare now, implement later):

```cmake
idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES ""
    PRIV_REQUIRES ""
)
```

### Expected Component Dependency Graph (for reference)

This graph shows the intended dependencies. Leave `REQUIRES` empty for now - actual dependencies will be added when implementation begins:

```
control/
├── motion_controller → motor, pulse_gen, position, events, config
├── safety_monitor → drivers/mcp23017, drivers/tpic6b595, events
└── command_executor → motor, interface/command_parser, events

interface/
├── usb_cdc → (ESP-IDF tinyusb)
└── command_parser → config

motor → pulse_gen, position, hal

drivers/
├── tpic6b595 → hal/spi_hal
└── oled → hal/i2c_hal

events/event_manager → (standalone)
config/ → events
```

### Directory Structure After This Story

```
firmware/
├── CMakeLists.txt              # (from Story 1.1)
├── main/                       # (from Story 1.1)
├── managed_components/         # (auto-managed by ESP Component Registry)
│   ├── espressif__mcp23017/
│   └── espressif__i2c_bus/
└── components/                 # NEW - created in this story
    ├── hal/
    │   ├── gpio_hal/
    │   │   ├── CMakeLists.txt
    │   │   └── include/
    │   ├── i2c_hal/
    │   │   ├── CMakeLists.txt
    │   │   └── include/
    │   └── spi_hal/
    │       ├── CMakeLists.txt
    │       └── include/
    ├── drivers/
    │   ├── tpic6b595/
    │   │   ├── CMakeLists.txt
    │   │   └── include/
    │   └── oled/
    │       ├── CMakeLists.txt
    │       └── include/
    ├── pulse_gen/
    │   ├── CMakeLists.txt
    │   └── include/
    ├── position/
    │   ├── CMakeLists.txt
    │   └── include/
    ├── motor/
    │   ├── CMakeLists.txt
    │   └── include/
    ├── control/
    │   ├── motion_controller/
    │   │   ├── CMakeLists.txt
    │   │   └── include/
    │   ├── safety_monitor/
    │   │   ├── CMakeLists.txt
    │   │   └── include/
    │   └── command_executor/
    │       ├── CMakeLists.txt
    │       └── include/
    ├── interface/
    │   ├── usb_cdc/
    │   │   ├── CMakeLists.txt
    │   │   └── include/
    │   └── command_parser/
    │       ├── CMakeLists.txt
    │       └── include/
    ├── config/
    │   ├── nvs_manager/
    │   │   ├── CMakeLists.txt
    │   │   └── include/
    │   └── yaml_parser/
    │       ├── CMakeLists.txt
    │       └── include/
    └── events/
        └── event_manager/
            ├── CMakeLists.txt
            └── include/
```

**Component Count:** 17 custom components (18 directories including parent grouping folders)

### Verification Commands

```bash
# Source environment
get_idf

# Navigate to firmware directory
cd /Users/sergeybarabash/robo/yarobot_control_unit/firmware

# Verify structure
find components -name CMakeLists.txt | wc -l  # Should be 17

# Build to verify no errors
idf.py build

# List all components recognized by build system
idf.py reconfigure 2>&1 | grep -E "Component|components"
```

### Why This Structure?

**Layered Architecture Benefits:**
1. **HAL Layer** - Abstracts ESP-IDF peripheral APIs, enables unit testing with mocks
2. **Drivers Layer** - Device-specific code isolated from business logic
3. **Control Layer** - Business logic separated from hardware concerns
4. **Interface Layer** - Communication protocols decoupled from processing

**ESP-IDF Component Benefits:**
- Automatic dependency resolution via `REQUIRES`/`PRIV_REQUIRES`
- Separate compilation units for faster incremental builds
- Clear public API boundaries via `include/` directories
- Easy to add unit tests per component

### References

**Project Documentation:**
- [Source: docs/architecture.md#Project-Structure] - Complete directory layout
- [Source: docs/architecture.md#FR-Category-to-Architecture-Mapping] - Component responsibilities
- [Source: docs/epics.md#Story-1.2] - Original acceptance criteria

**ESP-IDF Official Documentation:**
- [ESP-IDF Build System - Component CMakeLists](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/build-system.html#component-cmakelists-files)
- [ESP-IDF Component Requirements](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/build-system.html#component-requirements)

---

## Dev Agent Record

### Context Reference

- `docs/sprint-artifacts/1-2-component-directory-structure.context.xml`

### Agent Model Used

claude-opus-4-5-20251101

### Debug Log References

N/A - Structure-only story, no debugging required.

### Completion Notes List

- Created 16 component directories with CMakeLists.txt and include/ subdirectories
- All components use minimal `idf_component_register(INCLUDE_DIRS "include")` template
- Build verified successful with ESP-IDF 5.4 - no errors from empty components
- Note: Story mentioned 17 components but actual task count is 16 (MCP23017 uses managed component)

### File List

**New Files:**
- firmware/components/hal/gpio_hal/CMakeLists.txt
- firmware/components/hal/i2c_hal/CMakeLists.txt
- firmware/components/hal/spi_hal/CMakeLists.txt
- firmware/components/drivers/tpic6b595/CMakeLists.txt
- firmware/components/drivers/oled/CMakeLists.txt
- firmware/components/pulse_gen/CMakeLists.txt
- firmware/components/position/CMakeLists.txt
- firmware/components/motor/CMakeLists.txt
- firmware/components/control/motion_controller/CMakeLists.txt
- firmware/components/control/safety_monitor/CMakeLists.txt
- firmware/components/control/command_executor/CMakeLists.txt
- firmware/components/interface/usb_cdc/CMakeLists.txt
- firmware/components/interface/command_parser/CMakeLists.txt
- firmware/components/config/nvs_manager/CMakeLists.txt
- firmware/components/config/yaml_parser/CMakeLists.txt
- firmware/components/events/event_manager/CMakeLists.txt

**New Directories (include/):**
- firmware/components/hal/gpio_hal/include/
- firmware/components/hal/i2c_hal/include/
- firmware/components/hal/spi_hal/include/
- firmware/components/drivers/tpic6b595/include/
- firmware/components/drivers/oled/include/
- firmware/components/pulse_gen/include/
- firmware/components/position/include/
- firmware/components/motor/include/
- firmware/components/control/motion_controller/include/
- firmware/components/control/safety_monitor/include/
- firmware/components/control/command_executor/include/
- firmware/components/interface/usb_cdc/include/
- firmware/components/interface/command_parser/include/
- firmware/components/config/nvs_manager/include/
- firmware/components/config/yaml_parser/include/
- firmware/components/events/event_manager/include/

---

## Change Log

| Date | Author | Change |
|------|--------|--------|
| 2025-12-03 | SM Agent (Bob) | Initial story draft |
| 2025-12-03 | SM Agent (Bob) | Story context created, marked ready-for-dev |
| 2025-12-03 | Dev Agent (Amelia) | Implementation complete - all 16 components created, build verified |
