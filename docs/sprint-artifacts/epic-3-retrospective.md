# Epic 3: Motor Control Core - RETROSPECTIVE

**Date:** 2025-12-17
**Facilitator:** SM Agent (Bob)
**Participants:** Sergey (Project Lead)

---

## Epic 3 Summary

| Metric | Value |
|--------|-------|
| **Epic Status** | COMPLETE |
| **Stories Completed** | 13 of 13 (100%) |
| **Stories Reviewed** | 3-9, 3-9b, 3-10, 3-11 (formal AI reviews) |
| **Hardware Testing Sessions** | 4 documented sessions |
| **Critical Learnings Captured** | 5 major architectural insights |

### Story Completion Timeline

| Story | Title | Status |
|-------|-------|--------|
| 3-1 | Shift Register Driver | done |
| 3-2 | RMT Pulse Generator | done |
| 3-3 | MCPWM Pulse Generator with PCNT | done |
| 3-4 | LEDC Pulse Generator | done |
| 3-5 | Position Tracker Interface | done |
| 3-6 | Motor Base Class (ServoMotor) | done |
| 3-7 | Stepper Motor Implementation | done |
| 3-8 | Discrete Axis Implementation | done |
| 3-9 | Motion Controller & CMD_MOVE/MOVR | done |
| 3-9b | Motor System Integration | done |
| 3-9c | RMT FastAccelStepper Refactor | done |
| 3-10 | CMD_VEL/STOP/EN/POS Commands | done |
| 3-11 | Motion Completion Events | done |

---

## What Went Well

### 1. Story-to-Story Learning Propagation

Story 3-10 introduced the "Learnings from Previous Story" section which captured critical patterns:
- **"Position is a Promise"** - Position tracked at queue time, not ISR time
- **"Two-Phase Completion"** - RMT needs graceful shutdown with pause symbols
- **Direction handling** - target_pos always positive, queue_end_.position signed

These learnings were formally cited in Story 3-11's context, creating knowledge continuity.

### 2. Mandatory "No Magic Numbers" Constraint (AC14)

Every story included AC14 requiring all numeric values from config headers. This resulted in:
- 8 config header files (`config_*.h`) with comprehensive constants
- Grep verification checks as standard review practice
- Clean, maintainable code with single source of truth for values

### 3. Hardware Testing Documentation

Story 3-10 documented **4 extensive hardware testing sessions** with:
- Specific bug descriptions and root cause analysis
- Code snippets showing before/after fixes
- Test result tables with PASS/FAIL status
- FastAccelStepper library insights applied to our implementation

### 4. Proactive Integration Gap Identification

Story 3-9 identified that motors weren't wired to the system. Rather than leaving this undocumented:
- Dev explicitly called out the gap in Completion Notes
- Led directly to Story 3-9b creation
- Pattern of "integration stories" established

### 5. Comprehensive Code Reviews

AI-assisted code reviews with:
- AC-by-AC validation with file:line evidence
- Task completion verification against actual code
- Security notes (input validation, null pointer checks)
- Architectural alignment confirmation

---

## What Could Be Improved

### 1. Task Checkbox Discipline

Multiple stories had tasks marked `[ ]` when implementation was actually complete:
- Story 3-9: All 13 tasks implemented but shown unchecked
- Story 3-9b: 12 of 13 tasks done but marked incomplete

**Action Item:** Update task checkboxes immediately after completing work, not during review.

### 2. Minor AC Gaps Slipped Through

- Story 3-9b has hardcoded `1.0f` for E-axis units_per_rev (should be constant)
- Story 3-10 AC11 (notification-based blocking) uses polling instead

**Action Item:** Add minor AC deviations to explicit "Known Issues" section in story file.

### 3. Integration Tests Deferred

Several stories had integration tests marked as "pending hardware":
- Story 3-9c deferred 4-channel simultaneous verification
- Story 3-10 marked hardware tests as incomplete

**Action Item:** Create explicit "Hardware Test Backlog" tracking for deferred tests.

---

## Key Technical Learnings

### Learning 1: Position is a Promise

```cpp
// Position updated when commands QUEUED (not in ISR)
// position_tracker_ reflects where motor WILL BE after queued commands
if (position_tracker_ && cmd.steps > 0) {
    position_tracker_->addPulses(cmd.steps);
}
```

**Why it matters:** Avoids ISR timing issues, simplifies completion detection.

### Learning 2: Two-Phase Completion Detection

```
Phase 1: Queue empty -> set rmt_stopped_, fill final pause symbols
Phase 2: Next callback sees rmt_stopped_ -> return done=true
Phase 3: onTransmitDone() fires -> motion truly complete
```

**Why it matters:** RMT hardware needs time to transmit final symbols before stopping.

### Learning 3: Signed vs Absolute Position Handling

```cpp
// For FWD: remaining = target_pos - queue_end_.position
// For REV: remaining = target_pos + queue_end_.position  // position is negative
```

**Why it matters:** Reverse moves broke silently due to sign handling bugs. Test both directions!

### Learning 4: uint8_t Overflow in Comparisons

```cpp
// WRONG: static_cast<uint8_t>(2820) = 252 (overflow!)
if (cmd.steps > static_cast<uint8_t>(remaining)) { ... }

// CORRECT: Cast the smaller type UP
if (static_cast<int32_t>(cmd.steps) > remaining) { ... }
```

**Why it matters:** Integer overflow bugs are silent and cause unpredictable motion termination.

### Learning 5: ISR Position Tracking Double-Sign Bug

```cpp
// WRONG: addPulses() already applies direction internally
if (!(cmd.flags & CMD_FLAG_DIRECTION)) steps = -steps;
position_tracker_->addPulses(steps);  // Double-signed!

// CORRECT: Pass unsigned, let tracker handle direction
position_tracker_->addPulses(cmd.steps);  // Always positive
```

**Why it matters:** Position drifted continuously positive due to double negation.

---

## Epic 4 Preview: Safety & I/O Systems

Epic 4 contains **14 stories** focused on safety-critical systems:

| Story | Title | Priority |
|-------|-------|----------|
| 4-1 | MCP23017 I/O Expander Driver | Foundation |
| 4-2 | Limit Switch Monitoring | Safety |
| 4-3 | Limit Switch Motion Stop | Safety |
| 4-4 | Emergency Stop System | Safety (Critical) |
| 4-5 | Brake Control System | Safety |
| 4-6 | Position Loss Detection | Safety |
| 4-7 | General Purpose Digital I/O | Foundation |
| 4-8 | Servo Feedback Processing | Feedback |
| 4-9 | C-Axis Floating Switch | Application |
| 4-10 | Error Tracking & Recovery | Reliability |
| 4-11 | I2C Communication Health | Reliability |
| 4-12 | Error Logging | Observability |
| 4-13 | Homing Sequences | Motion |
| 4-14 | Driver Alarm Monitoring | Safety |

### Recommended Approach for Epic 4

1. **Context Epic 4 first** - Create tech-spec before drafting stories
2. **I2C stability first** - 4-1 and 4-11 establish MCP23017 communication
3. **Safety stories in sequence** - 4-2 -> 4-3 -> 4-4 -> 4-5 (builds on each other)
4. **Leverage Epic 3 event system** - Motion completion events from 3-11 provide foundation for limit/error events

---

## Action Items

### Process Improvements

- [ ] Update task checkboxes immediately upon completion
- [ ] Add "Known Deviations" section to story template
- [ ] Create Hardware Test Backlog tracking file
- [ ] Establish "Learnings from Previous Story" as mandatory section

### Technical Debt

- [ ] Add `E_AXIS_UNITS_PER_REV` constant to config_defaults.h
- [ ] Consider notification-based wake for motion_task (CPU efficiency)
- [ ] Complete deferred hardware tests from Epic 3

### Epic 4 Preparation

- [ ] Run `*epic-tech-context` for Epic 4 before first story draft
- [ ] Identify I2C communication patterns needed for MCP23017
- [ ] Review safety-critical coding standards for Epic 4 stories

---

## Retrospective Status

**Outcome:** Epic 3 successfully completed with strong documentation practices established. Key learnings captured for future reference. Ready to proceed to Epic 4.

**Next Steps:**
1. Context Epic 4 (`*epic-tech-context`)
2. Draft Story 4-1 (MCP23017 I/O Expander Driver)
3. Begin Safety & I/O Systems implementation

---

*Generated by SM Agent (Bob) - BMAD Method*
