# nRF54L15 Firmware (EMS + BLE + PPG)

## Overview
This firmware runs on `nrf54l15dk` and integrates:

- EMS waveform generation and control
- BLE control/telemetry (including flash data streaming)
- PPG + accelerometer processing pipeline
- Power management (sleep/wake via PMIC)
- Watchdog-based runtime health checks

## Runtime Data Flow
1. BLE writes commands (EMS/PPG/flash control).
2. Commands are parsed and pushed to internal message queues.
3. Main/PPG threads consume queue messages and update runtime state.
4. Sensor and algorithm pipeline produces PPG/features.
5. Data is streamed over BLE and/or stored in flash.
6. Watchdog and health logic monitor thread liveness.

## Test Strategy (Portable)
Unit-only algorithm tests are intentionally excluded for now.  
Focus is on integration, runtime robustness, HIL, and release gates.

### Test Flow
1. PR: run fast L1 integration suites on simulator.
2. Nightly: run L1 + selected L2/L3 on hardware.
3. Weekly: run full L2/L3 including soak and fault injection.
4. Release: run L4 regression gate (must fully pass).

## Compact Test Matrix

| Level | Suite | Runs On | Trigger | Primary Goal |
|---|---|---|---|---|
| L1 | Message Path Integration | Simulator | Every PR | Validate BLE write parsing -> queue payload shaping |
| L1 | PPG Control Integration | Simulator | Every PR | Validate START/STOP command mapping and queue behavior |
| L1 | Flash Stream Control Integration | Simulator | Every PR | Validate start/resume/reset transfer state behavior |
| L2 | RTOS Runtime Stability | Dev board | Nightly | Validate queue/timer/thread behavior under load |
| L2 | Connectivity Robustness | Dev board | Nightly | Validate connect/disconnect/reconnect recovery |
| L2 | Data Pipeline Robustness | Dev board | Nightly | Validate sustained ingest/process/stream behavior |
| L3 | Driver HIL Validation | HIL rig | Nightly/Weekly | Validate AS7058/BMA580/nPM1300/PWM/flash timing paths |
| L3 | Power Sleep/Wake Reliability | HIL rig | Nightly | Validate sleep entry/exit and state restore behavior |
| L3 | Endurance/Soak | HIL rig | Weekly | Validate long-run stability (memory/queues/watchdog) |
| L3 | Fault Injection | HIL rig | Weekly | Validate graceful recovery from bus/flash/BLE faults |
| L4 | Release Regression Gate | CI + HIL | Release | Validate end-to-end critical path before shipping |

## What Each Suite Does
- `Message Path Integration`: Feeds BLE write inputs and verifies correct internal message ID/payload shaping and queue insertion.
- `PPG Control Integration`: Verifies BLE PPG control commands (`START/STOP`) map to correct command queue behavior.
- `Flash Stream Control Integration`: Verifies flash transfer control flow (`start/resume/reset`) and legal transfer state transitions.
- `RTOS Runtime Stability`: Stress-checks thread/queue/timer behavior to catch deadlocks, starvation, and watchdog feed issues.
- `Connectivity Robustness`: Repeats connect/disconnect/reconnect cycles to verify advertising restart and link recovery.
- `Data Pipeline Robustness`: Validates sustained sensor ingest -> processing -> BLE/flash path under realistic load.
- `Driver HIL Validation`: Validates real hardware driver behavior (AS7058/BMA580/nPM1300/PWM/flash timing) on target setup.
- `Power Sleep/Wake Reliability`: Verifies sleep entry/exit, wake-source handling, and state restoration after wake/reset.
- `Endurance/Soak`: Runs long-duration scenarios (8h/24h) to catch leaks, drift, and runtime instability.
- `Fault Injection`: Injects controlled flash/I2C/BLE faults and verifies graceful degradation and recovery paths.
- `Release Regression Gate`: Executes critical-path end-to-end suite as final release blocker.

## Quality Gates

| Metric | Target |
|---|---|
| Integration pass rate | 100% |
| HIL critical pass rate | 100% |
| Reconnect reliability | >= 99% (configured cycle test) |
| Runtime crash/assert count | 0 |
| Unexpected watchdog reset | 0 |
| Data loss | Within product threshold |

## Portability Rules
- Keep tests under `tests/` with independent configs.
- Keep interface contracts versioned (message IDs, state transitions, thresholds).
- Use simulator for fast feedback; HIL for hardware truth.
- Reuse the same matrix across previous/next firmware versions.
