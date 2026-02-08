# nrf54l15

## test details
# Firmware Test Suite Matrix (Portable, No Unit-Only Algo Tests)

| Level | Suite Name | Scope | What It Validates | Method | Dependencies | Execution Target | Frequency | Pass Criteria | Portability Notes |
|---|---|---|---|---|---|---|---|---|---|
| L1 | Message Path Integration | BLE write parsing -> queue payload shaping | Correct BLE characteristic write mapping, message IDs, payload conversion, queue enqueue behavior | ztest + mocks/stubs | Mock BLE, mock queue, minimal wrappers | Simulator (`native_sim`) | Every PR | 100% test pass, no dropped/invalid mapped command | Keep UUID-to-message mapping test vectors in shared file for reuse across versions |
| L1 | PPG Control Integration | PPG command flow from BLE/start-stop path | Correct transition requests (`START`, `STOP`), command queue semantics, invalid command handling | ztest + mocks/stubs | Mock AS7058 API, mock queue, mock logger | Simulator | Every PR | Valid commands enqueue correctly, invalid commands rejected safely | Reuse same command contract in all branches/releases |
| L1 | Flash Read/Stream Control Integration | Flash control command path | Start/resume/reset behavior, transfer state machine transitions, resume offset integrity | ztest + flash/BLE stubs | Mock flash read/write, mock notify path | Simulator | Every PR | State transitions match contract, no illegal state transitions | Keep transfer-state contract stable via shared enum test cases |
| L2 | RTOS Runtime Stability | Thread/queue/timer behavior under load | Queue pressure handling, timer jitter tolerance, no deadlocks, watchdog heartbeat continuity | On-target scenario tests | Real RTOS, optional telemetry hooks | Dev board | Nightly | No deadlock/reset in test window, watchdog fed correctly | Same scenarios reusable; only board config changes |
| L2 | Connectivity Robustness | BLE connect/disconnect/reconnect loops | Recovery after disconnects, re-advertising correctness, reconnect reliability | On-target scripted test | BLE central script + board | Dev board + phone/dongle | Nightly | Success rate >= 99% over N cycles, no stuck advertising state | Keep test script transport-agnostic (nRF/Android/iOS variants) |
| L2 | Data Pipeline Robustness | Sensor ingest -> processing -> BLE/flash | Sustained data flow without starvation, bounded drops, graceful backpressure | On-target stress run | Real sensors or sensor replay source | Dev board | Nightly | Meets configured drop/loss thresholds, no crash/assert | Thresholds configurable by product version |
| L3 | Hardware-In-Loop Driver Validation | AS7058, BMA580, nPM1300, PWM, flash timing | Driver init/recovery, IRQ handling, bus stability, timing margin | HIL automated script | Real hardware setup | Dedicated HIL rig | Nightly/Weekly | No fatal driver faults, timing within limits, recoverable transient errors | Keep hardware abstraction in scripts for board revision portability |
| L3 | Power/Sleep-Wake Reliability | Sleep entry/exit and restore | Safe prepare-for-sleep behavior, wake source handling, state restoration | HIL scenario test | PMIC, wake lines, persistent storage | HIL rig | Nightly | Correct wake reason handling, no boot loops, state restored as designed | Store expected wake/restore contracts in shared YAML |
| L3 | Endurance/Soak | Long-duration runtime | Memory stability, queue stability, flash behavior, watchdog continuity | 8h/24h soak | Full firmware runtime | HIL rig | Weekly | No crash/reset leaks, acceptable performance drift | Same soak profile reusable; thresholds versioned |
| L3 | Fault Injection | Controlled failures | Safe handling of I2C/flash/BLE faults and partial data errors | Fault-injection hooks + HIL | Injected bus errors/timeouts | HIL rig | Weekly | No undefined state, graceful degradation, recoverability verified | Keep fault catalog shared across versions |
| L4 | Release Gate Regression | Full critical path regression | End-to-end confidence before release | Aggregated run of critical suites | CI + HIL infra | CI + HIL | Release candidate | All critical suites pass, no Sev-1/Sev-2 open defects | Same release gate checklist reused for previous/next versions |

## Global Quality Gates
| Gate | Threshold |
|---|---|
| Integration suite pass rate | 100% |
| HIL critical suite pass rate | 100% |
| Reconnect reliability | >= 99% over defined cycle count |
| Runtime crash/assert count | 0 during qualified runs |
| Watchdog unexpected reset count | 0 during qualified runs |
| Data loss metric | Must remain within product-defined threshold |

## Execution Policy
| Trigger | Suites |
|---|---|
| Every PR | L1 suites |
| Nightly | L1 + selected L2 + selected L3 |
| Weekly | Full L2/L3 including soak and fault injection |
| Release | L4 full regression gate |

## Notes
- Unit-only algorithm tests are intentionally excluded per current decision.
- Keep all contracts (message mapping, state transitions, thresholds) in versioned test data files to maximize portability.
- Prefer simulator for fast feedback and HIL for truth validation.
