# Smooth terminal star implementation plan

> Execute inline with superpowers:executing-plans; user approved design and implementation on 2026-11-17. No repeated approval gate.

**Goal:** Reduce phase-six speed cycling and complete a coordinated star approach without YOPO after endpoint takeover.
**Architecture:** Keep the existing official mission and controller. Add bounded soft-speed recovery and candidate admission consistent with final stopping checks. A group endpoint handover uses fresh observed certificates and continuous, synchronized references before vertical landing.
**Tech Stack:** Existing ROS Python/NumPy and Unity simulation, SSH-only execution.
**Spec:** ../specs/2026-11-17-smooth-terminal-star-design.md

## Global constraints

Preserve immutable combine-v1-success at 8158346. Route cap 6 m/s, formation policy floor 2 m/s, local avoidance 1.5 m/s. Endpoint convergence and landing may stop. No local Runtime/native imports or tests. Original server projects untouched. Use existing isolated fix/combine-independent-star checkout; server canonical project only for authorized tests.

## Review focus

- Fresh proposal/certificate ownership must prevent stale or partial group takeover.
- Endpoint polynomial must preserve P/V/A and stay within speed/clearance limits throughout, not only at its endpoints.
- Loss of heartbeat, tracking or free corridor after takeover must brake and report failure without silently returning to YOPO.
- Speed smoothing must not override urgent smaller safety limits or a malformed input.
- Metrics must distinguish intended endpoint deceleration from repeated route stopping and require real all-ten touchdown.

## Tasks

### 1. Continuous route speed admission

Files: Runtime/terminal_speed.py (new), mission_node.py, tests/test_terminal_speed.py.
Interface: bounded_speed(previous,target,dt,recovery_acc=.6) and select_continuous_velocity(p,v,a,desired,clear) return a no-faster admissible command. Cache bounded recovery state in Mission, reset per run. Full stop map and peer checks retain authority.
- [x] Write tests for jittering cap recovery, finite bounds, immediate smaller limit and safe reduced candidate selection; run RED via SSH.
- [x] Implement shared bounded recovery and evaluate reduced candidates before final stop latching; run GREEN and existing regressions.
- [ ] Replay recorded stop inputs and record actual reason/limits; do not weaken the stop model to obtain a pass.

### 2. Endpoint formation takeover

Files: Runtime/terminal_star_approach.py (new), point_mission_engine.py, point_mission_node.py, ten_follower.py, mission_node.py; tests/test_terminal_star_approach.py.
Interface: StarApproach builds a five-member synchronized quintic P/V/A plan from fresh references to model.landing_slots; proposal carries token/start/duration/coefficients. Each Mission validates its observed trajectory/landing column and reports token and stamp. Group commits only all-five fresh matching certificates; each controller executes its own reference continuously and reports mode.
- [x] Write RED cases: all five within 30 m, trajectory max3, boundary continuity, crossing rejection, stale/missing certificate rejection and no return to YOPO after committed ownership.
- [x] Implement group proposal/commit, own observation admission, endpoint reference execution, safety fault and star-settled landing integration; remove the 15m/.6 step for admitted formation approaches.
- [ ] Validate no network inference after takeover and same star coordinates for goals and landing.

### 3. Evidence and full flight

Files: tests/audit_smooth_terminal.py (new), full audit, validation docs.
- [ ] Server regression suite and phase-six simulation; inspect all ten results and fix failures with regressions.
- [ ] Run unmodified official mission from stage one with source manifest and owned cleanup.
- [ ] Compare route speed variation, sustained speed cycles/stop episodes, latch fractions and duration with full-pipeline-02; require all-ten arrival/star touchdown and zero collision/bounds.
- [ ] Fresh final review, refresh export hashes, publish tested changes and retain immutable V1 tag.

## Execution ledger

Start: baseline HEAD 78158ec (documentation atop tested8158346). User approved spec. Rule: native inline execution reuses existing isolated checkout and .combine-work transport ledger; no new worktree or local project execution. Full plan checkpoints recorded here and .combine-work/STATE.md.

Ruling: introduce an explicit near-endpoint PREPARE settling phase before certificates; a moving proposal became stale while acknowledgments arrived. This is endpoint deceleration, excluded from route smoothness comparison. Commit requires actual and reference boundary agreement. The nominal tracking target is capped at 2.5 m/s and 0.6 m/s² to reserve actuator authority; feedback command cap remains 3 m/s. Map admission includes a 0.5 m tracking tube, paired trajectories keep 3.8 m clearance, and tracking error over 0.5 m faults.

Pilot `phase6-smooth-star-01` was intentionally stopped for the above review fix. Pilot `phase6-smooth-star-02` stopped at a real ground-age contract mismatch: inner check 0.6 s versus existing controller/supervisor 0.7 s. A RED/GREEN regression now pins the shared 0.7 s limit while rejecting expired/future inputs. Ground is refreshed between complete candidate checks. Both pilots are not success evidence.

111 server regressions passed in deployment and independent installation. Full production run `full-smooth-star-01` started with all latest runtime inputs frozen; it supplies the required end-to-end and phase-six validation after the fixes. Whole-branch static review found no remaining P1/P2 before the ground-age consistency fix; final flight review remains outstanding.
