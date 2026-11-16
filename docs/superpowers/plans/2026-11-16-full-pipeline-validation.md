# Full pipeline validation plan

> Execute inline using the executing-plans workflow. All Runtime imports, tests, native programs and simulation run through SSH only.

**Goal:** Run the production ten-aircraft mission from stage one and verify all aircraft reach their own endpoints, settle in the upstream leading star, land and disarm.

**Architecture:** Use the existing `bash/run_mode.sh ten` entry without spawn overrides, skipped phases or injected slowdowns. A server-only external supervisor records source hashes and owns process cleanup. Add a telemetry auditor that rejects phase-six-only evidence as proof of the complete mission.

**Tech stack:** ROS Noetic, Unity Player, existing Swarm/YOPO Runtime, Python telemetry auditing.

**Constraints:** Keep formation speed policy at least 2 m/s; independent local avoidance may request 1.5 m/s. Preserve safety and landing control. Do not introduce coordinated headings or loiter paths. The current request concerns complete arrival and star landing; report distance peaks without making 50 m a new completion requirement. Preserve original server projects and baseline history.

- [x] Verify published flight inputs, all ten configured spawns, free flight ports and server resources.
- [x] Start unmodified production entry from stage one in a new run directory with source fingerprints.
- [x] Add server-tested full-scope auditing: ordered takeoff/assembly/climb/cruise/descent/YOPO/landing coverage, all-ten cruise evidence, real terminal inference and star touchdown.
- [x] Follow the real run through completion. Investigate any failure before changing control code; add regressions and restart the full mission after a necessary fix.
- [x] Audit final measurements, preserve reports and update validation scope.
- [x] Prepare the reviewed code and actual reports for publication to the existing combine branch; final static review found no actionable P1/P2 issues.

The existing phase-six result remains separate evidence and is never used as proof that stages one through five passed.

**Measured outcome (2026-11-17):** First full attempt safely aborted at the relay takeover clearance check. After reducing the CLOUD approach speed and observing fresh actual relay P/V/A, the second continuous production run `full-pipeline-02` passed from stage one: all ten endpoints, star touchdown/contact/disarm, 1304.899 s, zero collision or bounds events. The full auditor passed with no errors or source changes. All 94 regressions passed on the server in both the deployed and independent-install directories. See `docs/validation/full-pipeline-audit.json` and `VALIDATION.md` for measurements and limits.
