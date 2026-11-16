# Validation 2026-11-16

Executed on the SSH Linux server from this exported directory:

- Python production sources compile successfully.
- setup.sh checks Unity part SHA-256, extracts the player, and passes dependency/mode checks.
- Unity actually emits TRACKER_FIELD_READY drones=10.
- ROS control remains running for 35 seconds; /drone_0/odom publishes an actual message.
- No takeoff was dispatched. This is startup verification, not full mission/landing acceptance.

Full mission can be requested with bash bash/run_mode.sh ten RUN_DIRECTORY.
Server system dependencies remain required as documented in README.md.

Mode regression:
```text
....
----------------------------------------------------------------------
Ran 4 tests in 0.000s

OK
```
