# Historical margin-overlap observation

`historical-margin-overlap.npz` is the original `swarm-uav-combine/code/Runtime/run-4/drone_3_failure_input.npz`, preserved without modifying its observed occupancy, ground, position, velocity, goal or depth.

SHA-256: `6459620a4bb9402e11f8c59ca0b276420a4ab903104b9d8ea3ee6956af8899a2`.

It contains a stationary aircraft at approximately (1495.1602, -13.3246, 7.0142). Its observed 0.30 m body sphere is free, while its 0.35 m extra margin overlaps a voxel. A measured, monotonically outward retreat exists. This fixture tests preservation of that option during an active scan; it does not provide flight-path oracle data to a running aircraft. Peer constraints remain independently enforced by production safety checks.

Load or test only on the SSH server. Local Runtime/native imports are prohibited.

## Historical formation-order / distance conflict

`order-priority-distance-conflict.json` is a measured input from `phase6-min2-formation-cross-01`. At a fresh 50.0127 m adjacency, local leader 2 (global 7) retained a 6 m/s outward command because order-recovery priority skipped the other wing's active distance edge. This fixture verifies all incident distance edges still apply while inward catch-up remains available. It is diagnostic input only, never flight-path oracle data.

SHA-256: `bbf105d22bcd7f52a19b0ee534ac991df8cd17b7c488f86c5c7d32bd488aa5e9`.

## Full-mission relay speed boundary

`full-takeover-speed-sequence.json` contains 55 measured group-0 position/velocity snapshots preceding the failed takeover in `full-pipeline-01`. It intentionally excludes the old frozen mission acceleration as actual relay-command evidence. The regression checks that this still-fast sequence does not announce a cut under the lower approach-speed gate. It never supplies oracle flight paths to a running aircraft.

SHA-256: `bdfd6c215cd931c0a8cd3f3fe61aaebccc7e5588650bc7dca5a177fdd5c6e4ff`.
