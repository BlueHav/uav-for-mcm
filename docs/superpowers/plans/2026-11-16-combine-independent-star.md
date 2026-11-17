# Independent combine flight and star landing

Goal: make swarm-uav-combine independent of sibling projects, explain actual local path failures, preserve the upstream five-member final star through touchdown, then test hierarchical distance control.

User requirements: server execution only; phase-six maximum horizontal speed 6 m/s; use Swarm-Formation's existing final template and real leader IDs; vertical landing starts and ends in star; only the predecessor slows for a stretched follower, recovering below 25 m after a 50 m trigger.

Implementation proceeds inline in this chat. Source base: 216c21abf1590bbe56b3210402aa4bb86c713478. Preserve original server projects and baseline commits.

- [x] Test isolated env/path resolution, then replace external project env sourcing with checkout-relative paths and build controller/TCP dependencies from included sources. Extract the branch's own Unity archive, with SHA checks.
- [x] Test final slots against literal upstream template (leader first; two wing rows), then use consistent configured slots for descent boundary, independent endpoint, landing gate, and ground validation. Keep phase-six limit at 6 m/s.
- [x] Reproduce actual phase-six failure on server. Save odometry, local map rejections, peer stopping conflicts, planning state and Unity collision events. Fix identified causes with regression cases; retain stopping and sensor checks.
- [x] Verify independent terminal flight and touchdown before introducing distance control.
- [x] Test XY tree edges, 50/25 hysteresis, two-child aggregation, isolated groups, stale observations and safety priorities. Add per-predecessor speed caps to independent local planning, with decision telemetry and no forced follower acceleration.
- [ ] Repeat actual flight with distance control; force a lagging follower to exercise slowdown/recovery. Review changes, publish only verified results and explicit remaining limitations.

Geometry: formation_core.shape('final') is the existing five-member upstream template. Actual local leader index is 2, global leaders 2 and 7. Role order is leader, left second, right second, left third, right third; edges (2,0), (2,1), (0,3), (1,4) within each group. XY thresholds are configurable and default to 50/25 m; baseline link lengths are logged from the template.

Verification: all imports, unit tests, native builds and simulations run via SSH. Test deployment is /root/uav-combine-independent, separate from both original projects. System ROS/CUDA/Python installations remain normal system dependencies, but no sibling project's Unity, Runtime, env.sh or workspace may be required. Native test fixtures must be identified as focused stage tests; startup evidence is never reported as full flight acceptance.

## Latest user steering: speed-only floor and graph/order regulation

Supersedes the original permission to wait at zero. Distance/formation soft caps must be 2..6 m/s; preserve independent YOPO obstacle planning and independent safety/landing stops. Do not add loiter, lateral holding paths or heading coordination. Reuse upstream normalized-Laplacian XY coefficient and diagnostic member-repair attribution; supplement with metric spacing and immediate measured-course role ordering. Soft limits do not exempt positive-speed flights from the progress watchdog.

- [x] Draft coefficient/attribution governor, minimum-speed radial integration, instantaneous actual-course hierarchy and diagnostic/command separation.
- [x] Replace test-only zero pause with 2 m/s slow injection; extend audits to consumer caps, physical slowdown and eventual order recovery.
- [ ] Run all new and changed tests through SSH (currently TCP timeout).
- [ ] Rebuild Unity overlay for relative MapMarkers fallback, verify source/asset parity, fresh install.
- [ ] Execute formal-cross phase-six slow-injection flight and audit actual distance, order, coefficients and star landing.
- [ ] Complete review, metadata and authorized GitHub fast-forward publication after verified results.

Historical independent flight and old zero-policy results remain historical only; latest policy is unverified. An active old-policy harness must be confirmed exited before deployed Runtime changes.

## Actual final verification, known unmet distance acceptance

83 server regressions PASS in canonical deployment and independent install-check. Fresh setup uses own ROS/controller/TCP/dynamics, deterministic Unity overlay1172... matches29sources and realflight source manifest exactly. Latest CROSS02 completedall10 NN/owncrops/0swarm/full1.2km/starlanding/0collision/0bounds. Strictaudit FAIL: peak116.0677m + injection-parentresponse notobserved inlagwindow. Everyconsumerpolicycap2..6,0floorviolations; localobstacle1.5kept peruser latest clarification. Actualovertakingedgeshadconsumer2+physicalslowandultimatelyrecovered,butnotguaranteednoovertakethroughout.

CROSS01priorityearlyreturnbug removedandrealJSONregressionRED/GREEN. CROSS02remainingdistancefailure fromfront2/rear1.5oractualsafetystop, min2+speed-only cannot enforcehard50 inthatcase. Do not markstrict50 fulfilled or deletefailingaudits. Latestdocs explainallunmetitems; tests/snapshots/sourcechecks preserved. Publicationauthorized byoriginalrequest; baselinehistorypreserved, noforcepush. Furtherdistanceimprovement remainsopen underuserfixed speedconstraints.
