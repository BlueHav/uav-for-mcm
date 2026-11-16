"""Server-side provenance for the isolated command shaping override."""
import difflib
import hashlib
import json
from pathlib import Path

B = Path(__file__).parent.resolve()
baseline = Path(('__UAV_PROJECT__/code/Swarm-formation/Runtime/mission_node.py'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])))
override = B / 'mission_runtime_override.py'
old, new = baseline.read_text(), override.read_text()
assert new.count('self.shape_desired(') == 3
(B / 'runtime-shaping.patch').write_text(''.join(difflib.unified_diff(
    old.splitlines(True), new.splitlines(True),
    fromfile=str(baseline), tofile=str(override))))
manifest = dict(
    baseline_path=str(baseline),
    baseline_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),
    override_source_path=str(override),
    override_sha256=hashlib.sha256(override.read_bytes()).hexdigest(),
    hook_sites=3,
    scope='Default-noop shaping before original smooth_velocity; control and final_sweep; production file unchanged.')
(B / 'runtime-shaping-manifest.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps(manifest))
