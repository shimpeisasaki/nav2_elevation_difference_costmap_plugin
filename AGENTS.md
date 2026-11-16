# Repository maintenance

Preserve the sensor-independent TerrainModel and the published ElevationLayer plugin. Do not confuse this current plugin with retired terrain_nav2. Baseline master 9de12ec4b85ad2d847ad0b9fbfa6cae14e6533b7; adopted model 262dc4f8112253c8e2d288f75205676e418a1f12. README.md and docs/ROBUST_ELEVATION_INTEGRATION_JA.md own the model contract.

## Changes and evidence

Update the authoritative feature/interface/launch documentation in the same change. Link to the owner rather than copying current instructions; preserve Logbooks and historical evaluation conditions. Record repository SHA, dirty diff, environment, input identity, command, counts, exit status, failures/skips and unverified conditions. Distinguish offline checks, simulation and hardware evidence.

Start with related offline tests after reading their fixtures. Use ROS 2 Humble consistently and explicitly select actual source paths and fresh build/install outputs. Keep parallel workers at most two. Never start hardware nodes, enable motion, send robot commands, replay bags or launch simulation without explicit authorization. Preserve safety latches and require manual permission/rearm.

Inspect metadata, filenames and summaries before any large recording/model/log. Narrow searches to relevant source and exclude generated output/data/cache. Store full check logs in generated log/ paths, report concise failures and preserve the original exit code. Do not install dependencies, update upstream, push, rewrite history or change main/master as part of this implementation.
