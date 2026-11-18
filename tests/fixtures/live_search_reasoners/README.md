These two immutable `exploration_graph.json` recordings were copied byte for byte
from `HEAD` (`93ad5af`) under `outputs/live_runs/` on 2026-10-15. The first contains
a replayable live scene; the second deliberately contains no scene graph.

`SearchSessionArchive` prunes old WebUI sessions under `outputs/live_runs/`.
Replay tests use these copies so normal archive retention cannot delete their inputs.
The original working-tree recordings and their existing deletions were left intact.
