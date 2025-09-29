## Example run (200 agents, batch size 20):
PYTHONUNBUFFERED=1 python new_try/run_batches.py \
  --map smaller_batches/32x32_allEntry.map \
  --scen smaller_batches/all_tasks.scen \
  --agents 200 \
  --batch-size 20 \
  --init-random --seed 320 \
  --bridge-exe ./build/lacam/lacam_bridge \
  --csv results.csv \
  --run-id N200_bs20_near \
  --verbose 2 \
  --prefer-near

## smaller_batches/run_batches.py
What it does:
1. Runs batch rolling + single-phase replanning with LaCAM.
2. Each wave: call lacam_bridge once to get full paths, step through them.
3. When batch_size tasks finish, immediately assign new tasks and start the next wave.
4. Agents move in two phases: to_start → to_goal; reaching goal frees the agent for reassignment.
5. Assignment strategy = endpoint uniqueness + hotspot priority + tile balancing + (optional) nearest dispatch.

## If LaCAM gets stuck (no solution / timeout)
1. Increase hotspot quota → raise --hot-min-per-wave (e.g., 20 → 50).
Effect: more hotspot tasks are included together in the same wave → duplicates spread across different batches → fewer total waves.\
2. Decrease tile capacity (or increase number of tiles) → lower --tile-cap (e.g., 80 → 40) or raise --tiles (e.g., 4 → 6).\
Effect: limits how many tasks per region, encouraging spatial dispersion → reduces congestion at shared endpoints.
3. Change random seed → set a different --seed (e.g., 0 → 42).\
Effect: reshuffles initial positions and assignment order → may escape deadlock situations.

