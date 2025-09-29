#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
run_batches.py — Batch rolling + single-phase replanning (all N, one LaCAM call per wave)

Key points:
- Input to bridge uses "format: xy", and starts/goals are passed as "x,y x,y ..." (space-separated, no brackets).
- The bridge output is "walkable cell IDs" (compact indices that skip obstacles); we map them back to (x, y) with id2xy.
- paths_xy always "include the start": path[0] == start; when advancing step s use path[s], clamped via min().
- Clearer logs: wave start / plan horizon / per-step events / trigger reasons / end-of-wave summary.
"""

import argparse
import csv
import json
import re
import subprocess
import time
from collections import Counter, deque, defaultdict
from pathlib import Path

# ================= Custom exceptions =================
class BridgeTimeout(Exception):
    pass

# ================= Small logging helpers =================
def print_wave_header(wave_id, active_cnt, free_cnt, pending_left):
    print(f"\n=== WAVE {wave_id} START ===  active={active_cnt}  free={free_cnt}  pending_left={pending_left}")

def print_bridge_plan(T_full, active_cnt):
    print(f"[plan] T_full={T_full}  active={active_cnt}  (total steps)")

def print_step_debug(step, newly_to_start, newly_done, fin_since_assign, free_cnt, verbose=0):
    if verbose >= 2:
        print(
            f"[STEP] s={step:>3}  +to_start={newly_to_start:<2}  +done={newly_done:<2}  "
            f"fin_since_assign={fin_since_assign:<3}  free={free_cnt}"
        )

def print_trigger(wave_id, q_want, q, got, scanned, skipped, pending_left):
    print(
        f"[wave {wave_id}] TRIGGER  reason=batch  want={q_want}  q={q}  got={got}  "
        f"scanned={scanned}  skipped~={skipped}  pending_left={pending_left}"
    )

def print_wave_summary(
    wave_id, steps_advanced, T_full, finished_this_wave, arrive_start_this_wave,
    ts, tg, total_finished, makespan
):
    print(
        f"[wave {wave_id}] SUMMARY  steps={steps_advanced}/{T_full}  "
        f"done+={finished_this_wave}  to_start+={arrive_start_this_wave}  "
        f"TS={ts}  TG={tg}  total_done={total_finished}  makespan={makespan}"
    )

# ================= Map / task parsing =================
def load_map_grid(map_path):
    with open(map_path, "r", encoding="utf-8") as f:
        lines = [ln.rstrip("\n") for ln in f]
    # .map compatibility: extract only the grid section
    grid = [ln for ln in lines if ln and any(c in ln for c in ".@TGWS")]
    if not grid:
        raise ValueError("Failed to parse .map grid: " + map_path)
    H = len(grid)
    W = max(len(r) for r in grid)
    grid = [row.ljust(W) for row in grid]
    return grid, W, H

def is_free(grid, x, y):
    if y < 0 or y >= len(grid): return False
    if x < 0 or x >= len(grid[0]): return False
    return grid[y][x] == '.'

def parse_scen_tasks(path, max_tasks=None):
    """Parse MovingAI .scen -> [{'sx','sy','gx','gy'}, ...]"""
    tasks = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#") or s.lower().startswith("version"):
                continue
            parts = re.split(r"[\s,]+", s)
            map_col = None
            for i, tok in enumerate(parts):
                if tok.endswith(".map"):
                    map_col = i
                    break
            if map_col is None or len(parts) <= map_col + 6:
                continue
            try:
                sx = int(parts[map_col + 3]); sy = int(parts[map_col + 4])
                gx = int(parts[map_col + 5]); gy = int(parts[map_col + 6])
            except ValueError:
                continue
            tasks.append({"sx": sx, "sy": sy, "gx": gx, "gy": gy})
            if max_tasks is not None and len(tasks) >= max_tasks:
                break
    if not tasks:
        raise ValueError(f"No valid tasks could be parsed from {path}")
    return tasks

# ================= Basic utilities (bridge I/O and ID→XY decode) =================
def fmt_xy_list(pairs):
    """xy input format required by the bridge: 'x,y x,y ...' (space-separated, no brackets)"""
    return " ".join(f"{x},{y}" for (x, y) in pairs)

def _read_grid_tail(map_path: str):
    """Read the grid portion after the 'map' marker in a .map file (or a pure grid file); return list of lines."""
    lines = []
    with open(map_path, 'r', encoding="utf-8") as f:
        raw = [ln.rstrip('\r\n') for ln in f]
    if any(ln.strip().lower() == 'map' for ln in raw):
        started = False
        for ln in raw:
            if started:
                lines.append(ln)
            elif ln.strip().lower() == 'map':
                started = True
    else:
        lines = [ln for ln in raw if ln.strip()]
    if not lines:
        raise RuntimeError(f"empty map: {map_path}")
    return lines

def build_id2xy_from_map(map_path: str, obstacle_chars=('@','T')):
    """
    Assign compact IDs to "walkable cells" following the bridge/graph convention (scan rows, skip obstacles):
    id2xy[id] = (x, y)
    """
    grid = _read_grid_tail(map_path)
    id2xy = []
    for y, row in enumerate(grid):
        for x, ch in enumerate(row):
            if ch in obstacle_chars:
                continue
            id2xy.append((x, y))
    return id2xy

def run_lacam_bridge_with_paths(map_path, starts_xy, goals_xy, exe="lacam_bridge", timeout=None):
    """
    Call the C++ bridge and return (T_moves, paths_xy)
    - Inputs: starts/goals in the xy text format required by the bridge ("x,y x,y ...")
    - Output: bridge always returns "walkable cell IDs"; this function decodes IDs to (x,y)
    - Guarantee: paths_xy[k][0] == starts_xy[k] (includes start)
    - T_moves = number of advanceable steps (not counting the t=0 start frame)
    """
    N = len(starts_xy)
    stdin = [
        f"map: {map_path}",
        f"N: {N}",
        "format: xy",
        "starts: " + fmt_xy_list(starts_xy),
        "goals:  " + fmt_xy_list(goals_xy),
    ]
    payload = "\n".join(stdin) + "\n"

    p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, text=True)
    try:
        out, err = p.communicate(payload, timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            p.kill()
        finally:
            raise BridgeTimeout(f"bridge timeout (>{timeout}s)")
    if p.returncode != 0:
        raise RuntimeError(f"lacam_bridge failed (rc={p.returncode}). stderr:\n{err}\n---- payload ----\n{payload}")

    try:
        data = json.loads(out)
    except Exception as e:
        raise RuntimeError(f"Failed to parse bridge JSON. Raw output:\n{out}") from e

    if "solution" not in data:
        raise RuntimeError("bridge JSON missing field: solution")
    sol = data["solution"]  # [frames][N] (walkable cell IDs)

    # Empty solution: stay still (include start)
    if not sol:
        return 0, [[starts_xy[k]] for k in range(N)]
    if any(len(row) != N for row in sol):
        raise RuntimeError("bridge solution columns per frame != N")

    # ID -> (x,y)
    id2xy = build_id2xy_from_map(map_path, obstacle_chars=('@','T'))
    by_time_xy = [[id2xy[int(u)] for u in row] for row in sol]  # [frames][N] -> (x,y)

    # Ensure "includes start"
    includes_start = all(by_time_xy[0][k] == starts_xy[k] for k in range(N))
    paths_xy = [[] for _ in range(N)]
    frames = len(sol)
    if includes_start:
        for t in range(frames):
            for k in range(N):
                paths_xy[k].append(by_time_xy[t][k])
        T_moves = frames - 1
    else:
        for k in range(N):
            paths_xy[k].append(starts_xy[k])  # t=0 start
        for t in range(frames):
            for k in range(N):
                paths_xy[k].append(by_time_xy[t][k])
        T_moves = frames

    # Self-checks
    assert all(paths_xy[k][0] == starts_xy[k] for k in range(N)), "paths_xy does not include start"
    assert all(len(p) == T_moves + 1 for p in paths_xy), "len(path) should be T_moves+1 (including start)"

    return T_moves, paths_xy

# ================= Runtime structures =================
class AgentState:
    __slots__ = ("id", "pos", "has_task", "task_idx", "subphase")
    def __init__(self, i, pos):
        self.id = i
        self.pos = tuple(pos)
        self.has_task = False
        self.task_idx = None
        self.subphase = "to_start"  # 'to_start' | 'to_goal'

# ================= Main flow =================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--scen", required=True)
    ap.add_argument("--agents", type=int, required=True)
    ap.add_argument("--batch-size", type=int, default=50)
    ap.add_argument("--bridge-exe", default="lacam_bridge")
    ap.add_argument("--bridge-timeout", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--csv", default="results.csv")
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--max-tasks", type=int, default=None)
    # Initial positions
    ap.add_argument("--init-xy", default=None)
    ap.add_argument("--init-json", default=None)
    ap.add_argument("--init-random", action="store_true")
    # Spatial balancing
    ap.add_argument("--tiles", type=int, default=4)
    ap.add_argument("--tile-cap", type=int, default=50)
    ap.add_argument("--relax-threshold", type=int, default=20)
    # Nearest dispatch
    ap.add_argument("--prefer-near", action="store_true")
    # Hotspot control
    ap.add_argument("--hot-threshold", type=int, default=3)
    ap.add_argument("--hot-min-per-wave", type=int, default=50)
    ap.add_argument("--tail-pack", type=int, default=20)
    # Early break near the end (optional)
    ap.add_argument(
        "--early-break-slack",
        type=int,
        default=2,
        help="If remaining <= slack steps and the current wave finished tasks, trigger an early replenish once."
    )
    # Logging
    ap.add_argument("--verbose", type=int, default=1, choices=[0, 1, 2])
    args = ap.parse_args()

    # ---- Map & tasks ----
    grid, W, H = load_map_grid(args.map)
    tasks = parse_scen_tasks(args.scen, max_tasks=args.max_tasks)
    T_total = len(tasks)
    assert args.agents <= T_total, "agents must be <= number of tasks"

    # ---- Initial positions ----
    if args.init_xy:
        inits = []
        with open(args.init_xy, "r", encoding="utf-8") as f:
            for line in f:
                s = line.strip()
                if not s:
                    continue
                if "," in s:
                    xs, ys = s.split(",", 1)
                else:
                    xs, ys = s.split()
                inits.append((int(xs), int(ys)))
                if len(inits) >= args.agents:
                    break
        if len(inits) < args.agents:
            raise ValueError(f"init-xy has {len(inits)} lines, fewer than agents={args.agents}")
        init_pos = inits
    elif args.init_json:
        with open(args.init_json, "r", encoding="utf-8") as f:
            obj = json.load(f)
        arr = obj.get("inits", [])
        if len(arr) < args.agents:
            raise ValueError(f"init-json has {len(arr)} positions, fewer than agents={args.agents}")
        init_pos = [(int(d["x"]), int(d["y"])) for d in arr[:args.agents]]
    elif args.init_random:
        free_cells = [(x, y) for y in range(H) for x in range(W) if is_free(grid, x, y)]
        import random
        rng = random.Random(args.seed)
        init_pos = rng.sample(free_cells, args.agents)
        if args.verbose >= 2:
            print(f"[init] initial positions (first {min(100, len(init_pos))}): {init_pos[:100]}")
    else:
        init_pos = [(tasks[i]["sx"], tasks[i]["sy"]) for i in range(args.agents)]

    agents = [AgentState(i, init_pos[i]) for i in range(args.agents)]
    free_ids = list(range(args.agents))  # all idle at start

    # ---- Pending queue & assignment flags ----
    pending = deque(range(T_total))
    assigned_mask = [False] * T_total

    # ---- Tile utils & endpoint-uniqueness check ----
    tile_w = max(1, W // args.tiles)
    tile_h = max(1, H // args.tiles)

    def tile_of_xy(x, y):
        return (min(x // tile_w, args.tiles - 1), min(y // tile_h, args.tiles - 1))

    def active_endpoints():
        """Collect (start, goal) only for active tasks; idle current positions are not endpoints."""
        occ = set()
        for a in agents:
            if not a.has_task:
                continue
            t = tasks[a.task_idx]
            s = (t["sx"], t["sy"])
            g = (t["gx"], t["gy"])
            if s in occ or g in occ:
                raise RuntimeError("Endpoint uniqueness violated: duplicate/cross-duplicate within a single wave.")
            occ.add(s)
            occ.add(g)
        return occ

    # ---- Dispatcher: hotspots + nearest + spatial balancing ----
    def assign_from_pending_spatial_near(quota, relax_tiles=False):
        if quota <= 0 or not free_ids:
            return 0, 0, 0

        pending_left = len([i for i in pending if not assigned_mask[i]])
        tail_boost = (pending_left <= args.tail_pack)

        occ_now = active_endpoints()
        used_tile_S = defaultdict(int)
        used_tile_G = defaultdict(int)

        # Hotspot degree
        snap = list(pending)
        deg = Counter()
        for idx in snap:
            if assigned_mask[idx]:
                continue
            t = tasks[idx]
            deg[(t["sx"], t["sy"])] += 1
            deg[(t["gx"], t["gy"])] += 1

        scored = []
        for idx in snap:
            if assigned_mask[idx]:
                continue
            t = tasks[idx]
            score = deg[(t["sx"], t["sy"])] + deg[(t["gx"], t["gy"])]
            scored.append((score, idx))
        scored.sort(key=lambda x: x[0], reverse=True)

        def can_use(idx, occ_set, usedS, usedG):
            t = tasks[idx]
            s = (t["sx"], t["sy"])
            g = (t["gx"], t["gy"])
            if s in occ_set or g in occ_set:
                return False
            if not (relax_tiles or tail_boost):
                ts = tile_of_xy(*s)
                tg = tile_of_xy(*g)
                if usedS[ts] >= args.tile_cap or usedG[tg] >= args.tile_cap:
                    return False
            return True

        candidates_all, candidates_hot = [], []
        scanned = 0
        for score, idx in scored:
            if scanned >= len(snap):
                break
            scanned += 1
            if assigned_mask[idx]:
                continue
            if not can_use(idx, occ_now, used_tile_S, used_tile_G):
                continue
            candidates_all.append(idx)
            if score >= args.hot_threshold:
                candidates_hot.append(idx)

        def greedy_assign(indices, take_quota, occ_base, usedS_base, usedG_base):
            picked = set()
            if take_quota <= 0 or not indices or not free_ids:
                return picked

            if args.prefer_near:
                free_objs = [agents[aid] for aid in free_ids]
                cache = {idx: (tasks[idx]["sx"], tasks[idx]["sy"]) for idx in indices}

                def manhattan(a, s):
                    return abs(a.pos[0] - s[0]) + abs(a.pos[1] - s[1])

                free_rank = []
                for a in free_objs:
                    best = None
                    for idx in indices:
                        if idx in picked:
                            continue
                        d = manhattan(a, cache[idx])
                        if best is None or d < best:
                            best = d
                    free_rank.append((best if best is not None else 10**9, a.id))
                free_rank.sort()

                occ_local = set(occ_base)
                usedS, usedG = dict(usedS_base), dict(usedG_base)

                for _, aid in free_rank:
                    if len(picked) >= take_quota or not free_ids:
                        break
                    if aid not in free_ids:
                        continue
                    a = agents[aid]
                    best_idx, best_d = None, None
                    for idx in indices:
                        if idx in picked:
                            continue
                        t = tasks[idx]
                        s = (t["sx"], t["sy"])
                        g = (t["gx"], t["gy"])
                        if s in occ_local or g in occ_local:
                            continue
                        if not (relax_tiles or tail_boost):
                            ts = tile_of_xy(*s)
                            tg = tile_of_xy(*g)
                            if usedS.get(ts, 0) >= args.tile_cap or usedG.get(tg, 0) >= args.tile_cap:
                                continue
                        d = abs(a.pos[0] - s[0]) + abs(a.pos[1] - s[1])
                        if best_d is None or d < best_d:
                            best_d, best_idx = d, idx
                    if best_idx is None:
                        continue
                    t = tasks[best_idx]
                    s = (t["sx"], t["sy"])
                    g = (t["gx"], t["gy"])
                    if not (relax_tiles or tail_boost):
                        ts = tile_of_xy(*s)
                        tg = tile_of_xy(*g)
                    free_ids.remove(aid)
                    a.task_idx = best_idx
                    a.has_task = True
                    a.subphase = "to_start"
                    occ_local.add(s)
                    occ_local.add(g)
                    if not (relax_tiles or tail_boost):
                        usedS[ts] = usedS.get(ts, 0) + 1
                        usedG[tg] = usedG.get(tg, 0) + 1
                    assigned_mask[best_idx] = True
                    picked.add(best_idx)
                return picked
            else:
                occ_local = set(occ_base)
                usedS, usedG = dict(usedS_base), dict(usedG_base)
                for idx in indices:
                    if len(picked) >= take_quota or not free_ids:
                        break
                    t = tasks[idx]
                    s = (t["sx"], t["sy"])
                    g = (t["gx"], t["gy"])
                    if s in occ_local or g in occ_local:
                        continue
                    if not (relax_tiles or tail_boost):
                        ts = tile_of_xy(*s)
                        tg = tile_of_xy(*g)
                        if usedS.get(ts, 0) >= args.tile_cap or usedG.get(tg, 0) >= args.tile_cap:
                            continue
                    aid = free_ids.pop()
                    a = agents[aid]
                    a.task_idx = idx
                    a.has_task = True
                    a.subphase = "to_start"
                    occ_local.add(s)
                    occ_local.add(g)
                    if not (relax_tiles or tail_boost):
                        usedS[ts] = usedS.get(ts, 0) + 1
                        usedG[tg] = usedG.get(tg, 0) + 1
                    assigned_mask[idx] = True
                    picked.add(idx)
                return picked

        # Prioritize hotspots (optional)
        hot_take = min(quota, max(0, args.hot_min_per_wave))
        hot_take = min(hot_take, len(candidates_hot))
        picked_hot = greedy_assign(candidates_hot, hot_take, occ_now, used_tile_S, used_tile_G)

        occ_base = set(occ_now)
        for idx in picked_hot:
            t = tasks[idx]
            occ_base.add((t["sx"], t["sy"]))
            occ_base.add((t["gx"], t["gy"]))

        remain_quota = quota - len(picked_hot)
        picked_rest = set()
        if remain_quota > 0 and free_ids:
            rest = [idx for idx in candidates_all if idx not in picked_hot]
            picked_rest = greedy_assign(rest, remain_quota, occ_base, used_tile_S, used_tile_G)

        picked_set = set(picked_hot) | set(picked_rest)
        taken = len(picked_set)

        # Re-queue scanned-but-not-assigned to the front to retry first next time
        scanned_set = set(i for _, i in scored)
        new_q = deque()
        for _, idx in scored:
            if idx not in picked_set and not assigned_mask[idx]:
                new_q.append(idx)
        for idx in pending:
            if idx not in scanned_set and not assigned_mask[idx]:
                new_q.append(idx)
        pending.clear()
        pending.extend(new_q)

        skipped_count = len([1 for _, idx in scored if idx not in picked_set])
        return taken, len(scored), skipped_count

    # ---- Initial assignment (fill as much as possible)----
    want0 = args.agents
    got0, scanned0, skipped0 = assign_from_pending_spatial_near(want0, relax_tiles=True)
    if args.verbose >= 1:
        print(f"[init] assigned={got0}, free={len(free_ids)}, pending_left={len(pending)}, skipped~={skipped0}")

    # ---- Stats ----
    makespan = 0
    wall_t0 = time.time()
    finished_since_assign = 0
    total_finished = 0
    wave_id = 0

    # ============ Main loop: single-phase replanning (step through; break as soon as batch is reached) ============
    while True:
        # Wave header
        active_cnt = sum(a.has_task for a in agents)
        pending_left_now = len([i for i in pending if not assigned_mask[i]])
        if args.verbose >= 1:
            print_wave_header(wave_id, active_cnt, len(free_ids), pending_left_now)

        # If no active tasks: try assignment; else run planning
        if active_cnt == 0:
            remaining_unassigned = pending_left_now
            if remaining_unassigned > 0 and free_ids:
                q = min(len(free_ids), remaining_unassigned)
                got, scanned, skipped = assign_from_pending_spatial_near(
                    q, relax_tiles=(active_cnt < args.relax_threshold)
                )
                if args.verbose >= 1:
                    print(
                        f"[wave {wave_id}] ASSIGN(first)  want={q}  got={got}  scanned={scanned}  "
                        f"skipped~={skipped}  pending_left={remaining_unassigned - got}"
                    )
                finished_since_assign = 0
                if got == 0:
                    break
            else:
                break  # all done

        # --------- One LaCAM call: get full paths ---------
        _ = active_endpoints()

        starts = [a.pos for a in agents]
        goals = []
        for a in agents:
            if not a.has_task:
                goals.append(a.pos)  # idle: stay
            else:
                tsk = tasks[a.task_idx]
                goals.append((tsk["sx"], tsk["sy"]) if a.subphase == "to_start" else (tsk["gx"], tsk["gy"]))

        try:
            T_full, paths_xy = run_lacam_bridge_with_paths(
                args.map, starts, goals, exe=args.bridge_exe, timeout=args.bridge_timeout
            )
        except BridgeTimeout as e:
            print(f"[TIMEOUT] wave={wave_id} phase=single active={active_cnt} msg={e}")
            break

        if args.verbose >= 1:
            print_bridge_plan(T_full, active_cnt)

        # ---- Step forward; as soon as batch threshold is reached, break into next wave ----
        steps_advanced = 0
        finished_this_wave = 0      # tasks completed (goal reached) in this wave
        arrive_start_this_wave = 0  # #agents that reached their start in this wave (for logs)

        while steps_advanced < T_full:
            steps_advanced += 1

            # 1) Everyone moves one step (paths include start: take path[s], with clamp)
            for i, a in enumerate(agents):
                path = paths_xy[i]  # length = T_full + 1
                a.pos = path[min(steps_advanced, len(path) - 1)]

            # 2) Subphase switches & completion releases (count "newly completed / reached start" in this wave)
            newly_free = []
            newly_to_start = 0
            for a in agents:
                if not a.has_task:
                    continue
                tsk = tasks[a.task_idx]
                if a.subphase == "to_start" and a.pos == (tsk["sx"], tsk["sy"]):
                    a.subphase = "to_goal"  # reached start -> switch subphase only
                    newly_to_start += 1
                elif a.subphase == "to_goal" and a.pos == (tsk["gx"], tsk["gy"]):
                    a.has_task = False
                    a.task_idx = None
                    a.subphase = "to_start"
                    newly_free.append(a.id)

            if newly_to_start:
                arrive_start_this_wave += newly_to_start
            if newly_free:
                free_ids.extend(newly_free)
                finished_since_assign += len(newly_free)
                finished_this_wave += len(newly_free)
                total_finished += len(newly_free)

            # Per-step debug (only when verbose >= 2)
            # print_step_debug(steps_advanced, newly_to_start, len(newly_free),
            #                  finished_since_assign, len(free_ids), verbose=args.verbose)

            # 3) Trigger "immediate replenish + break"
            remaining_unassigned = len([i for i in pending if not assigned_mask[i]])
            if (finished_since_assign >= args.batch_size):
                q_want = finished_since_assign
                q = min(q_want, len(free_ids), remaining_unassigned)

                got = scanned = skipped = 0
                if q > 0:
                    got, scanned, skipped = assign_from_pending_spatial_near(
                        q, relax_tiles=(sum(a.has_task for a in agents) < args.relax_threshold)
                    )

                print_trigger(wave_id, q_want, q, got, scanned, skipped, remaining_unassigned - got)

                finished_since_assign = 0
                break  # break current execution and enter the next wave

        # 4) Accumulate steps advanced in this wave
        makespan += steps_advanced

        # 5) Per-wave summary
        if args.verbose >= 1:
            ts = sum(a.has_task and a.subphase == 'to_start' for a in agents)
            tg = sum(a.has_task and a.subphase == 'to_goal' for a in agents)
            print_wave_summary(
                wave_id, steps_advanced, T_full,
                finished_this_wave, arrive_start_this_wave,
                ts, tg, total_finished, makespan
            )
            if args.verbose >= 2:
                subphase_cnt = Counter(a.subphase for a in agents if a.has_task)
                print(f"[wave {wave_id}] subphase(after step)={dict(subphase_cnt)}")

        wave_id += 1

    # ================= Final outputs (CSV + JSON) =================
    wall_sec = time.time() - wall_t0
    csv_path = Path(args.csv)
    new_file = not csv_path.exists()
    with csv_path.open("a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        header = [
            "run_id", "seed", "map", "agents_total", "batch_size", "makespan", "wall_clock_s",
            "tiles", "tile_cap", "prefer_near", "hot_threshold", "hot_min_per_wave",
            "tail_pack", "relax_threshold", "early_break_slack", "timeout"
        ]
        if new_file:
            w.writerow(header)
        w.writerow([
            args.run_id or "", args.seed, args.map, args.agents, args.batch_size,
            makespan, f"{wall_sec:.3f}",
            args.tiles, args.tile_cap, int(args.prefer_near),
            args.hot_threshold, args.hot_min_per_wave, args.tail_pack, args.relax_threshold,
            args.early_break_slack,
            ""
        ])

    out = {
        "makespan": makespan,
        "wall_clock_s": wall_sec,
        "agents": args.agents,
        "batch_size": args.batch_size,
        "tiles": args.tiles,
        "tile_cap": args.tile_cap,
        "prefer_near": bool(args.prefer_near),
        "hot_threshold": args.hot_threshold,
        "hot_min_per_wave": args.hot_min_per_wave,
        "tail_pack": args.tail_pack,
        "relax_threshold": args.relax_threshold,
        "early_break_slack": args.early_break_slack,
        "timeout": None
    }
    print(json.dumps(out))


if __name__ == "__main__":
    main()
