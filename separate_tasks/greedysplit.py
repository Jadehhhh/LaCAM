#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
强注释版：Solver-friendly SCEN splitter for RHCR / LaCAM

目标：
  - 把 CSV 中的 (goal_1, goal_2) 任务切成若干批（每批“理想”为 500 点=250 任务）。
  - 保证“批内不重复端点”；允许“跨批重复端点”（现实中必然的）。
  - 让每批的起点/终点在地图上尽量空间均衡（按 tile 控制配额）。
  - 优先把“热点端点”（出现频次高）分散到不同批，避免集中。
  - 尾部两步处理：
      1) rebalance_tail：把倒数 K 批合并重装箱（仍零重复），尽量更满；
      2) absorb_tiny_tail_into_previous：把极小尾批（默认 ≤10 任务）塞回前面（允许超 250），仍保证批内不重复端点。

输出：
  - 多个 MovingAI SCEN 文件（version 1 + 9 列格式）。

运行：
  python3 scen_splitter_sf_explained.py
"""

import csv
import math
import random
from pathlib import Path
from collections import Counter, defaultdict
from typing import List, Dict, Tuple

# ===================== 参数区（按需修改） =====================
CSV_FILE      = "tasks_group800_blank_time_blank.csv"  # 输入 CSV（需包含列 goal_1, goal_2）
MAP_NAME      = "32x32_allEntry.map"                   # SCEN 第二列的地图文件名（与实际地图匹配）
WIDTH, HEIGHT = 32, 32                                 # 地图宽高（整数）
CAP_POINTS    = 800                                    # 每批“理想容量”（点数）→ 500 点 = 250 任务
GROUP_POINTS  = 10**9                                  # 切组大小（按点数）。设超大 = 全局一次性装箱
SEED          = 43                                    # 随机种子（影响组内打乱 / 轻微扰动）
TILES         = 4                                      # 地图划分成 TILES×TILES 个 tile（为空间均衡）
TILE_CAP      = 25                                    # 每批内：每个 tile 的起点/终点最多各用多少次
TAIL_LOOKBACK = 6                                    # 尾部重平衡回看批数（仅重装尾部，不动头部）
MIN_TAIL_SIZE = 20                                     # ≤ 此任务数的尾批视作“极小尾巴”→ 吸收到前面
RELAX_TILECAP_ON_ABSORB = True                         # 吸收尾巴时，必要时是否放宽 tile 配额限制
OUT_ROOT      = Path(".")                              # 输出根目录
HOT_TOPK      = 5                                      # 打印热点端点前 K 名
# ============================================================


# -------------------- I/O：读 CSV、写 SCEN --------------------
def read_tasks(csv_file: str) -> List[Dict]:
    """
    读取 CSV，过滤掉缺失 goal_1/goal_2 的行。
    每条记录 row 是 dict，后面需转 int 使用。
    """
    with open(csv_file, newline="", encoding="utf-8") as f:
        return [row for row in csv.DictReader(f) if row.get("goal_1") and row.get("goal_2")]

def write_scen(path: Path, rows: List[Dict], map_name: str, width: int, height: int):
    """
    写 MovingAI 的 SCEN 文件，9 列：
    id  map  w  h  sx  sy  gx  gy  len
    其中 sx=goal1%W, sy=goal1//W （把线性索引还原成网格坐标）
    len=直线欧氏距离（仅作为占位/估计，可被 solver 忽略或替换）
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as wf:
        wf.write("version 1\n")
        for local_id, row in enumerate(rows):
            s_loc = int(row["goal_1"])
            g_loc = int(row["goal_2"])
            sx, sy = s_loc % width, s_loc // width
            gx, gy = g_loc % width, g_loc // width
            length = math.hypot(gx - sx, gy - sy)  # 直线距离（可粗估）
            wf.write(
                f"{local_id}\t{map_name}\t{width}\t{height}\t"
                f"{sx}\t{sy}\t{gx}\t{gy}\t{length:.8f}\n"
            )


# -------------------- 统计/检查：热点、批内重复 --------------------
def print_hotspots(tasks: List[Dict], topk: int = 5):
    """
    打印端点出现频次（热点）。帮助判断冲突强度/是否需要放宽 tile_cap 等。
    """
    cnt = Counter()
    for r in tasks:
        g1, g2 = int(r["goal_1"]), int(r["goal_2"])
        cnt[g1] += 1
        cnt[g2] += 1
    if not cnt:
        print("⚠️ 没读到任何任务"); return
    max_point, max_count = cnt.most_common(1)[0]
    tops = ", ".join([f"{p}:{c}" for p, c in cnt.most_common(topk)])
    print(f"🔥 最热点端点 {max_point} 出现 {max_count} 次")
    print(f"🔥 前{topk}热点（端点:次数）= {tops}")

def has_dup_points(batch: List[Dict]) -> bool:
    """
    检查“批内是否有端点重复”。这是硬约束：必须为 False。
    """
    used = set()
    for r in batch:
        g1, g2 = int(r["goal_1"]), int(r["goal_2"])
        if g1 in used or g2 in used:
            return True
        used.add(g1); used.add(g2)
    return False


# -------------------- 按点数切组（可设超大=全局） --------------------
def split_groups(rows: List[Dict], group_points: int) -> List[List[Dict]]:
    """
    按“点数”切组（每任务=2点）。
    - GROUP_POINTS=10**9 → 实际上只有 1 组（一次性全局装箱）。
    - 用分组的原因：大数据时节省内存/加快每次装箱；也能减少 left-over 的级联。
    """
    tasks_per_group = max(1, group_points // 2)
    return [rows[i:i + tasks_per_group] for i in range(0, len(rows), tasks_per_group)]


# -------------------- 多批同时装箱（启发式） --------------------
def pack_multi_batches(pool: List[Dict], need: int, width: int, height: int,
                       tiles: int, tile_cap: int, seed: int) -> Tuple[List[List[Dict]], List[Dict]]:
    """
    在“一个任务池 pool”上，同时构造 K 个批（K≈ceil(|pool|/need)）：
       - 先算每个端点的度数（出现次数），按 (deg[g1]+deg[g2]) 从大到小排序 → 热点优先；
       - 维护每个批的：
           used_points（该批已用过的端点集合）→ 保证批内零重复；
           used_tile_S/G（起点/终点的 tile 计数）→ 控制空间均衡；
           sizes（当前批已塞的任务数）→ 负载均衡（谁小先塞谁）；
           next_batch（热点轮转：同一端点下次优先去下一个批）。
       - 对每条任务，按“从负载较小的批开始、环扫”的顺序尝试放入：
           满足：未满、端点不冲突、tile 配额不超 → 就放入。
       - 返回：已填充的批列表 filled（去掉空批）+ 没塞进去的 leftover。
    """
    rng = random.Random(seed)
    if not pool:
        return [], []

    # 1) 统计端点度数（热点）并给任务排序（难的先放）
    deg = Counter()
    for r in pool:
        g1, g2 = int(r["goal_1"]), int(r["goal_2"])
        deg[g1] += 1; deg[g2] += 1

    order = list(range(len(pool)))
    order.sort(key=lambda i: deg[int(pool[i]["goal_1"])] + deg[int(pool[i]["goal_2"])], reverse=True)

    # 2) 预估这轮需要多少批（上限）
    K = max(1, math.ceil(len(pool) / need))

    # 3) tile 划分函数（把线性索引映射到 tile 坐标）
    tile_w, tile_h = max(1, width // tiles), max(1, height // tiles)
    def tile_of(loc: int):
        x, y = loc % width, loc // width
        return (min(x // tile_w, tiles - 1), min(y // tile_h, tiles - 1))

    # 4) 初始化 K 个批的状态
    batches = [[] for _ in range(K)]
    used_points = [set() for _ in range(K)]            # 批内已用端点
    used_tile_S = [defaultdict(int) for _ in range(K)] # 批内起点 tile 次数
    used_tile_G = [defaultdict(int) for _ in range(K)] # 批内终点 tile 次数
    sizes = [0] * K                                    # 批大小（任务数）
    next_batch = defaultdict(int)                      # 热点轮转指针
    placed = [False] * len(pool)                       # 记录该任务是否成功放入某批

    # 5) 逐任务放入
    for idx in order:
        r = pool[idx]
        g1, g2 = int(r["goal_1"]), int(r["goal_2"])

        # 两个端点的轮转起点取较小值（让它们都尽量向后分散）
        start = min(next_batch[g1], next_batch[g2])

        # 负载均衡：按批大小从小到大排序；从 start 批起环扫
        ordered = sorted(range(K), key=lambda k: (sizes[k], k))
        start_pos = ordered.index(start) if start in ordered else 0
        scan = ordered[start_pos:] + ordered[:start_pos]

        put = None
        for k in scan:
            # 条件 1：该批未满
            if sizes[k] >= need:
                continue
            # 条件 2：该批未使用这两个端点（保证批内零重复）
            if g1 in used_points[k] or g2 in used_points[k]:
                continue
            # 条件 3：tile 配额未超（空间均衡）
            tS = tile_of(g1); tG = tile_of(g2)
            if used_tile_S[k][tS] >= tile_cap or used_tile_G[k][tG] >= tile_cap:
                continue
            put = k
            break

        if put is not None:
            # 放入该批，并更新所有状态
            batches[put].append(r)
            used_points[put].add(g1); used_points[put].add(g2)
            used_tile_S[put][tS] += 1
            used_tile_G[put][tG] += 1
            sizes[put] += 1
            placed[idx] = True

            # 热点轮转：下次涉及 g1/g2 的任务优先从下一个批开始找位置
            nb = (put + 1) % K
            next_batch[g1] = nb
            next_batch[g2] = nb

    # 6) 返回：去掉空批的 filled + 没塞进去的 leftover
    filled = [b for b in batches if b]
    leftover = [pool[i] for i, ok in enumerate(placed) if not ok]
    return filled, leftover


# -------------------- 直到耗尽（反复打包 leftover） --------------------
def greedy_pack_until_empty(pool: List[Dict], need: int, width: int, height: int,
                            tiles: int, tile_cap: int, seed: int) -> List[List[Dict]]:
    """
    反复调用 pack_multi_batches，对 leftover 再次装箱，直到耗尽。
    极端情况下（可能被 tile_cap/热点卡死），弹出 1 个任务单独成批，避免死循环。
    """
    batches = []
    local_pool = list(pool)
    step_seed = seed
    while local_pool:
        new_batches, local_pool = pack_multi_batches(local_pool, need, width, height, tiles, tile_cap, step_seed)
        if not new_batches:
            # 安全阀：避免无限循环（此举会产生小批，后续交给尾部均衡来处理）
            batches.append([local_pool[0]])
            local_pool = local_pool[1:]
        else:
            batches.extend(new_batches)
        step_seed += 1  # 每轮轻微扰动
    return batches


# -------------------- 尾部重平衡：合并倒数 K 批重装 --------------------
def rebalance_tail(batches: List[List[Dict]], need: int, lookback: int,
                   width: int, height: int, tiles: int, tile_cap: int, seed: int) -> List[List[Dict]]:
    """
    仅“动尾部”，保持前面批不变：
      - 把倒数 lookback 个批的任务合并成 tail_pool；
      - 对 tail_pool 再次做“零重复装箱”，通常能把尾部的小批变得更均衡。
    """
    if lookback <= 0 or not batches:
        return batches
    k = min(lookback, len(batches))
    head = batches[:-k]  # 头部保持不动
    tail_pool = []
    for b in batches[-k:]:
        tail_pool.extend(b)
    # 对尾部池进行“直到耗尽”的零重复装箱
    tail_batches = greedy_pack_until_empty(tail_pool, need, width, height, tiles, tile_cap, seed)
    return head + tail_batches


# -------------------- 吸收极小尾批：塞回前面（允许超 250） --------------------
def absorb_tiny_tail_into_previous(batches: List[List[Dict]],
                                   width:int, height:int, tiles:int, tile_cap:int,
                                   min_tail_size:int=10, relax_tile_cap:bool=True) -> List[List[Dict]]:
    """
    把所有“极小尾批”（大小 ≤ min_tail_size）的任务，尽量塞回前面的批：
      - 不再限制容量（允许 > need），但“批内不重复端点”仍是硬约束；
      - 优先遵守 tile_cap；若实在塞不下且 relax_tile_cap=True，则放宽 tile 配额再尝试；
      - 极端情况（所有前批都冲突）→ 少数任务仍会残留到一个尾批（很少见）。
    """
    if not batches:
        return batches

    # 计算 tile 的辅助函数
    tile_w, tile_h = max(1, width // tiles), max(1, height // tiles)
    def tile_of(loc:int):
        x, y = loc % width, loc // width
        return (min(x // tile_w, tiles - 1), min(y // tile_h, tiles - 1))

    # 为每个批建立状态（已用端点 / tile 使用计数）
    used_points=[]; used_tile_S=[]; used_tile_G=[]
    for b in batches:
        up=set(); tS=defaultdict(int); tG=defaultdict(int)
        for r in b:
            g1,g2=int(r["goal_1"]),int(r["goal_2"])
            up.add(g1); up.add(g2)
            tS[tile_of(g1)] += 1
            tG[tile_of(g2)] += 1
        used_points.append(up); used_tile_S.append(tS); used_tile_G.append(tG)

    # 收集所有“极小尾批”，并从列表末尾移除它们
    tail_tasks=[]; cut=len(batches)
    while cut>0 and len(batches[cut-1]) <= min_tail_size:
        cut -= 1
        tail_tasks.extend(batches.pop())

    if not tail_tasks:
        return batches  # 没有极小尾巴，就不处理

    # 简单排序（也可换热点/空间压力更强的排序）
    tail_tasks.sort(key=lambda r: (int(r["goal_1"]) + int(r["goal_2"])))

    # 逐个任务塞回前面的批
    leftovers=[]
    for r in tail_tasks:
        g1,g2=int(r["goal_1"]),int(r["goal_2"])
        tS,tG=tile_of(g1), tile_of(g2)
        placed=False

        # 两轮 pass：先严格遵守 tile_cap，再（必要时）放宽 tile_cap
        for pass_relax in (False, True) if relax_tile_cap else (False,):
            if placed:
                break
            # 从第1批开始尝试（也可改为从靠后批开始，减少对前面批的扰动）
            for k in range(len(batches)):
                # 容量不检查，但“批内不重复端点”要检查
                if (g1 in used_points[k]) or (g2 in used_points[k]):
                    continue
                # 第一轮：严格遵守 tile_cap；第二轮可放宽
                if not pass_relax:
                    if used_tile_S[k][tS] >= tile_cap or used_tile_G[k][tG] >= tile_cap:
                        continue
                # 可以放
                batches[k].append(r)
                used_points[k].add(g1); used_points[k].add(g2)
                used_tile_S[k][tS] += 1; used_tile_G[k][tG] += 1
                placed=True
                break

        if not placed:
            # 理论极端：所有前批都冲突 → 仍然保留到一个尾批
            leftovers.append(r)

    if leftovers:
        batches.append(leftovers)

    return batches


# -------------------- 主流程：组→装箱→尾部处理→输出 --------------------
def split_group_chain_solver_friendly(rows: List[Dict],
                                      cap_points:int, group_points:int,
                                      width:int, height:int,
                                      seed:int, tiles:int, tile_cap:int,
                                      tail_lookback:int) -> List[List[Dict]]:
    """
    主流程（可全局或分组）：
      - 先按 group_points（点数）把 rows 切成若干组（设超大=全局一次性装箱）；
      - 逐组处理：把“上一组 leftover + 本组任务”合并成 pool，在 pool 上做“多批同时装箱”；
      - 组末 leftover 留给下一组；
      - 所有组结束，若还有 leftover：用 greedy_pack_until_empty“直到耗尽”；
      - 做一次 rebalance_tail（只重装尾部 K 批）；
      - 最后 absorb_tiny_tail_into_previous（把极小尾批塞回前面，允许超 250）。
    """
    rng = random.Random(seed)
    groups = split_groups(rows, group_points)
    all_batches: List[List[Dict]] = []
    leftover: List[Dict] = []
    need = cap_points // 2  # 每批任务数（500 点 → 250 任务）

    for gi, g in enumerate(groups):
        shuffled = g[:]
        rng.shuffle(shuffled)  # 组内打乱，避免顺序偏置
        pool = leftover + shuffled
        leftover = []

        # 在 pool 上做一次“多批同时装箱”
        new_batches, pool_rem = pack_multi_batches(pool, need, width, height, tiles, tile_cap, seed + gi)
        all_batches.extend(new_batches)
        leftover = pool_rem  # 剩余继续带到下一组

    # 最后的 leftover 再做一次“直到耗尽”的零重复装箱
    if leftover:
        all_batches.extend(greedy_pack_until_empty(leftover, need, width, height, tiles, tile_cap, seed + 777))

    # 尾部重平衡（只动后 K 批）
    if tail_lookback > 0:
        all_batches = rebalance_tail(all_batches, need, tail_lookback, width, height, tiles, tile_cap, seed + 999)

    # 吸收极小尾批（允许超 250，但批内仍零重复）
    all_batches = absorb_tiny_tail_into_previous(
        all_batches, width, height, tiles, tile_cap,
        min_tail_size=MIN_TAIL_SIZE, relax_tile_cap=RELAX_TILECAP_ON_ABSORB
    )

    return all_batches


# -------------------- 入口：读→切→写→汇报 --------------------
def main():
    random.seed(SEED)

    tasks = read_tasks(CSV_FILE)
    total = len(tasks)
    print(f"✅ 总任务数: {total}, 总点数: {total*2}")

    print_hotspots(tasks, topk=HOT_TOPK)

    batches = split_group_chain_solver_friendly(
        rows=tasks,
        cap_points=CAP_POINTS,
        group_points=GROUP_POINTS,
        width=WIDTH, height=HEIGHT,
        seed=SEED,
        tiles=TILES, tile_cap=TILE_CAP,
        tail_lookback=TAIL_LOOKBACK,
    )

    out_dir = OUT_ROOT / f"scen_group_chain_{CAP_POINTS}"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 逐批写 SCEN 文件
    for i, batch in enumerate(batches, start=1):
        scen_path = out_dir / f"batch{i}.scen"
        write_scen(scen_path, batch, MAP_NAME, WIDTH, HEIGHT)

    # 汇报统计
    sizes = [len(b) for b in batches]
    dups = sum(1 for b in batches if has_dup_points(b))
    need = CAP_POINTS // 2
    print("===================================")
    print(f"📦 批总数: {len(batches)}")
    print(f"🧩 每批理想容量(任务): {need}（注意：吸收尾巴后个别批可能 > {need}）")
    print(f"📏 批大小范围(任务): min={min(sizes)}, max={max(sizes)}, avg={sum(sizes)/len(sizes):.2f}")
    print(f"🚫 批内重复端点: {dups}（应为 0）")
    print(f"♻️ 尾部重平衡 lookback={TAIL_LOOKBACK} | 极小尾批阈值={MIN_TAIL_SIZE} | 吸收放宽tile={RELAX_TILECAP_ON_ABSORB}")
    print(f"📂 输出目录: {out_dir}/")
    print("===================================")


if __name__ == "__main__":
    main()
