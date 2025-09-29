import csv, random
from pathlib import Path

def generate_tasks(grid_file: str, out_file: str,
                   total_tasks=2000, points_per_group=800, seed=123):
    """
    生成任务文件：
    - 总共 total_tasks 个任务（= total_tasks*2 个点）
    - 每 group 内 points_per_group 个点（两两配对 -> points_per_group/2 个任务）
    - 每组内部点不重复，不同组之间允许重复
    - agent_id, current_location, time 都留空
    """

    random.seed(seed)
    GRID_FILE = Path(grid_file)
    OUT_FILE = Path(out_file)

    # 1. 读取 Entry 点
    entries = []
    with GRID_FILE.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(("grid size", "id,", "32,")):
                continue
            parts = line.split(",")
            if len(parts) < 4:
                continue
            cell_id = int(parts[0])
            cell_type = parts[1].lower()
            if cell_type != "obstacle":
                entries.append(cell_id)

    if len(entries) < points_per_group:
        raise RuntimeError(f"地图可用点不足: 需要 {points_per_group}, 实际只有 {len(entries)}")

    tasks_per_group = points_per_group // 2
    groups = total_tasks // tasks_per_group

    rows = []
    for g in range(groups):
        pool = entries.copy()
        random.shuffle(pool)
        group_points = pool[:points_per_group]
        assert len(set(group_points)) == points_per_group, "组内出现了重复点"

        for i in range(0, points_per_group, 2):
            goal1 = group_points[i]
            goal2 = group_points[i+1]
            # agent_id, current_location, time 留空
            rows.append(["", "", goal1, goal2, ""])

    # 2. 写入 CSV
    with OUT_FILE.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["agent_id", "current_location", "goal_1", "goal_2", "time"])
        w.writerows(rows)

    print(f"✅ 已生成 {len(rows)} 个任务，保存到 {OUT_FILE}")

if __name__ == "__main__":
    generate_tasks("32x32_allEntry.grid", "tasks_group800_blank_time_blank.csv")
