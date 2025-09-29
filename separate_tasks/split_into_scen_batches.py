import csv, math
from pathlib import Path

# ---------- 配置 ----------
CSV_FILE = "tasks_group800_blank_time_blank.csv"  # 输入任务CSV
MAP_NAME = "32x32_allEntry.map"                  # 地图名
WIDTH, HEIGHT = 32, 32                           # 地图尺寸
BATCH_SIZES = [100, 200, 400, 800]               # 要切分的 batch 大小
OUT_ROOT = Path(".")                             # 输出目录

# ---------- 读取任务 ----------
with open(CSV_FILE, newline="", encoding="utf-8") as f:
    tasks = [row for row in csv.DictReader(f) if row.get("goal_1") and row.get("goal_2")]
total_tasks = len(tasks)
print(f"✅ 总任务数: {total_tasks}")

# ---------- 写 scen 文件 ----------
def write_scen(path: Path, rows, start_id=0):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as wf:
        wf.write("version 1\n")
        for local_id, row in enumerate(rows, start=start_id):
            s_loc = int(row["goal_1"])
            g_loc = int(row["goal_2"])
            sx, sy = s_loc % WIDTH, s_loc // WIDTH
            gx, gy = g_loc % WIDTH, g_loc // WIDTH
            length = math.hypot(gx - sx, gy - sy)
            wf.write(f"{local_id}\t{MAP_NAME}\t{WIDTH}\t{HEIGHT}\t"
                     f"{sx}\t{sy}\t{gx}\t{gy}\t{length:.8f}\n")

# ---------- 按不同 batch size 切分 ----------
for bs in BATCH_SIZES:
    out_dir = OUT_ROOT / f"scen_{bs}"
    out_dir.mkdir(parents=True, exist_ok=True)
    num_batches = (total_tasks + bs - 1) // bs  # 向上取整
    for i in range(num_batches):
        start = i * bs
        end = min((i + 1) * bs, total_tasks)
        if start >= end:
            break
        scen_path = out_dir / f"batch{i}.scen"
        write_scen(scen_path, tasks[start:end], start_id=0)
    print(f"📂 已生成 {num_batches} 个文件到 {out_dir}/")
