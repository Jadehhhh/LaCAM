import csv
import math
import pathlib

# ---------- Config ----------
CSV_FILE = "tasks_group800_blank_time_blank.csv"   # 输入 CSV
MAP_NAME = "32x32_allEntry.map"                   # 地图名
WIDTH, HEIGHT = 32, 32
OUT_FILE = pathlib.Path("all_tasks.scen")         # 输出文件

# ---------- Load tasks ----------
with open(CSV_FILE, newline='') as f:
    tasks = [row for row in csv.DictReader(f) if row.get("goal_1") and row.get("goal_2")]
print(f"Total tasks: {len(tasks)}")

# ---------- Write scen ----------
with open(OUT_FILE, "w", newline="") as wf:
    wf.write("version 1\n")
    for idx, row in enumerate(tasks):
        s_loc = int(row["goal_1"])
        g_loc = int(row["goal_2"])
        sx, sy = s_loc % WIDTH, s_loc // WIDTH
        gx, gy = g_loc % WIDTH, g_loc // WIDTH
        length = math.hypot(gx - sx, gy - sy)
        wf.write(
            f"{idx}\t{MAP_NAME}\t{WIDTH}\t{HEIGHT}\t{sx}\t{sy}\t{gx}\t{gy}\t{length:.8f}\n"
        )

print(f"✅ Wrote {len(tasks)} tasks to {OUT_FILE}")
