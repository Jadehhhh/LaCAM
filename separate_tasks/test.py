# 把 batch0.scen 交给 LaCAM 自带 check 工具（若有）
# 没有的话用脚本检测：
python3 - <<'PY'
import re, sys, pathlib
MAP = pathlib.Path("32x32_allEntry.map").read_text().splitlines()[4:]  # 从第5行开始是真地图
H, W = len(MAP), len(MAP[0])
bad = 0
for ln,line in enumerate(open("batch0.scen")):
    if ln < 3: continue
    sx, sy, gx, gy = map(int, re.findall(r'\d+', line)[1:])  # 跳过 id
    for x,y,tag in [(sx,sy,"start"), (gx,gy,"goal")]:
        if MAP[y][x] != '.':
            print(f"[bad] line {ln+1}: {tag} ({x},{y}) on '{MAP[y][x]}'")
            bad += 1
print("bad =", bad)
PY

