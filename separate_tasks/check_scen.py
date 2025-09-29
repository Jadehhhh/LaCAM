import sys, collections

if len(sys.argv) < 2:
    print("用法: python check_scen.py file.scen")
    sys.exit(1)

scen = sys.argv[1]
with open(scen, "r", encoding="utf-8") as f:
    lines = [l.strip() for l in f if l.strip()]
rows = [l.split() for l in lines[1:]]  # 跳过 header

starts = collections.Counter()
goals  = collections.Counter()
pairs  = collections.Counter()

for r in rows:
    sx, sy, gx, gy = map(int, (r[4], r[5], r[6], r[7]))
    starts[(sx,sy)] += 1
    goals[(gx,gy)]  += 1
    pairs[((sx,sy),(gx,gy))] += 1

dupe_starts = [(k,v) for k,v in starts.items() if v>1]
dupe_goals  = [(k,v) for k,v in goals.items() if v>1]
self_same   = [p for p in pairs if p[0]==p[1]]
cross       = [k for k in starts if k in goals]

print("文件:", scen)
print("总任务数:", len(rows))
print("重复 starts:", len(dupe_starts), dupe_starts[:10])
print("重复 goals :", len(dupe_goals),  dupe_goals[:10])
print("start==goal:", len(self_same), self_same[:10])
print("start 出现在其他 goal 的点数:", len(cross), cross[:10])
