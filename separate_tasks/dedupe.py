#!/usr/bin/env python3
import sys

def dedupe_scen(infile, outfile):
    seen_starts = set()
    seen_goals  = set()
    with open(infile) as fin, open(outfile, 'w') as fout:
        for line in fin:
            line = line.rstrip('\n')
            if not line or line.startswith("version"):
                # 保留版本行和空行
                fout.write(line + "\n")
                continue
            parts = line.split()
            if len(parts) < 8:
                # 格式异常，直接保留
                fout.write(line + "\n")
                continue

            # scen 格式：ID map W H sx sy gx gy cost
            sx, sy, gx, gy = map(int, parts[4:8])
            start = (sx, sy)
            goal  = (gx, gy)

            if start in seen_starts or goal in seen_goals:
                # 跳过这条重复的任务
                continue

            # 第一次见到的起点和终点都保留
            seen_starts.add(start)
            seen_goals .add(goal)
            fout.write(line + "\n")

    print(f"过滤完成，保留 {len(seen_starts)} 条无重复任务，输出到 {outfile}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法: python dedupe_scen.py  batch0.scen  batch0_clean.scen")
        sys.exit(1)
    dedupe_scen(sys.argv[1], sys.argv[2])

