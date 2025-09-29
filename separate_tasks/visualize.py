import matplotlib.pyplot as plt
import numpy as np

def visualize_scen(map_file, scen_file):
    # 1. 读地图
    with open(map_file) as f:
        lines = [l.rstrip() for l in f if l.strip()]
    W = int(lines[2].split()[1])
    grid = lines[4:]
    H = len(grid)
    obs = np.zeros((H, W), dtype=bool)
    for y, row in enumerate(grid):
        for x, ch in enumerate(row):
            if ch != '.':
                obs[y, x] = True

    # 2. 读场景
    starts, goals = [], []
    with open(scen_file) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 7 or parts[0].startswith("version"):
                continue
            sx, sy, gx, gy = map(int, parts[4:8])
            starts.append((sx, sy))
            goals.append((gx, gy))

    if not starts:
        raise RuntimeError("没有读取到任何 start/goal，请确认 scen 文件格式")

    # 3. 绘制
    fig, ax = plt.subplots(figsize=(6,6))
    # 地图
    ax.imshow(obs, cmap='gray_r', origin='upper', zorder=0)
    # 目标
    gx, gy = zip(*goals)
    ax.scatter(gx, gy, c='blue', marker='x', s=40, label='Goal', zorder=1)
    # 起点
    sx, sy = zip(*starts)
    ax.scatter(sx, sy, c='red', marker='o', s=40, label='Start', zorder=2)

    # 箭头
    for (x0, y0), (x1, y1) in zip(starts, goals):
        ax.arrow(x0, y0, x1-x0, y1-y0,
                 head_width=0.2, length_includes_head=True,
                 color='gray', alpha=0.3, zorder=1)

    # 坐标设置
    ax.set_xlim(-0.5, W-0.5)
    ax.set_ylim(H-0.5, -0.5)
    ax.invert_yaxis()
    ax.set_xticks(range(W))
    ax.set_yticks(range(H))
    ax.grid(True, linewidth=0.2, zorder=3)
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()

#if __name__ == '__main__':
    #visualize_scen('random-32-32-10.map', 'random-32-32-10-random-1.scen')

if __name__ == '__main__':
   visualize_scen('32x32_allEntry.map', 'scen_300/batch10.scen')