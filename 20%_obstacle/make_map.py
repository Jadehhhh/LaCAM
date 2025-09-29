import csv, pathlib, random
random.seed(0)

N = 32
density = 0.20           # 20% 障碍
path = pathlib.Path("32x32_allEntry.grid")
path.parent.mkdir(exist_ok=True)

with open(path, "w", newline="") as f:
    w = csv.writer(f)
    f.write("grid size\n")
    w.writerow([N, N])
    w.writerow(["id","type","x","y","w0","w1","w2","w3","w4"])

    obstacles = set()
    tile_size = 4  # 把地图划成 4x4 的 tile，每块 8x8
    block = N // tile_size

    for tr in range(tile_size):
        for tc in range(tile_size):
            cells = []
            for r in range(tr*block, (tr+1)*block):
                for c in range(tc*block, (tc+1)*block):
                    cells.append(r*N + c)
            k = int(len(cells) * density)
            obstacles |= set(random.sample(cells, k))

    for idx in range(N*N):
        r, c = divmod(idx, N)
        typ = "Obstacle" if idx in obstacles else "Entry"

        # 按 East, North, West, South 顺序写权重
        east  = 1 if c < N-1 else "inf"
        north = 1 if r > 0   else "inf"
        west  = 1 if c > 0   else "inf"
        south = 1 if r < N-1 else "inf"

        if typ == "Obstacle":
            row = [idx, typ, c, r, "inf","inf","inf","inf","inf"]
        else:
            row = [idx, typ, c, r, east, north, west, south, 1]
        w.writerow(row)

print("32x32_allEntry.grid 生成完成")
