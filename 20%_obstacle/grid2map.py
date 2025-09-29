import csv

WALL = '@'
FREE = '.'

grid_file = "./32x32_allEntry.grid"
map_file  = "./32x32_allEntry.map"

with open(grid_file) as f:
    rdr = list(csv.reader(f))
    line = rdr[1]
    if len(line) == 1:
        H, W = map(int, line[0].strip().split(','))
    else:
        H, W = map(int, line)
    cells = [['?'] * W for _ in range(H)]

    for row in rdr[3:]:  # 跳过前3行
        if len(row) < 4:
            continue
        _, cell_type, x, y = row[:4]
        x, y = int(x), int(y)
        cells[y][x] = WALL if cell_type == "Obstacle" else FREE

with open(map_file, "w") as f:
    f.write(f"type octile\nheight {H}\nwidth {W}\nmap\n")
    for r in range(H):
        f.write(''.join(cells[r]) + "\n")

print("wrote", map_file)
