#!/usr/bin/env bash
set -euo pipefail

# === 基本配置（可用环境变量覆盖） ===
BIN="${BIN:-../build/main}"           # 可执行文件路径，亦可在命令行前加 BIN=/path/to/bin 覆盖
MAP="${MAP:-32x32_allEntry.map}"      # 地图文件名
TIME_LIMIT="${TIME_LIMIT:-240}"        # 超时
VERBOSE="${VERBOSE:-3}"               # 日志级别

SCEN_PREFIX="${SCEN_PREFIX:-batch}"   # scen 文件前缀
SCEN_SUFFIX="${SCEN_SUFFIX:-.scen}"   # scen 文件后缀

# === 用法 ===
# 1) 显式传入要遍历的目录：
#    ./run_dirs.sh scen_300 scen_500 scen_600 scen_700
# 2) 不传参数则自动遍历当前目录下匹配 scen_* 的目录：
#    ./run_dirs.sh

# 收集要处理的目录
if [[ "$#" -gt 0 ]]; then
  DIRS=("$@")
else
  # 默认匹配 scen_* 目录
  mapfile -t DIRS < <(find . -maxdepth 1 -type d -name "scen_*" -printf "%f\n" | sort)
fi

if [[ "${#DIRS[@]}" -eq 0 ]]; then
  echo "未发现需要处理的目录（请传入目录名或在当前目录下确保存在 scen_* 目录）。" >&2
  exit 1
fi

echo "可执行文件: $BIN"
echo "地图文件  : $MAP"
echo "目录清单  : ${DIRS[*]}"
echo

grand_total_ms=0
grand_total_agents=0

for DIR in "${DIRS[@]}"; do
  [[ -d "$DIR" ]] || { echo "跳过：$DIR 不是目录"; continue; }

  echo "===== 处理目录: $DIR ====="
  dir_total_ms=0
  dir_total_agents=0
  scen_count=0

  # 只处理形如 batch*.scen 的文件（可用 SCEN_PREFIX/SCEN_SUFFIX 定制）
  shopt -s nullglob
  files=( "$DIR"/"${SCEN_PREFIX}"*"${SCEN_SUFFIX}" )
  shopt -u nullglob

  # 按文件名排序保证可重复性
  IFS=$'\n' files=( $(printf "%s\n" "${files[@]}" | sort) ); unset IFS

  if [[ "${#files[@]}" -eq 0 ]]; then
    echo "  目录中未找到 ${SCEN_PREFIX}*${SCEN_SUFFIX} 文件，跳过。"
    echo
    continue
  fi

  for SCEN in "${files[@]}"; do
    scen_count=$((scen_count + 1))
    base="$(basename "$SCEN")"
    echo "--- $base ---"

    if [[ ! -f "$SCEN" ]]; then
      echo "文件不存在: $SCEN" >&2
      exit 1
    fi

    # 计算任务数（去掉 header 行）
    agents=$(($(wc -l < "$SCEN") - 1))
    if (( agents <= 0 )); then
      echo "  文件 $base 中任务数为 0，跳过。"
      continue
    fi
    echo "N (tasks) = $agents"

    # 运行可执行文件
    output="$("$BIN" -i "$SCEN" -m "$MAP" -N "$agents" -v "$VERBOSE" -t "$TIME_LIMIT" 2>&1 || true)"

    # 解析 makespan: 后的数字
    ms="$(awk '
      /makespan:/ {
        for (i=1; i<=NF; i++) if ($i=="makespan:") { print $(i+1); exit }
      }' <<<"$output")"

    if [[ -z "$ms" ]]; then
      echo "  ⚠️ 未能解析 makespan，原始输出如下：" >&2
      echo "$output"
      exit 1
    fi

    echo "makespan = $ms"
    dir_total_ms=$((dir_total_ms + ms))
    dir_total_agents=$((dir_total_agents + agents))
  done

  echo "------------------------------------------"
  echo "目录小计：$DIR | 文件数=${scen_count} | 总任务=${dir_total_agents} | makespan总和=${dir_total_ms}"
  echo "------------------------------------------"
  echo

  grand_total_ms=$((grand_total_ms + dir_total_ms))
  grand_total_agents=$((grand_total_agents + dir_total_agents))
done

echo "=========================================="
echo "全部目录汇总：总任务=${grand_total_agents} | makespan总和=${grand_total_ms}"
echo "=========================================="
