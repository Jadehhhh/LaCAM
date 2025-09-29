// bridge/bridge.cpp — supports "format: xy" (recommended) and legacy integer input.
// Optional: `output: xy` to include id/index/x/y per entry; default is legacy id-only arrays.

#include "instance.hpp"   // Instance(map, starts, goals)
#include "planner.hpp"    // Solution solve(...)
#include "utils.hpp"      // Deadline
#include "graph.hpp"      // Graph (for map dims and U table)

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <random>
#include <cstddef>

// ---------- small helpers ----------
static std::string trim(std::string s){
  auto p = s.find_first_not_of(" \t\r\n");
  if (p == std::string::npos) return "";
  auto q = s.find_last_not_of(" \t\r\n");
  return s.substr(p, q - p + 1);
}

static bool parse_after_colon(const std::string& line, std::string& out){
  auto pos = line.find(':');
  if (pos == std::string::npos) return false;
  out = trim(line.substr(pos + 1));
  return true;
}

static bool parse_int_list(const std::string& s, int K, std::vector<int>& out){
  std::istringstream iss(s);
  out.assign(K, 0);
  for (int i = 0; i < K; ++i) if (!(iss >> out[i])) return false;
  return true;
}

static bool parse_xy_pairs(const std::string& s, int K, std::vector<std::pair<int,int>>& out){
  std::istringstream iss(s);
  out.clear(); out.reserve(K);
  for (int i = 0; i < K; ++i) {
    std::string tok; if (!(iss >> tok)) return false;
    auto p = tok.find(',');
    if (p == std::string::npos) return false;
    int x = std::stoi(tok.substr(0, p));
    int y = std::stoi(tok.substr(p + 1));
    out.emplace_back(x, y);
  }
  return (int)out.size() == K;
}

int main(){
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);

  std::string line, map_path, tmp;
  int K = 0;

  // map:
  if (!std::getline(std::cin, line) || !parse_after_colon(line, map_path)) {
    std::cerr << "ERR: bad 'map:' line\n"; return 2;
  }
  // N:
  if (!std::getline(std::cin, line) || !parse_after_colon(line, tmp)) {
    std::cerr << "ERR: bad 'N:' line\n"; return 2;
  }
  K = std::stoi(tmp);
  if (K <= 0) { std::cerr << "ERR: N must be > 0\n"; return 2; }

  // ---- Robustly parse optional directives (do not push back to stdin) ----
  std::string format = "uindex";
  std::string output_mode = "id";

  // Pull subsequent lines; cache the first non-optional line as starts_line
  std::string starts_line;
  while (std::getline(std::cin, line)) {
    std::string key = trim(line);
    if (key.rfind("format:", 0) == 0) {
      parse_after_colon(line, format);
      continue;
    }
    if (key.rfind("output:", 0) == 0) {
      parse_after_colon(line, output_mode);
      continue;
    }
    // First non-optional line should be 'starts:'
    starts_line = line;
    break;
  }
  if (starts_line.empty()) { std::cerr << "ERR: missing 'starts:' line\n"; return 2; }

  std::vector<int> starts_k(K), goals_k(K); // k = y*W + x (uniform index)
  Graph Gv(map_path);
  int W = Gv.width, H = Gv.height;
  if (W <= 0 || H <= 0) { std::cerr << "ERR: invalid map dims\n"; return 2; }

  if (format == "uindex") {
    // Parse starts_line itself
    if (!parse_after_colon(starts_line, tmp) || !parse_int_list(tmp, K, starts_k)) {
      std::cerr << "ERR: bad 'starts:' (uindex)\n"; return 2;
    }
    // goals: read the next line
    if (!std::getline(std::cin, line) || !parse_after_colon(line, tmp) || !parse_int_list(tmp, K, goals_k)) {
      std::cerr << "ERR: bad 'goals:' (uindex)\n"; return 2;
    }
    // Sanity check
    for (int i = 0; i < K; ++i) {
      if (starts_k[i] < 0 || starts_k[i] >= W*H || goals_k[i] < 0 || goals_k[i] >= W*H) {
        std::cerr << "ERR: index OOB at i=" << i << "\n"; return 2;
      }
      if (Gv.U[starts_k[i]] == nullptr || Gv.U[goals_k[i]] == nullptr) {
        std::cerr << "ERR: index maps to obstacle at i=" << i << "\n"; return 2;
      }
    }
  } else if (format == "xy") {
    // starts_line is: starts: x,y x,y ...
    std::vector<std::pair<int,int>> starts_xy, goals_xy;
    if (!parse_after_colon(starts_line, tmp) || !parse_xy_pairs(tmp, K, starts_xy)) {
      std::cerr << "ERR: bad 'starts:' (xy)\n"; return 2;
    }
    if (!std::getline(std::cin, line) || !parse_after_colon(line, tmp) || !parse_xy_pairs(tmp, K, goals_xy)) {
      std::cerr << "ERR: bad 'goals:' (xy)\n"; return 2;
    }
    for (int i = 0; i < K; ++i) {
      auto [sx, sy] = starts_xy[i];
      auto [gx, gy] = goals_xy[i];
      if (sx < 0 || sx >= W || sy < 0 || sy >= H || gx < 0 || gx >= W || gy < 0 || gy >= H) {
        std::cerr << "ERR: xy OOB at i=" << i << "\n"; return 2;
      }
      int si = sy * W + sx, gi = gy * W + gx;
      if (Gv.U[si] == nullptr || Gv.U[gi] == nullptr) {
        std::cerr << "ERR: xy maps to obstacle at i=" << i << "\n"; return 2;
      }
      starts_k[i] = si; goals_k[i] = gi;
    }
  } else {
    std::cerr << "ERR: unknown format: " << format << "\n"; return 2;
  }

  // ---- Call LaCAM and output (keep your existing logic; if you add `output: xy`, handle it like your previous version) ----
  Instance ins(map_path, starts_k, goals_k);
  std::mt19937 MT(0);
  Deadline dl(1e9);
  const int verbose = 0;
  Solution sol = solve(ins, verbose, &dl, &MT);

  // Minimal output (id arrays). If you used an xy-outputting version before, you can extend here similarly.
  const std::size_t T = sol.size();
  std::cout << "{\"T\":" << T
            << ",\"K\":" << K
            << ",\"W\":" << W
            << ",\"H\":" << H
            << ",\"solution\":[";
  for (std::size_t t = 0; t < T; ++t) {
    if (t) std::cout << ",";
    std::cout << "[";
    for (int k = 0; k < K; ++k) {
      if (k) std::cout << ",";
      std::cout << sol[t][k]->id;  // grid u-index
    }
    std::cout << "]";
  }
  std::cout << "]}\n";
  return 0;
}
