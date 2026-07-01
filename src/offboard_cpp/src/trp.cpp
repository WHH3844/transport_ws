#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace {
constexpr int ROWS = 9;
constexpr int COLS = 7;

std::pair<int, int> id_to_rc(int id) {
    return {id / 10 - 1, id % 10 - 1};
}

int rc_to_id(int row, int col) {
    return (row + 1) * 10 + (col + 1);
}

bool is_valid_id(int id) {
    auto [r, c] = id_to_rc(id);
    return r >= 0 && r < ROWS && c >= 0 && c < COLS;
}

std::vector<int> plan_path(int start_id, int target_id, const std::set<int>& forbidden_ids) {
    auto [sr, sc] = id_to_rc(start_id);
    auto [tr, tc] = id_to_rc(target_id);

    std::vector<std::vector<bool>> grid(ROWS, std::vector<bool>(COLS, true));
    for (int id : forbidden_ids) {
        auto [r, c] = id_to_rc(id);
        if (r >= 0 && r < ROWS && c >= 0 && c < COLS) {
            grid[r][c] = false;
        }
    }

    if (!grid[sr][sc] || !grid[tr][tc]) {
        return {};
    }

    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<std::pair<int, int>>> parent(
        ROWS, std::vector<std::pair<int, int>>(COLS, {-1, -1}));

    std::queue<std::pair<int, int>> q;
    q.push({sr, sc});
    visited[sr][sc] = true;

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    bool found = false;
    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop();

        if (r == tr && c == tc) {
            found = true;
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && grid[nr][nc] && !visited[nr][nc]) {
                visited[nr][nc] = true;
                parent[nr][nc] = {r, c};
                q.push({nr, nc});
            }
        }
    }

    if (!found) {
        return {};
    }

    std::vector<int> path;
    int r = tr;
    int c = tc;
    while (!(r == sr && c == sc)) {
        path.push_back(rc_to_id(r, c));
        std::tie(r, c) = parent[r][c];
    }
    path.push_back(start_id);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> plan_full_traversal(int start_id, const std::set<int>& forbidden_ids) {
    std::set<int> all_ids;
    for (int row = 0; row < ROWS; ++row) {
        for (int col = 0; col < COLS; ++col) {
            int id = rc_to_id(row, col);
            if (forbidden_ids.find(id) == forbidden_ids.end()) {
                all_ids.insert(id);
            }
        }
    }

    if (all_ids.find(start_id) == all_ids.end()) {
        return {};
    }

    std::vector<int> final_path;
    std::set<int> unvisited = all_ids;

    int current = start_id;
    unvisited.erase(current);
    final_path.push_back(current);

    while (!unvisited.empty()) {
        std::vector<int> best_segment;
        int next_target = -1;

        for (int candidate : unvisited) {
            std::vector<int> path = plan_path(current, candidate, forbidden_ids);
            if (!path.empty() && (best_segment.empty() || path.size() < best_segment.size())) {
                best_segment = std::move(path);
                next_target = candidate;
            }
        }

        if (best_segment.empty()) {
            break;
        }

        final_path.insert(final_path.end(), best_segment.begin() + 1, best_segment.end());
        current = next_target;
        unvisited.erase(next_target);
    }

    std::vector<int> back_path = plan_path(current, start_id, forbidden_ids);
    if (!back_path.empty()) {
        final_path.insert(final_path.end(), back_path.begin() + 1, back_path.end());
    }

    return final_path;
}

void print_path(const std::vector<int>& path) {
    std::cout << "Planned path length: " << path.size() << "\n";
    std::cout << "Path IDs:";
    for (int id : path) {
        std::cout << ' ' << id;
    }
    std::cout << "\n";

    std::cout << "Path (row, col):";
    for (int id : path) {
        auto [r, c] = id_to_rc(id);
        std::cout << " (" << r + 1 << ',' << c + 1 << ')';
    }
    std::cout << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    const int start_id = 91;

    // 默认使用 3 个禁飞区，可通过命令行参数覆盖，例如: ./trp 33 54 72
    std::set<int> forbidden_ids = {33, 54, 72};
    if (argc == 4) {
        forbidden_ids.clear();
        for (int i = 1; i <= 3; ++i) {
            int id = std::stoi(argv[i]);
            if (!is_valid_id(id)) {
                std::cerr << "Invalid no-fly ID: " << id << "\n";
                return 1;
            }
            forbidden_ids.insert(id);
        }
        if (forbidden_ids.size() != 3) {
            std::cerr << "Please provide 3 distinct no-fly IDs.\n";
            return 1;
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<int> path = plan_full_traversal(start_id, forbidden_ids);
    auto t2 = std::chrono::high_resolution_clock::now();

    if (path.empty()) {
        std::cerr << "Path planning failed.\n";
        return 1;
    }

    std::chrono::duration<double, std::milli> elapsed_ms = t2 - t1;

    std::cout << "Grid: 9x7\n";
    std::cout << "Start ID: " << start_id << "\n";
    std::cout << "No-fly IDs:";
    for (int id : forbidden_ids) {
        std::cout << ' ' << id;
    }
    std::cout << "\n";

    print_path(path);
    std::cout << std::fixed << std::setprecision(3)
              << "Planning time: " << elapsed_ms.count() << " ms\n";
    return 0;
}
