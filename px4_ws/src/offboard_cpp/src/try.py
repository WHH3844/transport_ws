from collections import deque
from time import perf_counter
import argparse
from typing import List, Set, Tuple

ROWS = 9
COLS = 7


def id_to_rc(map_id: int) -> Tuple[int, int]:
    return map_id // 10 - 1, map_id % 10 - 1


def rc_to_id(row: int, col: int) -> int:
    return (row + 1) * 10 + (col + 1)


def is_valid_id(map_id: int) -> bool:
    r, c = id_to_rc(map_id)
    return 0 <= r < ROWS and 0 <= c < COLS


def plan_path(start_id: int, target_id: int, forbidden_ids: Set[int]) -> List[int]:
    sr, sc = id_to_rc(start_id)
    tr, tc = id_to_rc(target_id)

    grid = [[True] * COLS for _ in range(ROWS)]
    for fid in forbidden_ids:
        r, c = id_to_rc(fid)
        if 0 <= r < ROWS and 0 <= c < COLS:
            grid[r][c] = False

    if not grid[sr][sc] or not grid[tr][tc]:
        return []

    visited = [[False] * COLS for _ in range(ROWS)]
    parent = [[(-1, -1)] * COLS for _ in range(ROWS)]

    q = deque([(sr, sc)])
    visited[sr][sc] = True

    for_found = False
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while q:
        r, c = q.popleft()
        if (r, c) == (tr, tc):
            for_found = True
            break

        for dr, dc in directions:
            nr = r + dr
            nc = c + dc
            if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] and not visited[nr][nc]:
                visited[nr][nc] = True
                parent[nr][nc] = (r, c)
                q.append((nr, nc))

    if not for_found:
        return []

    path = []
    r, c = tr, tc
    while (r, c) != (sr, sc):
        path.append(rc_to_id(r, c))
        r, c = parent[r][c]
    path.append(start_id)
    path.reverse()
    return path


def plan_full_traversal(start_id: int, forbidden_ids: Set[int]) -> List[int]:
    all_ids = {
        rc_to_id(r, c)
        for r in range(ROWS)
        for c in range(COLS)
        if rc_to_id(r, c) not in forbidden_ids
    }

    if start_id not in all_ids:
        return []

    final_path = [start_id]
    unvisited = set(all_ids)
    unvisited.remove(start_id)
    current = start_id

    while unvisited:
        best_segment = []
        next_target = None

        for candidate in unvisited:
            segment = plan_path(current, candidate, forbidden_ids)
            if segment and (not best_segment or len(segment) < len(best_segment)):
                best_segment = segment
                next_target = candidate

        if not best_segment:
            break

        final_path.extend(best_segment[1:])
        current = next_target
        unvisited.remove(next_target)

    back_path = plan_path(current, start_id, forbidden_ids)
    if back_path:
        final_path.extend(back_path[1:])

    return final_path


def print_path(path: List[int]) -> None:
    print(f"Planned path length: {len(path)}")
    print("Path IDs:", " ".join(map(str, path)))

    rc_path = [f"({r + 1},{c + 1})" for r, c in (id_to_rc(pid) for pid in path)]
    print("Path (row, col):", " ".join(rc_path))


def main() -> int:
    parser = argparse.ArgumentParser(description="9x7 grid BFS + greedy traversal planner")
    parser.add_argument(
        "--no-fly",
        nargs=3,
        type=int,
        default=[33, 54, 72],
        help="3 no-fly IDs, e.g. --no-fly 33 54 72",
    )
    args = parser.parse_args()

    forbidden_ids = set(args.no_fly)
    if len(forbidden_ids) != 3:
        print("Please provide 3 distinct no-fly IDs.")
        return 1
    if any(not is_valid_id(fid) for fid in forbidden_ids):
        print("Invalid no-fly ID detected.")
        return 1

    start_id = 91

    t1 = perf_counter()
    path = plan_full_traversal(start_id, forbidden_ids)
    t2 = perf_counter()

    if not path:
        print("Path planning failed.")
        return 1

    print("Grid: 9x7")
    print(f"Start ID: {start_id}")
    print("No-fly IDs:", " ".join(map(str, sorted(forbidden_ids))))
    print_path(path)
    print(f"Planning time: {(t2 - t1) * 1000:.3f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
