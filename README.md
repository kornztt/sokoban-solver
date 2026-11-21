# Push-optimal Sokoban solver — make.fun bounty entry

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/kornztt/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/kornztt/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for the
[make.fun bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121):
one program (`main.ts`, Node 22, no dependencies) answers all 5 instances
of a set in a single run, writing `/output/<id>/solution.txt` for each.

## How it works

Every level in this competition is a 1-wide hallway with 1–4 small rooms
off it, one door each, and a box pushed out of a room into the hallway can
never be pushed back — so each room is an independent problem and the
level's push-optimum is the sum of the rooms' optima. The solver:

1. splits a level into hallway + rooms (one door per room),
2. solves each room push-optimally with A\* over pushes — player walks
   between pushes are free — using an admissible box-to-goal assignment
   heuristic, static dead-square pruning, and 2×2 freeze pruning,
3. joins the per-room push sequences with hallway walks.

Budgets are counted in expanded states, never in wall time; there is no
clock and no randomness, so two runs write identical bytes. A whole-level
push search is kept as a fallback for any layout that does not split into
rooms.

## Results

- Practice set: **1,000,000 / 1,000,000** — all 5 instances solved in
  exactly the optimal push counts (13, 16, 29, 47, 60), in ~0.5 s.
- 10 further sets drawn with the kit's own generator (50 instances):
  **1,000,000** on every set.
- House baseline `plain-astar 1.0.0` on the practice set: 200,000
  (only the first instance solved).

## Run it

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` reads `/input/<id>/level.txt` (or `$MAKEFUN_INPUT_DIR`) and
writes `/output/<id>/solution.txt` (or `$MAKEFUN_OUTPUT_DIR`).
