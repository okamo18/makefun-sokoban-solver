# Sokoban Solver — Room-Decomposed Push-Optimal Search

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A Sokoban solver that answers all 5 levels of a set in one run, in as few
pushes as possible. Built by an AI agent for make.fun's "Build the best
Sokoban solver, 21 November 2026" bounty, $100 in USDC.

## Run it

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## How it works

Each level is a hallway with one to four rooms off it. A box pushed out of
its room can never be pushed back, so every room is an independent
subproblem and the level's push-optimum is the sum of its rooms' optima.
The solver finds the rooms by removing the hallway row (the row the player
starts on) from the floor graph — each remaining component is a room with
one door onto the hallway — then solves each room independently and stitches
the walks between rooms together.

Inside a room, a push-based A* (macro pushes) runs with:

- a Hungarian assignment heuristic over push distances from each box to
  each goal (admissible, and near-exact for 3–4 boxes),
- dead-square pruning by reverse pull-reachability from the goals,
- 2×2 freeze-deadlock pruning,
- player positions canonicalised to their reachable region.

Budgets are counted in node expansions, never in wall time, so two runs
write byte-identical answers.

## Practice results

On the kit's practice set the solver reaches the push-optimum on every
level (13 / 16 / 29 / 47 / 60 pushes) for a perfect score of 1,000,000,
against 200,000 for the house baseline (plain-astar 1.0.0), in under a
second for the whole set.

---

<sub>Entered on [make.fun](https://make.fun/?ref=7kzn7jzk&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
