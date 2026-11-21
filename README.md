# Roomwise Push-Optimal Sokoban Solver

A push-optimal Sokoban solver for make.fun's "Build the best Sokoban solver"
bounty. Each generated level is a one-wide hallway with independent rooms off
it (one door each, boxes never leave their room), so the solver splits the
level at the hallway row, solves every room on its own with a push-based A*
(cost = pushes, admissible box-to-goal assignment heuristic from reverse pull
distances, dead-square and 2x2 deadlock pruning), then stitches the room
solutions together with hallway walks. A whole-level push A* runs as a
fallback when a level does not decompose. Search budgets are expansion
counts, never the clock, so two runs write identical bytes.

Run: `node main.ts` with `/input/<id>/level.txt` in, `/output/<id>/solution.txt` out.
