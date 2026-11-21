// Push-optimal Sokoban solver for make.fun sokoban-solver-01.
// Levels are a 1-wide hallway with independent rooms off it (one door each,
// boxes never leave their room), so each room is solved on its own with a
// push-based A* (cost = pushes, admissible assignment heuristic from reverse
// pull distances, dead-square pruning) and the room solutions are stitched
// together with hallway walks. A whole-level push A* is the fallback when a
// level does not decompose. Deterministic: budgets are expansion counts,
// never the clock.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];
const INF = 1e9;

function parse(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Array(w * h).fill(true);
  const goals = [];
  const boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = rows[y][x] ?? '#';
      const i = y * w + x;
      wall[i] = ch === '#';
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS walk (no pushes) from start to target avoiding walls and boxes.
// allowed: optional Set of cells the player may stand on (default: all floor).
function walkPath(lv, start, target, boxSet, allowed) {
  if (start === target) return '';
  const prev = new Map([[start, -1]]);
  const step = new Map();
  const q = [start];
  for (let qi = 0; qi < q.length; qi++) {
    const u = q[qi];
    for (const [dx, dy, ch] of DIRS) {
      const x = (u % lv.w) + dx;
      const y = Math.floor(u / lv.w) + dy;
      if (x < 0 || y < 0 || x >= lv.w || y >= lv.h) continue;
      const v = y * lv.w + x;
      if (lv.wall[v] || boxSet.has(v) || prev.has(v)) continue;
      if (allowed && !allowed.has(v)) continue;
      prev.set(v, u);
      step.set(v, ch);
      if (v === target) {
        let s = '';
        for (let t = v; t !== start; t = prev.get(t)) s = step.get(t) + s;
        return s;
      }
      q.push(v);
    }
  }
  return null;
}

// Flood fill of player-reachable cells; returns {set, prev, step} for walks.
function flood(lv, start, boxSet, allowed) {
  const prev = new Map([[start, -1]]);
  const step = new Map();
  const q = [start];
  for (let qi = 0; qi < q.length; qi++) {
    const u = q[qi];
    for (const [dx, dy, ch] of DIRS) {
      const x = (u % lv.w) + dx;
      const y = Math.floor(u / lv.w) + dy;
      if (x < 0 || y < 0 || x >= lv.w || y >= lv.h) continue;
      const v = y * lv.w + x;
      if (lv.wall[v] || boxSet.has(v) || prev.has(v)) continue;
      if (allowed && !allowed.has(v)) continue;
      prev.set(v, u);
      step.set(v, ch);
      q.push(v);
    }
  }
  return { set: new Set(q), prev, step };
}

function walkFromFlood(fl, start, target) {
  let s = '';
  for (let t = target; t !== start; t = fl.prev.get(t)) s = fl.step.get(t) + s;
  return s;
}

// Reverse-pull distances: for each goal, the minimum pushes to bring one box
// from every cell to that goal (other boxes ignored). INF = dead square.
function goalDistances(lv, boxFloor, standFloor, goals) {
  const maps = [];
  for (const goal of goals) {
    const dist = new Array(lv.w * lv.h).fill(INF);
    dist[goal] = 0;
    const q = [goal];
    for (let qi = 0; qi < q.length; qi++) {
      const b = q[qi];
      for (const [dx, dy] of DIRS) {
        const ax = (b % lv.w) - dx;
        const ay = Math.floor(b / lv.w) - dy;
        const sx = ax - dx;
        const sy = ay - dy;
        if (ax < 0 || ay < 0 || ax >= lv.w || ay >= lv.h) continue;
        if (sx < 0 || sy < 0 || sx >= lv.w || sy >= lv.h) continue;
        const a = ay * lv.w + ax;
        const s = sy * lv.w + sx;
        if (!boxFloor.has(a) || !standFloor.has(s) || dist[a] !== INF) continue;
        dist[a] = dist[b] + 1;
        q.push(a);
      }
    }
    maps.push(dist);
  }
  return maps;
}

// Minimum assignment cost boxes->goals (permutations for small n, else the
// sum of per-box minima, still a lower bound). INF when a box is dead.
function heuristic(boxes, maps) {
  const n = boxes.length;
  if (n === 0) return 0;
  const cost = boxes.map((b) => maps.map((m) => m[b]));
  for (const row of cost) if (Math.min(...row) >= INF) return INF;
  if (n > 7) return cost.reduce((a, row) => a + Math.min(...row), 0);
  let best = INF;
  const used = new Array(n).fill(false);
  const rec = (i, acc) => {
    if (acc >= best) return;
    if (i === n) {
      best = acc;
      return;
    }
    for (let j = 0; j < n; j++) {
      if (used[j] || cost[i][j] >= INF) continue;
      used[j] = true;
      rec(i + 1, acc + cost[i][j]);
      used[j] = false;
    }
  };
  rec(0, 0);
  return best;
}

// A 2x2 block of walls/boxes with a box off goal can never be undone.
function has2x2Deadlock(lv, boxSet, goalSet) {
  for (const b of boxSet) {
    const x = b % lv.w;
    const y = Math.floor(b / lv.w);
    for (const [ox, oy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]]) {
      const cells = [];
      let ok = true;
      let offGoal = false;
      for (let dy = 0; dy < 2 && ok; dy++) {
        for (let dx = 0; dx < 2; dx++) {
          const cx = x + ox + dx;
          const cy = y + oy + dy;
          if (cx < 0 || cy < 0 || cx >= lv.w || cy >= lv.h) { ok = false; break; }
          const c = cy * lv.w + cx;
          cells.push(c);
          if (!lv.wall[c] && !boxSet.has(c)) { ok = false; break; }
          if (boxSet.has(c) && !goalSet.has(c)) offGoal = true;
        }
      }
      if (ok && offGoal) return true;
    }
  }
  return false;
}

// Push-based A*. boxes/player are cell indexes; boxFloor = cells a box may
// occupy, standFloor = cells the player may stand on. exitCell (or -1): the
// player must be able to walk there once every box is on a goal.
// Returns {moves, finalPlayer, finalBoxes} or null.
function solvePushes(lv, startBoxes, startPlayer, boxFloor, standFloor, goals, exitCell, maxExpansions) {
  const goalSet = new Set(goals);
  const maps = goalDistances(lv, boxFloor, standFloor, goals);
  const h0 = heuristic(startBoxes, maps);
  if (h0 >= INF) return null;
  const boxesList = [startBoxes.slice()];
  const playerList = [startPlayer];
  const gList = [0];
  const parent = [-1];
  const segment = [''];
  const repOf = (boxes, player) => {
    const fl = flood(lv, player, new Set(boxes), standFloor);
    let rep = player;
    for (const c of fl.set) if (c < rep) rep = c;
    return rep;
  };
  const keyOf = (boxes, player) => boxes.join(',') + '|' + repOf(boxes, player);
  const best = new Map([[keyOf(startBoxes, startPlayer), 0]]);
  const heap = [[h0, h0, 0, 0]];
  let counter = 1;
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const hpop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };
  let expansions = 0;
  while (heap.length > 0 && expansions < maxExpansions) {
    const [, , , s] = hpop();
    const boxes = boxesList[s];
    const player = playerList[s];
    const g = gList[s];
    if (best.get(keyOf(boxes, player)) !== s) continue;
    const boxSet = new Set(boxes);
    const fl = flood(lv, player, boxSet, standFloor);
    if (boxes.every((b) => goalSet.has(b)) && (exitCell < 0 || fl.set.has(exitCell))) {
      let moves = '';
      for (let t = s; t > 0; t = parent[t]) moves = segment[t] + moves;
      return { moves, finalPlayer: player, finalBoxes: boxes.slice() };
    }
    expansions++;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      const bx = b % lv.w;
      const by = Math.floor(b / lv.w);
      for (const [dx, dy, ch] of DIRS) {
        const sx = bx - dx;
        const sy = by - dy;
        const tx = bx + dx;
        const ty = by + dy;
        if (sx < 0 || sy < 0 || sx >= lv.w || sy >= lv.h) continue;
        if (tx < 0 || ty < 0 || tx >= lv.w || ty >= lv.h) continue;
        const stand = sy * lv.w + sx;
        const dest = ty * lv.w + tx;
        if (!fl.set.has(stand)) continue;
        if (!boxFloor.has(dest) || boxSet.has(dest)) continue;
        const nb = boxes.slice();
        nb[bi] = dest;
        nb.sort((a, c) => a - c);
        const nbSet = new Set(nb);
        if (has2x2Deadlock(lv, nbSet, goalSet)) continue;
        const hh = heuristic(nb, maps);
        if (hh >= INF) continue;
        const nplayer = b;
        const k = keyOf(nb, nplayer);
        const seen = best.get(k);
        if (seen !== undefined && gList[seen] <= g + 1) continue;
        const t = boxesList.length;
        boxesList.push(nb);
        playerList.push(nplayer);
        gList.push(g + 1);
        parent.push(s);
        segment.push(walkFromFlood(fl, player, stand) + ch.toUpperCase());
        best.set(k, t);
        hpush([g + 1 + hh, hh, counter++, t]);
      }
    }
  }
  return null;
}

// Replay a move string on the level; returns the push count, or -1 when the
// replay is illegal or does not end solved.
function replayPushes(lv, moves) {
  const boxes = new Set(lv.boxes);
  const goalSet = new Set(lv.goals);
  let p = lv.player;
  let pushes = 0;
  const dirOf = { u: [0, -1], d: [0, 1], l: [-1, 0], r: [1, 0] };
  for (const raw of moves) {
    const ch = raw.toLowerCase();
    if (!(ch in dirOf)) return -1;
    const [dx, dy] = dirOf[ch];
    const x = (p % lv.w) + dx;
    const y = Math.floor(p / lv.w) + dy;
    if (x < 0 || y < 0 || x >= lv.w || y >= lv.h) return -1;
    const q = y * lv.w + x;
    if (lv.wall[q]) return -1;
    if (boxes.has(q)) {
      const x2 = x + dx;
      const y2 = y + dy;
      if (x2 < 0 || y2 < 0 || x2 >= lv.w || y2 >= lv.h) return -1;
      const q2 = y2 * lv.w + x2;
      if (lv.wall[q2] || boxes.has(q2)) return -1;
      boxes.delete(q);
      boxes.add(q2);
      pushes++;
    }
    p = q;
  }
  for (const b of boxes) if (!goalSet.has(b)) return -1;
  return boxes.size === goalSet.size ? pushes : -1;
}

function solveLevel(text) {
  const lv = parse(text);
  const floor = new Set();
  for (let i = 0; i < lv.w * lv.h; i++) if (!lv.wall[i]) floor.add(i);
  // Try the hallway/rooms decomposition first.
  const hallwayY = Math.floor(lv.player / lv.w);
  const rest = new Set([...floor].filter((c) => Math.floor(c / lv.w) !== hallwayY));
  const comps = [];
  const seenCells = new Set();
  for (const c of [...rest].sort((a, b) => a - b)) {
    if (seenCells.has(c)) continue;
    const comp = [];
    const stack = [c];
    seenCells.add(c);
    while (stack.length > 0) {
      const u = stack.pop();
      comp.push(u);
      const ux = u % lv.w;
      const uy = Math.floor(u / lv.w);
      for (const [dx, dy] of DIRS) {
        const vx = ux + dx;
        const vy = uy + dy;
        if (vx < 0 || vy < 0 || vx >= lv.w || vy >= lv.h) continue;
        const v = vy * lv.w + vx;
        if (rest.has(v) && !seenCells.has(v)) {
          seenCells.add(v);
          stack.push(v);
        }
      }
    }
    comps.push(new Set(comp));
  }
  const rooms = [];
  let decomposes = lv.boxes.every((b) => Math.floor(b / lv.w) !== hallwayY)
    && lv.goals.every((g) => Math.floor(g / lv.w) !== hallwayY);
  if (decomposes) {
    for (const comp of comps) {
      const rb = lv.boxes.filter((b) => comp.has(b));
      const rg = lv.goals.filter((g) => comp.has(g));
      if (rb.length === 0 && rg.length === 0) continue;
      if (rb.length !== rg.length) { decomposes = false; break; }
      const doors = [...comp].filter((c) => {
        const cy = Math.floor(c / lv.w);
        return Math.abs(cy - hallwayY) === 1 && floor.has(hallwayY * lv.w + (c % lv.w));
      });
      if (doors.length !== 1) { decomposes = false; break; }
      const door = doors[0];
      const entry = hallwayY * lv.w + (door % lv.w);
      rooms.push({ comp, boxes: rb, goals: rg, door, entry });
    }
    if (rooms.length === 0) decomposes = false;
    const covered = rooms.reduce((a, r) => a + r.boxes.length, 0);
    if (covered !== lv.boxes.length) decomposes = false;
  }
  if (decomposes) {
    rooms.sort((a, b) => (a.door % lv.w) - (b.door % lv.w) || a.door - b.door);
    let moves = '';
    let curPlayer = lv.player;
    const curBoxes = new Set(lv.boxes);
    let ok = true;
    for (const room of rooms) {
      const toEntry = walkPath(lv, curPlayer, room.entry, curBoxes, null);
      if (toEntry === null) { ok = false; break; }
      moves += toEntry;
      const standFloor = new Set(room.comp);
      standFloor.add(room.entry);
      const sol = solvePushes(lv, room.boxes, room.entry, room.comp, standFloor, room.goals, room.entry, 600000);
      if (sol === null) { ok = false; break; }
      moves += sol.moves;
      for (const b of room.boxes) curBoxes.delete(b);
      for (const b of sol.finalBoxes) curBoxes.add(b);
      const exitWalk = walkPath(lv, sol.finalPlayer, room.entry, curBoxes, standFloor);
      if (exitWalk === null) { ok = false; break; }
      moves += exitWalk;
      curPlayer = room.entry;
    }
    if (ok && replayPushes(lv, moves) >= 0) return moves;
  }
  // Fallback: whole-level push A*.
  const sol = solvePushes(lv, lv.boxes, lv.player, floor, floor, lv.goals, -1, 1200000);
  if (sol !== null && replayPushes(lv, sol.moves) >= 0) return sol.moves;
  return '';
}

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
