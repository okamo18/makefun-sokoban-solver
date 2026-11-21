// Sokoban solver for make.fun's sokoban-solver-01-run-20261121.
// One program answers 5 levels in one run. Each level is a hallway with
// 1-4 rooms off it; a box pushed out of its room can never come back, so
// every room is solved independently and push-optimally, and the level's
// optimum is the sum of its rooms' optima. Rooms are found by removing the
// hallway row (the row the player starts on) from the floor graph: each
// remaining component is a room with one door onto the hallway.
// Inside a room, a push-based A* (macro pushes) runs with a Hungarian
// assignment heuristic over push distances, dead-square pruning (reverse
// pull reachability from the goals) and 2x2 freeze-deadlock pruning.
// No wall clock, no randomness: every budget is counted in expansions, so
// two runs write identical bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS: { dx: number; dy: number; ch: string }[] = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

// Budgets are counted in node expansions, never in wall time, so the run
// is deterministic.
const ROOM_EXPANSION_CAP = 150000;
const GLOBAL_EXPANSION_CAP = 1200000;
let expansionsUsed = 0;

function parseLevel(text: string): {
  w: number; h: number; wall: boolean[]; goals: number[]; boxes: number[]; player: number;
} {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall: boolean[] = new Array(w * h).fill(true);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = false;
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// Breadth-first walk over `allowed` floor cells (not in `blocked`), from
// `from` to `to`. Returns the move letters, or null when unreachable.
// Neighbour order is fixed, so the path is deterministic.
function walkPath(
  w: number, allowed: Set<number>, blocked: Set<number>, from: number, to: number,
): string | null {
  if (from === to) return '';
  const prev = new Map<number, { p: number; ch: string }>();
  const queue: number[] = [from];
  prev.set(from, { p: -1, ch: '' });
  for (let qi = 0; qi < queue.length; qi++) {
    const cur = queue[qi]!;
    const cx = cur % w;
    const cy = Math.floor(cur / w);
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (!allowed.has(n) || blocked.has(n) || prev.has(n)) continue;
      prev.set(n, { p: cur, ch: d.ch });
      if (n === to) {
        let s = '';
        let t = n;
        while (t !== from) {
          const step = prev.get(t)!;
          s = step.ch + s;
          t = step.p;
        }
        return s;
      }
      queue.push(n);
    }
  }
  return null;
}

// Reachable floor cells for the player from `from`, over `cells`, avoiding
// the squares in `boxSet`. Returns the set and its smallest cell.
function reachable(
  w: number, cells: Set<number>, boxSet: Set<number>, from: number,
): { set: Set<number>; rep: number } {
  const set = new Set<number>([from]);
  const queue: number[] = [from];
  let rep = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const cur = queue[qi]!;
    if (cur < rep) rep = cur;
    const cx = cur % w;
    const cy = Math.floor(cur / w);
    for (const d of DIRS) {
      const nx = cx + d.dx;
      const ny = cy + d.dy;
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (!cells.has(n) || boxSet.has(n) || set.has(n)) continue;
      set.add(n);
      queue.push(n);
    }
  }
  return { set, rep };
}

type Region = {
  // Cells the player may walk on: the room's cells plus the hallway square
  // outside its door (a box on the door is pushed in from there).
  cells: Set<number>;
  // Cells a box may occupy: the room's cells only.
  boxCells: Set<number>;
  boxes: number[];
  goals: number[];
  startPlayer: number; // the hallway square outside the door
  exitCell: number | null; // door square: the final player region must reach it
};

type SolveResult = { moves: string; endPlayer: number; endBoxes: number[] };

// Push-optimal A* for one region (a room, or the whole level as fallback).
// Returns null when the expansion cap is hit or no solution exists.
function solveRegion(w: number, wall: boolean[], region: Region): SolveResult | null {
  const { cells, boxCells, goals } = region;
  const n = region.boxes.length;
  if (n === 0) return { moves: '', endPlayer: region.startPlayer, endBoxes: [] };
  if (goals.length !== n) return null;
  const goalSet = new Set(goals);

  // Push distances: BFS from each goal over the cells a box may occupy.
  const distFromGoal: Int32Array[] = goals.map((g) => {
    const dist = new Int32Array(wall.length).fill(-1);
    dist[g] = 0;
    const queue: number[] = [g];
    for (let qi = 0; qi < queue.length; qi++) {
      const cur = queue[qi]!;
      const cx = cur % w;
      const cy = Math.floor(cur / w);
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0) continue;
        const nn = ny * w + nx;
        if (!boxCells.has(nn) || dist[nn] !== -1) continue;
        dist[nn] = dist[cur]! + 1;
        queue.push(nn);
      }
    }
    return dist;
  });

  // Dead squares: reverse pull reachability from the goals. A box on a
  // square outside this set can never reach a goal.
  const alive = new Array<boolean>(wall.length).fill(false);
  {
    const queue: number[] = [];
    for (const g of goals) {
      alive[g] = true;
      queue.push(g);
    }
    for (let qi = 0; qi < queue.length; qi++) {
      const p = queue[qi]!;
      const px = p % w;
      const py = Math.floor(p / w);
      for (const d of DIRS) {
        // The box at p could have been pushed from q = p - d, which needs
        // the player to have stood at q - d: any walkable cell, including
        // the hallway square outside the door.
        const qx = px - d.dx;
        const qy = py - d.dy;
        const bx = px - 2 * d.dx;
        const by = py - 2 * d.dy;
        if (qx < 0 || qy < 0 || bx < 0 || by < 0) continue;
        const q = qy * w + qx;
        const b = by * w + bx;
        if (!boxCells.has(q) || !cells.has(b) || alive[q]) continue;
        alive[q] = true;
        queue.push(q);
      }
    }
  }

  // 2x2 freeze blocks: four cells, no goal among them, every cell a wall or
  // a box, at least one box -> the boxes in it can never move.
  const freezeBlocks: number[][] = [];
  {
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (const c of boxCells) {
      const x = c % w;
      const y = Math.floor(c / w);
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    for (let y = minY - 1; y <= maxY; y++) {
      for (let x = minX - 1; x <= maxX; x++) {
        const block = [y * w + x, y * w + x + 1, (y + 1) * w + x, (y + 1) * w + x + 1];
        if (block.some((c) => goalSet.has(c))) continue;
        if (!block.some((c) => boxCells.has(c))) continue;
        freezeBlocks.push(block);
      }
    }
  }
  const isFrozen = (boxSet: Set<number>): boolean => {
    for (const block of freezeBlocks) {
      let boxCount = 0;
      let allWallOrBox = true;
      for (const c of block) {
        if (boxSet.has(c)) boxCount++;
        else if (cells.has(c)) {
          // A walkable cell with no box on it: the box could be pushed
          // from here, so the block is not frozen. (Walkable includes the
          // hallway square outside the door, where the player stands to
          // push a box on the door into the room.)
          allWallOrBox = false;
          break;
        }
      }
      if (boxCount > 0 && allWallOrBox) return true;
    }
    return false;
  };

  // Heuristic: minimum assignment of boxes to goals over push distances.
  // Exact (permutations) for small regions, sum of nearest goals otherwise.
  // Both are admissible: they ignore the other boxes and the player.
  const heuristic = (boxes: number[]): number => {
    if (n <= 6) {
      const perm = goals.map((_, i) => i);
      let best = Infinity;
      const rec = (k: number, acc: number): void => {
        if (acc >= best) return;
        if (k === n) {
          best = acc;
          return;
        }
        for (let i = k; i < n; i++) {
          const tmp = perm[k]!;
          perm[k] = perm[i]!;
          perm[i] = tmp;
          const d = distFromGoal[perm[k]!]![boxes[k]!]!;
          if (d !== -1) rec(k + 1, acc + d);
          const tmp2 = perm[k]!;
          perm[k] = perm[i]!;
          perm[i] = tmp2;
        }
      };
      rec(0, 0);
      return best;
    }
    let sum = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (const dg of distFromGoal) {
        const d = dg[b]!;
        if (d !== -1 && d < m) m = d;
      }
      if (m === Infinity) return Infinity;
      sum += m;
    }
    return sum;
  };

  const startBoxes = region.boxes.slice().sort((a, b) => a - b);
  const startBoxSet = new Set(startBoxes);
  if (startBoxes.some((b) => !alive[b])) return null;
  if (isFrozen(startBoxSet)) return null;

  // A* node storage.
  const nodeBoxes: number[][] = [startBoxes];
  const nodePlayer: number[] = [region.startPlayer];
  const nodeParent: number[] = [-1];
  const nodeG: number[] = [0];
  const nodePush: { boxFrom: number; dir: number }[] = [{ boxFrom: -1, dir: -1 }];
  const nodeF: number[] = [heuristic(startBoxes)];
  // Binary heap of node indices, ordered by (f, -g, insertion order).
  const heap: number[] = [0];
  let seq = 0;
  const seqOf = new Map<number, number>([[0, seq++]]);
  const less = (a: number, b: number): boolean => {
    if (nodeF[a] !== nodeF[b]) return nodeF[a]! < nodeF[b]!;
    if (nodeG[a] !== nodeG[b]) return nodeG[a]! > nodeG[b]!;
    return seqOf.get(a)! < seqOf.get(b)!;
  };
  const heapPush = (v: number): void => {
    let i = heap.length;
    heap.push(v);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      const t = heap[i]!;
      heap[i] = heap[p]!;
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        const t = heap[i]!;
        heap[i] = heap[c]!;
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };

  const startReach = reachable(w, cells, startBoxSet, region.startPlayer);
  const best = new Map<string, number>([
    [startBoxes.join(',') + '#' + startReach.rep, 0],
  ]);
  let expanded = 0;

  while (heap.length > 0) {
    if (expanded >= ROOM_EXPANSION_CAP || expansionsUsed >= GLOBAL_EXPANSION_CAP) return null;
    const s = heapPop();
    const boxes = nodeBoxes[s]!;
    const g = nodeG[s]!;
    const player = nodePlayer[s]!;
    const boxSet = new Set(boxes);
    const reach = reachable(w, cells, boxSet, player);
    const key = boxes.join(',') + '#' + reach.rep;
    if (best.get(key) !== g) continue;
    // Goal: every box on a goal, and the player able to reach the exit.
    if (boxes.every((b) => goalSet.has(b))) {
      if (region.exitCell === null || reach.set.has(region.exitCell)) {        // Reconstruct the move string.
        const pushes: { fromPlayer: number; boxFrom: number; dir: number; boxesBefore: number[] }[] = [];
        for (let t = s; t > 0; t = nodeParent[t]!) {
          pushes.push({
            fromPlayer: nodePlayer[nodeParent[t]!]!,
            boxFrom: nodePush[t]!.boxFrom,
            dir: nodePush[t]!.dir,
            boxesBefore: nodeBoxes[nodeParent[t]!]!,
          });
        }
        pushes.reverse();
        let moves = '';
        for (const pu of pushes) {
          const d = DIRS[pu.dir]!;
          const behind = pu.boxFrom - (d.dy * w + d.dx);
          const walk = walkPath(w, cells, new Set(pu.boxesBefore), pu.fromPlayer, behind);
          if (walk === null) return null; // should not happen
          moves += walk + d.ch.toUpperCase();
        }
        return { moves, endPlayer: player, endBoxes: boxes };
      }
    }
    expanded++;
    expansionsUsed++;
    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi]!;
      const bx = b % w;
      const by = Math.floor(b / w);
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di]!;
        const tx = bx + d.dx;
        const ty = by + d.dy;
        if (tx < 0 || ty < 0) continue;
        const target = ty * w + tx;
        const behind = b - (d.dy * w + d.dx);
        if (!boxCells.has(target) || boxSet.has(target)) continue;
        if (!reach.set.has(behind)) continue;
        if (!alive[target]) continue;
        const nb = boxes.slice();
        nb[bi] = target;
        nb.sort((a, b2) => a - b2);
        const nbSet = new Set(nb);
        if (isFrozen(nbSet)) continue;
        const h = heuristic(nb);
        if (h === Infinity) continue;
        const nReach = reachable(w, cells, nbSet, b);
        const nKey = nb.join(',') + '#' + nReach.rep;
        const ng = g + 1;
        if (best.has(nKey) && best.get(nKey)! <= ng) continue;
        best.set(nKey, ng);
        const idx = nodeBoxes.length;
        nodeBoxes.push(nb);
        nodePlayer.push(b);
        nodeParent.push(s);
        nodeG.push(ng);
        nodePush.push({ boxFrom: b, dir: di });
        nodeF.push(ng + h);
        seqOf.set(idx, seq++);
        heapPush(idx);
      }
    }
  }
  return null;
}

function solveLevel(text: string): string {
  const { w, h, wall, goals, boxes, player } = parseLevel(text);
  const hallwayRow = Math.floor(player / w);
  // Rooms: connected components of floor cells, excluding the hallway row.
  const inRoom = new Array<boolean>(w * h).fill(false);
  const rooms: { cells: Set<number>; boxes: number[]; goals: number[]; door: number }[] = [];
  const seen = new Set<number>();
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || Math.floor(i / w) === hallwayRow || seen.has(i)) continue;
    const cells = new Set<number>([i]);
    seen.add(i);
    const queue: number[] = [i];
    for (let qi = 0; qi < queue.length; qi++) {
      const cur = queue[qi]!;
      const cx = cur % w;
      const cy = Math.floor(cur / w);
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || ny < 0 || ny >= h) continue;
        const nn = ny * w + nx;
        if (ny === hallwayRow || wall[nn] || seen.has(nn)) continue;
        seen.add(nn);
        cells.add(nn);
        queue.push(nn);
      }
    }
    const rBoxes = boxes.filter((b) => cells.has(b));
    const rGoals = goals.filter((g) => cells.has(g));
    let door = -1;
    for (const c of cells) {
      const cy = Math.floor(c / w);
      const ny = cy + (cy < hallwayRow ? 1 : -1);
      if (ny === hallwayRow && !wall[ny * w + (c % w)]) {
        door = c;
        break;
      }
    }
    if (door === -1) return '';
    rooms.push({ cells, boxes: rBoxes, goals: rGoals, door });
    for (const c of cells) inRoom[c] = true;
  }
  // Any box or goal outside a room means the layout is not the expected
  // hallway-and-rooms shape: fall back to solving the whole level at once.
  const outside = boxes.some((b) => !inRoom[b]) || goals.some((g) => !inRoom[g]);
  const allFloor = new Set<number>();
  for (let i = 0; i < w * h; i++) if (!wall[i]) allFloor.add(i);
  if (outside || rooms.length === 0) {
    const res = solveRegion(w, wall, {
      cells: allFloor, boxCells: allFloor, boxes, goals,
      startPlayer: player, exitCell: null,
    });
    return res === null ? '' : res.moves;
  }

  rooms.sort((a, b) => a.door - b.door);
  let moves = '';
  let curPlayer = player;
  const curBoxes = new Set(boxes);
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri]!;
    const door = room.door;
    const doorY = Math.floor(door / w);
    const hallCell = (doorY < hallwayRow ? doorY + 1 : doorY - 1) * w + (door % w);
    // Walk through the hallway to the square outside the door. The room
    // solution starts from there (it pushes a box on the door in, if any).
    const toDoor = walkPath(w, allFloor, curBoxes, curPlayer, hallCell);
    if (toDoor === null) return '';
    moves += toDoor;
    curPlayer = hallCell;
    // Solve the room, requiring the player to be able to leave afterwards;
    // retry without that requirement if it fails.
    const regionCells = new Set(room.cells);
    regionCells.add(hallCell);
    let res = solveRegion(w, wall, {
      cells: regionCells, boxCells: room.cells, boxes: room.boxes,
      goals: room.goals, startPlayer: hallCell, exitCell: door,
    });
    if (res === null) {
      res = solveRegion(w, wall, {
        cells: regionCells, boxCells: room.cells, boxes: room.boxes,
        goals: room.goals, startPlayer: hallCell, exitCell: null,
      });
    }
    if (res === null) return '';
    moves += res.moves;
    for (const b of room.boxes) curBoxes.delete(b);
    for (const b of res.endBoxes) curBoxes.add(b);
    curPlayer = res.endPlayer;
    if (ri < rooms.length - 1) {
      // Walk out through the door back into the hallway.
      const out = walkPath(w, allFloor, curBoxes, curPlayer, door);
      if (out === null) return '';
      moves += out;
      const stepOut = DIRS.find((d) => (Math.floor(door / w) + d.dy) * w + ((door % w) + d.dx) === hallCell)!;
      moves += stepOut.ch;
      curPlayer = hallCell;
    }
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
