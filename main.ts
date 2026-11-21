// Push-optimal Sokoban solver for make.fun sokoban-solver-01.
//
// Strategy (from the competition's own description of its levels): every
// level is a 1-wide hallway with 1-4 small rooms off it, one door each; a
// box pushed out of a room into the hallway can never come back, so each
// room is an independent problem and the level optimum is the sum of the
// rooms' optima. Each room (3-4 boxes, ~6x6 floor) is solved push-optimally
// with A* over pushes (player walks between pushes are free), an admissible
// assignment heuristic, static dead-square pruning and 2x2 freeze pruning.
// The per-room push sequences are then joined with hallway walks. A room
// that A* cannot finish within budget gets a greedy best-first retry at the
// same budget (a suboptimal solve still scores, a failed room scores 0);
// if both fail, the level is left unsolved rather than burning the set's
// remaining budget on a whole-level search, which cannot succeed where the
// per-room searches failed.
//
// Determinism: budgets are counted in expanded states, never in wall time;
// no clock, no randomness, fixed neighbour order everywhere, so two runs
// write the same bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets, counted in expanded states (deterministic, clock-free).
const ASTAR_BUDGET = 400_000;
// Greedy (best-first) fallback gets the same budget as A*: it is the last
// chance at a valid, if suboptimal, room solution, and a suboptimal solve
// still earns 500*P*/pushes where a failed room earns 0 for the level.
const GREEDY_BUDGET = 400_000;
const GLOBAL_BUDGET = 4_000_000;

let globalExpansions = 0;

// Directions in fixed order: up, down, left, right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = rows[y][x] ?? '#';
      const i = y * w + x;
      if (ch !== '#') wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS distances from `from` over cells where passable[c] is 1. -1 = unreachable.
function bfsDist(w, h, passable, from) {
  const dist = new Int32Array(w * h).fill(-1);
  const q = [from];
  dist[from] = 0;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!passable[n] || dist[n] >= 0) continue;
      dist[n] = dist[c] + 1;
      q.push(n);
    }
  }
  return dist;
}

// Shortest player walk from `from` to `to` over passable cells avoiding
// `blocked` (boxes). Returns the move letters, or null when unreachable.
function bfsPath(w, h, passable, blocked, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(w * h).fill(-1);
  const prevDir = new Int8Array(w * h).fill(-1);
  const q = [from];
  prev[from] = from;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!passable[n] || blocked[n] || prev[n] >= 0) continue;
      prev[n] = c;
      prevDir[n] = d;
      if (n === to) {
        let moves = '';
        for (let t = n; t !== from; t = prev[t]) moves = LETTER[prevDir[t]] + moves;
        return moves;
      }
      q.push(n);
    }
  }
  return null;
}

// Cells the player can reach from `from` over passable cells avoiding boxes.
function reachable(w, h, passable, blocked, from) {
  const seen = new Uint8Array(w * h);
  const q = [from];
  seen[from] = 1;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!passable[n] || blocked[n] || seen[n]) continue;
      seen[n] = 1;
      q.push(n);
    }
  }
  return seen;
}

// Static dead squares for a room: floor cells from which a box can never be
// pushed onto a goal, ignoring other boxes. A box moves c -> m only when m
// is room floor and the square behind c (opposite m) is floor for the player
// to stand on; a cell is live when it reaches a goal in that directed graph.
function deadSquares(w, h, roomFloor, floorPlus, goals) {
  const live = new Uint8Array(w * h);
  const q = [];
  for (const g of goals) {
    live[g] = 1;
    q.push(g);
  }
  for (let qi = 0; qi < q.length; qi++) {
    const m = q[qi];
    const mx = m % w;
    const my = (m / w) | 0;
    // Reverse edge m -> c: a forward push c -> m needs the square behind c.
    for (let d = 0; d < 4; d++) {
      const cx = mx - DX[d];
      const cy = my - DY[d];
      if (cx < 0 || cy < 0 || cx >= w || cy >= h) continue;
      const c = cy * w + cx;
      if (!roomFloor[c] || live[c]) continue;
      const bx = cx - DX[d];
      const by = cy - DY[d];
      if (bx < 0 || by < 0 || bx >= w || by >= h) continue;
      if (!floorPlus[by * w + bx]) continue;
      live[c] = 1;
      q.push(c);
    }
  }
  const dead = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) if (roomFloor[i] && !live[i]) dead[i] = 1;
  return dead;
}

function permutations(n) {
  const out = [];
  const a = [];
  for (let i = 0; i < n; i++) a.push(i);
  const gen = (k) => {
    if (k === 1) {
      out.push(a.slice());
      return;
    }
    for (let i = 0; i < k; i++) {
      gen(k - 1);
      const j = k % 2 === 0 ? i : 0;
      const t = a[j];
      a[j] = a[k - 1];
      a[k - 1] = t;
    }
  };
  gen(n);
  return out;
}

// Push search for one room (or, for the fallback, the whole level).
// floorPlus: cells the player may stand on (room cells + door entry cell).
// roomFloor: cells a box may occupy. goals: the room's goals.
// startBoxes: sorted box cells. startPlayer: entry cell.
// Returns { pushes: [{box, dir}], expansions } or null.
function pushSearch(w, h, wall, floorPlus, roomFloor, goals, startBoxes, startPlayer, budget, greedy) {
  const n = startBoxes.length;
  const goalSet = new Uint8Array(w * h);
  for (const g of goals) goalSet[g] = 1;
  const dead = deadSquares(w, h, roomFloor, floorPlus, goals);
  for (const b of startBoxes) if (dead[b]) return null;

  // distFromGoal[g][cell]: box steps from cell to goal g over room floor.
  const distFromGoal = goals.map((g) => bfsDist(w, h, roomFloor, g));
  const perms = permutations(n);
  const heuristic = (boxes) => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = distFromGoal[p[i]][boxes[i]];
        if (d < 0) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };

  // 2x2 freeze: a 2x2 block whose four cells are all wall-or-box, holding a
  // box that is not on a goal, can never move again (any push out of the
  // block needs the player to stand inside it, on a wall or a box).
  const occ2 = new Uint8Array(w * h);
  const isFrozen = (boxes) => {
    for (const b of boxes) occ2[b] = 1;
    let frozen = false;
    outer: for (const b of boxes) {
      const bx = b % w;
      const by = (b / w) | 0;
      for (let oy = -1; oy <= 0; oy++) {
        for (let ox = -1; ox <= 0; ox++) {
          const x0 = bx + ox;
          const y0 = by + oy;
          if (x0 < 0 || y0 < 0 || x0 + 1 >= w || y0 + 1 >= h) continue;
          const cells = [y0 * w + x0, y0 * w + x0 + 1, (y0 + 1) * w + x0, (y0 + 1) * w + x0 + 1];
          let allBlocked = true;
          let hasLooseBox = false;
          for (const c of cells) {
            if (occ2[c]) {
              if (!goalSet[c]) hasLooseBox = true;
            } else if (!wall[c]) {
              allBlocked = false;
            }
          }
          if (allBlocked && hasLooseBox) {
            frozen = true;
            break outer;
          }
        }
      }
    }
    for (const b of boxes) occ2[b] = 0;
    return frozen;
  };

  // State key: boxes + player packed in base (w*h), one digit per cell.
  // Base 256 would collide if a board ever held more than 256 cells; the
  // generator's boards are at most 15x17=255, but base (w*h) is injective
  // for any board size whose key stays under 2^53, so the key can never
  // silently merge two different states. Only equality is ever tested on
  // the key (heap order uses f/h/seq), so the search order is unchanged.
  const BASE = w * h;
  const packKey = (boxes, player) => {
    let k = player;
    for (let i = n - 1; i >= 0; i--) k = k * BASE + boxes[i];
    for (let i = n; i < 4; i++) k = k * BASE + 0;
    return k;
  };

  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;

  // Binary heap ordered by (f, h, seq). Elements: [f, h, seq, stateIdx].
  const heap = [];
  const heapLess = (a, b) =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1], heap[c])) c++;
        if (!heapLess(heap[c], heap[i])) break;
        const t = heap[i];
        heap[i] = heap[c];
        heap[c] = t;
        i = c;
      }
    }
    return top;
  };

  const sBoxes = [startBoxes.slice()];
  const sPlayer = [startPlayer];
  const sG = [0];
  const sParent = [-1];
  const sPushBox = [-1];
  const sPushDir = [-1];
  const bestG = new Map();
  bestG.set(packKey(startBoxes, startPlayer), 0);
  let seq = 0;
  heapPush([h0, h0, seq++, 0]);
  let expansions = 0;

  const occ = new Uint8Array(w * h);
  while (heap.length > 0) {
    if (expansions >= budget || globalExpansions >= GLOBAL_BUDGET) return null;
    const entry = heapPop();
    const si = entry[3];
    const g = sG[si];
    const boxes = sBoxes[si];
    const player = sPlayer[si];
    if (bestG.get(packKey(boxes, player)) !== g) continue;
    let solved = true;
    for (const b of boxes) if (!goalSet[b]) { solved = false; break; }
    if (solved) {
      const pushes = [];
      for (let t = si; t > 0; t = sParent[t]) pushes.push({ box: sPushBox[t], dir: sPushDir[t] });
      pushes.reverse();
      return { pushes, expansions };
    }
    expansions++;
    globalExpansions++;
    for (const b of boxes) occ[b] = 1;
    const reach = reachable(w, h, floorPlus, occ, player);
    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = bx + DX[d];
        const ny = by + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const dest = ny * w + nx;
        if (!roomFloor[dest] || occ[dest]) continue;
        const px = bx - DX[d];
        const py = by - DY[d];
        if (px < 0 || py < 0 || px >= w || py >= h) continue;
        if (!reach[py * w + px]) continue;
        const nb = boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        let bad = false;
        for (const q of nb) if (dead[q]) { bad = true; break; }
        if (bad) continue;
        if (isFrozen(nb)) continue;
        const ng = g + 1;
        const key = packKey(nb, b);
        const seen = bestG.get(key);
        if (seen !== undefined && seen <= ng) continue;
        const hh = heuristic(nb);
        if (hh === Infinity) continue;
        bestG.set(key, ng);
        const ti = sBoxes.length;
        sBoxes.push(nb);
        sPlayer.push(b);
        sG.push(ng);
        sParent.push(si);
        sPushBox.push(b);
        sPushDir.push(d);
        heapPush([greedy ? hh : ng + hh, hh, seq++, ti]);
      }
    }
    for (const b of boxes) occ[b] = 0;
  }
  return null;
}

// Decompose a level into hallway + rooms. Returns null when the layout does
// not match the expected shape (caller falls back to a whole-level search).
function decompose(level) {
  const { w, h, wall, goals, boxes, player } = level;
  const hallRow = (player / w) | 0;
  const floor = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) floor[i] = wall[i] ? 0 : 1;
  const hall = new Uint8Array(w * h);
  for (let x = 0; x < w; x++) if (floor[hallRow * w + x]) hall[hallRow * w + x] = 1;
  const seen = new Uint8Array(w * h);
  const rooms = [];
  for (let i = 0; i < w * h; i++) {
    if (!floor[i] || hall[i] || seen[i]) continue;
    const comp = [];
    const q = [i];
    seen[i] = 1;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      comp.push(c);
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nn = ny * w + nx;
        if (!floor[nn] || hall[nn] || seen[nn]) continue;
        seen[nn] = 1;
        q.push(nn);
      }
    }
    const roomSet = new Uint8Array(w * h);
    for (const c of comp) roomSet[c] = 1;
    const rBoxes = boxes.filter((b) => roomSet[b]);
    const rGoals = goals.filter((g) => roomSet[g]);
    const doors = [];
    for (const c of comp) {
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nn = ny * w + nx;
        if (hall[nn]) doors.push({ door: c, entry: nn });
      }
    }
    if (rBoxes.length === 0 || rBoxes.length !== rGoals.length) return null;
    if (doors.length !== 1) return null;
    rooms.push({ cells: comp, set: roomSet, boxes: rBoxes, goals: rGoals, door: doors[0].door, entry: doors[0].entry });
  }
  if (rooms.length === 0) return null;
  const total = rooms.reduce((a, r) => a + r.boxes.length, 0);
  if (total !== boxes.length) return null;
  rooms.sort((a, b) => a.door - b.door);
  return { hallRow, hall, rooms };
}

// Emit the move letters for a room's push list, starting with the player at
// `start` (the room's entry cell).
function emitRoomPushes(level, room, pushes, start) {
  const { w, h } = level;
  const floorPlus = new Uint8Array(w * h);
  for (const c of room.cells) floorPlus[c] = 1;
  floorPlus[room.entry] = 1;
  const boxes = room.boxes.slice();
  const occ = new Uint8Array(w * h);
  for (const b of boxes) occ[b] = 1;
  let player = start;
  let moves = '';
  for (const p of pushes) {
    const bi = boxes.indexOf(p.box);
    if (bi < 0) return null;
    const b = boxes[bi];
    const bx = b % w;
    const by = (b / w) | 0;
    const behind = (by - DY[p.dir]) * w + (bx - DX[p.dir]);
    const dest = (by + DY[p.dir]) * w + (bx + DX[p.dir]);
    const walk = bfsPath(w, h, floorPlus, occ, player, behind);
    if (walk === null) return null;
    moves += walk;
    moves += LETTER[p.dir];
    occ[b] = 0;
    occ[dest] = 1;
    boxes[bi] = dest;
    player = b;
  }
  return { moves, player };
}

function solveByRooms(level, dec) {
  const { w, h, player } = level;
  const hallPass = dec.hall;
  let moves = '';
  let cur = player;
  const occ = new Uint8Array(w * h);
  for (const room of dec.rooms) {
    const toEntry = bfsPath(w, h, hallPass, occ, cur, room.entry);
    if (toEntry === null) return null;
    moves += toEntry;
    cur = room.entry;
    const floorPlus = new Uint8Array(w * h);
    for (const c of room.cells) floorPlus[c] = 1;
    floorPlus[room.entry] = 1;
    let res = pushSearch(w, h, level.wall, floorPlus, room.set, room.goals, room.boxes, room.entry, ASTAR_BUDGET, false);
    if (!res) res = pushSearch(w, h, level.wall, floorPlus, room.set, room.goals, room.boxes, room.entry, GREEDY_BUDGET, true);
    if (!res) return null;
    const emitted = emitRoomPushes(level, room, res.pushes, cur);
    if (emitted === null) return null;
    moves += emitted.moves;
    cur = emitted.player;
    // Exit to the hallway through the door (the generator guarantees a
    // finished room never shuts the player in).
    for (const b of room.goals) occ[b] = 1;
    const exit = bfsPath(w, h, floorPlus, occ, cur, room.entry);
    if (exit === null) return null;
    moves += exit;
    cur = room.entry;
  }
  return moves;
}

// Emit moves for a whole-level push list (fallback path).
function emitPushes(level, pushes, start, floorPlus) {
  const { w, h } = level;
  const boxes = level.boxes.slice();
  const occ = new Uint8Array(w * h);
  for (const b of boxes) occ[b] = 1;
  let player = start;
  let moves = '';
  for (const p of pushes) {
    const bi = boxes.indexOf(p.box);
    if (bi < 0) return null;
    const b = boxes[bi];
    const bx = b % w;
    const by = (b / w) | 0;
    const behind = (by - DY[p.dir]) * w + (bx - DX[p.dir]);
    const dest = (by + DY[p.dir]) * w + (bx + DX[p.dir]);
    const walk = bfsPath(w, h, floorPlus, occ, player, behind);
    if (walk === null) return null;
    moves += walk;
    moves += LETTER[p.dir];
    occ[b] = 0;
    occ[dest] = 1;
    boxes[bi] = dest;
    player = b;
  }
  return moves;
}

// Solve one level. Returns the move string, or '' when unsolved.
function solveLevel(level) {
  const { w, h, wall, boxes, player } = level;
  if (boxes.every((b) => level.goals.includes(b))) return '';
  const dec = decompose(level);
  if (dec) {
    const solved = solveByRooms(level, dec);
    if (solved !== null) return solved;
    // Decomposition succeeded but a room failed: a whole-level search over
    // 8-16 boxes is the product of the rooms' state spaces and cannot
    // succeed where the per-room searches failed, so do not burn the
    // remaining global budget (and wall time) on it. Return '' for this
    // level rather than risking the whole set on a hopeless search.
    return '';
  }
  // No decomposition: the layout does not match the expected hallway+rooms
  // shape, so a whole-level search is the only chance.
  const floorPlus = new Uint8Array(w * h);
  const roomFloor = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) if (!wall[i]) { floorPlus[i] = 1; roomFloor[i] = 1; }
  const res = pushSearch(w, h, wall, floorPlus, roomFloor, level.goals, level.boxes, player, ASTAR_BUDGET, false)
    ?? pushSearch(w, h, wall, floorPlus, roomFloor, level.goals, level.boxes, player, GREEDY_BUDGET, true);
  if (!res) return '';
  return emitPushes(level, res.pushes, player, floorPlus) ?? '';
}

for (const id of readdirSync(I).sort()) {
  const text = readFileSync(join(I, id, 'level.txt'), 'utf8');
  const level = parseLevel(text);
  const moves = solveLevel(level);
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
