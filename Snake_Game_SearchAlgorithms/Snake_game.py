# robot_snake.py
# Robot collecteur style "Snake" (Tkinter) - 4 directions (cout=1), UCS / A* (H1/H2/H3/H4-MST),
# re-optimise apres chaque objet, apparitions dynamiques, retour a START, stats (expanded / temps).

import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from typing import List, Tuple, Dict, Set, Optional, Callable
from heapq import heappush, heappop
import time, math, random

Coord = Tuple[int, int]  # (row, col)

# Heuristiques (consistantes)

def h_manhattan(a: Coord, b: Coord) -> int:
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def h_euclidean(a: Coord, b: Coord) -> float:
    dr, dc = a[0]-b[0], a[1]-b[1]
    return math.sqrt(dr*dr + dc*dc)

def h_chebyshev(a: Coord, b: Coord) -> int:
    return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

HEURISTICS: Dict[str, Callable[[Coord, Coord], float]] = {
    "H1 Manhattan": h_manhattan,
    "H2 Euclidienne": h_euclidean,
    "H3 Chebyshev": h_chebyshev,
    # "H4 MST (multi-obj)" est geree au niveau de la selection de cible (cf. plus bas)
}
# Aides distances & MST

def manhattan_dist(a: Coord, b: Coord) -> int:
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def mst_cost_manhattan(points: List[Coord]) -> int:
    """
    Cout de l'arbre couvrant minimal (MST) sur 'points' avec distances de Manhattan (Prim).
    Si 0/1 point -> 0.
    """
    if not points or len(points) <= 1:
        return 0
    n = len(points)
    visited = [False]*n
    dist = [10**9]*n
    # Demarrer de 0
    dist[0] = 0
    total = 0
    for _ in range(n):
        # choisir le non-visite de plus petite dist
        u = -1
        best = 10**9
        for i in range(n):
            if not visited[i] and dist[i] < best:
                best = dist[i]; u = i
        visited[u] = True
        total += dist[u]
        # relax
        pu = points[u]
        for v in range(n):
            if not visited[v]:
                w = manhattan_dist(pu, points[v])
                if w < dist[v]:
                    dist[v] = w
    return total

# =========================
# Grille
# =========================
@dataclass
class Grid:
    rows: int
    cols: int
    walls: Set[Coord]

    def in_bounds(self, p: Coord) -> bool:
        r, c = p
        return 0 <= r < self.rows and 0 <= c < self.cols

    def passable(self, p: Coord) -> bool:
        return p not in self.walls

    def neighbors4(self, p: Coord) -> List[Coord]:
        r, c = p
        cand = [(r-1,c), (r+1,c), (r,c-1), (r,c+1)]
        return [q for q in cand if self.in_bounds(q) and self.passable(q)]

# =========================
# Recherche: UCS & A*
# =========================
@dataclass
class SearchResult:
    path: List[Coord]
    expanded: int
    plan_time_sec: float

def reconstruct(came: Dict[Coord, Coord], start: Coord, goal: Coord) -> List[Coord]:
    if goal != start and goal not in came:
        return []
    cur = goal
    path = [cur]
    while cur != start:
        cur = came[cur]
        path.append(cur)
    path.reverse()
    return path

def ucs(grid: Grid, start: Coord, goal: Coord) -> SearchResult:
    t0 = time.perf_counter()
    pq = []
    heappush(pq, (0, start))
    came: Dict[Coord, Coord] = {}
    cost: Dict[Coord, int] = {start: 0}
    closed: Set[Coord] = set()
    expanded = 0

    while pq:
        g, u = heappop(pq)
        if u in closed:
            continue
        closed.add(u); expanded += 1
        if u == goal:
            return SearchResult(reconstruct(came, start, goal), expanded, time.perf_counter()-t0)
        for v in grid.neighbors4(u):
            ng = g + 1
            if v not in cost or ng < cost[v]:
                cost[v] = ng
                came[v] = u
                heappush(pq, (ng, v))
    return SearchResult([], expanded, time.perf_counter()-t0)

def astar(grid: Grid, start: Coord, goal: Coord, h) -> SearchResult:
    t0 = time.perf_counter()
    pq = []
    heappush(pq, (0, 0, start))  # (f, g, node)
    came: Dict[Coord, Coord] = {}
    gscore: Dict[Coord, float] = {start: 0.0}
    closed: Set[Coord] = set()
    expanded = 0

    while pq:
        f, g, u = heappop(pq)
        if u in closed:
            continue
        closed.add(u); expanded += 1
        if u == goal:
            return SearchResult(reconstruct(came, start, goal), expanded, time.perf_counter()-t0)
        for v in grid.neighbors4(u):
            ng = g + 1
            if v in closed and ng >= gscore.get(v, float("inf")):
                continue
            if ng < gscore.get(v, float("inf")):
                gscore[v] = ng
                came[v] = u
                heappush(pq, (ng + h(v, goal), ng, v))
    return SearchResult([], expanded, time.perf_counter()-t0)

# =========================
# Outils collecte
# =========================
@dataclass
class RunStats:
    expanded_total: int = 0
    plan_time_total: float = 0.0
    steps_moved: int = 0

def best_target_and_path(grid: Grid,
                         start: Coord,
                         targets: List[Coord],
                         planner: Callable[[Grid, Coord, Coord], SearchResult],
                         use_mst_scoring: bool) -> Tuple[Optional[Coord], SearchResult]:
    """
    Si use_mst_scoring=True : on choisit la cible qui minimise
    (longueur_chemin(start->t)) + MST des autres cibles restantes (Manhattan).
    Sinon : on choisit la cible au plus court chemin (puis moins d'expansions / temps).
    """
    best_t = None
    best_res = SearchResult([], 0, 0.0)
    best_score = float("inf")
    best_len = float("inf")
    best_exp = float("inf")
    best_time = float("inf")

    for t in targets:
        res = planner(grid, start, t)
        if not res.path:
            continue
        L = len(res.path) - 1

        if use_mst_scoring:
            remaining_after = [p for p in targets if p != t]
            mst_extra = mst_cost_manhattan(remaining_after)
            score = L + mst_extra
            if score < best_score or (score == best_score and res.expanded < best_exp) or (score == best_score and res.expanded == best_exp and res.plan_time_sec < best_time):
                best_t, best_res = t, res
                best_score, best_exp, best_time = score, res.expanded, res.plan_time_sec
        else:
            if (L < best_len) or (L == best_len and res.expanded < best_exp) or (L == best_len and res.expanded == best_exp and res.plan_time_sec < best_time):
                best_t, best_res = t, res
                best_len, best_exp, best_time = L, res.expanded, res.plan_time_sec

    return best_t, best_res

# =========================
# Snake UI (Tkinter)
# =========================
class SnakeApp:
    def __init__(self, root):
        self.root = root
        root.title("Robot Collecteur - style Snake")
        self.cell = 24

        # Controles
        ctrl = ttk.Frame(root); ctrl.pack(fill="x", padx=6, pady=6)
        ttk.Label(ctrl, text="Taille:").pack(side="left")
        self.size_var = tk.StringVar(value="20x20")
        ttk.Combobox(ctrl, textvariable=self.size_var,
                     values=["10x10","20x20","50x50"], width=6, state="readonly").pack(side="left", padx=4)

        ttk.Label(ctrl, text="Algo:").pack(side="left", padx=(10,0))
        self.algo_var = tk.StringVar(value="A*")
        ttk.Combobox(ctrl, textvariable=self.algo_var,
                     values=["UCS","A*"], width=5, state="readonly").pack(side="left", padx=4)

        ttk.Label(ctrl, text="Heuristique:").pack(side="left", padx=(10,0))
        self.heur_var = tk.StringVar(value="H1 Manhattan")
        self.heur_cb = ttk.Combobox(ctrl, textvariable=self.heur_var,
                                    values=list(HEURISTICS.keys()) + ["H4 MST (multi-obj)"],
                                    width=16, state="readonly")
        self.heur_cb.pack(side="left", padx=4)

        ttk.Label(ctrl, text="Objets fixes:").pack(side="left", padx=(10,0))
        self.items_var = tk.IntVar(value=10)
        ttk.Spinbox(ctrl, from_=1, to=100, textvariable=self.items_var, width=5).pack(side="left", padx=4)

        ttk.Label(ctrl, text="Densite murs:").pack(side="left", padx=(10,0))
        self.wall_var = tk.DoubleVar(value=0.10)
        ttk.Spinbox(ctrl, from_=0.0, to=0.4, increment=0.01, textvariable=self.wall_var, width=5).pack(side="left", padx=4)

        ttk.Label(ctrl, text="Spawn dynamique:").pack(side="left", padx=(10,0))
        self.spawn_var = tk.DoubleVar(value=0.10)  # proba par pas
        ttk.Spinbox(ctrl, from_=0.0, to=0.5, increment=0.01, textvariable=self.spawn_var, width=5).pack(side="left", padx=4)

        ttk.Label(ctrl, text="Max dynamique:").pack(side="left", padx=(10,0))
        self.maxdyn_var = tk.IntVar(value=6)
        ttk.Spinbox(ctrl, from_=0, to=200, textvariable=self.maxdyn_var, width=5).pack(side="left", padx=4)

        ttk.Button(ctrl, text="Nouvelle instance", command=self.new_instance).pack(side="left", padx=(10,4))
        ttk.Button(ctrl, text="Lancer", command=self.run).pack(side="left")
        ttk.Button(ctrl, text="Reset", command=self.reset).pack(side="left", padx=4)

        # Canvas
        self.canvas = tk.Canvas(root, bg="black")
        self.canvas.pack(padx=6, pady=6)

        # Status
        self.status = tk.StringVar(value="Pret.")
        ttk.Label(root, textvariable=self.status).pack(fill="x", padx=6, pady=(0,6))

        # Etat runtime
        self.grid: Optional[Grid] = None
        self.start: Coord = (0,0)
        self.robot: Coord = (0,0)
        self.items: List[Coord] = []
        self.dynamic_count = 0
        self.stats = RunStats()
        self.anim_path: List[Coord] = []
        self.anim_i = 0
        self.running = False
        self.rng = random.Random(123)

        self.new_instance()

        # Activer/desactiver heuristique selon algo
        self.algo_var.trace_add("write", self._toggle_heur)

    # ------------- generation instance -------------
    def _toggle_heur(self, *_):
        if self.algo_var.get() == "A*":
            self.heur_cb.configure(state="readonly")
        else:
            self.heur_cb.configure(state="disabled")

    def new_instance(self):
        if self.running: return
        rows, cols = map(int, self.size_var.get().split("x"))
        cell = self.cell
        self.canvas.config(width=cols*cell, height=rows*cell)

        # make grid
        seed = 42
        self.rng = random.Random(seed)
        walls = self._random_walls(rows, cols, float(self.wall_var.get()), self.rng)
        self.start = (rows//2, cols//2)
        walls.discard(self.start)
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:  # liberer autour de START
            p = (self.start[0]+dr, self.start[1]+dc)
            if 0 <= p[0] < rows and 0 <= p[1] < cols: walls.discard(p)

        self.grid = Grid(rows, cols, walls)
        self.robot = self.start
        self.items = self._random_items(rows, cols, int(self.items_var.get()))
        self.dynamic_count = 0
        self.stats = RunStats()
        self.anim_path = []; self.anim_i = 0
        self.running = False
        self._draw()

        self.status.set("Instance prete. START au centre. Objets: %d." % len(self.items))

    def _random_walls(self, rows, cols, density, rng) -> Set[Coord]:
        W: Set[Coord] = set()
        for r in range(rows):
            for c in range(cols):
                if rng.random() < density:
                    W.add((r,c))
        return W

    def _random_items(self, rows, cols, n) -> List[Coord]:
        out = []
        tries = 0
        while len(out) < n and tries < 5000:
            tries += 1
            p = (self.rng.randrange(rows), self.rng.randrange(cols))
            if p != self.start and self.grid.passable(p) and p not in out:
                out.append(p)
        return out

    # ------------- dessin -------------
    def _draw(self, path: Optional[List[Coord]] = None):
        self.canvas.delete("all")
        rows, cols = self.grid.rows, self.grid.cols
        cell = self.cell

        # grille
        for r in range(rows):
            for c in range(cols):
                x0, y0 = c*cell, r*cell
                x1, y1 = x0+cell, y0+cell
                fill = "#111"
                if (r,c) in self.grid.walls:
                    fill = "#333"
                self.canvas.create_rectangle(x0, y0, x1, y1, outline="#222", fill=fill)

        # path (vert)
        if path:
            for i in range(1, len(path)):
                r0,c0 = path[i-1]; r1,c1 = path[i]
                x0,y0 = c0*cell+cell//2, r0*cell+cell//2
                x1,y1 = c1*cell+cell//2, r1*cell+cell//2
                self.canvas.create_line(x0, y0, x1, y1, width=3, fill="#00ff6a")

        # objets (pommes)
        for (r,c) in self.items:
            x0,y0 = c*cell+5, r*cell+5
            x1,y1 = x0+cell-10, y0+cell-10
            self.canvas.create_oval(x0,y0,x1,y1, fill="#ff4b4b", outline="#ff9b9b", width=2)

        # START (marqueur)
        sr, sc = self.start
        self.canvas.create_text(sc*cell+cell-6, sr*cell+12, text="S", fill="#4aa3ff", font=("TkDefaultFont", 10, "bold"))

        # robot (tete du snake)
        rr, cc = self.robot
        self.canvas.create_rectangle(cc*cell+4, rr*cell+4, cc*cell+cell-4, rr*cell+cell-4, fill="#39e75f", outline="#0b9a38", width=2)

    # ------------- run / reset -------------
    def reset(self):
        if self.running: return
        self.robot = self.start
        self.anim_path = []; self.anim_i = 0
        self.stats = RunStats()
        self.dynamic_count = 0
        self._draw()
        self.status.set("Reset.")

    def run(self):
        if self.running: return
        if not self.items:
            self.status.set("Aucun objet. Genere une nouvelle instance.")
            return
        self.running = True
        self.status.set("Planification")
        self._plan_and_follow_next_segment(first_call=True)

    # ------------- planification et animation -------------
    def _planner(self) -> Callable[[Grid, Coord, Coord], SearchResult]:
        algo = self.algo_var.get().upper()
        if algo == "UCS":
            return lambda g,s,t: ucs(g,s,t)
        else:
            # Par defaut, on reste sur une heuristique simple pour A* interne (Manhattan si inconnue)
            h = HEURISTICS.get(self.heur_var.get(), h_manhattan)
            return lambda g,s,t: astar(g,s,t,h)

    def _maybe_spawn_item(self):
        # apparition dynamique (comme une nouvelle "pomme")
        if self.dynamic_count >= int(self.maxdyn_var.get()):
            return
        if random.random() < float(self.spawn_var.get()):
            rows, cols = self.grid.rows, self.grid.cols
            for _ in range(600):
                p = (random.randrange(rows), random.randrange(cols))
                if p != self.robot and p != self.start and self.grid.passable(p) and (p not in self.items):
                    self.items.append(p)
                    self.dynamic_count += 1
                    break

    def _use_mst_mode(self) -> bool:
        return self.algo_var.get().upper() == "A*" and self.heur_var.get() == "H4 MST (multi-obj)"

    def _plan_and_follow_next_segment(self, first_call=False):
        # Choisir la prochaine cible (replan a chaque cible)
        if self.items:
            plan_to = self._planner()
            use_mst = self._use_mst_mode()
            goal, res = best_target_and_path(self.grid, self.robot, self.items, plan_to, use_mst_scoring=use_mst)
            if res.path:
                self.stats.expanded_total += res.expanded
                self.stats.plan_time_total += res.plan_time_sec
                self.anim_path = res.path
                self.anim_i = 0
                tag = " (MST)" if use_mst else ""
                self.status.set(f"Chemin{tag} vers {goal} - expanded +{res.expanded}, plan {res.plan_time_sec*1000:.1f} ms")
                self._step_along_path()
                return
            else:
                # Aucun chemin vers au moins un objet ? on abandonne la collecte, on tente retour
                self.items.clear()

        # Si plus d'objets, retour a START
        if self.robot != self.start:
            plan_to = self._planner()
            back = plan_to(self.grid, self.robot, self.start)
            if back.path:
                self.stats.expanded_total += back.expanded
                self.stats.plan_time_total += back.plan_time_sec
                self.anim_path = back.path
                self.anim_i = 0
                self.status.set(f"Retour START - expanded +{back.expanded}, plan {back.plan_time_sec*1000:.1f} ms")
                self._step_along_path(final=True)
                return

        # Fini
        self.running = False
        self.status.set(f"Termine. Expanded: {self.stats.expanded_total} | Plan time: {self.stats.plan_time_total:.3f}s | Steps: {self.stats.steps_moved}")

    def _step_along_path(self, final=False):
        # Animation style Snake: avancer case par case
        if self.anim_i >= len(self.anim_path):
            # Segment fini: si on n'est pas en retour final, ramasser et replanifier
            if not final and self.robot in self.items:
                self.items.remove(self.robot)
            self._draw()
            self._plan_and_follow_next_segment()
            return

        next_pos = self.anim_path[self.anim_i]
        self.robot = next_pos
        self.stats.steps_moved += 1

        # Apparitions dynamiques pendant le deplacement
        self._maybe_spawn_item()

        # Ramassage immediat si on passe sur un objet
        if self.robot in self.items:
            self.items.remove(self.robot)

        # Dessin du chemin progressif
        self._draw(self.anim_path[:self.anim_i+1])
        self.anim_i += 1

        # Vitesse d'animation (ms)
        self.root.after(35, lambda: self._step_along_path(final=final))


if __name__ == "__main__":
    root = tk.Tk()
    SnakeApp(root)
    root.mainloop()