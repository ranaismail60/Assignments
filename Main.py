import tkinter as tk
import heapq, random, time, math

# Colors
BG = "#020817"
PB = "#0f172a"
TX = "#e2e8f0"
MU = "#64748b"
AC = "#f97316"
GR = "#22c55e"
BL = "#1d4ed8"
YE = "#eab308"
RE = "#ef4444"
PC = "#10b981"
AG = "#f43f5e"
WC = "#334155"
EC = "#0d1117"
SEP = "#1e293b"

WALL = 1

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def neighbors(grid, R, C, r, c):
    for dr, dc in ((0,1),(1,0),(0,-1),(-1,0)):
        nr, nc = r+dr, c+dc
        if 0 <= nr < R and 0 <= nc < C and grid[nr][nc] != WALL:
            yield (nr, nc)
#Path Rebuild 
def rebuild_path(came_from, node):
    path = []
    while node:
        path.append(node)
        node = came_from.get(node)
    return path[::-1]

def run_search(grid, R, C, start, goal, alg, h):
    pq = []
    counter = 0
    came_from = {start: None}
    g = {start: 0}
    visited = []
    heapq.heappush(pq, (h(start, goal), counter, start))

    while pq:
        _, _, cur = heapq.heappop(pq)
        visited.append(cur)
        if cur == goal:
            return rebuild_path(came_from, cur), visited
        for nb in neighbors(grid, R, C, *cur):
            tg = g.get(cur, 0) + 1
            if alg == "astar":
                if tg < g.get(nb, 1e9):
                    came_from[nb] = cur
                    g[nb] = tg
                    counter += 1
                    heapq.heappush(pq, (tg + h(nb, goal), counter, nb))
            else:
                if nb not in came_from:
                    came_from[nb] = cur
                    counter += 1
                    heapq.heappush(pq, (h(nb, goal), counter, nb))

    return None, visited


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dynamic Pathfinding Agent")
        self.configure(bg=BG)
        self.resizable(True, True)

        self.Rv  = tk.IntVar(value=20)
        self.Cv  = tk.IntVar(value=28)
        self.Dv  = tk.IntVar(value=30)
        self.SPv = tk.IntVar(value=8)
        self.alg = tk.StringVar(value="astar")
        self.hur = tk.StringVar(value="manhattan")
        self.dyn = tk.BooleanVar()
        self.edt = tk.StringVar(value="wall")

        self.grid  = []
        self.start = (1, 1)
        self.goal  = None
        self.rows  = 20
        self.cols  = 28
        self.cs    = 26

        self.running = False
        self.aid = None
        self.did = None
        self.path = []
        self.aidx = 0
        self.vset = set()
        self.pset = set()
        self.apos = None

        self._build_ui()
        self._maze()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        panel = tk.Frame(self, bg=PB, width=230)
        panel.pack(side="left", fill="y", padx=(10,0), pady=10)
        panel.pack_propagate(False)
        self._build_panel(panel)

        right = tk.Frame(self, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        cf = tk.Frame(right, bg=PB)
        cf.pack(fill="both", expand=True)

        self.cv = tk.Canvas(cf, bg=EC, highlightthickness=0, cursor="crosshair")
        self.cv.pack(fill="both", expand=True, padx=6, pady=6)
        self.cv.bind("<Button-1>", self._click)
        self.cv.bind("<B1-Motion>", self._click)

    def _sec(self, p, title):
        tk.Label(p, text=title, bg=PB, fg=MU, font=("Courier",8,"bold")).pack(anchor="w", padx=6, pady=(6,0))
        tk.Frame(p, bg=SEP, height=1).pack(fill="x", padx=6, pady=(2,3))
        body = tk.Frame(p, bg=PB)
        body.pack(fill="x", padx=6)
        return body

    def _spin(self, p, var, lo, hi, label):
        row = tk.Frame(p, bg=PB)
        row.pack(fill="x", pady=1)
        tk.Label(row, text=label, bg=PB, fg=TX, font=("Courier",9), width=11, anchor="w").pack(side="left")
        tk.Spinbox(row, textvariable=var, from_=lo, to=hi, width=5,
                   bg=SEP, fg=TX, relief="flat", font=("Courier",9),
                   highlightthickness=0).pack(side="right")

    def _radios(self, p, var, options):
        row = tk.Frame(p, bg=PB)
        row.pack(fill="x", pady=2)
        for val, lbl in options:
            rb = tk.Radiobutton(row, text=lbl, variable=var, value=val,
                                bg=PB, fg=TX, selectcolor=BL,
                                activebackground=PB, font=("Courier",9),
                                indicatoron=0, relief="flat",
                                padx=5, pady=3, cursor="hand2")
            rb.pack(side="left", padx=2)
            def _refresh(rb=rb, val=val):
                if var.get() == val:
                    rb.configure(bg=BL, fg="white")
                else:
                    rb.configure(bg=SEP, fg=MU)
            var.trace_add("write", lambda *a, fn=_refresh: fn())
            _refresh()

    def _btn(self, p, text, cmd, color=BL):
        tk.Button(p, text=text, command=cmd, bg=color, fg="white",
                  relief="flat", font=("Courier",9,"bold"),
                  cursor="hand2", padx=6, pady=4,
                  activebackground=color).pack(fill="x", pady=2)

    def _build_panel(self, p):
        tk.Label(p, text="PATHFINDER AGENT", bg=PB, fg=AC, font=("Courier",11,"bold")).pack(pady=(10,2))
        tk.Label(p, text="Dynamic Informed Search", bg=PB, fg=MU, font=("Courier",8)).pack(pady=(0,4))

        s = self._sec(p, "GRID SETTINGS")
        self._spin(s, self.Rv, 5, 40, "Rows")
        self._spin(s, self.Cv, 5, 60, "Columns")
        self._spin(s, self.Dv, 0, 70, "Density %")
        self._btn(s, "↺ GENERATE MAZE", self._maze, AC)

        self._radios(self._sec(p, "ALGORITHM"), self.alg, [("astar","A*"), ("gbfs","GBFS")])
        self._radios(self._sec(p, "HEURISTIC"), self.hur, [("manhattan","Manhattan"), ("euclidean","Euclidean")])
        self._radios(self._sec(p, "EDIT TOOL"), self.edt, [("wall","Wall"), ("start","Start"), ("goal","Goal")])

        s5 = self._sec(p, "DYNAMIC MODE")
        tk.Checkbutton(s5, text="Enable dynamic obstacles", variable=self.dyn,
                       bg=PB, fg=TX, selectcolor=PB, activebackground=PB,
                       font=("Courier",9), cursor="hand2").pack(anchor="w")
        self._spin(s5, self.SPv, 1, 30, "Spawn %")

        tk.Frame(p, bg=SEP, height=1).pack(fill="x", padx=6, pady=6)
        rf = tk.Frame(p, bg=PB)
        rf.pack(fill="x", padx=6)
        self._btn(rf, "▶ RUN SEARCH", self._run, GR)
        self._btn(rf, "■ RESET", self._reset, RE)

        tk.Frame(p, bg=SEP, height=1).pack(fill="x", padx=6, pady=(2,0))
        sf = tk.Frame(p, bg=PB)
        sf.pack(fill="x", padx=6, pady=3)
        tk.Label(sf, text="STATUS", bg=PB, fg=MU, font=("Courier",8,"bold")).pack(anchor="w")
        self.stv = tk.StringVar(value="IDLE")
        self.stl = tk.Label(sf, textvariable=self.stv, bg=EC, fg=MU, font=("Courier",10,"bold"), pady=3)
        self.stl.pack(fill="x")

        mf = tk.Frame(p, bg=PB)
        mf.pack(fill="x", padx=6, pady=3)
        tk.Label(mf, text="METRICS", bg=PB, fg=MU, font=("Courier",8,"bold")).pack(anchor="w")
        tk.Frame(mf, bg=SEP, height=1).pack(fill="x", pady=(0,3))
        self.mv = {}
        for key, label in [("v","Nodes Visited"), ("p","Path Cost"), ("t","Exec Time(ms)")]:
            row = tk.Frame(mf, bg=PB)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=label, bg=PB, fg=MU, font=("Courier",8), width=14, anchor="w").pack(side="left")
            var = tk.StringVar(value="—")
            self.mv[key] = var
            tk.Label(row, textvariable=var, bg=PB, fg=AC, font=("Courier",9,"bold")).pack(side="right")

        lf = tk.Frame(p, bg=PB)
        lf.pack(fill="x", padx=6, pady=3)
        tk.Label(lf, text="LEGEND", bg=PB, fg=MU, font=("Courier",8,"bold")).pack(anchor="w")
        tk.Frame(lf, bg=SEP, height=1).pack(fill="x", pady=(0,3))
        for color, label in [(GR,"Start"),(AC,"Goal"),(AG,"Agent"),(PC,"Path"),(YE,"Frontier"),(BL,"Visited"),(WC,"Wall")]:
            row = tk.Frame(lf, bg=PB)
            row.pack(fill="x", pady=1)
            tk.Canvas(row, width=10, height=10, bg=color, highlightthickness=0).pack(side="left", padx=(0,5))
            tk.Label(row, text=label, bg=PB, fg=TX, font=("Courier",8)).pack(side="left")

    # ── Grid ─────────────────────────────────────────────────────────────────

    def _maze(self):
        self._stop()
        self.rows = self.Rv.get()
        self.cols = self.Cv.get()
        d = self.Dv.get() / 100
        self.start = (1, 1)
        self.goal  = (self.rows-2, self.cols-2)
        self.grid  = [
            [WALL if random.random() < d and (r,c) not in ((1,1), self.goal) else 0
             for c in range(self.cols)]
            for r in range(self.rows)
        ]
        self._reset()
        self._stat("READY", GR)

    def _reset(self):
        self._stop()
        self.running = False
        self.vset = set()
        self.pset = set()
        self.apos = None
        self.path = []
        self.aidx = 0
        for v in self.mv.values():
            v.set("—")
        self._stat("READY", TX)
        self._draw()

    # ── Drawing ──────────────────────────────────────────────────────────────

    def _draw(self):
        self.cv.delete("all")
        W = self.cv.winfo_width()
        H = self.cv.winfo_height()
        if W < 10:
            self.after(50, self._draw)
            return
        cs = max(10, min(W // self.cols, H // self.rows, 36))
        self.cs = cs
        ox = (W - cs * self.cols) // 2
        oy = (H - cs * self.rows) // 2

        for r in range(self.rows):
            for c in range(self.cols):
                pos = (r, c)
                x1  = ox + c * cs
                y1  = oy + r * cs

                if pos == self.apos:
                    color = AG
                elif pos in self.pset:
                    color = PC
                elif pos in self.vset:
                    color = BL
                elif pos == self.start:
                    color = GR
                elif pos == self.goal:
                    color = AC
                elif self.grid[r][c] == WALL:
                    color = WC
                else:
                    color = EC

                self.cv.create_rectangle(x1, y1, x1+cs-1, y1+cs-1,
                                         fill=color, outline=SEP, width=1)

                if pos == self.apos:
                    label = "▲"
                elif pos == self.start:
                    label = "S"
                elif pos == self.goal:
                    label = "G"
                else:
                    label = ""

                if label and cs >= 14:
                    self.cv.create_text(x1+cs//2, y1+cs//2, text=label,
                                        fill="white", font=("Courier", max(7, cs//3), "bold"))

    def _cell(self, e):
        cs = self.cs
        ox = (self.cv.winfo_width()  - cs * self.cols) // 2
        oy = (self.cv.winfo_height() - cs * self.rows) // 2
        r  = (e.y - oy) // cs
        c  = (e.x - ox) // cs
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return (r, c)
        return None

    def _click(self, e):
        if self.running:
            return
        pos = self._cell(e)
        if not pos:
            return
        r, c = pos
        mode = self.edt.get()

        if mode == "wall":
            if pos not in (self.start, self.goal):
                self.grid[r][c] = WALL if self.grid[r][c] == 0 else 0
        elif mode == "start":
            if pos != self.goal:
                self.grid[self.start[0]][self.start[1]] = 0
                self.start = pos
                self.grid[r][c] = 0
        elif mode == "goal":
            if pos != self.start:
                self.grid[self.goal[0]][self.goal[1]] = 0
                self.goal = pos
                self.grid[r][c] = 0

        self._reset()

    # ── Search ───────────────────────────────────────────────────────────────

    def _run(self):
        if not self.goal:
            return
        self._stop()
        self.running = True
        self.vset = set()
        self.pset = set()
        self.apos = None
        self._stat("SEARCHING...", YE)

        h = manhattan if self.hur.get() == "manhattan" else euclidean
        t0 = time.perf_counter()
        path, vis = run_search(self.grid, self.rows, self.cols,
                               self.start, self.goal, self.alg.get(), h)
        ms = round((time.perf_counter() - t0) * 1000, 2)

        self.mv["v"].set(str(len(vis)))
        self.mv["t"].set(f"{ms} ms")

        if not path:
            self.vset = set(vis)
            self.mv["p"].set("0")
            self._stat("NO PATH", RE)
            self.running = False
            self._draw()
            return

        self.mv["p"].set(str(len(path) - 1))
        self._animate(vis, path)

    def _animate(self, vis, path):
        vis_list = vis[:]
        step = [0]

        def tick():
            if step[0] < len(vis_list):
                self.vset.add(vis_list[step[0]])
                step[0] += 1
                self._draw()
                self.aid = self.after(12, tick)
            else:
                self.pset = set(path)
                self._draw()
                self.path = path
                self.aidx = 0
                self._stat("MOVING AGENT", GR)
                self.after(200, self._move)
                if self.dyn.get():
                    self._dtick()

        self.aid = self.after(12, tick)

    def _move(self):
        if self.aidx >= len(self.path):
            self._stat("GOAL REACHED ", GR)
            self.running = False
            self._stop_dyn()
            self.apos = None
            self._draw()
            return
        self.apos = self.path[self.aidx]
        self.pset = set(self.path[self.aidx:])
        self.aidx += 1
        self._draw()
        self.aid = self.after(90, self._move)

    # ── Dynamic obstacles ─────────────────────────────────────────────────────

    def _dtick(self):
        if not self.running or not self.dyn.get():
            return
        if random.random() < self.SPv.get() / 100:
            empty = [
                (r, c)
                for r in range(self.rows)
                for c in range(self.cols)
                if self.grid[r][c] == 0
                and (r, c) not in (self.start, self.goal)
                and (r, c) != self.apos
            ]
            if empty:
                nr, nc = random.choice(empty)
                self.grid[nr][nc] = WALL
                if (nr, nc) in self.path[self.aidx:]:
                    self._replan()
        self.did = self.after(300, self._dtick)

    def _replan(self):
        if not self.apos:
            return
        h = manhattan if self.hur.get() == "manhattan" else euclidean
        t0 = time.perf_counter()
        path, vis = run_search(self.grid, self.rows, self.cols,
                               self.apos, self.goal, self.alg.get(), h)
        ms = round((time.perf_counter() - t0) * 1000, 2)
        self.mv["v"].set(str(len(vis)))
        self.mv["t"].set(f"{ms} ms")

        if not path:
            self._stat("REPLAN FAILED", RE)
            self._stop()
            self.running = False
            return

        self.mv["p"].set(str(len(path) - 1))
        if self.aid:
            self.after_cancel(self.aid)
            self.aid = None
        self.path = path
        self.aidx = 0
        self.pset = set(path)
        self._stat("RE-PLANNED ", YE)
        self.aid = self.after(90, self._move)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        if self.aid:
            self.after_cancel(self.aid)
            self.aid = None
        self._stop_dyn()

    def _stop_dyn(self):
        if self.did:
            self.after_cancel(self.did)
            self.did = None

    def _stat(self, text, color=TX):
        self.stv.set(text)
        self.stl.configure(fg=color)


if __name__ == "__main__":
    app = App()
    app.geometry("1050x680")
    app.minsize(700, 480)
    app.bind("<Configure>", lambda e: app._draw() if e.widget is app else None)
    app.after(100, app._draw)
    app.mainloop()
