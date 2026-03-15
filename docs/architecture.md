# Architecture Documentation — UAV/WSN GA→ACO Path Planner

> Render UML diagrams at [plantuml.com](https://www.plantuml.com/plantuml/uml)
> by pasting `uml-legacy.puml` (legacy) or `uml-refactored.puml` (current).

---

## 1. Layer Overview

The codebase uses a strict layered architecture with a one-way dependency rule.
Each layer may only import from layers below it:

```
┌────────────────────────────────────────────────────┐
│  ui/                                               │
│  SimulationGui · SimulationPanel                   │
│  SimulationSnapshot · VisualizationListener        │
└─────────────────────────┬──────────────────────────┘
                          │ depends on ↓
┌────────────────────────────────────────────────────┐
│  config/                                           │
│  AppConfig  (intelligence registry + properties    │
│              loader; owns REGISTRY map)            │
└─────────────────────────┬──────────────────────────┘
                          │ depends on ↓
┌────────────────────────────────────────────────────┐
│  intelligence/                                     │
│  PredefinedPatrolIntelligence                      │
│  GaAcoIntelligence  (frontier + gated GA-ACO)      │
└─────────────────────────┬──────────────────────────┘
                          │ depends on ↓
┌────────────────────────────────────────────────────┐
│  simulation/                                       │
│  UAV · SimulationRunner · SimulationConfig         │
│  UAVIntelligence (interface)                       │
│  CoverageGrid · KnowledgeBase · Environment        │
│  NodeAgent · NodeMover · EventScheduler            │
│  MarkovChain · MovementStrategy · LevyWalkStrategy │
└─────────────────────────┬──────────────────────────┘
                          │ depends on ↓
┌────────────────────────────────────────────────────┐
│  algorithm/                                        │
│  PathPlanner (interface)                           │
│  gaaco/  GaAcoPlanner · GaAcoConfig                │
│          GeneticAlgorithm · AntColonyOptimization  │
│          LocalSearch · PheromoneMatrix             │
│  Route · Population                                │
└─────────────────────────┬──────────────────────────┘
                          │ depends on ↓
┌────────────────────────────────────────────────────┐
│  domain/                                           │
│  SensorNode                                        │
└────────────────────────────────────────────────────┘
```

> **Note on `config/ ↔ simulation/` dependency**: `AppConfig` reads
> `config.properties` and calls `SimulationConfig.defaults()`, which in
> turn calls back into `AppConfig.load()`.  This circular package reference
> compiles and runs safely — there is no static initialiser cycle — but
> `AppConfig` is the single source of truth for all default values.

---

## 2. Legacy Architecture (original thesis code)

The original codebase contains thirteen classes in the default package with no
layering. The central problem is `Run.java` (~1,750 lines): it owns the simulation
grid, drives the event loop, performs all range scanning, manages the UAV patrol,
and directly invokes the GA and ACO algorithms.

### Legacy class inventory

| Class | Role | Key issues |
|---|---|---|
| `Run` | Monolithic orchestrator | Couples environment, event loop, UAV patrol, range scanning, and path planning in one file. |
| `Nodes` | Grid container | Outer class whose only purpose is to hold `Node`. |
| `Nodes.Node` | Sensor node (inner) | **Bug E**: `drone_cache_from_waypoint` and `drone_direct_caching` are `static ArrayList` — shared across every instance. |
| `Events` | Event queue | Hand-rolled sorted linked list. O(n) insertion. |
| `Movement` | Grid movement | All nine methods are `static`. Each scans the entire `field[w][h]` — O(width×height) — to locate a node. |
| `MarkOvChain` | Markov chain | Four separate `state0/1/2/3()` methods; no shared constants. |
| `Ant` | Single ACO ant | **Bug B**: pheromone index written `[to][from]` in some paths, reversed vs. deposit step. |
| `AntColonyOptimization` | ACO runner | Sequential. **Bug D2**: deposit uses step distance not tour length. **Bug D6**: no global evaporation. |
| `GeneticAlgorithm` | GA runner | Reads all parameters from `Params` static fields. |
| `Population` | Chromosome bag | Mutable `ArrayList<Path>`. |
| `Path` | Route (legacy) | **Bug C**: `isFitnessChanged` permanently dead code; `getFitness()` always recomputes. |
| `Params` | Global constants | Eleven `static final` fields; no validation. |
| `Node` | Algorithm node | Separate from `Nodes.Node`. Used by `Ant` and `Path`. |

### Legacy dependency structure

```
Run ──────────────────────────────────────── (drives everything)
 ├─ Nodes ──── Nodes.Node (inner, static caches — Bug E)
 ├─ Events (sorted linked list)
 ├─ Movement (static, O(w×h) per call)
 ├─ MarkOvChain (called per-event)
 ├─ GeneticAlgorithm ─── Population ─── Path ─── Node
 ├─ AntColonyOptimization ─── Ant ─── Path ─── Node
 └─ Params (global constants)
```

---

## 3. Refactored Architecture

### 3.0 Config Layer — `config/`

| Class | Role | Key design |
|---|---|---|
| `AppConfig` | Runtime configuration loader and intelligence registry | Reads `config.properties` from the project root (falls back to compiled-in defaults). Maintains a `static Map<String, Function<SimulationConfig, UAVIntelligence>> REGISTRY` that maps short string keys (`"patrol"`, `"gaaco"`, …) to factory lambdas. `load().buildSimulationConfig()` produces the `SimulationConfig` used throughout the run. |

#### Intelligence registry

`AppConfig.REGISTRY` is the single place where keys are associated with
implementations:

```java
static {
    REGISTRY.put("patrol", sim -> new PredefinedPatrolIntelligence());
    REGISTRY.put("gaaco",  sim -> new GaAcoIntelligence(sim, GaAcoConfig.defaults()));
}
```

`Main` reads `run.intelligences` from `config.properties`, looks each key up in
`REGISTRY`, and passes the resulting list to `SimulationRunner`. Adding a new
strategy requires only three steps: implement `UAVIntelligence`, add a
`REGISTRY.put(...)` line, and add the key to `config.properties`.

---

### 3.1 Domain Layer — `domain/`

| Class | Role | Key design |
|---|---|---|
| `SensorNode` | Immutable sensor-node value object | Equality/hashing by `id` only. `withPosition()` returns a new instance. Carries `direction` and `speed` captured at observation time; `hasVelocity()` distinguishes direct (full state) from transitive (position only) observations. Thread-safe; shareable across layers. |

`SensorNode` is the shared vocabulary between the simulation (where nodes move
on a grid) and the algorithm (which plans routes through their coordinates). The
velocity fields are consumed exclusively by the UAV's interception logic;
the algorithm layer uses only position fields for distance calculations.

---

### 3.2 Algorithm Layer — `algorithm/`

#### PathPlanner interface (Strategy pattern)

```java
public interface PathPlanner {
    Route planRoute(List<SensorNode> nodes);
}
```

The `PathPlanner` interface decouples the UAV from any specific planning algorithm.
`UAV` holds a `PathPlanner` reference injected at construction time; calling code
(currently `SimulationRunner`) decides which implementation to use:

```java
PathPlanner planner = new GaAcoPlanner(GaAcoConfig.defaults());
UAV uav = new UAV(simConfig, planner);
```

To add a new algorithm, implement `PathPlanner` and pass it in — no changes to
`UAV`, `SimulationRunner`, or any other class are required.

#### Algorithm class inventory

| Class | Package | Role | Key design |
|---|---|---|---|
| `PathPlanner` | `algorithm` | Strategy interface | Single method `planRoute(List<SensorNode>): Route`. Swappable without touching any simulation code. |
| `Route` | `algorithm` | Immutable ordered node sequence | Replaces `Path`. Distance computed once at construction. `getNodes()` returns unmodifiable view. |
| `GaAcoConfig` | `algorithm.gaaco` | Immutable GA-ACO + planning-schedule config | Owns all GA/ACO hyperparameters **and** the planning schedule used by `GaAcoIntelligence` (`planningThreshold`, `planningInterval`, `minCoverageBeforePlan`, `maxRouteWaypoints`). Validated on construction; `withXxx()` copy-and-override. |
| `GaAcoPlanner` | `algorithm.gaaco` | Default `PathPlanner` implementation | Three-stage pipeline: GA → ACO → LocalSearch. Stateless; fresh instances per call; concurrent-safe. |
| `Population` | `algorithm.gaaco` | Mutable route container for GA | `List<Route>` replaced slot-by-slot per generation. |
| `GeneticAlgorithm` | `algorithm.gaaco` | GA stage | Tournament selection, order crossover (O(n) via HashSet), swap mutation, elitism. 2-opt on best before returning. |
| `PheromoneMatrix` | `algorithm.gaaco` | Pheromone state | `snapshot()` returns a defensive copy used by concurrent ant workers — no synchronisation needed during tour construction. |
| `AntColonyOptimization` | `algorithm.gaaco` | Concurrent ACO stage | Ant 1 seeds matrix with GA route. Ants 2–N build tours concurrently from one read-only snapshot. Batch evaporate-then-deposit. Fixes bugs B, D2, D6. |
| `LocalSearch` | `algorithm.gaaco` | Route post-processing | `twoOpt()` eliminates edge crossings. `orOpt()` relocates individual waypoints. Both run as stage 3 of `GaAcoPlanner`. |

#### Three-stage fusion pipeline (`GaAcoPlanner.planRoute`)

```
GaAcoPlanner.planRoute(nodes)
  │
  ├─ Stage 1: GeneticAlgorithm.run(nodes, gaAcoConfig)
  │    Random init → tournament selection → order crossover → swap mutation
  │    → elitism → repeat for generationCount
  │    → twoOpt(gaBest)           ← clean crossings before ACO seed
  │
  ├─ Stage 2: AntColonyOptimization.refine(gaRoute, nodes, gaAcoConfig)
  │    Ant 1: deposit GA route onto pheromone matrix
  │    Ants 2-N: concurrent tour construction from pheromone snapshot
  │    → batch evaporate + deposit → repeat for acoIterationCount
  │    → elitist deposit of best route found
  │
  └─ Stage 3: LocalSearch.twoOpt(acoRoute) → LocalSearch.orOpt(result)
       twoOpt: reverse segments to eliminate crossings
       orOpt:  relocate individual waypoints to cheaper positions
       → return polished route
```

---

### 3.3 Simulation Layer — `simulation/`

#### Simulation class inventory

| Class | Role | Key design |
|---|---|---|
| `UAVIntelligence` | Strategy interface for per-tick UAV behaviour | `onTick(tick, uav, env): Route` — called after scan+move each tick. Implement to add a new intelligence model without touching `UAV`, `SimulationRunner`, or any other class. Optional `getCoverageGrid()` exposes the coverage tracker to the GUI. |
| `UAVIntelligence` | Strategy interface | `onTick(tick, uav, env): Route` — called after scan+move each tick. Concrete implementations live in `intelligence/`. |
| `NodeAgent` | Mutable simulation entity | Live grid position, direction, speed, per-instance neighbour cache (Bug E fix). `toSensorNode()` captures direction+speed for UAV interception. |
| `NodeMover` | Grid movement helper | O(1); reads `agent.(x,y)` directly. |
| `KnowledgeBase` | Per-UAV knowledge accumulator | `NodeAgent → SensorNode+velocity` bridge. Transitive neighbour-cache ingestion. No static fields. |
| `Environment` | Grid + agent registry | O(n) range queries; squared-distance comparison; excludes node at query centre. |
| `EventScheduler` | Priority-queue event scheduler | O(log n) insert and poll. Replaces O(n) sorted linked list. |
| `SimulationConfig` | Immutable environment parameters | Field dimensions, node count/speed/range, drone range/speed/placement/duration. **Does not** contain planning thresholds or route-length limits — those belong to the algorithm config (e.g. `GaAcoConfig`). `withXxx()` copy-and-override; also has a `builder()`. |
| `CoverageGrid` | Frontier-based exploration tracker | Divides field into `sectorSize×sectorSize` sectors. `nearestUnvisitedCenter()` drives autonomous UAV toward unexplored areas. `isSectorVisited(x, y)` used by `UAV.replan(coverage)` to exclude nodes in already-scanned sectors. Owned by intelligence implementations; exposed via `getCoverageGrid()` for GUI rendering. |
| `MovementStrategy` | Interface for mobility models | Implemented by `LevyWalkStrategy` (`movement/` package). |
| `UAV` | Drone entity — pluggable planner + frontier exploration | See §3.4. |
| `SimulationRunner` | Discrete-event simulation driver | Multi-pass experiment runner. Dispatches `UAVIntelligence.onTick()` each drone tick. Accepts any number of strategies; prints a ranked comparison at the end. Checks the universal return-home condition (remaining ticks ≤ Chebyshev distance to home + 10) for every strategy. |

> **`movement/` package**: `MarkovChain` and `LevyWalkStrategy` live in the
> `movement` package (not `simulation`). `MarkovChain` provides factory methods
> `levyWalk()` (95% straight-run) and `exploratoryWalk()` (70% straight-run);
> `LevyWalkStrategy` implements `MovementStrategy` and accepts any chain.

#### CoverageGrid — frontier-based exploration

The `CoverageGrid` replaces the fixed four-corner patrol in the autonomous pass.
The 1500×1500 field is divided into a grid of 140×140-cell sectors (sector size
= drone range). Each tick the UAV marks all sectors within its scan radius as
visited. During `PATROL` mode, `GaAcoIntelligence` calls
`nearestUnvisitedCenter()` and passes the result to `UAV.setExplorationTarget()`,
steering the UAV toward the nearest unscanned area. When all sectors are visited
the UAV falls back to the fixed four-corner loop.

This gives the autonomous UAV systematic full-field coverage rather than the
perimeter-biased baseline patrol, building a larger and more representative
knowledge base before each `replan()`.

---

### 3.4 UAV Entity — Drive Modes, Token Ledger, Interception, and Strategy

The `UAV` class has four orthogonal responsibilities:

#### Drive modes (`DriveMode` enum)

| Mode | When active | Behaviour |
|---|---|---|
| `PATROL` | Startup; after each planned route is fully traversed | Steers to nearest unvisited `CoverageGrid` sector (autonomous pass) or fixed W1→W2→W3→W4 corners (patrol pass). |
| `EXECUTE` | Immediately after each successful `replan()` | Navigates `PathPlanner` waypoints using predictive interception. Reverts to `PATROL` when last waypoint is reached. |
| `RETURN_HOME` | When remaining ticks ≤ Chebyshev distance to W1 + 10 | Navigates directly to the deployment cell (W1). Activated universally by `SimulationRunner` for every strategy. No frontier steering or replanning fires in this mode. |

#### PathPlanner injection (Strategy pattern)

`UAV` holds a `PathPlanner` field and calls it via interface — it has no
knowledge of `GaAcoPlanner` or any concrete algorithm. This is the
Scala-ready form: the planner can be replaced or mocked without changing `UAV`:

```java
// Swap the planner without touching UAV
PathPlanner planner = new GaAcoPlanner(GaAcoConfig.defaults());
// PathPlanner planner = new GreedyNearestNeighbour();  // future
UAV uav = new UAV(simConfig, planner);
```

#### Smart `replan(CoverageGrid)` strategy

The no-arg `replan()` delegates to `replan(null)`; `GaAcoIntelligence` calls `replan(coverage)` to pass its owned grid.

1. **Filter tokenized nodes** — only nodes not yet physically scanned. Routing back to tokenized nodes earns nothing.
2. **Filter scanned sectors** — when a `CoverageGrid` is provided, nodes whose sector has already been marked visited are also excluded. The UAV's range covers an entire sector when passing through its centre; routing back there wastes travel and creates local-minima loops. This is the primary anti-local-minima mechanism.
3. **Require ≥ 3 targets** — prevents long detours to stale-position transitive nodes for minimal gain. Returns `null` if fewer than 3 valid candidates remain after both filters.
4. **Greedy max-spread selection** — instead of nearest-first (which clusters routes locally), uses a farthest-insertion algorithm: seed on the node farthest from the UAV, then repeatedly add the candidate whose minimum distance to all already-selected nodes is greatest. Produces routes that span the whole known field, not just the nearest pocket.
5. **Cap to `maxRouteWaypoints`** — keeps routes completable within remaining ticks (applied as part of step 4).

#### Token ledger (thesis comparison metric)

Each sensor node grants exactly one token on the first direct scan contact
(default `droneRange = 140` cells). Tokens are split by drive mode:

```
patrolTokens   — contacts during fixed patrol  (baseline)
executeTokens  — contacts during GA→ACO routes (experimental)
```

#### Predictive node interception

When in `EXECUTE` mode the UAV solves the interception equation each tick:

```
Node at time t: (nx + vx·t,  ny + vy·t)

(vx² + vy² − s²)·t²  +  2·(dx·vx + dy·vy)·t  +  (dx² + dy²)  =  0

Smallest positive root t → target cell (nx + vx·t, ny + vy·t)
```

Falls back to last-known static position when `hasVelocity()` is false, the
discriminant is negative, or all roots are non-positive.

---

### 3.5 Intelligence Layer — `intelligence/`

All concrete `UAVIntelligence` implementations live here.  Each class
implements the `UAVIntelligence` interface from the `simulation` package and is
wired in via `AppConfig.REGISTRY`.

| Class | Key | Description |
|---|---|---|
| `PredefinedPatrolIntelligence` | `patrol` | Baseline. No-op `onTick`; UAV follows the built-in W1→W2→W3→W4 corner loop for the entire run. Zero planning overhead. |
| `GaAcoIntelligence` | `gaaco` | Autonomous strategy. Owns a `CoverageGrid`; per tick marks scanned sectors, steers toward the nearest frontier during `PATROL`, and triggers coverage-gated replanning via `uav.replan(coverage)` (scanned-sector filter included). |

---

### 3.6 UI Layer — `ui/`

| Class / Interface | Role | Key design |
|---|---|---|
| `VisualizationListener` | `@FunctionalInterface` GUI callback | `onTick(SimulationSnapshot)` called on simulation thread after every drone event. Implementor posts to EDT via `SwingUtilities.invokeLater`. Pass `null` to runner for headless mode. |
| `SimulationSnapshot` | Immutable state snapshot | Deep-copied coverage grid, unmodifiable node/waypoint lists, frontier target. Safe to hand to EDT without synchronisation. Inner class `NodeSnap` captures per-node state. |
| `SimulationPanel` | Custom `JPanel` renderer | Dark-theme canvas. Nodes coloured by state (grey/amber/green). UAV blue/cyan by mode. Red route lines, orange frontier crosshair, faint coverage-grid overlay. Anti-aliased. |
| `SimulationGui` | `JFrame` with N panels + stats sidebar | Constructed as `new SimulationGui(intelligences.size())`. Renders one `SimulationPanel` per strategy (dynamic `GridLayout`). Speed slider (0–200 ms/tick, quadratic). Pause/Resume button. Live sidebar: tick, pass, mode, tokens, KB size, coverage, route info. Each panel has its own accent colour from a 5-colour palette. |

#### GUI threading model

```
SimulationRunner thread:
  → processDroneEvent()
  → buildSnapshot()          ← deep copies all state
  → vizListener.onTick(snap) ← blocks for tick delay; sleeps if paused
  → SwingUtilities.invokeLater(() -> {
        simPanel.setSnapshot(snap);  ← triggers repaint on EDT
        updateStats(snap);
    })

EDT:
  → SimulationPanel.paintComponent() ← reads snap (volatile field)
  → updateStats labels
```

---

### 3.7 Cross-layer data flow

```
Startup:
  config.properties
    → AppConfig.load()
    → REGISTRY.get("gaaco") / ...
    → List<UAVIntelligence>   ← one instance per configured key
    → SimulationGui(n)        ← n panels, one per strategy

Per-tick (GaAcoIntelligence):
  NodeAgent.toSensorNode()        ← captures (x, y, direction, speed)
      → KnowledgeBase.observe()
      → KnowledgeBase.snapshot()  ← List<SensorNode> with velocity
      → PathPlanner.planRoute(nodes)      ← interface; GaAcoPlanner default
           → GeneticAlgorithm.run()
           → AntColonyOptimization.refine()
           → LocalSearch.twoOpt() + orOpt()
      → Route                     ← ordered waypoints
      → UAV.activateRoute(route)  ← mode = EXECUTE
      → UAV.doExecuteStep()
      → UAV.moveTowardIntercept() ← velocity consumed here; quadratic solve per tick

SimulationRunner:
    → UAVIntelligence.onTick()    ← dispatched to each strategy each tick
    → CoverageGrid.markScanned()
    → buildSnapshot()
    → VisualizationListener.onTick(snap) → SimulationGui → SimulationPanel (per strategy)
    → printComparison(results)    ← ranked console report at end of all passes
```

---

## 4. Scala/TDD Readiness

### 4.1 Immutable Value Objects → Scala Case Classes

```java
// Java (current)
public final class SensorNode {
    private final int id;
    private final double x, y;
    private final int direction, speed;
    public SensorNode withPosition(double newX, double newY) { ... }
}
```
```scala
// Scala equivalent
case class SensorNode(id: Int, x: Double, y: Double,
                      direction: Int = -1, speed: Int = 0) {
  def withPosition(newX: Double, newY: Double): SensorNode =
    copy(x = newX, y = newY)
  def hasVelocity: Boolean = direction >= 0 && speed > 0
}
```

`GaAcoConfig`'s `withXxx()` methods collapse to `copy(field = value)`.
`SimulationConfig` follows the same pattern.  Both become `case class` with no boilerplate.

### 4.2 PathPlanner → Scala Trait (strategy boundary)

```scala
// Scala
trait PathPlanner {
  def planRoute(nodes: List[SensorNode]): Route
}

// In tests: anonymous implementation, no mocking framework
val fixed = new PathPlanner {
  def planRoute(nodes: List[SensorNode]): Route = Route(nodes)
}
val uav = new UAV(simConfig, fixed)
```

The interface is the primary seam point for testing UAV behaviour independently
of any algorithm. In Scala this requires no framework — a single-method trait
can be satisfied with a lambda or anonymous class.

### 4.3 Strategy Interfaces → Scala Traits

Both `PathPlanner` and `MovementStrategy` are single-method interfaces:

```scala
trait MovementStrategy {
  def nextState(state: Int, rng: Random): Int
}
val alwaysIdle = (_: Int, _: Random) => MarkovChain.IDLE
```

`VisualizationListener` is `@FunctionalInterface` in Java:
```scala
// Scala — function type replaces interface
type VisualizationListener = SimulationSnapshot => Unit
runner.setVisualizationListener(snap => panel.setSnapshot(snap))
```

### 4.4 Zero Static Mutable State → Full Parallel Test Execution

```scala
// Safe to run all replicates in parallel
(1 to 1000).par.map { seed =>
  val algConfig = GaAcoConfig.defaults().withPlanningThreshold(10)
  val planner   = GaAcoPlanner(algConfig)
  val simConfig = SimulationConfig.defaults()
  val runner    = SimulationRunner.create(simConfig)
  runner.run(List(new GaAcoIntelligence(simConfig, algConfig)))
}
```

All simulation state is local to `run()`. `GaAcoPlanner.planRoute()` is
stateless. No static fields anywhere — `sbt test` with `parallelExecution := true`
works without coordination.

### 4.5 Layer Boundaries → Clean Mock Seams

| Seam | Java type | Test double strategy |
|---|---|---|
| Algorithm input | `List<SensorNode>` | Construct directly |
| Path planner | `PathPlanner` interface | Anonymous trait / lambda |
| UAV intelligence | `UAVIntelligence` interface | Anonymous class; `onTick` returns fixed route or null |
| Mobility model | `MovementStrategy` interface | Anonymous trait |
| Knowledge bridge | `KnowledgeBase.snapshot()` | Subclass returning fixed list |
| Planning result | `Route` | Construct known `Route`; inject via `activateRoute()` |
| Interception | `SensorNode.hasVelocity()` | Constructor args `(id, x, y, dir, spd)` |
| GUI callback | `VisualizationListener` | Lambda / no-op |

### 4.6 Summary

| Property | Status | Scala / TDD benefit |
|---|---|---|
| Value objects immutable | `SensorNode`, `Route`, `GaAcoConfig`, `SimulationConfig` | Directly `case class`; structural equality. All config classes use `withXxx()` copy-and-override. |
| Pluggable planner | `PathPlanner` interface | Swap algorithm without touching UAV |
| Pluggable intelligence | `UAVIntelligence` interface | Add intelligence model without touching `UAV` or `SimulationRunner` |
| Algorithm has no sim imports | Verified by package structure | Algorithm tests need no simulation setup |
| Strategy interfaces | `PathPlanner`, `MovementStrategy`, `VisualizationListener` | Zero-framework test doubles |
| No static mutable state | All fields `final`; no `static` mutable fields | Full parallel test execution |
| Pure algorithmic functions | `constructTourFromSnapshot`, `twoOpt`, `orOpt` | Deterministic with seeded `Random` |
| One-way dependency flow | `domain ← algorithm ← simulation ← intelligence ← config ← ui` | Mock at any layer boundary |
| `activateRoute()` public | `UAV.activateRoute(Route)` | UAV token/intercept tests without planner |
| Frontier exploration | `CoverageGrid` + `UAV.setExplorationTarget()` | Testable in isolation; inject known visited map |
| Scanned-sector filter | `CoverageGrid.isSectorVisited()` + `UAV.replan(coverage)` | Prevents planner revisiting explored sectors; pass null coverage to disable |
| Return-home guarantee | `UAV.returnHome()` triggered by `SimulationRunner` | Applies to all strategies; no per-intelligence change needed |
| Null-safe planner result | `replan()` returns `null` → `Option[Route]` in Scala | ScalaTest matchers apply directly |
| GUI fully decoupled | `VisualizationListener` optional; null = headless | Simulation tests never need Swing |
