# Developer Guide — DroneLab

A practical guide for researchers and engineers who want to understand, run, and
extend the simulator.

---

## Quick Start (3 steps)

```bash
# 1. Clone and build
git clone https://github.com/TechnoTwelve/drone-lab.git
cd drone-lab
mvn package -DskipTests

# 2. Run the GUI
java -jar target/drone-lab.jar

# 3. Run the test suite
mvn test
```

Prerequisites: Java 17+, Maven 3.6+. See [REQUIREMENTS.md](../REQUIREMENTS.md)
for full setup instructions.

---

## How the Simulation Works

### The simulation loop

Each run is a **discrete-event simulation**. Time advances in *ticks*. On each
tick:

1. Every sensor node moves one step according to its Lévy-walk mobility model.
2. The UAV scans all nodes within `droneRange` cells (direct contact → +1 token).
3. Nodes observed by neighbours of scanned nodes are ingested transitively into
   the UAV's knowledge base.
4. The active `UAVIntelligence` strategy is called: `onTick(tick, uav, env)`.
   - The intelligence may steer the UAV toward an exploration frontier.
   - It may trigger a `replan()` to build a new route through known nodes.
5. The UAV moves one step toward its current target.
6. A `SimulationSnapshot` is sent to the GUI for rendering.

Total ticks: `simulation.duration` (default 5434 ≈ one perimeter lap + 10).

### What a "token" is

A **token** is a unique direct-scan contact. Each node grants exactly one token
the first time the UAV flies within `droneRange` of it. Tokens are the primary
comparison metric between strategies.

### Multiple passes

`SimulationRunner` runs each configured intelligence against the *same* randomly
generated field (same seed → same node positions and velocities). All passes run
concurrently in separate threads, each with its own UAV and intelligence instance.

---

## Project Structure

```
drone-lab/
├── config.properties          Runtime parameters (edit to change behaviour)
├── pom.xml                    Maven build descriptor
├── src/
│   ├── main/java/
│   │   ├── Main.java                  Entry point
│   │   ├── config/AppConfig.java      Intelligence registry + properties loader
│   │   ├── domain/SensorNode.java     Immutable node value object
│   │   ├── algorithm/                 PathPlanner interface + GA-ACO implementation
│   │   ├── simulation/                UAV, Environment, CoverageGrid, event loop
│   │   ├── intelligence/              Concrete UAVIntelligence implementations
│   │   ├── movement/                  Lévy-walk mobility model
│   │   └── ui/                        Swing GUI (panels, snapshots, listener)
│   └── test/java/                     JUnit 5 test suite (265 tests)
├── docs/
│   ├── architecture.md                Deep-dive architecture documentation
│   ├── developer-guide.md             This file
│   ├── api.md                         Public API reference
│   ├── uml-refactored.puml            Current architecture UML (paste at plantuml.com)
│   └── uml-legacy.puml                Original monolithic architecture UML
├── README.md                          Feature overview + algorithm reference
└── REQUIREMENTS.md                    Setup and installation guide
```

### Layer dependency rule

Dependencies flow strictly downward — no layer imports from a layer above it:

```
ui  →  config  →  intelligence  →  simulation  →  algorithm  →  domain
```

This means you can test `algorithm/` code without any `simulation/` setup, and
`simulation/` code without any `ui/` or `config/` setup.

---

## Key Classes

| Class | Package | What it does |
|---|---|---|
| `Main` | root | Reads config, builds intelligences, starts GUI and runner |
| `AppConfig` | `config` | Loads `config.properties`; owns the intelligence `REGISTRY` |
| `SimulationRunner` | `simulation` | Discrete-event loop; runs all passes; prints ranked results |
| `UAV` | `simulation` | Drone entity — scan, move, replan, return-home |
| `UAVIntelligence` | `simulation` | **Interface** — implement this to add a new strategy |
| `CoverageGrid` | `simulation` | Tracks explored sectors; provides nearest-frontier query |
| `KnowledgeBase` | `simulation` | Per-UAV accumulator of observed sensor nodes |
| `Environment` | `simulation` | Grid + O(n) range queries |
| `PathPlanner` | `algorithm` | **Interface** — implement this to add a new route planner |
| `GaAcoPlanner` | `algorithm.gaaco` | Default planner: GA → ACO → LocalSearch pipeline |
| `GaAcoConfig` | `algorithm.gaaco` | Immutable config for GA-ACO hyperparameters |
| `SimulationConfig` | `simulation` | Immutable field/drone/node parameters |
| `SimulationSnapshot` | `ui` | Immutable state copy passed to the GUI each tick |
| `SimulationPanel` | `ui` | Custom dark-theme Swing renderer |

---

## Adding a New Intelligence Strategy

An intelligence strategy decides what the UAV should do each tick. This is the
main extension point for research into new path planning approaches.

### Step 1 — Create the class

```java
package intelligence;

import algorithm.Route;
import simulation.*;

public final class MyIntelligence implements UAVIntelligence {

    private final SimulationConfig config;
    private final CoverageGrid     coverage;

    public MyIntelligence(SimulationConfig config) {
        this.config   = config;
        this.coverage = new CoverageGrid(
            config.getFieldWidth(), config.getFieldHeight(), config.getDroneRange());
    }

    @Override
    public Route onTick(long tick, UAV uav, Environment env) {
        // Mark what the UAV has scanned this tick
        coverage.markScanned(uav.getX(), uav.getY(), config.getDroneRange());

        // During PATROL: steer toward the nearest unvisited sector
        if (uav.getDriveMode() == UAV.DriveMode.PATROL) {
            int[] frontier = coverage.nearestUnvisitedCenter(uav.getX(), uav.getY());
            if (frontier != null) uav.setExplorationTarget(frontier[0], frontier[1]);
            else                  uav.clearExplorationTarget();
        }

        // Trigger a replan when the knowledge base is large enough
        if (uav.getDriveMode() == UAV.DriveMode.PATROL
                && uav.getKnowledgeBase().size() >= 15) {
            return uav.replan(coverage);  // returns null if too few valid targets
        }

        return null;  // no route change this tick
    }

    @Override public String getLabel()              { return "My Intelligence"; }
    @Override public CoverageGrid getCoverageGrid() { return coverage; }
}
```

### Step 2 — Register it

In `src/main/java/config/AppConfig.java`, add one line to the static block:

```java
static {
    REGISTRY.put("patrol", sim -> new PredefinedPatrolIntelligence());
    REGISTRY.put("gaaco",  sim -> new GaAcoIntelligence(sim, GaAcoConfig.defaults()));
    REGISTRY.put("mine",   sim -> new MyIntelligence(sim));  // ← add this
}
```

### Step 3 — Enable it in config.properties

```properties
run.intelligences = patrol,gaaco,mine
```

The GUI automatically adds a new panel; the console report includes the new
strategy in its ranked comparison.

### UAVIntelligence interface contract

| Method | Required | Notes |
|---|---|---|
| `onTick(tick, uav, env): Route` | Yes | Return a `Route` to activate it; return `null` to keep current behaviour. |
| `getLabel(): String` | Yes | Name shown in the GUI panel title and console report. |
| `getCoverageGrid(): CoverageGrid` | No (default `null`) | Expose your tracker for GUI rendering and final coverage stats. |

### Useful UAV methods inside `onTick`

| Method | What it returns / does |
|---|---|
| `uav.getDriveMode()` | `PATROL`, `EXECUTE`, or `RETURN_HOME` |
| `uav.getX()` / `uav.getY()` | Current UAV grid position |
| `uav.getTotalTokens()` | Tokens collected so far |
| `uav.getKnowledgeBase().size()` | Number of nodes known to the UAV |
| `uav.getKnowledgeBase().snapshot()` | Unmodifiable `List<SensorNode>` of known nodes |
| `uav.isTokenized(nodeId)` | Whether a node has been directly scanned already |
| `uav.replan(coverage)` | Plan a route; pass `null` to skip the sector filter |
| `uav.setExplorationTarget(x, y)` | Steer UAV toward `(x, y)` during `PATROL` |
| `uav.clearExplorationTarget()` | Revert to fixed four-corner patrol |

> **Return-home is automatic.** `SimulationRunner` calls `uav.returnHome()` for
> every intelligence when remaining ticks ≤ Chebyshev distance to home + 10.
> The `DriveMode.PATROL` guard in `onTick` already prevents frontier steering
> and replanning from firing during `RETURN_HOME`.

---

## Adding a New Path Planner

A path planner takes a list of sensor nodes and returns an ordered route. The UAV
calls it through the `PathPlanner` interface — it has no knowledge of any specific
algorithm.

```java
package algorithm;

import domain.SensorNode;
import java.util.*;

public final class GreedyNearestNeighbour implements PathPlanner {

    @Override
    public Route planRoute(List<SensorNode> nodes) {
        if (nodes.isEmpty()) return new Route(nodes);

        List<SensorNode> ordered = new ArrayList<>();
        List<SensorNode> remaining = new ArrayList<>(nodes);

        // Seed with the first node
        ordered.add(remaining.remove(0));

        // Greedily pick the nearest unvisited node
        while (!remaining.isEmpty()) {
            SensorNode last = ordered.get(ordered.size() - 1);
            SensorNode nearest = remaining.stream()
                .min(Comparator.comparingDouble(n ->
                    Math.hypot(n.getX() - last.getX(), n.getY() - last.getY())))
                .orElseThrow();
            ordered.add(nearest);
            remaining.remove(nearest);
        }

        return new Route(ordered);
    }
}
```

Inject it into the UAV in `SimulationRunner.runOnce()` (replace the
`GaAcoPlanner` instantiation):

```java
PathPlanner planner = new GreedyNearestNeighbour();
UAV uav = new UAV(config, planner);
```

---

## Running & Interpreting Tests

```bash
mvn test                    # run all 265 tests
mvn test -pl . -Dtest=UAVTest  # run a specific test class
```

Tests are in `src/test/java/`. They cover:

- `algorithm/` — route construction, fitness calculation, GA/ACO correctness
- `simulation/` — environment range queries, node movement, Markov chain transitions

All simulation state is local (no static mutable fields), so tests run fully in
parallel and are deterministic when seeded.

After any code change, confirm `mvn test` still shows `BUILD SUCCESS` and
`Tests run: 265`.

---

## Simulation Configuration Reference

`config.properties` at the project root controls every aspect of the simulation.
`AppConfig` loads it once at startup.

```properties
# ── Run settings ──────────────────────────────────────────────
run.headless      = false       # true = no GUI; useful for batch sweeps
run.seed          =             # fixed integer for reproducible runs; empty = random
run.intelligences = patrol,gaaco # which algorithms to run (comma-separated keys)

# ── Field and sensor nodes ─────────────────────────────────────
simulation.fieldWidth     = 1500   # grid columns
simulation.fieldHeight    = 1500   # grid rows
simulation.nodeCount      = 50     # total sensor nodes deployed at startup
simulation.nodeIdStart    = 5000   # first node ID
simulation.nodeRange      = 100    # node-to-node detection radius (cells)
simulation.nodeMinSpeed   = 1      # minimum node movement speed (cells/tick)
simulation.nodeMaxSpeed   = 1      # maximum node movement speed (cells/tick)

# ── UAV ────────────────────────────────────────────────────────
simulation.droneRange     = 140    # scan radius (cells); also = CoverageGrid sector size
simulation.dronePlacement = 72     # UAV home row and column
simulation.droneSpeed     = 1      # UAV movement speed (cells/tick)
simulation.duration       = 5434   # total simulation ticks
```

### Tips for parameter tuning

| Goal | Change |
|---|---|
| More nodes for denser networks | Increase `nodeCount` (try 100–200) |
| Faster UAV for larger fields | Increase `droneSpeed` (try 2–3) |
| Longer runs for full coverage | Increase `duration` |
| Reproducible experiment | Set `run.seed` to any integer |
| Batch comparison, no window | Set `run.headless = true` |
| Add a third algorithm | Add its key to `run.intelligences` |

---

## Research Extension Points

### Reinforcement Learning / ML

The cleanest integration point is `UAVIntelligence.onTick()`. Your RL agent
receives the same observation that `GaAcoIntelligence` uses:

- `uav.getX()`, `uav.getY()` — UAV position
- `uav.getKnowledgeBase().snapshot()` — known node positions
- `uav.getDriveMode()` — current mode
- `coverage.coverageFraction()` — proportion of field explored
- `coverage.nearestUnvisitedCenter(x, y)` — direction to nearest frontier
- `uav.getTotalTokens()` — cumulative reward signal

The action space maps to `uav.setExplorationTarget(x, y)` (continuous steering)
or returning a `Route` (planned waypoints).

For Python-based training (e.g. with Stable-Baselines3), a
`gymnasium.Env` wrapper can communicate over a socket using a
`SynchronousQueue`-based bridge: the Java `onTick()` sends an observation,
blocks on the queue, and the Python agent writes an action back. See
`docs/architecture.md §4` for design notes.

### MCTS / Tree Search

MCTS requires a *rollout* from any intermediate state. The key requirement is
a deep-copyable world state. The four classes to make cloneable are:
`Environment`, `UAV`, `EventScheduler`, and `CoverageGrid`. Once cloneable,
`SimulationRunner.runOnce()` can be driven tick-by-tick from a node in the MCTS
tree rather than end-to-end.

### Alternative mobility models

Implement `MovementStrategy` in the `movement/` package and inject it into
`NodeAgent` during `SimulationRunner.runOnce()`. The existing `LevyWalkStrategy`
and `MarkovChain` classes show the expected interface.

---

## Docs Map

| File | Contents |
|---|---|
| `README.md` | Feature overview, config reference, algorithm descriptions, empirical results |
| `REQUIREMENTS.md` | Prerequisites, installation, IDE setup, troubleshooting |
| `docs/developer-guide.md` | This file — how to extend and work with the codebase |
| `docs/architecture.md` | Deep-dive: layer overview, class inventory, data flow, Scala readiness |
| `docs/api.md` | Public API reference for key classes |
| `docs/uml-refactored.puml` | Current architecture (paste at plantuml.com) |
| `CITATION.cff` | Machine-readable citation metadata |
