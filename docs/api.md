# API Reference

The full Javadoc API reference is generated from source-code comments on every
push to `main` and published alongside this site.

[**Open Full Javadoc →**](https://technotwelve.github.io/DroneLab/javadoc/){ .md-button .md-button--primary }

---

## Package overview

| Package | Description |
|---|---|
| `config` | `AppConfig` — intelligence registry and `config.properties` loader |
| `domain` | `SensorNode` — immutable value object (position + velocity) |
| `algorithm` | `PathPlanner` interface, `Route` |
| `algorithm.gaaco` | GA → ACO → LocalSearch planner (`GaAcoPlanner`, `GaAcoConfig`) |
| `simulation` | Discrete-event loop, `UAV`, `SimulationConfig`, `CoverageGrid`, `KnowledgeBase` |
| `intelligence` | `PredefinedPatrolIntelligence`, `GaAcoIntelligence` |
| `movement` | `MarkovChain`, `LevyWalkStrategy` |
| `ui` | `SimulationGui`, `SimulationPanel`, `VisualizationListener` |
| `scenarios` | `WSNDataCollectionScenario` |
| `algorithmTest` | `GaAcoTest` — self-contained GA-ACO demo program |

## Key interfaces

=== "UAVIntelligence"

    ```java
    public interface UAVIntelligence {
        /** Called after scan + move each tick. Return a Route to activate it. */
        Route onTick(long tick, UAV uav, Environment env);
        String getLabel();
        default CoverageGrid getCoverageGrid() { return null; }
    }
    ```

=== "PathPlanner"

    ```java
    public interface PathPlanner {
        /** Plan an optimised route through the given nodes. */
        Route planRoute(List<SensorNode> nodes);
    }
    ```
