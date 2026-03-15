# DroneLab

**Discrete-event UAV/WSN simulator for comparing path planning strategies.**

DroneLab runs multiple UAV intelligence algorithms side-by-side against an
identical randomly-generated wireless sensor network, then ranks them by tokens
collected and field coverage in a live multi-panel GUI and a structured console
report.

![DroneLab — completed simulation](images/sim-full-1.png)

*Predefined Patrol (left) vs Autonomous GA→ACO (right) — seed 42, 50 nodes, 1500×1500 field.*

---

## Key Features

<div class="grid cards" markdown>

-   **Hybrid GA→ACO Planner**

    Novel three-stage pipeline: Genetic Algorithm seeds the pheromone matrix for
    Ant Colony Optimisation, refined by 2-opt + or-opt local search.

-   **Frontier Exploration**

    Coverage-grid sector tracking drives the UAV toward unvisited areas
    systematically — no more perimeter-only loops.

-   **Side-by-Side Comparison**

    Any number of strategies run concurrently against the same field and seed.
    Results are ranked automatically at the end of every run.

-   **Pluggable Architecture**

    Add a new intelligence in three steps: implement `UAVIntelligence`, register
    a key, add the key to `config.properties`. No other files touched.

-   **Reproducible Experiments**

    Fixed `run.seed` guarantees identical node layout across all strategies and
    all runs. Deterministic from top to bottom.

-   **265 Tests, Zero Static State**

    All simulation state is thread-local. The full test suite runs in parallel.
    `mvn test` is the single command to verify everything.

</div>

---

## Quick Start

**Prerequisites:** Java 17+, Maven 3.6+

```bash
git clone https://github.com/TechnoTwelve/DroneLab.git
cd DroneLab
mvn package -DskipTests
java -jar target/drone-lab.jar
```

See [REQUIREMENTS.md](https://github.com/TechnoTwelve/DroneLab/blob/main/REQUIREMENTS.md)
for full setup instructions including IntelliJ IDEA and troubleshooting.

---

## Empirical Results

**50-node · 1500×1500 field · droneRange=140 · 20-seed sweep**

| Strategy | Avg tokens | Win rate |
|---|---|---|
| **Autonomous GA→ACO** | — | **16 / 20 seeds** |
| Predefined Patrol | baseline | 4 / 20 seeds |

Average improvement: **+17.84%** more tokens collected by the GA→ACO strategy.

The gain comes from three mechanisms working together:

1. **Frontier exploration** — systematic full-field coverage instead of perimeter-only patrol
2. **Coverage gate** — waits for 40% field coverage before committing to routes
3. **Scanned-sector filter** — prevents re-routing into already-explored sectors

---

## Documentation

| Section | Contents |
|---|---|
| [Architecture](architecture.md) | Layer overview, class inventory, data flow diagrams |
| [Interface Guide](interface.md) | GUI walkthrough with annotated screenshots |
| [Developer Guide](developer-guide.md) | Adding intelligences, path planners, running tests |
| [API Reference](api.md) | Package overview, key interfaces, full Javadoc |

---

## Citation

If you use DroneLab in your research, please cite:

```bibtex
@mastersthesis{ahmed2022dronelab,
  author  = {Tarek Uddin Ahmed},
  title   = {Path Planning And Trajectory for UAVs},
  school  = {Queen Mary University of London},
  year    = {2022}
}
```

Or use the **Cite this repository** button on
[GitHub](https://github.com/TechnoTwelve/DroneLab) (reads from `CITATION.cff`).

---

*Developed by Tarek Uddin Ahmed as a post-MSc extension of his 2022 dissertation project at Queen Mary University of London, supervised by Dr Umair Bilal Chaudhry. Built on Dr Chaudhry's original discrete-event simulation framework.*
