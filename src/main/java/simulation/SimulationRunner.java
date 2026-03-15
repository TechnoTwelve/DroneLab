/*
 * DroneLab — UAV/WSN Simulator
 * Copyright (C) 2022 Tarek Uddin Ahmed
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
package simulation;

import algorithm.PathPlanner;
import algorithm.Route;
import domain.SensorNode;
import movement.LevyWalkStrategy;
import movement.MarkovChain;
import ui.SimulationSnapshot;
import ui.VisualizationListener;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

/**
 * Discrete-event simulation runner for UAV/WSN experiments.
 *
 * <p>Supports two execution modes:
 * <ol>
 *   <li><b>Sequential</b> ({@link #run(List)}) — runs each intelligence
 *       strategy one after another on a single thread.  Used for headless
 *       / batch operation.</li>
 *   <li><b>Parallel</b> ({@link #runInParallel(List, List, PassCompleteCallback)})
 *       — runs strategies concurrently on separate threads so they can be
 *       visualised side-by-side in the GUI.</li>
 * </ol>
 *
 * <h3>Comparison methodology</h3>
 * Both modes use the same random seed for every pass so that sensor-node
 * deployment positions and movement sequences are identical.  Only the
 * {@link UAVIntelligence} strategy differs between passes.
 *
 * <h3>Typical usage</h3>
 * <pre>
 *   SimulationRunner.fromScenario(new WSNDataCollectionScenario())
 *       .withSeed(17L)
 *       .run();
 * </pre>
 *
 * <p>To run a single intelligence without a scenario:
 * <pre>
 *   new SimulationRunner(SimulationConfig.defaults())
 *       .withPlannerFactory(() -&gt; new GaAcoPlanner(GaAcoConfig.defaults()))
 *       .run(List.of(new GaAcoIntelligence(config, GaAcoConfig.defaults())));
 * </pre>
 */
public final class SimulationRunner {

    // ── Callback interface ────────────────────────────────────────────────────

    /**
     * Notified when a single simulation pass finishes.
     * Implementations must be thread-safe (called from a background thread).
     */
    public interface PassCompleteCallback {
        /**
         * @param pass             1-based pass index
         * @param tokens           total unique tokens collected
         * @param totalNodes       total nodes deployed
         * @param coverageFraction fraction of field sectors visited [0,1],
         *                         or {@link Double#NaN} if not tracked
         */
        void onComplete(int pass, int tokens, int totalNodes, double coverageFraction);
    }

    // ── Fields ────────────────────────────────────────────────────────────────

    private final SimulationConfig config;

    /**
     * Optional fixed seed for reproducible runs.
     * {@code Long.MIN_VALUE} (the default) means "generate fresh seed per run".
     */
    private long fixedSeed = Long.MIN_VALUE;

    /**
     * Factory that produces a fresh {@link PathPlanner} for each simulation
     * pass.  Defaults to a pass-through planner (no route optimisation).
     * Override with {@link #withPlanner} or {@link #withPlannerFactory}.
     */
    private Supplier<PathPlanner> plannerFactory = () -> targets -> new Route(targets);

    /**
     * Factory that produces a fresh {@link MovementStrategy} for each
     * simulation pass.  Defaults to Lévy-walk.
     * Override with {@link #withMovementStrategy} or {@link #withMovementStrategyFactory}.
     */
    private Supplier<MovementStrategy> movementStrategyFactory =
            () -> new LevyWalkStrategy(MarkovChain.exploratoryWalk());

    /**
     * Controls how targets are placed at the start of each pass.
     * Defaults to {@link NodeDeploymentStrategy#stratified()}.
     * Override via {@link #withDeploymentStrategy}.
     */
    private NodeDeploymentStrategy deploymentStrategy =
            NodeDeploymentStrategy.stratified();

    /** Legacy single-listener used by the sequential {@link #run(List)} path. */
    private VisualizationListener vizListener = null;

    /**
     * Live scan-range override set from the GUI dashboard.
     * A value &le; 0 means "use {@link SimulationConfig#getDroneRange()}".
     * Updated atomically so the change takes effect on the very next drone tick
     * without interrupting a running simulation.
     */
    private final AtomicInteger liveRange = new AtomicInteger(-1);

    // ── Construction ──────────────────────────────────────────────────────────

    /**
     * Overrides the drone scan range at runtime.
     *
     * <p>The new range takes effect on the very next drone tick, allowing the
     * GUI dashboard to adjust the scan radius while a simulation is running.
     * Pass a value &le; 0 to revert to the configured range.
     *
     * @param range scan radius in cells; &le; 0 reverts to {@link SimulationConfig#getDroneRange()}
     * @return {@code this} for chaining
     */
    public SimulationRunner setLiveRange(int range) {
        this.liveRange.set(range);
        return this;
    }

    /**
     * Creates a runner for the given field/drone configuration.
     * Use {@link #withPlannerFactory} to set a planning algorithm, and
     * {@link #fromScenario} to configure from a complete scenario.
     *
     * @param config simulation configuration; must not be null
     */
    public SimulationRunner(SimulationConfig config) {
        if (config == null) throw new IllegalArgumentException("config must not be null");
        this.config = config;
    }

    /**
     * Creates a runner fully configured from a {@link SimulationScenario}.
     *
     * <p>The scenario provides the {@link SimulationConfig}, the
     * {@link PathPlanner} factory, the {@link MovementStrategy}, and the
     * {@link NodeDeploymentStrategy}.
     *
     * <pre>
     *   SimulationRunner.fromScenario(new WSNDataCollectionScenario())
     *       .withSeed(17L)
     *       .run();
     * </pre>
     *
     * @param scenario scenario to run; must not be null
     * @return a configured runner, ready for {@code withSeed} and execution
     */
    public static SimulationRunner fromScenario(SimulationScenario scenario) {
        if (scenario == null) throw new IllegalArgumentException("scenario must not be null");
        SimulationRunner runner = new SimulationRunner(scenario.getConfig());
        runner.withPlannerFactory(scenario.getPlannerFactory());
        runner.withMovementStrategyFactory(scenario::getMovementStrategy);
        runner.withDeploymentStrategy(scenario.getDeploymentStrategy());
        return runner;
    }

    /**
     * Locks the random seed used for every pass so results are reproducible.
     * Call before {@link #run(List)} or {@link #runInParallel}.
     *
     * @param seed any long value; all strategies will share this seed
     * @return {@code this} for chaining
     */
    public SimulationRunner withSeed(long seed) {
        this.fixedSeed = seed;
        return this;
    }

    /**
     * Overrides the path-planning algorithm used by the UAV.
     *
     * <p>The supplied instance is reused across all passes.  If the planner
     * holds mutable state between calls to {@link PathPlanner#planRoute}, use
     * {@link #withPlannerFactory} instead so each pass gets a fresh instance.
     *
     * @param planner planner to use; must not be null
     * @return {@code this} for chaining
     */
    public SimulationRunner withPlanner(PathPlanner planner) {
        if (planner == null) throw new IllegalArgumentException("planner must not be null");
        this.plannerFactory = () -> planner;
        return this;
    }

    /**
     * Overrides the path-planning algorithm with a factory called once per
     * simulation pass.  Use this when the planner has per-run state, or when
     * running passes in parallel.
     *
     * @param factory supplier of fresh planner instances; must not be null
     * @return {@code this} for chaining
     */
    public SimulationRunner withPlannerFactory(Supplier<PathPlanner> factory) {
        if (factory == null) throw new IllegalArgumentException("factory must not be null");
        this.plannerFactory = factory;
        return this;
    }

    /**
     * Overrides the movement model used by all targets in the simulation.
     *
     * @param strategy movement strategy to use; must not be null
     * @return {@code this} for chaining
     */
    public SimulationRunner withMovementStrategy(MovementStrategy strategy) {
        if (strategy == null) throw new IllegalArgumentException("strategy must not be null");
        this.movementStrategyFactory = () -> strategy;
        return this;
    }

    /**
     * Overrides the movement model with a factory called once per simulation
     * pass.  Use this when the strategy carries per-run state.
     *
     * @param factory supplier of fresh strategy instances; must not be null
     * @return {@code this} for chaining
     */
    public SimulationRunner withMovementStrategyFactory(Supplier<MovementStrategy> factory) {
        if (factory == null) throw new IllegalArgumentException("factory must not be null");
        this.movementStrategyFactory = factory;
        return this;
    }

    /**
     * Overrides the node deployment strategy used at the start of each pass.
     *
     * @param strategy deployment strategy to use; must not be null
     * @return {@code this} for chaining
     */
    public SimulationRunner withDeploymentStrategy(NodeDeploymentStrategy strategy) {
        if (strategy == null) throw new IllegalArgumentException("strategy must not be null");
        this.deploymentStrategy = strategy;
        return this;
    }

    /**
     * Registers a single {@link VisualizationListener} used by the sequential
     * {@link #run(List)} entry point.
     * Pass {@code null} to remove the listener.
     */
    public void setVisualizationListener(VisualizationListener listener) {
        this.vizListener = listener;
    }

    // ── Public entry points ───────────────────────────────────────────────────

    /**
     * Sequential run with an explicit list of intelligence strategies.
     * Each strategy runs in order on the calling thread; all passes use
     * the same random seed.
     *
     * @param intelligences ordered list of strategies to compare (≥ 1)
     */
    public void run(List<UAVIntelligence> intelligences) {
        long seed = (fixedSeed != Long.MIN_VALUE) ? fixedSeed : new Random().nextLong();
        int  totalNodes = config.getNodeCount();
        printHeader(totalNodes);

        List<SimResult> results = new ArrayList<>(intelligences.size());
        for (int i = 0; i < intelligences.size(); i++) {
            UAVIntelligence intel = intelligences.get(i);
            System.out.printf("%n[Pass %d/%d] %s...%n",
                    i + 1, intelligences.size(), intel.getLabel());
            results.add(runOnce(seed, i + 1, intel, this.vizListener));
        }

        printComparison(results, totalNodes);
    }

    /**
     * Runs all intelligences from the given scenario sequentially.
     * Equivalent to {@code run(scenario.getIntelligences())}.
     *
     * @param scenario scenario whose intelligences to compare
     */
    public void run(SimulationScenario scenario) {
        run(scenario.getIntelligences());
    }

    /**
     * Parallel run with an explicit list of intelligence strategies.
     *
     * <p>Each strategy runs on its own background thread; all passes share the
     * same random seed so sensor deployment and movement sequences are
     * identical across strategies.  This method <em>blocks</em> until every
     * thread finishes.
     *
     * <pre>
     *   runner.runInParallel(
     *       List.of(new PatrolIntelligence(),
     *               new MyACOIntelligence(),
     *               new MyRLIntelligence()),
     *       List.of(gui.createListener(0),
     *               gui.createListener(1),
     *               gui.createListener(2)),
     *       gui::notifyPassComplete);
     * </pre>
     *
     * @param intelligences ordered list of strategies to run (must not be empty)
     * @param listeners     per-tick GUI callbacks, one per strategy; entries
     *                      beyond the intelligence count are ignored; a
     *                      {@code null} entry or a shorter list means headless
     *                      for that pass
     * @param onComplete    called (from the finishing thread) when each pass
     *                      completes; may be {@code null}
     */
    public void runInParallel(List<UAVIntelligence> intelligences,
                               List<VisualizationListener> listeners,
                               PassCompleteCallback onComplete) {
        runInParallel(intelligences, listeners, onComplete, false);
    }

    /**
     * Parallel run with optional <em>sync mode</em>.
     *
     * <p>When {@code syncMode} is {@code true} a {@link CyclicBarrier} is
     * inserted after each pass's per-tick listener call so that all passes
     * advance to the next tick together.  This makes the side-by-side
     * visualisation frame-for-frame comparable — useful for research studies
     * where you want both strategies to be rendered at exactly the same logical
     * time.
     *
     * <p>Sync mode only activates when every pass has a non-null listener; if
     * any pass is headless the barrier is skipped for that run.
     *
     * @param intelligences ordered list of strategies to run (must not be empty)
     * @param listeners     per-tick GUI callbacks, one per strategy
     * @param onComplete    called when each pass completes; may be {@code null}
     * @param syncMode      {@code true} to lock-step all passes tick-by-tick
     */
    public void runInParallel(List<UAVIntelligence> intelligences,
                               List<VisualizationListener> listeners,
                               PassCompleteCallback onComplete,
                               boolean syncMode) {
        if (intelligences == null || intelligences.isEmpty())
            throw new IllegalArgumentException("intelligences must not be empty");

        long seed = (fixedSeed != Long.MIN_VALUE) ? fixedSeed : new Random().nextLong();
        printHeader(config.getNodeCount());

        // ── Sync-mode barrier ─────────────────────────────────────────────────
        // Only create a barrier when sync is requested AND every pass has a
        // non-null listener (otherwise the barrier would deadlock headless passes).
        final boolean allHaveListeners = listeners != null
                && listeners.size() >= intelligences.size()
                && listeners.stream().allMatch(l -> l != null);
        final CyclicBarrier syncBarrier = (syncMode && allHaveListeners && intelligences.size() > 1)
                ? new CyclicBarrier(intelligences.size())
                : null;

        if (syncMode && syncBarrier != null) {
            System.out.println("[Runner] Sync mode ON — " + intelligences.size()
                    + " passes will advance in lock-step.");
        }

        List<AtomicReference<SimResult>> results = new ArrayList<>(intelligences.size());
        List<Thread> threads = new ArrayList<>(intelligences.size());

        for (int i = 0; i < intelligences.size(); i++) {
            final int passNum = i + 1;
            final UAVIntelligence intel = intelligences.get(i);
            final VisualizationListener rawListener =
                    (listeners != null && i < listeners.size()) ? listeners.get(i) : null;

            // Wrap the listener with a barrier rendezvous when sync mode is on.
            final VisualizationListener listener;
            if (syncBarrier != null && rawListener != null) {
                listener = snap -> {
                    rawListener.onTick(snap);
                    try {
                        syncBarrier.await();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } catch (BrokenBarrierException ignored) {
                        // A peer thread was interrupted or timed out; continue.
                    }
                };
            } else {
                listener = rawListener;
            }

            AtomicReference<SimResult> ref = new AtomicReference<>();
            results.add(ref);

            Thread t = new Thread(() -> {
                System.out.printf("%n[Parallel] Starting Pass %d -- %s...%n",
                        passNum, intel.getLabel());
                SimResult r = runOnce(seed, passNum, intel, listener);
                ref.set(r);
                System.out.printf("%n[Parallel] Pass %d complete -- %d / %d tokens%n",
                        passNum, r.totalTokens, r.totalNodes);
                if (onComplete != null)
                    onComplete.onComplete(passNum, r.totalTokens, r.totalNodes,
                                          r.coverageFraction);
                // Break the barrier so other passes are not left waiting after
                // this pass finishes (e.g. if one pass has fewer ticks).
                if (syncBarrier != null) syncBarrier.reset();
            }, "sim-" + passNum);

            t.setDaemon(true);
            threads.add(t);
        }

        threads.forEach(Thread::start);
        for (Thread t : threads) {
            try { t.join(); } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }

        List<SimResult> resolved = new ArrayList<>();
        for (AtomicReference<SimResult> ref : results) {
            SimResult r = ref.get();
            if (r != null) resolved.add(r);
        }
        printComparison(resolved, config.getNodeCount());
    }

    /**
     * Headless parallel run of all intelligences from the given scenario.
     * Equivalent to {@code runInParallel(scenario.getIntelligences(), null, onComplete)}.
     *
     * @param scenario   scenario whose intelligences to compare
     * @param onComplete called when each pass completes; may be {@code null}
     */
    public void runInParallel(SimulationScenario scenario, PassCompleteCallback onComplete) {
        runInParallel(scenario.getIntelligences(), null, onComplete);
    }

    // ── Single-pass simulation ─────────────────────────────────────────────────

    /**
     * Runs one complete simulation pass and returns the aggregated result.
     *
     * @param seed        random seed (shared across all passes for reproducibility)
     * @param passNumber  1-based pass index for display and snapshot labelling
     * @param intelligence UAV decision-making strategy for this pass
     * @param listener    per-tick GUI callback; {@code null} for headless
     * @return tokens collected and best route found in this pass
     */
    private SimResult runOnce(long seed, int passNumber,
                               UAVIntelligence intelligence,
                               VisualizationListener listener) {
        Random rng = new Random(seed);

        // ── 1. Environment setup ──────────────────────────────────────────────
        Environment env = new Environment(config.getFieldWidth(),
                                          config.getFieldHeight());
        deploymentStrategy.deploy(env, config, rng);

        // ── 2. UAV setup ──────────────────────────────────────────────────────
        PathPlanner planner = plannerFactory.get();
        UAV uav = new UAV(config, planner);
        int dp = config.getDronePlacement();
        if (!env.setCell(dp, dp, UAV.UAV_ID)) {
            System.out.printf("  Warning: UAV start cell (%d,%d) occupied.%n", dp, dp);
        }

        // ── 3. Event scheduling ───────────────────────────────────────────────
        EventScheduler   scheduler = new EventScheduler();
        NodeMover        mover     = new NodeMover();
        MovementStrategy strategy  = movementStrategyFactory.get();

        scheduleInitialNodeEvents(scheduler, env);
        scheduler.schedule(1L, UAV.UAV_ID, MarkovChain.MOVE);

        // ── 4. Main event loop ────────────────────────────────────────────────
        Route  bestRoute      = null;
        long   droneEventTime = 0;
        String passLabel      = intelligence.getLabel();

        while (!scheduler.isEmpty()) {
            EventScheduler.Event ev = scheduler.poll();
            if (ev.time > config.getDuration()) break;

            if (ev.nodeId == UAV.UAV_ID) {
                Route r = processDroneEvent(ev, uav, env, droneEventTime,
                                             bestRoute, intelligence);
                if (r != null) bestRoute = r;
                droneEventTime++;
                scheduler.schedule(droneEventTime + 1, UAV.UAV_ID, MarkovChain.MOVE);

                if (listener != null) {
                    SimulationSnapshot snap = buildSnapshot(
                            ev.time, passNumber, passLabel,
                            uav, env, intelligence.getCoverageGrid(), bestRoute);
                    listener.onTick(snap);
                }

            } else {
                NodeAgent agent = env.agentById().get(ev.nodeId);
                if (agent == null) continue;
                int newState = processNodeEvent(agent, ev.motionState,
                                                env, mover, strategy, rng);
                scheduler.schedule(ev.time + agent.getSpeed(), ev.nodeId, newState);
            }
        }

        CoverageGrid coverage = intelligence.getCoverageGrid();
        double covFraction = (coverage == null) ? Double.NaN : coverage.coverageFraction();
        return new SimResult(uav.getTotalTokens(), env.allAgents().size(),
                             bestRoute, covFraction, passLabel);
    }

    // ── Snapshot builder ──────────────────────────────────────────────────────

    private SimulationSnapshot buildSnapshot(long tick, int pass, String passLabel,
                                             UAV uav, Environment env,
                                             CoverageGrid coverage, Route bestRoute) {
        List<SimulationSnapshot.NodeSnap> nodeSnaps =
                new ArrayList<>(env.allAgents().size());
        for (NodeAgent agent : env.allAgents()) {
            boolean tok   = uav.isTokenized(agent.getId());
            boolean known = uav.getKnowledgeBase().knows(agent.getId());
            nodeSnaps.add(new SimulationSnapshot.NodeSnap(
                    agent.getId(), agent.getX(), agent.getY(), tok, known));
        }

        List<SensorNode> wps = uav.getRouteWaypoints();
        List<double[]> wpCoords;
        if (wps.isEmpty()) {
            wpCoords = Collections.emptyList();
        } else {
            wpCoords = new ArrayList<>(wps.size());
            for (SensorNode sn : wps)
                wpCoords.add(new double[]{ sn.getX(), sn.getY() });
        }

        boolean[][] sectors    = (coverage == null) ? null : coverage.visitedCopy();
        int         sectorSize = (coverage == null) ? 0    : coverage.getSectorSize();
        double      covFrac    = (coverage == null) ? 0.0  : coverage.coverageFraction();

        int homePlacement = config.getDronePlacement();

        return new SimulationSnapshot(
                tick, pass, passLabel,
                config.getDuration(), config.getNodeCount(),
                config.getFieldWidth(), config.getFieldHeight(),
                uav.getX(), uav.getY(),
                homePlacement, homePlacement,
                uav.getDriveMode(),
                config.getDroneRange(),
                uav.getTotalTokens(),
                uav.getKnowledgeBase().size(),
                Collections.unmodifiableList(nodeSnaps),
                Collections.unmodifiableList(wpCoords),
                uav.getExplorationTarget(),
                sectors, sectorSize, covFrac);
    }

    // ── Setup helpers ─────────────────────────────────────────────────────────

    private void scheduleInitialNodeEvents(EventScheduler scheduler, Environment env) {
        long t = 1;
        for (NodeAgent agent : env.allAgents()) {
            scheduler.schedule(t, agent.getId(), MarkovChain.IDLE);
            t++;
        }
    }

    // ── Event processors ──────────────────────────────────────────────────────

    private Route processDroneEvent(EventScheduler.Event ev,
                                    UAV uav,
                                    Environment env,
                                    long droneEventTime,
                                    Route previousBest,
                                    UAVIntelligence intelligence) {
        int range = liveRange.get();
        if (range <= 0) range = config.getDroneRange();
        uav.scan(env, range);

        UAV.DriveMode modeBefore = uav.getDriveMode();
        uav.moveStep(env.getField(), config.getFieldWidth(), config.getFieldHeight(),
                     intelligence);

        if (modeBefore == UAV.DriveMode.EXECUTE
                && uav.getDriveMode() == UAV.DriveMode.PATROL) {
            System.out.printf("  [t=%d] Route complete --UAV reverted to PATROL%n", ev.time);
        }

        Route r = intelligence.onTick(ev.time, uav, env);
        if (r != null) {
            System.out.printf(
                    "  [t=%d] Replanned: %d nodes, distance=%.2f --UAV now EXECUTING%n",
                    ev.time, r.getNodes().size(), r.getTotalDistance());
            return r;
        }

        if (uav.getDriveMode() != UAV.DriveMode.RETURN_HOME) {
            long remaining = config.getDuration() - ev.time;
            int  homeX     = config.getDronePlacement();
            int  homeY     = config.getDronePlacement();
            int  chebyshev = Math.max(Math.abs(uav.getX() - homeX),
                                      Math.abs(uav.getY() - homeY));
            if (remaining <= chebyshev + 10) {
                uav.returnHome();
                System.out.printf(
                        "  [t=%d] Returning home: %d ticks remaining, %d needed%n",
                        ev.time, remaining, chebyshev);
            }
        }

        return previousBest;
    }

    private int processNodeEvent(NodeAgent agent, int motionState,
                                 Environment env, NodeMover mover,
                                 MovementStrategy strategy, Random rng) {
        int newState;
        switch (motionState) {
            case MarkovChain.IDLE:
                newState = strategy.nextState(MarkovChain.IDLE, rng);
                break;
            case MarkovChain.MOVE:
            case MarkovChain.CONTINUE:
                int actualDir = mover.move(env.getField(), agent,
                        config.getFieldWidth(), config.getFieldHeight(), rng);
                agent.setDirection(actualDir);
                newState = strategy.nextState(motionState, rng);
                break;
            case MarkovChain.CHANGE_DIR:
                agent.setDirection(rng.nextInt(NodeAgent.DIRECTION_COUNT));
                newState = strategy.nextState(MarkovChain.CHANGE_DIR, rng);
                break;
            default:
                newState = MarkovChain.IDLE;
        }
        updateNodeNeighbors(agent, env);
        return newState;
    }

    private void updateNodeNeighbors(NodeAgent agent, Environment env) {
        for (NodeAgent neighbor : env.findAgentsInRange(
                agent.getX(), agent.getY(), config.getNodeRange())) {
            agent.addNeighbor(neighbor.getId());
        }
    }

    // ── Result container ──────────────────────────────────────────────────────

    private static final class SimResult {
        final int    totalTokens;
        final int    totalNodes;
        final Route  bestRoute;
        final double coverageFraction;
        final String label;

        SimResult(int totalTokens, int totalNodes, Route bestRoute,
                  double coverageFraction, String label) {
            this.totalTokens      = totalTokens;
            this.totalNodes       = totalNodes;
            this.bestRoute        = bestRoute;
            this.coverageFraction = coverageFraction;
            this.label            = label;
        }
    }

    // ── Console output ────────────────────────────────────────────────────────

    private static final String LINE_THICK = "========================================";
    private static final String LINE_THIN  = "  --------------------------------------";

    private void printHeader(int totalNodes) {
        System.out.println();
        System.out.println(LINE_THICK);
        System.out.printf("  UAV/WSN Experiment%n");
        System.out.printf("  %d nodes | %d ticks | field %dx%d%n",
                totalNodes, config.getDuration(),
                config.getFieldWidth(), config.getFieldHeight());
        System.out.println(LINE_THICK);
    }

    /**
     * Prints a formatted summary for all completed passes and ranks them by
     * tokens collected.  Works for any number of strategies (>= 1).
     */
    private void printComparison(List<SimResult> results, int totalNodes) {
        if (results.isEmpty()) return;

        System.out.println();
        System.out.println(LINE_THICK);
        System.out.printf("  RESULTS  --  %d %s%n",
                results.size(), results.size() == 1 ? "strategy" : "strategies");
        System.out.println(LINE_THICK);
        System.out.printf("  Environment : %d nodes, %d ticks (same for all)%n",
                totalNodes, config.getDuration());

        // ── Per-strategy details ──────────────────────────────────────────────
        for (SimResult r : results) {
            System.out.println();
            System.out.printf("  -- %s%n", r.label);
            System.out.printf("  Tokens collected : %d / %d (%.1f%%)%n",
                    r.totalTokens, totalNodes, pct(r.totalTokens, totalNodes));
            if (!Double.isNaN(r.coverageFraction) && r.coverageFraction > 0) {
                System.out.printf("  Field coverage   : %.1f%% of sectors%n",
                        r.coverageFraction * 100.0);
            }
            if (r.bestRoute != null) {
                System.out.printf("  Best route       : %d waypoints, dist=%.2f%n",
                        r.bestRoute.getNodes().size(), r.bestRoute.getTotalDistance());
            }
        }

        // ── Ranking ───────────────────────────────────────────────────────────
        if (results.size() >= 2) {
            List<SimResult> ranked = new ArrayList<>(results);
            ranked.sort((a, b) -> Integer.compare(b.totalTokens, a.totalTokens));
            SimResult leader = ranked.get(0);

            System.out.println();
            System.out.println(LINE_THIN);
            for (int i = 0; i < ranked.size(); i++) {
                SimResult r    = ranked.get(i);
                int       diff = r.totalTokens - leader.totalTokens;
                if (diff == 0) {
                    System.out.printf("  %d.  %-26s  %d / %d  (%.1f%%)  [winner]%n",
                            i + 1, r.label,
                            r.totalTokens, totalNodes, pct(r.totalTokens, totalNodes));
                } else {
                    System.out.printf("  %d.  %-26s  %d / %d  (%.1f%%)  %+d vs leader%n",
                            i + 1, r.label,
                            r.totalTokens, totalNodes, pct(r.totalTokens, totalNodes), diff);
                }
            }
        }

        System.out.println();
        System.out.println(LINE_THICK);
    }

    private static double pct(int part, int total) {
        return total > 0 ? 100.0 * part / total : 0.0;
    }
}
