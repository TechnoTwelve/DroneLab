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
package algorithm.gaaco;

import algorithm.Route;
import domain.SensorNode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Ant Colony Optimisation (ACO) stage of the GA→ACO fusion path planner,
 * with concurrent ant tour construction and multiple iteration loops.
 *
 * <h3>Algorithm outline</h3>
 * <ol>
 *   <li>A fresh {@link PheromoneMatrix} is initialised with uniform level τ₀.</li>
 *   <li>A precomputed <em>heuristic matrix</em>
 *       {@code heuristic[i][j] = (1/dist[i][j])^β} is built once and shared
 *       read-only across all ants and all iterations, eliminating repeated
 *       {@code Math.pow} calls in the hot selection loop.</li>
 *   <li><b>Ant 1</b> follows the GA best route verbatim, seeding the pheromone
 *       matrix with high-quality trail information before any independent ant
 *       explores.</li>
 *   <li>For each of {@code acoIterationCount} iterations:
 *     <ol type="a">
 *       <li>A read-only snapshot of the pheromone matrix is taken; every ant
 *           reads from this fixed snapshot without writes — zero contention.</li>
 *       <li><b>Ants 2 … antCount</b> each build a tour concurrently via an
 *           {@link ExecutorService}.</li>
 *       <li>After all ants finish, the master thread performs a single batch
 *           update: global evaporation, deposits from every ant route
 *           ({@code Q / tourLength} per edge), plus an <em>elitist deposit</em>
 *           from the best route found so far (MMAS-inspired convergence
 *           acceleration).</li>
 *     </ol>
 *   </li>
 *   <li>The best route across all ants and all iterations is returned.</li>
 * </ol>
 *
 * <h3>Concurrency model — no shared mutable state</h3>
 * <ul>
 *   <li>Each concurrent ant receives its own forked {@link Random}, its own
 *       {@code boolean[] visited} array, and its own local node list — no
 *       shared mutable fields between worker threads.</li>
 *   <li>The pheromone snapshot ({@code double[][]}) and heuristic matrix
 *       ({@code double[][]}) are read-only after construction; sharing them
 *       requires no synchronisation.</li>
 *   <li>{@link PheromoneMatrix} is only ever accessed by the master thread
 *       (ant-1 deposit and batch updates after collection).</li>
 *   <li>{@link Route} and {@link SensorNode} are immutable — safe to pass
 *       across thread boundaries via {@link Future}.</li>
 *   <li>The {@link ExecutorService} is created once before the iteration loop
 *       and shut down in a {@code finally} block after all iterations complete.</li>
 * </ul>
 *
 * <h3>Pheromone update semantics</h3>
 * Each iteration uses a <em>batch</em> Ant System update: one global evaporation
 * followed by deposits from all ant routes, then an elitist deposit.  This is
 * behaviourally sounder than the legacy per-ant sequential evaporate-and-deposit.
 *
 * <h3>Bug corrections relative to the legacy code</h3>
 * <ul>
 *   <li><b>B — Reversed pheromone index</b>: consistent
 *       {@code snapshot[from][to]} where {@code from = current, to = next}.</li>
 *   <li><b>D2 — Deposit uses cumulative step distance</b>: fixed to
 *       {@code Q / totalTourLength}.</li>
 *   <li><b>D6 — Global evaporation missing</b>: {@code evaporateAll(rho)}
 *       is applied globally after every ant batch.</li>
 * </ul>
 */
public final class AntColonyOptimization {

    private final GaAcoConfig config;
    private final Random          random;

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * Constructs an ACO instance with an externally supplied random source.
     *
     * @param config algorithm parameters; must not be null
     * @param random random source; must not be null
     */
    public AntColonyOptimization(GaAcoConfig config, Random random) {
        if (config == null) throw new IllegalArgumentException("config must not be null");
        if (random == null) throw new IllegalArgumentException("random must not be null");
        this.config = config;
        this.random = random;
    }

    /**
     * Constructs an ACO instance using an internally created, non-seeded random.
     *
     * @param config algorithm parameters; must not be null
     */
    public AntColonyOptimization(GaAcoConfig config) {
        this(config, new Random());
    }

    // ── Public entry point ────────────────────────────────────────────────────

    /**
     * Refines a GA route using ACO over {@code acoIterationCount} iterations
     * and returns the best route found.
     *
     * <p>Ant 1 seeds the pheromone matrix with the GA route (sequential, master
     * thread).  Each subsequent iteration takes a pheromone snapshot, dispatches
     * {@code antCount − 1} concurrent ants, performs a batch pheromone update,
     * and applies an elitist deposit from the best route found so far.
     *
     * @param gaRoute best route produced by the GA stage; must not be null
     * @param nodes   ordered list of all sensor nodes
     * @return best {@link Route} found (fitness ≥ {@code gaRoute.getFitness()})
     * @throws IllegalStateException if the executor is interrupted during collection
     * @throws RuntimeException      if an ant task throws during execution
     */
    public Route refine(Route gaRoute, List<SensorNode> nodes) {
        int                  n          = nodes.size();
        double[][]           dist       = buildDistanceMatrix(nodes);
        // Precompute heuristic[i][j] = (1/dist[i][j])^β once — reused by every ant,
        // every step, every iteration; eliminates repeated Math.pow in the hot path.
        double[][]           heuristic  = buildHeuristicMatrix(dist, config.getBeta());
        PheromoneMatrix      pheromones = new PheromoneMatrix(n, config.getPheromoneInit());
        Map<Integer,Integer> idToIdx    = buildIdToIndexMap(nodes);

        // ── Ant 1: seed pheromone matrix with GA route (master thread) ────────
        depositRoute(gaRoute, pheromones, idToIdx);
        Route best = gaRoute;

        int remainingAnts = config.getAntCount() - 1;
        if (remainingAnts <= 0) return best;

        // One executor shared across all iterations — created once, shut down once.
        int nThreads = Math.min(remainingAnts,
                                Runtime.getRuntime().availableProcessors());
        ExecutorService executor = Executors.newFixedThreadPool(
                Math.max(1, nThreads));

        try {
            for (int iter = 0; iter < config.getAcoIterationCount(); iter++) {

                // ── Snapshot: every ant reads from this — zero contention ─────
                double[][] snapshot = pheromones.snapshot();

                // ── Dispatch concurrent ants ──────────────────────────────────
                // Both RNG calls advance the master random on the master thread only,
                // before submit() — no concurrent access to this.random.
                List<Future<Route>> futures = new ArrayList<>(remainingAnts);
                for (int a = 0; a < remainingAnts; a++) {
                    final int    startIndex = random.nextInt(n);
                    final Random antRng     = new Random(random.nextLong());
                    futures.add(executor.submit(() ->
                        constructTourFromSnapshot(startIndex, nodes, snapshot,
                                                  heuristic, antRng)));
                }

                // ── Collect results ───────────────────────────────────────────
                List<Route> antRoutes = new ArrayList<>(futures.size());
                for (Future<Route> future : futures) {
                    try {
                        antRoutes.add(future.get());
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        throw new IllegalStateException(
                                "ACO interrupted while waiting for ant to complete", e);
                    } catch (ExecutionException e) {
                        throw new RuntimeException(
                                "ACO ant task failed during tour construction",
                                e.getCause());
                    }
                }

                // ── Batch pheromone update ────────────────────────────────────
                // Standard Ant System: one global evaporation, then all-ant deposits.
                pheromones.evaporateAll(config.getRho());
                for (Route antRoute : antRoutes) {
                    if (antRoute.getFitness() > best.getFitness()) {
                        best = antRoute;
                    }
                    depositEdges(antRoute, pheromones, idToIdx);
                }

                // ── Elitist deposit ───────────────────────────────────────────
                // Best-so-far route receives an extra pheromone reinforcement each
                // iteration, accelerating convergence toward the current optimum
                // (MMAS-inspired elitist strategy).
                depositEdges(best, pheromones, idToIdx);
            }
        } finally {
            // Allows all submitted tasks to finish; rejects any new submissions.
            executor.shutdown();
        }

        return best;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Builds a symmetric n×n Euclidean distance matrix.
     * {@code dist[i][j]} is the distance from {@code nodes.get(i)} to
     * {@code nodes.get(j)}; the diagonal is 0.0.
     */
    private double[][] buildDistanceMatrix(List<SensorNode> nodes) {
        int        n    = nodes.size();
        double[][] dist = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                dist[i][j] = nodes.get(i).calculateDistance(nodes.get(j));
            }
        }
        return dist;
    }

    /**
     * Precomputes the heuristic matrix {@code h[i][j] = (1 / dist[i][j])^β}.
     *
     * <p>This is computed once per {@link #refine} call and shared read-only
     * across all ants and all iterations, eliminating repeated {@code Math.pow}
     * invocations from the hot node-selection loop.  With the default β = 11.0,
     * the savings are significant.  Diagonal entries (i = j) and entries for
     * zero-distance pairs remain 0.0 — those candidates are never selected.
     *
     * @param dist distance matrix (n × n, symmetric)
     * @param beta heuristic exponent β (typically ≥ 1)
     * @return heuristic matrix of the same dimensions
     */
    private double[][] buildHeuristicMatrix(double[][] dist, double beta) {
        int        n = dist.length;
        double[][] h = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j && dist[i][j] > 0.0) {
                    h[i][j] = Math.pow(1.0 / dist[i][j], beta);
                }
                // h[i][i] = 0.0 by default — diagonal never used in selection
            }
        }
        return h;
    }

    /** Returns a map from node ID to its zero-based index in {@code nodes}. */
    private Map<Integer, Integer> buildIdToIndexMap(List<SensorNode> nodes) {
        Map<Integer, Integer> map = new HashMap<>(nodes.size() * 2);
        for (int i = 0; i < nodes.size(); i++) {
            map.put(nodes.get(i).getId(), i);
        }
        return map;
    }

    /**
     * Evaporates all pheromone, then deposits {@code Q / tourLength} on each
     * directed edge of {@code route}.
     *
     * <p>Called only by the master thread for the ant-1 (GA-seed) deposit.
     *
     * @param route      route whose edges receive the deposit
     * @param pheromones live pheromone matrix (master-thread access only)
     * @param idToIdx    mapping from node ID to matrix index
     */
    private void depositRoute(Route route,
                              PheromoneMatrix pheromones,
                              Map<Integer, Integer> idToIdx) {
        pheromones.evaporateAll(config.getRho());
        depositEdges(route, pheromones, idToIdx);
    }

    /**
     * Deposits {@code Q / tourLength} pheromone on each directed edge of
     * {@code route}, without evaporating first.
     *
     * <p>Used for per-ant batch deposits and the elitist deposit within each
     * ACO iteration.  Separation from {@link #depositRoute} keeps evaporation
     * under explicit control of the master loop.
     *
     * @param route      route whose edges receive pheromone
     * @param pheromones live pheromone matrix (master-thread access only)
     * @param idToIdx    mapping from node ID to matrix index
     */
    private void depositEdges(Route route,
                              PheromoneMatrix pheromones,
                              Map<Integer, Integer> idToIdx) {
        double tourLength = route.getTotalDistance();
        if (tourLength <= 0.0) return;
        double           delta      = config.getQ() / tourLength;
        List<SensorNode> routeNodes = route.getNodes();
        for (int i = 0; i < routeNodes.size() - 1; i++) {
            int from = idToIdx.get(routeNodes.get(i).getId());
            int to   = idToIdx.get(routeNodes.get(i + 1).getId());
            pheromones.deposit(from, to, delta);
        }
    }

    /**
     * Constructs a complete ant tour from a read-only pheromone snapshot.
     *
     * <p>This method is <em>purely functional</em> with respect to shared
     * state: it reads only from its arguments ({@code pheromoneSnapshot},
     * {@code heuristic}, {@code nodes}), writes only to its own local arrays
     * ({@code visited[]}, {@code tour}), and returns an immutable
     * {@link Route}.  Safe to call concurrently from multiple threads provided
     * each invocation receives its own {@code antRng} (which is guaranteed
     * since each is forked from the master random on the master thread before
     * submission).
     *
     * @param startIndex        zero-based index of the starting node
     * @param nodes             ordered node list (shared read-only)
     * @param pheromoneSnapshot read-only copy from {@link PheromoneMatrix#snapshot()}
     * @param heuristic         precomputed {@code (1/dist)^β} matrix (shared read-only)
     * @param antRng            this ant's own random source (not shared)
     * @return the route traversed by this ant
     */
    private Route constructTourFromSnapshot(int startIndex,
                                             List<SensorNode> nodes,
                                             double[][] pheromoneSnapshot,
                                             double[][] heuristic,
                                             Random antRng) {
        int              n       = nodes.size();
        boolean[]        visited = new boolean[n];
        List<SensorNode> tour    = new ArrayList<>(n);

        int current = startIndex;
        visited[current] = true;
        tour.add(nodes.get(current));

        for (int step = 1; step < n; step++) {
            int next = selectNextNode(current, visited,
                                      pheromoneSnapshot, heuristic, n, antRng);
            if (next == -1) break;
            visited[next] = true;
            tour.add(nodes.get(next));
            current = next;
        }

        return new Route(tour);
    }

    /**
     * Selects the next unvisited node from {@code current} via roulette-wheel
     * selection weighted by the ACO transition probability.
     *
     * <h3>Transition probability</h3>
     * <pre>
     *   p(i→j) = τ_ij^α · η_ij^β / Σ_k ( τ_ik^α · η_ik^β )
     * </pre>
     * where {@code η_ij^β = heuristic[i][j]} (precomputed) and the sum runs
     * over all unvisited k ≠ i.
     *
     * <p>Falls back to uniform random selection if the denominator is zero.
     *
     * <p><b>Bug B fix</b>: pheromone is read as {@code snapshot[current][j]}
     * (from → to), never reversed.
     *
     * <p><b>Performance</b>: unvisited node tracking uses a pre-allocated
     * {@code int[]} array rather than an {@code ArrayList<Integer>}, avoiding
     * allocation and boxing overhead inside the hot loop.
     *
     * @param current   index of the current node
     * @param visited   per-ant boolean mask (not shared)
     * @param snapshot  read-only pheromone snapshot
     * @param heuristic precomputed {@code (1/dist)^β} matrix (shared read-only)
     * @param n         total node count
     * @param antRng    this ant's own random source
     * @return index of the chosen next node, or {@code -1} if all visited
     */
    private int selectNextNode(int current,
                               boolean[] visited,
                               double[][] snapshot,
                               double[][] heuristic,
                               int n,
                               Random antRng) {
        double[] numerators    = new double[n];
        double   denominator   = 0.0;
        // Pre-allocated int array instead of ArrayList<Integer> — no boxing overhead
        int[]    unvisited     = new int[n];
        int      unvisitedCount = 0;

        for (int j = 0; j < n; j++) {
            if (!visited[j] && j != current) {
                unvisited[unvisitedCount++] = j;
                double tau = snapshot[current][j];      // Bug B fix: [from][to]
                double eta = heuristic[current][j];     // precomputed: (1/dist)^β
                double num = (tau > 0.0 && eta > 0.0)
                        ? Math.pow(tau, config.getAlpha()) * eta
                        : 0.0;
                numerators[j]  = num;
                denominator   += num;
            }
        }

        if (unvisitedCount == 0) return -1;

        if (denominator == 0.0) {
            // Fallback: uniform random among unvisited nodes
            return unvisited[antRng.nextInt(unvisitedCount)];
        }

        // Roulette-wheel selection
        double r = antRng.nextDouble();
        for (int j = 0; j < n; j++) {
            if (!visited[j] && j != current) {
                double prob = numerators[j] / denominator;
                if (prob > r) return j;
                r -= prob;
            }
        }

        // Numerical safety fallback: return last unvisited node
        return unvisited[unvisitedCount - 1];
    }
}
