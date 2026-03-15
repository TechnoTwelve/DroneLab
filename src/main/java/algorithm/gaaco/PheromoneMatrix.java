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

/**
 * Encapsulates the pheromone trail matrix used by the Ant Colony Optimisation
 * stage.
 *
 * <h3>Matrix layout</h3>
 * {@code matrix[i][j]} holds the pheromone level on the directed edge from
 * node {@code i} to node {@code j}, where {@code i} and {@code j} are
 * zero-based indices into the node list supplied at construction time.  The
 * diagonal is always 0.0 — a node cannot travel to itself.
 *
 * <h3>Pheromone update rules</h3>
 * <ul>
 *   <li><b>Evaporation</b> — {@link #evaporateAll(double)} multiplies every
 *       off-diagonal entry by {@code (1 − ρ)}, uniformly decaying the trail
 *       on all edges, including those not visited in the current round.  This
 *       corrects the original code's bug where unvisited edges were never
 *       decayed.</li>
 *   <li><b>Deposit</b> — {@link #deposit(int, int, double)} adds a positive
 *       delta to a single directed edge.  The ACO class calls this per-edge
 *       after a complete ant tour, using {@code Q / tourLength} as the delta.
 *       Using tour-total length (not cumulative in-step distance) matches the
 *       standard Ant System formula.</li>
 * </ul>
 *
 * <h3>Thread safety</h3>
 * This class is not thread-safe, and it does not need to be.  In the
 * concurrent ACO design the live {@code PheromoneMatrix} is accessed only by
 * the master thread: ant-1 deposits the GA route, and the master thread
 * applies the batch evaporate-then-deposit after collecting all ant results.
 * Concurrent ant workers (ants 2..N) receive a read-only {@code double[][]}
 * snapshot via {@link #snapshot()} and never touch this object directly.
 * (The legacy code used {@code AtomicDouble} to serialise concurrent ant
 * updates, which introduced a read-modify-write race anyway.)
 */
public final class PheromoneMatrix {

    private final double[][] matrix;
    private final int        n;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Constructs a fresh pheromone matrix for {@code nodeCount} nodes.
     *
     * <p>Every off-diagonal entry is initialised to {@code initialValue};
     * the diagonal is set to 0.0.
     *
     * @param nodeCount    number of nodes (matrix dimension); must be ≥ 1
     * @param initialValue uniform initial pheromone level τ₀; must be &gt; 0
     * @throws IllegalArgumentException if any constraint is violated
     */
    public PheromoneMatrix(int nodeCount, double initialValue) {
        if (nodeCount < 1)    throw new IllegalArgumentException("nodeCount must be >= 1");
        if (initialValue <= 0) throw new IllegalArgumentException("initialValue must be > 0");
        this.n      = nodeCount;
        this.matrix = new double[n][n];
        reset(initialValue);
    }

    // ── Read / write operations ───────────────────────────────────────────────

    /**
     * Returns the pheromone level on the edge from node {@code from} to node
     * {@code to}.
     *
     * @param from source node index [0, n)
     * @param to   destination node index [0, n)
     * @return pheromone level (≥ 0)
     */
    public double get(int from, int to) {
        return matrix[from][to];
    }

    /**
     * Adds {@code delta} pheromone to the edge from {@code from} to {@code to}.
     *
     * <p>The deposit amount should be {@code Q / tourLength} per the standard
     * Ant System update rule.
     *
     * @param from  source node index [0, n)
     * @param to    destination node index [0, n)
     * @param delta pheromone amount to deposit; must be ≥ 0
     */
    public void deposit(int from, int to, double delta) {
        matrix[from][to] += delta;
    }

    // ── Global pheromone operations ───────────────────────────────────────────

    /**
     * Applies global evaporation to every off-diagonal entry.
     *
     * <pre>  τ_ij ← (1 − ρ) · τ_ij   for all i ≠ j</pre>
     *
     * <p>Calling this before depositing pheromone for a completed ant tour
     * implements the standard Ant System update sequence and ensures that
     * unvisited edges decay over time rather than retaining their initial level
     * indefinitely.
     *
     * @param rho evaporation coefficient ρ ∈ [0, 1]
     */
    public void evaporateAll(double rho) {
        double retain = 1.0 - rho;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                matrix[i][j] *= retain;
            }
            matrix[i][i] = 0.0; // keep diagonal at exactly zero
        }
    }

    /**
     * Resets every off-diagonal entry to {@code initialValue} and every
     * diagonal entry to 0.0.
     *
     * <p>Call this to start a fresh ACO run without allocating a new matrix.
     *
     * @param initialValue new uniform pheromone level; must be &gt; 0
     */
    public void reset(double initialValue) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                matrix[i][j] = (i == j) ? 0.0 : initialValue;
            }
        }
    }

    // ── Snapshot ──────────────────────────────────────────────────────────────

    /**
     * Returns a defensive copy of the full pheromone matrix.
     *
     * <p>The returned array is completely independent of this object: writes to
     * the copy do not affect the live matrix, and subsequent calls to
     * {@link #deposit} or {@link #evaporateAll} on this object do not affect
     * the copy.
     *
     * <p>Intended for use by {@link AntColonyOptimization} to hand a read-only
     * pheromone view to each concurrent ant worker.  Because ants only read
     * from the returned array, no synchronisation is required during tour
     * construction.
     *
     * @return n×n copy of the current pheromone levels
     */
    public double[][] snapshot() {
        double[][] copy = new double[n][n];
        for (int i = 0; i < n; i++) {
            copy[i] = matrix[i].clone();
        }
        return copy;
    }

    // ── Query ─────────────────────────────────────────────────────────────────

    /**
     * Returns the number of nodes (the matrix dimension).
     *
     * @return node count
     */
    public int size() {
        return n;
    }
}
