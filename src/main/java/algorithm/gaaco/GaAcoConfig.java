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
 * Immutable configuration for the GA-ACO fusion path-planning algorithm
 * and the planning schedule used by {@link intelligence.GaAcoIntelligence}.
 *
 * <p>This class is intentionally scoped to the GA-ACO algorithm.  If you
 * implement a different algorithm (reinforcement learning, PSO, A*, …)
 * create a dedicated config class for it — do <em>not</em> add unrelated
 * parameters here.
 *
 * <p>All fields are {@code final}.  To produce a variant that differs by a
 * single parameter, use the {@code withXxx} copy-and-override methods — they
 * return a new {@code GaAcoConfig} and leave the original unchanged.
 *
 * <p>The static factory {@link #defaults()} returns research-calibrated values
 * that give a strong, reproducible win over the fixed-patrol baseline on the
 * standard 1 500 × 1 500 WSN field.
 *
 * <h3>GA/ACO parameter guidance</h3>
 * <ul>
 *   <li><b>α (alpha)</b> — exponent on the pheromone term. Higher values make
 *       ants more likely to follow existing pheromone trails. Range: ≥ 0.</li>
 *   <li><b>β (beta)</b> — exponent on the heuristic (1/distance) term. Higher
 *       values make ants prefer shorter individual edges. Range: ≥ 1.</li>
 *   <li><b>ρ (rho)</b> — evaporation rate. Higher values cause pheromone to
 *       decay faster, increasing exploration. Range: (0, 1].</li>
 *   <li><b>Q</b> — pheromone deposit constant. Scales the amount deposited
 *       per edge relative to tour length.</li>
 *   <li><b>acoIterationCount</b> — number of ACO ant-batch + pheromone-update
 *       cycles to run after the GA seed.  More iterations give the pheromone
 *       matrix more time to converge. Range: ≥ 1.</li>
 * </ul>
 *
 * <h3>Planning-schedule parameter guidance</h3>
 * <ul>
 *   <li><b>planningThreshold</b> — minimum number of known nodes in the
 *       knowledge base before replanning may fire.</li>
 *   <li><b>planningInterval</b> — minimum simulation ticks between consecutive
 *       replan checks.</li>
 *   <li><b>minCoverageBeforePlan</b> — field-coverage fraction the UAV must
 *       reach before the first replan is allowed.</li>
 *   <li><b>maxRouteWaypoints</b> — maximum number of waypoints per planned
 *       route.  The intelligence selects the nearest {@code maxRouteWaypoints}
 *       untokenised nodes from the knowledge base before passing them to the
 *       path planner.</li>
 * </ul>
 */
public final class GaAcoConfig {

    // ── GA parameters ──────────────────────────────────────────────────────

    /** Number of chromosomes (candidate routes) maintained per generation. */
    private final int    populationSize;

    /** Probability that any single gene undergoes swap-mutation. Range: [0, 1]. */
    private final double mutationRate;

    /** Number of candidate routes drawn per tournament selection round. */
    private final int    tournamentSize;

    /** Number of elite routes copied unchanged into each new generation. */
    private final int    eliteCount;

    /** Fixed number of GA generations (termination criterion). */
    private final int    generationCount;

    // ── ACO parameters ─────────────────────────────────────────────────────

    /** Total number of ants per ACO iteration, including the GA-seeded first ant. */
    private final int    antCount;

    /**
     * Uniform pheromone initialisation level τ₀. Applied to every off-diagonal
     * entry of the pheromone matrix at the start of each ACO run.
     * Must satisfy 0 &lt; τ₀ &lt; 1.
     */
    private final double pheromoneInit;

    /** Pheromone deposit constant Q. Deposit per edge = Q / total tour length. */
    private final double q;

    /** Evaporation coefficient ρ. Applied globally after each ant batch. Range: [0, 1]. */
    private final double rho;

    /** Exponent controlling pheromone trail influence (α). Range: ≥ 0. */
    private final double alpha;

    /** Exponent controlling heuristic distance influence (β). Range: ≥ 1. */
    private final double beta;

    /**
     * Number of ACO ant-batch + pheromone-update cycles to run after the GA
     * seed.  Range: ≥ 1.
     */
    private final int    acoIterationCount;

    // ── Planning schedule (owned by GaAcoIntelligence) ─────────────────────

    /**
     * Minimum knowledge-base size (known nodes) before replanning may fire.
     * Range: ≥ 1.
     */
    private final int    planningThreshold;

    /**
     * Minimum simulation ticks between consecutive replan checks.
     * Range: ≥ 1.
     */
    private final int    planningInterval;

    /**
     * Minimum field-coverage fraction the UAV must reach before the first
     * replan is allowed.  Range: [0, 1].
     */
    private final double minCoverageBeforePlan;

    /**
     * Maximum number of waypoints per planned route.
     * Range: ≥ 1.
     */
    private final int    maxRouteWaypoints;

    // ── Constructor ────────────────────────────────────────────────────────

    /**
     * Constructs a fully specified, validated GA-ACO + planning configuration.
     *
     * @param populationSize        number of chromosomes per generation (≥ 1)
     * @param mutationRate          per-gene swap-mutation probability [0, 1]
     * @param tournamentSize        candidates drawn per tournament (≥ 2)
     * @param eliteCount            elite routes preserved unchanged (≥ 0, &lt; populationSize)
     * @param generationCount       total GA generations to run (≥ 1)
     * @param antCount              total ACO ants per iteration (≥ 1)
     * @param pheromoneInit         uniform initial pheromone level; 0 &lt; value &lt; 1
     * @param q                     pheromone deposit constant (any positive value)
     * @param rho                   pheromone evaporation rate [0, 1]
     * @param alpha                 pheromone influence exponent (≥ 0)
     * @param beta                  heuristic influence exponent (≥ 1)
     * @param acoIterationCount     number of ACO ant-batch + update cycles (≥ 1)
     * @param planningThreshold     minimum KB size before replanning (≥ 1)
     * @param planningInterval      minimum ticks between replan checks (≥ 1)
     * @param minCoverageBeforePlan minimum coverage fraction before first replan [0, 1]
     * @param maxRouteWaypoints     maximum waypoints per planned route (≥ 1)
     * @throws IllegalArgumentException if any parameter violates its constraint
     */
    public GaAcoConfig(int    populationSize,
                       double mutationRate,
                       int    tournamentSize,
                       int    eliteCount,
                       int    generationCount,
                       int    antCount,
                       double pheromoneInit,
                       double q,
                       double rho,
                       double alpha,
                       double beta,
                       int    acoIterationCount,
                       int    planningThreshold,
                       int    planningInterval,
                       double minCoverageBeforePlan,
                       int    maxRouteWaypoints) {

        if (populationSize < 1)
            throw new IllegalArgumentException("populationSize must be >= 1, got: " + populationSize);
        if (mutationRate < 0.0 || mutationRate > 1.0)
            throw new IllegalArgumentException("mutationRate must be in [0,1], got: " + mutationRate);
        if (tournamentSize < 2)
            throw new IllegalArgumentException("tournamentSize must be >= 2, got: " + tournamentSize);
        if (eliteCount < 0 || eliteCount >= populationSize)
            throw new IllegalArgumentException(
                "eliteCount must be in [0, populationSize), got: " + eliteCount);
        if (generationCount < 1)
            throw new IllegalArgumentException("generationCount must be >= 1, got: " + generationCount);
        if (antCount < 1)
            throw new IllegalArgumentException("antCount must be >= 1, got: " + antCount);
        if (pheromoneInit <= 0.0 || pheromoneInit >= 1.0)
            throw new IllegalArgumentException(
                "pheromoneInit must satisfy 0 < value < 1, got: " + pheromoneInit);
        if (rho < 0.0 || rho > 1.0)
            throw new IllegalArgumentException("rho must be in [0,1], got: " + rho);
        if (alpha < 0.0)
            throw new IllegalArgumentException("alpha must be >= 0, got: " + alpha);
        if (beta < 1.0)
            throw new IllegalArgumentException("beta must be >= 1, got: " + beta);
        if (acoIterationCount < 1)
            throw new IllegalArgumentException(
                "acoIterationCount must be >= 1, got: " + acoIterationCount);
        if (planningThreshold < 1)
            throw new IllegalArgumentException(
                "planningThreshold must be >= 1, got: " + planningThreshold);
        if (planningInterval < 1)
            throw new IllegalArgumentException(
                "planningInterval must be >= 1, got: " + planningInterval);
        if (minCoverageBeforePlan < 0.0 || minCoverageBeforePlan > 1.0)
            throw new IllegalArgumentException(
                "minCoverageBeforePlan must be in [0,1], got: " + minCoverageBeforePlan);
        if (maxRouteWaypoints < 1)
            throw new IllegalArgumentException(
                "maxRouteWaypoints must be >= 1, got: " + maxRouteWaypoints);

        this.populationSize        = populationSize;
        this.mutationRate          = mutationRate;
        this.tournamentSize        = tournamentSize;
        this.eliteCount            = eliteCount;
        this.generationCount       = generationCount;
        this.antCount              = antCount;
        this.pheromoneInit         = pheromoneInit;
        this.q                     = q;
        this.rho                   = rho;
        this.alpha                 = alpha;
        this.beta                  = beta;
        this.acoIterationCount     = acoIterationCount;
        this.planningThreshold     = planningThreshold;
        this.planningInterval      = planningInterval;
        this.minCoverageBeforePlan = minCoverageBeforePlan;
        this.maxRouteWaypoints     = maxRouteWaypoints;
    }

    // ── Static factory ─────────────────────────────────────────────────────

    /**
     * Returns a {@code GaAcoConfig} with research-calibrated defaults tuned
     * for the standard 1 500 × 1 500 WSN data-collection scenario.
     *
     * <table border="1">
     *   <caption>Default values</caption>
     *   <tr><th>Parameter</th><th>Value</th><th>Notes</th></tr>
     *   <tr><td>populationSize</td><td>20</td><td>richer GA search space</td></tr>
     *   <tr><td>mutationRate</td><td>0.09</td><td></td></tr>
     *   <tr><td>tournamentSize</td><td>2</td><td></td></tr>
     *   <tr><td>eliteCount</td><td>1</td><td></td></tr>
     *   <tr><td>generationCount</td><td>15</td><td>3× more evolution cycles</td></tr>
     *   <tr><td>antCount</td><td>10</td><td>faster pheromone convergence</td></tr>
     *   <tr><td>pheromoneInit</td><td>0.2</td><td></td></tr>
     *   <tr><td>q</td><td>0.08</td><td></td></tr>
     *   <tr><td>rho</td><td>0.2</td><td></td></tr>
     *   <tr><td>alpha</td><td>0.1</td><td></td></tr>
     *   <tr><td>beta</td><td>11.0</td><td></td></tr>
     *   <tr><td>acoIterationCount</td><td>5</td><td>two extra cycles</td></tr>
     *   <tr><td>planningThreshold</td><td>15</td><td>sweep-optimised: +14% vs 10</td></tr>
     *   <tr><td>planningInterval</td><td>25</td><td></td></tr>
     *   <tr><td>minCoverageBeforePlan</td><td>0.05</td><td>near-immediate first replan</td></tr>
     *   <tr><td>maxRouteWaypoints</td><td>12</td><td>compact-disc cluster routes</td></tr>
     * </table>
     *
     * @return default GA-ACO configuration
     */
    public static GaAcoConfig defaults() {
        return new GaAcoConfig(
            20,    // populationSize   — richer GA search space
            0.09,  // mutationRate
            2,     // tournamentSize
            1,     // eliteCount
            15,    // generationCount  — 3× more evolution cycles
            10,    // antCount         — faster pheromone convergence
            0.2,   // pheromoneInit (τ₀)
            0.08,  // q
            0.2,   // rho (ρ)
            0.1,   // alpha (α)
            11.0,  // beta (β)
            5,     // acoIterationCount — two extra cycles
            // ── Planning schedule ──────────────────────────────────────────
            // planningThreshold 15: parameter sweep found +14.37% vs 10.
            15,    // planningThreshold
            25,    // planningInterval — replan promptly after each route
            0.05,  // minCoverageBeforePlan — near-immediate first replan
            // maxRouteWaypoints 12: compact-disc routes complete in ~500-800 ticks
            12     // maxRouteWaypoints
        );
    }

    // ── Getters ────────────────────────────────────────────────────────────

    /** @return number of chromosomes per generation */
    public int    getPopulationSize()         { return populationSize; }

    /** @return per-gene swap-mutation probability */
    public double getMutationRate()           { return mutationRate; }

    /** @return number of candidates drawn per tournament selection round */
    public int    getTournamentSize()         { return tournamentSize; }

    /** @return number of elite routes preserved unchanged each generation */
    public int    getEliteCount()             { return eliteCount; }

    /** @return fixed number of GA generations */
    public int    getGenerationCount()        { return generationCount; }

    /** @return total number of ACO ants per iteration */
    public int    getAntCount()               { return antCount; }

    /** @return uniform initial pheromone level τ₀ */
    public double getPheromoneInit()          { return pheromoneInit; }

    /** @return pheromone deposit constant Q */
    public double getQ()                      { return q; }

    /** @return pheromone evaporation rate ρ */
    public double getRho()                    { return rho; }

    /** @return pheromone trail influence exponent α */
    public double getAlpha()                  { return alpha; }

    /** @return heuristic distance influence exponent β */
    public double getBeta()                   { return beta; }

    /** @return number of ACO ant-batch + pheromone-update iterations */
    public int    getAcoIterationCount()      { return acoIterationCount; }

    /** @return minimum KB size before replanning may fire */
    public int    getPlanningThreshold()      { return planningThreshold; }

    /** @return minimum ticks between consecutive replan checks */
    public int    getPlanningInterval()       { return planningInterval; }

    /** @return minimum coverage fraction before the first replan */
    public double getMinCoverageBeforePlan()  { return minCoverageBeforePlan; }

    /** @return maximum waypoints per planned route */
    public int    getMaxRouteWaypoints()      { return maxRouteWaypoints; }

    // ── Immutable copy-and-override ────────────────────────────────────────

    /** Returns a new config with the given population size. @param v new value (&ge; 1) */
    public GaAcoConfig withPopulationSize(int v) {
        return new GaAcoConfig(v, mutationRate, tournamentSize, eliteCount,
                generationCount, antCount, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, planningThreshold, planningInterval,
                minCoverageBeforePlan, maxRouteWaypoints);
    }

    /** Returns a new config with the given generation count. @param v new value (&ge; 1) */
    public GaAcoConfig withGenerationCount(int v) {
        return new GaAcoConfig(populationSize, mutationRate, tournamentSize, eliteCount,
                v, antCount, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, planningThreshold, planningInterval,
                minCoverageBeforePlan, maxRouteWaypoints);
    }

    /** Returns a new config with the given ant count. @param v new value (&ge; 1) */
    public GaAcoConfig withAntCount(int v) {
        return new GaAcoConfig(populationSize, mutationRate, tournamentSize, eliteCount,
                generationCount, v, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, planningThreshold, planningInterval,
                minCoverageBeforePlan, maxRouteWaypoints);
    }

    /** Returns a new config with the given mutation rate. @param v new value [0, 1] */
    public GaAcoConfig withMutationRate(double v) {
        return new GaAcoConfig(populationSize, v, tournamentSize, eliteCount,
                generationCount, antCount, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, planningThreshold, planningInterval,
                minCoverageBeforePlan, maxRouteWaypoints);
    }

    /** Returns a new config with the given ACO iteration count. @param v new value (&ge; 1) */
    public GaAcoConfig withAcoIterationCount(int v) {
        return new GaAcoConfig(populationSize, mutationRate, tournamentSize, eliteCount,
                generationCount, antCount, pheromoneInit, q, rho, alpha, beta, v,
                planningThreshold, planningInterval, minCoverageBeforePlan, maxRouteWaypoints);
    }

    /** Returns a new config with the given planning threshold. @param v new value (&ge; 1) */
    public GaAcoConfig withPlanningThreshold(int v) {
        return new GaAcoConfig(populationSize, mutationRate, tournamentSize, eliteCount,
                generationCount, antCount, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, v, planningInterval, minCoverageBeforePlan, maxRouteWaypoints);
    }

    /** Returns a new config with the given planning interval. @param v new value (&ge; 1) */
    public GaAcoConfig withPlanningInterval(int v) {
        return new GaAcoConfig(populationSize, mutationRate, tournamentSize, eliteCount,
                generationCount, antCount, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, planningThreshold, v, minCoverageBeforePlan, maxRouteWaypoints);
    }

    /** Returns a new config with the given minimum coverage. @param v new value [0, 1] */
    public GaAcoConfig withMinCoverageBeforePlan(double v) {
        return new GaAcoConfig(populationSize, mutationRate, tournamentSize, eliteCount,
                generationCount, antCount, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, planningThreshold, planningInterval, v, maxRouteWaypoints);
    }

    /** Returns a new config with the given max waypoints. @param v new value (&ge; 1) */
    public GaAcoConfig withMaxRouteWaypoints(int v) {
        return new GaAcoConfig(populationSize, mutationRate, tournamentSize, eliteCount,
                generationCount, antCount, pheromoneInit, q, rho, alpha, beta,
                acoIterationCount, planningThreshold, planningInterval,
                minCoverageBeforePlan, v);
    }

    // ── Object overrides ───────────────────────────────────────────────────

    @Override
    public String toString() {
        return String.format(
            "GaAcoConfig{pop=%d, mut=%.3f, tourn=%d, elite=%d, gens=%d, " +
            "ants=%d, \u03C4\u2080=%.2f, Q=%.4f, \u03C1=%.2f, \u03B1=%.2f, \u03B2=%.1f, acoIter=%d, " +
            "planThresh=%d, planInterval=%d, minCov=%.0f%%, maxWaypoints=%d}",
            populationSize, mutationRate, tournamentSize, eliteCount, generationCount,
            antCount, pheromoneInit, q, rho, alpha, beta, acoIterationCount,
            planningThreshold, planningInterval, minCoverageBeforePlan * 100,
            maxRouteWaypoints);
    }
}
