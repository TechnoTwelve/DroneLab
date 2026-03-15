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

import algorithm.PathPlanner;
import algorithm.Route;
import domain.SensorNode;

import java.util.List;
import java.util.Random;

/**
 * Stateless implementation of {@link PathPlanner} using a three-stage
 * GA → ACO → LocalSearch fusion pipeline.
 *
 * <h3>Stage 1 — Genetic Algorithm (global search)</h3>
 * The GA explores the route space through random initialisation, tournament
 * selection, order crossover, and swap mutation over a fixed number of
 * generations.  A {@link LocalSearch#twoOpt 2-opt} pass is applied to the
 * GA best route before it is handed to the ACO, eliminating any crossings
 * produced during evolution.
 *
 * <h3>Stage 2 — Ant Colony Optimisation (local refinement)</h3>
 * The ACO receives the 2-opt-cleaned GA route and uses it to seed the
 * pheromone matrix.  Subsequent ants explore guided by the pheromone trail
 * and the precomputed inverse-distance heuristic over
 * {@code acoIterationCount} iterations.  An elitist deposit reinforces the
 * best route found so far after each batch update.
 *
 * <h3>Stage 3 — Local search (final polish)</h3>
 * The ACO best route is further improved by two complementary passes:
 * <ol>
 *   <li>{@link LocalSearch#twoOpt 2-opt} — eliminates any crossing edges that
 *       probabilistic ant construction may have re-introduced.</li>
 *   <li>{@link LocalSearch#orOpt Or-opt} — relocates individual waypoints to
 *       cheaper positions; addresses structural defects that 2-opt cannot fix
 *       by segment reversal alone.</li>
 * </ol>
 *
 * <h3>Statelessness guarantee</h3>
 * Each call to {@link #planRoute} creates fresh {@link GeneticAlgorithm} and
 * {@link AntColonyOptimization} instances with a new {@link Random} source.
 * No state is carried between calls; the planner may therefore be called
 * concurrently from multiple threads without synchronisation.
 *
 * <h3>Ownership model</h3>
 * A {@link simulation.UAV} (or any other caller) holds a {@link PathPlanner}
 * reference pointing to this implementation.  The planner receives the
 * current sensor-node positions at each planning call, so it always works on
 * up-to-date coordinates without any coupling to the simulation layer.
 */
public final class GaAcoPlanner implements PathPlanner {

    private final GaAcoConfig config;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Constructs a planner that will use the given algorithm configuration for
     * every route-planning call.
     *
     * @param config algorithm parameters; must not be null
     * @throws IllegalArgumentException if {@code config} is null
     */
    public GaAcoPlanner(GaAcoConfig config) {
        if (config == null) throw new IllegalArgumentException("config must not be null");
        this.config = config;
    }

    // ── PathPlanner implementation ────────────────────────────────────────────

    /**
     * Plans the optimal route through the given sensor nodes using the
     * GA → ACO → LocalSearch three-stage fusion algorithm.
     *
     * <p>The planner creates fresh GA and ACO instances for this call, sharing
     * a single {@link Random} source between them.
     *
     * <p>The returned route is guaranteed to be at least as good as the GA's
     * best output after 2-opt cleaning, because the ACO stage seeds from that
     * route and can only improve on it, and the final local-search passes can
     * only improve the ACO result further.
     *
     * @param nodes sensor nodes to visit; must not be null or empty; each node
     *              must have a unique ID
     * @return best {@link Route} found by the fusion algorithm; never null
     * @throws IllegalArgumentException if {@code nodes} is null or empty
     */
    @Override
    public Route planRoute(List<SensorNode> nodes) {
        if (nodes == null || nodes.isEmpty()) {
            throw new IllegalArgumentException("nodes must not be null or empty");
        }

        Random random = new Random();

        // Stage 1: GA global search (includes 2-opt on GA best before return)
        GeneticAlgorithm ga      = new GeneticAlgorithm(config, random);
        Route            gaRoute = ga.run(nodes);

        // Stage 2: ACO local refinement over acoIterationCount iterations
        AntColonyOptimization aco      = new AntColonyOptimization(config, random);
        Route                 acoRoute = aco.refine(gaRoute, nodes);

        // Stage 3: local-search polish on the ACO result
        //   2-opt: eliminate any crossings re-introduced by probabilistic ant construction
        //   Or-opt: relocate individual waypoints that are cheaper elsewhere
        Route polished = LocalSearch.twoOpt(acoRoute);
        polished       = LocalSearch.orOpt(polished);
        return polished;
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /**
     * Returns the algorithm configuration used by this planner.
     *
     * @return algorithm config; never null
     */
    public GaAcoConfig getConfig() {
        return config;
    }

    // ── Object overrides ──────────────────────────────────────────────────────

    @Override
    public String toString() {
        return "GaAcoPlanner{" + config + "}";
    }
}
