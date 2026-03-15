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
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 * Genetic Algorithm stage of the GA→ACO fusion path planner.
 *
 * <h3>Algorithm outline</h3>
 * <ol>
 *   <li><b>Initialisation</b> — creates {@code populationSize} routes by
 *       independently shuffling the node list at random.</li>
 *   <li><b>Evaluation</b> — each route's fitness is defined as
 *       {@code 1 / totalDistance} (higher is better).</li>
 *   <li><b>Selection</b> — tournament selection: {@code tournamentSize}
 *       candidates are drawn at random and the fittest is returned as a
 *       parent.</li>
 *   <li><b>Crossover</b> — one-point crossover at the midpoint: the first half
 *       of a randomly-ordered pair of parents is copied verbatim; the second
 *       half is filled with the remaining nodes in the order they appear in the
 *       other parent.  A 50 % coin flip decides which parent contributes the
 *       first half (per the original code's {@code Math.random() &lt; 0.5}
 *       swap).</li>
 *   <li><b>Mutation</b> — swap mutation: each node is independently selected
 *       for mutation with probability {@code mutationRate}; selected nodes are
 *       swapped with a uniformly random other node.</li>
 *   <li><b>Elitism</b> — the {@code eliteCount} best routes from the current
 *       generation are copied unchanged into the next.</li>
 *   <li><b>Termination</b> — after a fixed {@code generationCount} of
 *       generations, the best route in the final population is returned.</li>
 * </ol>
 *
 * <h3>No static state</h3>
 * All parameters are sourced from the injected {@link GaAcoConfig}.
 * A {@link Random} instance is also injected, enabling deterministic tests
 * by passing a seeded random.
 */
public final class GeneticAlgorithm {

    private final GaAcoConfig config;
    private final Random          random;

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * Constructs a GA instance with an externally supplied random source.
     *
     * <p>Passing a seeded {@link Random} makes runs fully deterministic, which
     * is useful for unit tests and reproducibility studies.
     *
     * @param config algorithm parameters; must not be null
     * @param random random source; must not be null
     */
    public GeneticAlgorithm(GaAcoConfig config, Random random) {
        if (config == null) throw new IllegalArgumentException("config must not be null");
        if (random == null) throw new IllegalArgumentException("random must not be null");
        this.config = config;
        this.random = random;
    }

    /**
     * Constructs a GA instance using an internally created, non-seeded random.
     *
     * @param config algorithm parameters; must not be null
     */
    public GeneticAlgorithm(GaAcoConfig config) {
        this(config, new Random());
    }

    // ── Public entry point ────────────────────────────────────────────────────

    /**
     * Runs the full GA over {@code generationCount} generations and returns the
     * best route found, after a final 2-opt local-search pass.
     *
     * @param nodes sensor nodes to visit; must not be null or empty
     * @return best {@link Route} found across all generations, locally optimised
     * @throws IllegalArgumentException if {@code nodes} is null or empty
     */
    public Route run(List<SensorNode> nodes) {
        if (nodes == null || nodes.isEmpty()) {
            throw new IllegalArgumentException("nodes must not be null or empty");
        }
        Population population = initialPopulation(nodes);
        population.sortByFitness();

        for (int gen = 0; gen < config.getGenerationCount(); gen++) {
            population = evolve(population);
            population.sortByFitness();
        }

        // 2-opt post-processing: eliminate any remaining crossings in the best route
        return LocalSearch.twoOpt(population.getBest());
    }

    // ── Package-visible helpers (used by tests) ───────────────────────────────

    /**
     * Creates the initial population by shuffling the node list independently
     * for each chromosome.
     *
     * @param nodes node list to shuffle
     * @return initial {@link Population}
     */
    Population initialPopulation(List<SensorNode> nodes) {
        List<Route> routes = new ArrayList<>(config.getPopulationSize());
        for (int i = 0; i < config.getPopulationSize(); i++) {
            List<SensorNode> shuffled = new ArrayList<>(nodes);
            Collections.shuffle(shuffled, random);
            routes.add(new Route(shuffled));
        }
        return new Population(routes);
    }

    /**
     * Applies one full generation: crossover followed by mutation.
     *
     * @param population current generation (sorted best-first)
     * @return new generation
     */
    Population evolve(Population population) {
        return mutatePopulation(crossoverPopulation(population));
    }

    /**
     * Builds the next generation by crossover.
     *
     * <p>Elite routes (indices 0 … {@code eliteCount − 1}) are copied
     * unchanged.  The remaining slots are filled by tournament selection and
     * one-point crossover.
     *
     * @param population current generation (must be sorted best-first)
     * @return new generation containing elites and crossed-over offspring
     */
    Population crossoverPopulation(Population population) {
        List<Route> next = new ArrayList<>(population.size());
        for (int i = 0; i < config.getEliteCount(); i++) {
            next.add(population.get(i));
        }
        for (int i = config.getEliteCount(); i < population.size(); i++) {
            Route parent1 = tournamentSelect(population);
            Route parent2 = tournamentSelect(population);
            next.add(crossover(parent1, parent2));
        }
        return new Population(next);
    }

    /**
     * Applies swap mutation to all non-elite routes in the population.
     *
     * <p>Elite routes (indices 0 … {@code eliteCount − 1}) are left unchanged.
     *
     * @param population population to mutate (modified in-place for non-elites)
     * @return the same population object, with non-elite routes possibly mutated
     */
    Population mutatePopulation(Population population) {
        for (int i = config.getEliteCount(); i < population.size(); i++) {
            population.set(i, mutate(population.get(i)));
        }
        return population;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Order-crossover (OX-style) with a random cut point.
     *
     * <p>A 50 % coin flip decides which parent donates the prefix.  A cut
     * point is chosen uniformly at random in {@code [1, size-1]} to vary
     * the segment length across generations.  The suffix is filled with
     * nodes from the other parent in the order they appear, skipping any
     * already present in the child.
     *
     * <p><b>Performance</b>: duplicate detection uses a {@link HashSet}
     * (O(1) per lookup) rather than {@code List.contains} (O(n)), reducing
     * crossover from O(n²) to O(n).
     *
     * @param route1 first parent
     * @param route2 second parent
     * @return child route containing all nodes from both parents without
     *         duplicates
     */
    Route crossover(Route route1, Route route2) {
        Route tempRoute1 = route1;
        Route tempRoute2 = route2;
        if (random.nextDouble() < 0.5) {
            tempRoute1 = route2;
            tempRoute2 = route1;
        }

        int size = tempRoute1.size();
        // Random cut point in [1, size-1]; guard single-node edge case
        int cutPoint = (size < 2) ? size : 1 + random.nextInt(size - 1);

        List<SensorNode> child   = new ArrayList<>(size);
        Set<SensorNode>  inChild = new HashSet<>(size * 2); // O(1) membership check

        // Copy prefix [0, cutPoint) from tempRoute1
        for (int i = 0; i < cutPoint; i++) {
            SensorNode node = tempRoute1.getNodes().get(i);
            child.add(node);
            inChild.add(node);
        }

        // Append remaining nodes from tempRoute2 in order, skipping duplicates — O(n) total
        for (SensorNode node : tempRoute2.getNodes()) {
            if (!inChild.contains(node)) {
                child.add(node);
            }
        }

        return new Route(child);
    }

    /**
     * Swap mutation: each node position is independently selected for mutation
     * with probability {@code mutationRate} and swapped with a randomly chosen
     * other position.
     *
     * @param route original route (not modified)
     * @return new route after mutation (may be identical if no swap occurred)
     */
    Route mutate(Route route) {
        List<SensorNode> nodes = new ArrayList<>(route.getNodes());
        for (int i = 0; i < nodes.size(); i++) {
            if (random.nextDouble() < config.getMutationRate()) {
                int j = random.nextInt(nodes.size());
                SensorNode tmp = nodes.get(i);
                nodes.set(i, nodes.get(j));
                nodes.set(j, tmp);
            }
        }
        return new Route(nodes);
    }

    /**
     * Tournament selection: draws {@code tournamentSize} routes at random from
     * the population and returns the fittest.
     *
     * @param population pool to draw from
     * @return fittest route among the sampled contestants
     */
    Route tournamentSelect(Population population) {
        Route best = null;
        for (int i = 0; i < config.getTournamentSize(); i++) {
            Route candidate = population.get(random.nextInt(population.size()));
            if (best == null || candidate.getFitness() > best.getFitness()) {
                best = candidate;
            }
        }
        return best;
    }

}
