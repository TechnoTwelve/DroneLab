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
import movement.LevyWalkStrategy;
import movement.MarkovChain;

import java.util.List;
import java.util.function.Supplier;

/**
 * Composition root for a simulation experiment.
 *
 * <p>Implement this interface to define a complete, self-contained experiment:
 * field configuration, target movement model, UAV strategies to compare, and
 * the path-planning algorithm to use.
 *
 * <h3>Quickstart — implement three methods</h3>
 * <pre>
 *   public class MyScenario implements SimulationScenario {
 *
 *       {@literal @}Override
 *       public String getName() { return "My Experiment"; }
 *
 *       {@literal @}Override
 *       public SimulationConfig getConfig() {
 *           return SimulationConfig.builder()
 *               .fieldSize(3000, 3000)
 *               .nodeCount(100)
 *               .duration(8000L)
 *               .build();
 *       }
 *
 *       {@literal @}Override
 *       public List&lt;UAVIntelligence&gt; getIntelligences() {
 *           return Arrays.asList(
 *               new PredefinedPatrolIntelligence(),
 *               new MyCustomIntelligence(getConfig()));
 *       }
 *   }
 * </pre>
 *
 * <h3>Running a scenario</h3>
 * <pre>
 *   SimulationRunner runner = SimulationRunner.fromScenario(new MyScenario())
 *       .withSeed(42L);
 *
 *   runner.run();                              // headless, sequential
 *   runner.runInParallel(listeners, callback); // side-by-side GUI
 * </pre>
 *
 * <h3>Extension points</h3>
 * Override {@link #getPlannerFactory()} to plug in a path-planning algorithm,
 * and {@link #getMovementStrategy()} to change how targets move.
 * Both have sensible defaults so you only override what your scenario needs.
 */
public interface SimulationScenario {

    /** Human-readable name shown in run reports and the GUI header. */
    String getName();

    /** Optional one-line description for documentation / README purposes. */
    default String getDescription() {
        return "";
    }

    /**
     * Field dimensions, timing, drone range, and node parameters for this
     * scenario.  Use {@link SimulationConfig#builder()} to construct.
     */
    SimulationConfig getConfig();

    /**
     * Ordered list of UAV intelligence strategies to compare in this scenario.
     * The first entry is treated as the baseline in comparison reports.
     *
     * <p>Each call to this method should return fresh instances because
     * intelligences are stateful (they own coverage grids, knowledge bases,
     * etc.).
     */
    List<UAVIntelligence> getIntelligences();

    /**
     * Factory called once per simulation pass to produce a fresh
     * {@link PathPlanner}.  Using a factory (rather than a shared instance)
     * is important when passes run in parallel.
     *
     * <p>Defaults to a pass-through planner that returns the provided target
     * list as-is (no route optimisation).  Override to plug in a custom
     * planning algorithm without touching any runner or UAV code:
     *
     * <pre>
     *   {@literal @}Override
     *   public Supplier&lt;PathPlanner&gt; getPlannerFactory() {
     *       GaAcoConfig cfg = GaAcoConfig.defaults();
     *       return () -&gt; new GaAcoPlanner(cfg);
     *   }
     * </pre>
     */
    default Supplier<PathPlanner> getPlannerFactory() {
        return () -> targets -> new Route(targets);
    }

    /**
     * Movement model applied to all targets (sensor nodes, drones, etc.)
     * throughout the simulation.  A new instance is created for each
     * simulation pass via this method.
     *
     * <p>Defaults to a Lévy-walk / Markov-chain model matching the original
     * thesis scenario.  Override to model faster targets, directed movement,
     * ballistic trajectories, or any other mobility pattern.
     */
    default MovementStrategy getMovementStrategy() {
        return new LevyWalkStrategy(MarkovChain.exploratoryWalk());
    }

    /**
     * Strategy controlling how targets are spatially deployed at the start of
     * each simulation pass.
     *
     * <p>Defaults to {@link NodeDeploymentStrategy#stratified()} — stratified
     * random sampling — which divides the field into a uniform grid and places
     * exactly one node per cell.  This guarantees spatial uniformity regardless
     * of the random seed, removing the need to cherry-pick seeds and making
     * results comparable across runs.
     *
     * <p>Override with {@link NodeDeploymentStrategy#random()} to reproduce
     * the original fully-random placement, or supply a custom strategy for
     * non-uniform scenarios (e.g. clustered deployment near disaster sites).
     */
    default NodeDeploymentStrategy getDeploymentStrategy() {
        return NodeDeploymentStrategy.stratified();
    }
}
