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
package algorithm;

import domain.SensorNode;

import java.util.List;

/**
 * Strategy interface for UAV route-planning algorithms.
 *
 * <p>Implementations receive a snapshot of sensor-node positions and return
 * the best {@link Route} they can find through those nodes.  The contract is
 * intentionally minimal — only the input type ({@code List<SensorNode>}) and
 * return type ({@link Route}) are fixed — so that alternative planners (greedy
 * nearest-neighbour, random restart, simulated annealing, etc.) can be
 * substituted without changing any simulation code.
 *
 * <h3>Pluggable design</h3>
 * {@link simulation.UAV} holds a {@code PathPlanner} reference and calls
 * {@link #planRoute} whenever it needs an optimised route.  The concrete
 * implementation is injected at construction time by
 * {@link simulation.SimulationRunner}, decoupling the UAV from any specific
 * algorithm.
 *
 * <pre>
 *   PathPlanner planner = new GaAcoPlanner(algConfig);
 *   UAV uav = new UAV(simConfig, planner);
 * </pre>
 *
 * <h3>Known implementations</h3>
 * <ul>
 *   <li>{@link algorithm.gaaco.GaAcoPlanner} — three-stage GA → ACO → LocalSearch
 *       pipeline (Tarek Ahmed / Dr. Chaudhry, MSc thesis 2022).</li>
 * </ul>
 *
 * <h3>Thread-safety contract</h3>
 * Implementations are not required to be thread-safe unless documented
 * otherwise.  {@link simulation.UAV} calls {@code planRoute} sequentially
 * from a single thread.
 */
public interface PathPlanner {

    /**
     * Plans the best route through the given sensor nodes and returns it.
     *
     * @param nodes snapshot of sensor-node positions to visit; must not be
     *              null or empty; each node must have a unique ID
     * @return best route found; never null
     * @throws IllegalArgumentException if {@code nodes} is null or empty
     */
    Route planRoute(List<SensorNode> nodes);
}
