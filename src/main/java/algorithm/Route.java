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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.StringJoiner;

/**
 * An immutable, ordered sequence of {@link SensorNode} objects representing
 * one planned UAV path through the sensor network.
 *
 * <h3>Distance model</h3>
 * Total distance is the sum of consecutive Euclidean segment lengths along the
 * open path — from the first node to the last. The return leg (closing the
 * circuit back to the start) is intentionally excluded: the UAV collects data
 * in a single pass and is not required to return to its origin.
 *
 * <h3>Fitness</h3>
 * Fitness is defined as the reciprocal of total distance, per the thesis
 * specification: {@code fitness = 1 / totalDistance}. A shorter route has a
 * higher fitness value. For a single-node route (distance = 0), fitness is
 * {@link Double#MAX_VALUE}.
 *
 * <h3>Immutability guarantee</h3>
 * Total distance is computed exactly once at construction time. Because the
 * route cannot change, no caching flag is needed — the fitness value is always
 * derived directly from the stored distance. The node list returned by
 * {@link #getNodes()} is unmodifiable; callers that need a mutable copy must
 * construct one explicitly.
 *
 * <p>This class replaces the legacy {@code Path.java}, correcting:
 * <ul>
 *   <li>The broken {@code isFitnessChanged} flag (which was always reset by the
 *       {@code getNodes()} getter, making the cache guard dead code).</li>
 *   <li>The raw-typed {@code ArrayList} — nodes are now typed as
 *       {@code List<SensorNode>}.</li>
 * </ul>
 */
public final class Route {

    private final List<SensorNode> nodes;
    private final double           totalDistance;

    // ── Constructors ──────────────────────────────────────────────────────

    /**
     * Constructs a {@code Route} from the given ordered node list.
     *
     * <p>A defensive copy of {@code nodes} is made internally; modifications to
     * the caller's list after construction have no effect on this route.
     *
     * @param nodes ordered sequence of nodes to visit; must not be null or empty
     * @throws IllegalArgumentException if {@code nodes} is null or empty
     */
    public Route(List<SensorNode> nodes) {
        if (nodes == null || nodes.isEmpty()) {
            throw new IllegalArgumentException("A route must contain at least one node.");
        }
        this.nodes         = Collections.unmodifiableList(new ArrayList<>(nodes));
        this.totalDistance = computeDistance(this.nodes);
    }

    // ── Distance computation ──────────────────────────────────────────────

    /**
     * Computes the total open-path distance: the sum of consecutive segment
     * lengths, without a return to the starting node.
     *
     * @param nodes ordered node list (must be non-empty)
     * @return total Euclidean path length (≥ 0)
     */
    private static double computeDistance(List<SensorNode> nodes) {
        double total = 0.0;
        for (int i = 0; i < nodes.size() - 1; i++) {
            total += nodes.get(i).calculateDistance(nodes.get(i + 1));
        }
        return total;
    }

    // ── Accessors ─────────────────────────────────────────────────────────

    /**
     * Returns the ordered node sequence for this route.
     *
     * <p>The returned list is unmodifiable. Callers requiring mutation must
     * wrap it: {@code new ArrayList<>(route.getNodes())}.
     *
     * @return unmodifiable ordered list of nodes
     */
    public List<SensorNode> getNodes() {
        return nodes;
    }

    /**
     * Returns the pre-computed total path distance.
     *
     * <p>Calculated once at construction; subsequent calls incur no computation.
     *
     * @return total Euclidean path length (≥ 0)
     */
    public double getTotalDistance() {
        return totalDistance;
    }

    /**
     * Returns the fitness of this route.
     *
     * <p>Defined as the reciprocal of total distance per the thesis specification:
     * <pre>  fitness = 1 / totalDistance</pre>
     *
     * A shorter route yields a higher fitness value. For a degenerate single-node
     * route (distance = 0), returns {@link Double#MAX_VALUE} to avoid division by
     * zero while ensuring such a route is never selected as optimal over any real
     * multi-node path.
     *
     * @return fitness value; higher is better
     */
    public double getFitness() {
        return totalDistance > 0.0 ? 1.0 / totalDistance : Double.MAX_VALUE;
    }

    /**
     * Returns the number of nodes in this route.
     *
     * @return node count (≥ 1)
     */
    public int size() {
        return nodes.size();
    }

    // ── Object overrides ──────────────────────────────────────────────────

    @Override
    public String toString() {
        StringJoiner sj = new StringJoiner(" → ");
        for (SensorNode n : nodes) {
            sj.add(String.valueOf(n.getId()));
        }
        return String.format("[%s] | dist=%.2f | fitness=%.6f", sj, totalDistance, getFitness());
    }
}
