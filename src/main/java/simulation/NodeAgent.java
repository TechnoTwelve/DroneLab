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

import domain.SensorNode;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * A mutable sensor-node agent in the discrete-event simulation.
 *
 * <p>Replaces the inner {@code Nodes.Node} class from the legacy code.
 * Unlike the algorithm layer's immutable {@link SensorNode}, a
 * {@code NodeAgent} tracks the node's live state as it evolves through the
 * simulation: current grid position, movement direction, speed, and the set
 * of neighbour nodes it has directly encountered.
 *
 * <h3>Per-instance caches — Bug E fix</h3>
 * The legacy {@code Nodes.Node} declared two {@code static ArrayList} fields
 * ({@code drone_cache_from_waypoint_with_node_caching} and
 * {@code drone_direct_caching_with_node_caching}), which were shared across
 * ALL node instances.  This completely broke per-node isolation.  In this
 * class every cache is a non-static instance field; each agent manages its
 * own independent set.
 *
 * <h3>Snapshot for the algorithm layer</h3>
 * Call {@link #toSensorNode()} to obtain an immutable {@link SensorNode}
 * snapshot reflecting the agent's current position.  This snapshot is safe
 * to pass to the algorithm layer and is never affected by subsequent
 * position updates.
 */
public final class NodeAgent {

    // ── Constants ─────────────────────────────────────────────────────────────

    /** Number of supported movement directions (N/S/E/W/NE/NW/SE/SW). */
    public static final int DIRECTION_COUNT = 8;

    // ── Identity (immutable) ──────────────────────────────────────────────────

    private final int id;
    private final int speed;

    // ── Mutable state ─────────────────────────────────────────────────────────

    private int x;
    private int y;
    /** Current movement direction: 0 = N, 1 = S, 2 = E, 3 = W,
     *  4 = NE, 5 = NW, 6 = SE, 7 = SW. */
    private int direction;

    /**
     * Nodes that this agent has detected within range during movement.
     * Per-instance, ordered by first encounter (insertion order preserved).
     * Duplicates are suppressed automatically.
     */
    private final Set<Integer> neighborCache;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Constructs a sensor-node agent with the given identity and initial state.
     *
     * @param id        unique node identifier (assigned from {@code nodeIdStart})
     * @param x         initial row index on the grid
     * @param y         initial column index on the grid
     * @param direction initial movement direction (0–7)
     * @param speed     movement speed in grid cells per tick (determines
     *                  inter-event interval: next event scheduled at
     *                  {@code currentTime + speed})
     */
    public NodeAgent(int id, int x, int y, int direction, int speed) {
        this.id            = id;
        this.x             = x;
        this.y             = y;
        this.direction     = direction;
        this.speed         = speed;
        this.neighborCache = new LinkedHashSet<>();
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /** @return unique node ID */
    public int getId()        { return id; }

    /** @return current row index on the grid */
    public int getX()         { return x; }

    /** @return current column index on the grid */
    public int getY()         { return y; }

    /** @return current movement direction (0–7) */
    public int getDirection() { return direction; }

    /** @return movement speed (cells per tick, also used as inter-event gap) */
    public int getSpeed()     { return speed; }

    // ── Mutation ──────────────────────────────────────────────────────────────

    /**
     * Updates the agent's grid position.  Should only be called by
     * {@link NodeMover} after a successful grid-cell move.
     *
     * @param newX new row index
     * @param newY new column index
     */
    public void updatePosition(int newX, int newY) {
        this.x = newX;
        this.y = newY;
    }

    /**
     * Sets the agent's movement direction.
     *
     * @param direction new direction (0–7)
     */
    public void setDirection(int direction) {
        this.direction = direction;
    }

    // ── Neighbour caching ─────────────────────────────────────────────────────

    /**
     * Records that this agent encountered another node with the given ID.
     * Duplicate entries are silently ignored.
     *
     * @param nodeId ID of the encountered neighbour
     */
    public void addNeighbor(int nodeId) {
        neighborCache.add(nodeId);
    }

    /**
     * Returns {@code true} if this agent has previously encountered the node
     * with the given ID.
     *
     * @param nodeId ID to check
     * @return {@code true} if {@code nodeId} is in this agent's cache
     */
    public boolean hasNeighbor(int nodeId) {
        return neighborCache.contains(nodeId);
    }

    /**
     * Returns an unmodifiable view of this agent's neighbour cache.
     *
     * <p>The iteration order reflects the order in which neighbours were first
     * encountered (insertion order of the underlying {@link LinkedHashSet}).
     *
     * @return unmodifiable set of neighbour node IDs
     */
    public Set<Integer> getNeighborCache() {
        return Collections.unmodifiableSet(neighborCache);
    }

    // ── Algorithm-layer bridge ────────────────────────────────────────────────

    /**
     * Creates an immutable {@link SensorNode} snapshot from this agent's
     * current position, direction, and speed.
     *
     * <p>The returned snapshot is independent of this agent; subsequent
     * position or direction changes do not affect it.  The velocity
     * information (direction + speed) allows the UAV to compute a predicted
     * interception point rather than chasing the node's last-known static
     * location.
     *
     * <p>Pass the snapshot to the algorithm layer
     * ({@link algorithm.PathPlanner}) via the {@link KnowledgeBase}.
     *
     * @return immutable snapshot of this agent's current state including velocity
     */
    public SensorNode toSensorNode() {
        return new SensorNode(id, x, y, direction, speed);
    }

    // ── Object overrides ──────────────────────────────────────────────────────

    @Override
    public String toString() {
        return String.format("NodeAgent{id=%d, pos=(%d,%d), dir=%d, speed=%d, neighbors=%d}",
                id, x, y, direction, speed, neighborCache.size());
    }
}
