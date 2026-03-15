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
package domain;

/**
 * Represents a wireless sensor node in the UAV data-collection network.
 *
 * <p>Immutable value object. Position updates return a new instance via
 * {@link #withPosition(double, double)}, preserving the original. Equality
 * and hashing are based solely on {@code id}, so two nodes with the same
 * identifier compare equal regardless of their current coordinates (a node
 * retains its identity as it moves through the environment).
 *
 * <h3>Velocity snapshot</h3>
 * When a sensor node is observed directly by the UAV, the snapshot also
 * captures the node's movement direction (0–7, same encoding as
 * {@code NodeMover}) and speed (cells per tick) at the time of observation.
 * This enables the UAV to compute a predicted intercept position rather than
 * chasing the node's last-known static location.  Nodes discovered only via
 * transitive neighbour-cache transfer use the no-velocity constructor and
 * return {@code false} from {@link #hasVelocity()}.
 *
 * <p>This class belongs to the domain layer and is shared by both the
 * algorithm layer (path planning) and the simulation layer (environment).
 */
public final class SensorNode {

    private final int    id;
    private final double x;
    private final double y;

    /**
     * Movement direction at the time of observation (0–7, NodeMover encoding).
     * {@code -1} signals "velocity unknown" for transitively-discovered nodes.
     */
    private final int direction;

    /**
     * Movement speed at the time of observation (cells per tick).
     * {@code 0} signals "velocity unknown".
     */
    private final int speed;

    // ── Constructors ──────────────────────────────────────────────────────

    /**
     * Constructs a sensor node with position only (no velocity information).
     *
     * <p>Use this constructor for algorithm-layer objects and tests where
     * movement direction is not relevant.  {@link #hasVelocity()} will return
     * {@code false} on the resulting instance.
     *
     * @param id unique node identifier (typically assigned sequentially from 5000)
     * @param x  x-coordinate in the environment grid
     * @param y  y-coordinate in the environment grid
     */
    public SensorNode(int id, double x, double y) {
        this(id, x, y, -1, 0);
    }

    /**
     * Constructs a sensor node with full velocity information.
     *
     * <p>Used by {@code NodeAgent.toSensorNode()} to capture a complete
     * state snapshot for interception-aware path following.
     *
     * @param id        unique node identifier
     * @param x         x-coordinate in the environment grid
     * @param y         y-coordinate in the environment grid
     * @param direction movement direction at observation time (0–7); use
     *                  {@code -1} if unknown
     * @param speed     movement speed at observation time (cells per tick);
     *                  use {@code 0} if unknown
     */
    public SensorNode(int id, double x, double y, int direction, int speed) {
        this.id        = id;
        this.x         = x;
        this.y         = y;
        this.direction = direction;
        this.speed     = speed;
    }

    // ── Accessors ──────────────────────────────────────────────────────────

    /**
     * Returns the unique identifier of this node.
     *
     * @return node ID
     */
    public int getId() { return id; }

    /**
     * Returns the current x-coordinate of this node.
     *
     * @return x-coordinate
     */
    public double getX() { return x; }

    /**
     * Returns the current y-coordinate of this node.
     *
     * @return y-coordinate
     */
    public double getY() { return y; }

    /**
     * Returns the movement direction at the time of observation (0–7,
     * same encoding as {@code NodeMover}).  Returns {@code -1} if no
     * velocity information was captured.
     *
     * @return direction (0–7) or {@code -1} if unknown
     */
    public int getDirection() { return direction; }

    /**
     * Returns the movement speed (cells per tick) at the time of observation.
     * Returns {@code 0} if no velocity information was captured.
     *
     * @return speed (cells per tick) or {@code 0} if unknown
     */
    public int getSpeed() { return speed; }

    /**
     * Returns {@code true} if this snapshot carries valid velocity information
     * (direction and speed were recorded at observation time).
     *
     * <p>When {@code false}, the UAV falls back to targeting the last-known
     * static position rather than computing a predicted intercept.
     *
     * @return {@code true} if direction ≥ 0 and speed > 0
     */
    public boolean hasVelocity() { return direction >= 0 && speed > 0; }

    // ── Geometry ──────────────────────────────────────────────────────────

    /**
     * Computes the Euclidean distance between this node and another.
     *
     * <pre>  distance = √((x₂−x₁)² + (y₂−y₁)²)</pre>
     *
     * @param other the target node; must not be null
     * @return Euclidean distance (≥ 0); returns 0 when both nodes occupy
     *         the same position
     * @throws IllegalArgumentException if {@code other} is null
     */
    public double calculateDistance(SensorNode other) {
        if (other == null) throw new IllegalArgumentException("other must not be null");
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns a new {@code SensorNode} with the same ID and velocity but
     * updated coordinates.  The original instance is unchanged (immutable
     * update pattern).
     *
     * @param newX updated x-coordinate
     * @param newY updated y-coordinate
     * @return new instance with the same ID, same velocity, and updated position
     */
    public SensorNode withPosition(double newX, double newY) {
        return new SensorNode(this.id, newX, newY, this.direction, this.speed);
    }

    // ── Object overrides ──────────────────────────────────────────────────

    /**
     * Two sensor nodes are equal if and only if their IDs are equal,
     * regardless of current position.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof SensorNode)) return false;
        return this.id == ((SensorNode) o).id;
    }

    @Override
    public int hashCode() {
        return Integer.hashCode(id);
    }

    @Override
    public String toString() {
        if (hasVelocity()) {
            return String.format("Node(%d, %.1f, %.1f, dir=%d, spd=%d)",
                    id, x, y, direction, speed);
        }
        return String.format("Node(%d, %.1f, %.1f)", id, x, y);
    }
}
