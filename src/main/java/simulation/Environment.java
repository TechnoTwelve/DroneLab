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

import java.util.ArrayList;
import java.util.Collections;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The simulation grid and sensor-node registry.
 *
 * <p>Owns the raw {@code int[][] field} grid (where a cell holds the node ID
 * that occupies it, or 0 if empty) and a {@code Map<Integer, NodeAgent>}
 * agent registry indexed by node ID.
 *
 * <h3>Range queries — no O(width × height) scan</h3>
 * {@link #findAgentsInRange(int, int, int)} iterates over the agent registry
 * in O(n) time (n = deployed node count), not over the full grid.  This
 * replaces the O(width × height) field scans used for neighbour detection in
 * the legacy {@code Run.individual_distance()}, {@code Run.drone_distance()},
 * and {@code Run.waypoint_distance()} methods.
 *
 * <h3>Single source of truth for position</h3>
 * The grid cell and the {@link NodeAgent}'s own {@code (x, y)} fields are
 * always kept in sync: {@link NodeMover} updates both atomically.  Any
 * caller that needs a node's position should read from the agent directly
 * ({@code agent.getX()} / {@code agent.getY()}) rather than scanning the
 * field.
 */
public final class Environment {

    // ── Grid ──────────────────────────────────────────────────────────────────

    private final int fieldWidth;
    private final int fieldHeight;
    /**
     * Simulation grid.  {@code field[row][col]} holds the node ID occupying
     * that cell, or 0 if the cell is empty.  The UAV is represented by the
     * sentinel value {@link UAV#UAV_ID} (747).
     */
    private final int[][] field;

    // ── Agent registry ────────────────────────────────────────────────────────

    /**
     * Sensor-node agents keyed by ID.  {@link LinkedHashMap} preserves
     * deployment order for deterministic iteration (e.g. when scheduling
     * initial events).
     */
    private final Map<Integer, NodeAgent> agents;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Constructs an empty simulation environment.
     *
     * @param fieldWidth  number of rows in the grid (≥ 1)
     * @param fieldHeight number of columns in the grid (≥ 1)
     */
    public Environment(int fieldWidth, int fieldHeight) {
        this.fieldWidth  = fieldWidth;
        this.fieldHeight = fieldHeight;
        this.field       = new int[fieldWidth][fieldHeight];
        this.agents      = new LinkedHashMap<>();
    }

    // ── Deployment ────────────────────────────────────────────────────────────

    /**
     * Deploys a sensor-node agent into the environment.
     *
     * <p>Writes the agent's ID into {@code field[agent.getX()][agent.getY()]}
     * and registers it in the agent map.
     *
     * @param agent the node to deploy
     * @return {@code true} if deployment succeeded; {@code false} if the
     *         target cell was already occupied
     */
    public boolean deployAgent(NodeAgent agent) {
        int x = agent.getX();
        int y = agent.getY();
        if (field[x][y] != 0) return false;
        field[x][y] = agent.getId();
        agents.put(agent.getId(), agent);
        return true;
    }

    /**
     * Writes an arbitrary value into a cell.
     *
     * <p>Used to place the UAV sentinel ({@link UAV#UAV_ID} = 747) at its
     * starting position before the simulation begins.
     *
     * @param x     row index
     * @param y     column index
     * @param value value to write
     * @return {@code true} if the cell was empty and the write succeeded
     */
    public boolean setCell(int x, int y, int value) {
        if (field[x][y] != 0) return false;
        field[x][y] = value;
        return true;
    }

    // ── Grid accessors ────────────────────────────────────────────────────────

    /**
     * Returns the raw simulation grid.
     *
     * <p>The reference is intentionally direct (not defensive-copied) so that
     * {@link NodeMover} and {@link UAV} can update cells in-place.
     *
     * @return the grid array ({@code field[row][col]})
     */
    public int[][] getField() { return field; }

    /** @return number of rows in the grid */
    public int getFieldWidth()  { return fieldWidth; }

    /** @return number of columns in the grid */
    public int getFieldHeight() { return fieldHeight; }

    // ── Agent accessors ───────────────────────────────────────────────────────

    /**
     * Returns an unmodifiable view of all deployed sensor-node agents,
     * in deployment order.
     *
     * @return ordered, unmodifiable collection of agents
     */
    public Collection<NodeAgent> allAgents() {
        return Collections.unmodifiableCollection(agents.values());
    }

    /**
     * Returns an unmodifiable view of the agent registry.
     *
     * <p>Used by {@link KnowledgeBase#observeWithTransitiveCache} for O(1)
     * neighbour lookups by ID.
     *
     * @return unmodifiable map from node ID to {@link NodeAgent}
     */
    public Map<Integer, NodeAgent> agentById() {
        return Collections.unmodifiableMap(agents);
    }

    // ── Range query ───────────────────────────────────────────────────────────

    /**
     * Returns all sensor-node agents within Euclidean distance {@code range}
     * of {@code (cx, cy)}, excluding any agent located exactly at that point
     * (squared distance = 0, i.e. a node sitting at the same cell).
     *
     * <p>Distance is compared using squared integer arithmetic to avoid
     * floating-point overhead.  The inclusive upper bound (≤ range²) matches
     * the {@code root_mid[i][j] <= range_of_nodes} check from the legacy
     * {@code Run.individual_distance()}.
     *
     * <p>Runs in O(n) time over the agent registry — no O(width × height)
     * field scan.
     *
     * @param cx    centre row
     * @param cy    centre column
     * @param range detection radius (grid cells, inclusive)
     * @return mutable list of agents within range; may be empty
     */
    public List<NodeAgent> findAgentsInRange(int cx, int cy, int range) {
        List<NodeAgent> result = new ArrayList<>();
        long rangeSq = (long) range * range;
        for (NodeAgent agent : agents.values()) {
            long dx = agent.getX() - cx;
            long dy = agent.getY() - cy;
            long distSq = dx * dx + dy * dy;
            if (distSq > 0 && distSq <= rangeSq) {
                result.add(agent);
            }
        }
        return result;
    }
}
