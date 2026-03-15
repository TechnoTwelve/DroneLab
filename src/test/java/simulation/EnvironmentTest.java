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

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link Environment}.
 *
 * <p>Key invariants verified:
 * <ul>
 *   <li>Deployment to an empty cell succeeds; deployment to an occupied cell fails.</li>
 *   <li>{@link Environment#findAgentsInRange} uses inclusive squared-distance
 *       ({@code distSq <= rangeSq}) and excludes the query centre itself
 *       ({@code distSq > 0}).</li>
 *   <li>{@link Environment#allAgents()} preserves deployment order
 *       (backed by {@link java.util.LinkedHashMap}).</li>
 * </ul>
 *
 * <h3>IntelliJ IDEA setup</h3>
 * See {@code test/algorithm/RouteTest.java} for instructions on adding JUnit 5
 * and marking the test source root.
 */
class EnvironmentTest {

    // ── Deployment ────────────────────────────────────────────────────────────

    @Test
    void deployAgent_toEmptyCell_returnsTrue() {
        Environment env = new Environment(20, 20);
        NodeAgent agent = new NodeAgent(5000, 5, 5, 0, 3);

        assertTrue(env.deployAgent(agent));
        assertTrue(env.agentById().containsKey(5000));
    }

    @Test
    void deployAgent_toOccupiedCell_returnsFalse() {
        Environment env = new Environment(20, 20);
        NodeAgent first  = new NodeAgent(5000, 5, 5, 0, 3);
        NodeAgent second = new NodeAgent(5001, 5, 5, 0, 3); // same cell

        assertTrue(env.deployAgent(first));
        assertFalse(env.deployAgent(second));

        // Only the first agent should be registered
        assertEquals(1, env.agentById().size());
    }

    // ── Range queries ─────────────────────────────────────────────────────────

    @Test
    void findAgentsInRange_agentWithinRange_included() {
        Environment env = new Environment(100, 100);
        NodeAgent agent = new NodeAgent(5000, 10, 10, 0, 3);
        env.deployAgent(agent);

        // Agent at (10,10); query centre (10,13); dist = 3 ≤ range 5 → included
        List<NodeAgent> found = env.findAgentsInRange(10, 13, 5);

        assertEquals(1, found.size());
        assertEquals(5000, found.get(0).getId());
    }

    @Test
    void findAgentsInRange_agentAtExactCentre_excluded() {
        Environment env = new Environment(100, 100);
        NodeAgent agent = new NodeAgent(5000, 10, 10, 0, 3);
        env.deployAgent(agent);

        // distSq == 0 → condition (distSq > 0) is false → excluded
        List<NodeAgent> found = env.findAgentsInRange(10, 10, 5);

        assertTrue(found.isEmpty());
    }

    @Test
    void findAgentsInRange_agentExactlyOnBoundary_included() {
        Environment env = new Environment(100, 100);
        NodeAgent agent = new NodeAgent(5000, 10, 15, 0, 3);
        env.deployAgent(agent);

        // dist = 5, range = 5 → distSq == rangeSq → inclusive upper bound, included
        List<NodeAgent> found = env.findAgentsInRange(10, 10, 5);

        assertEquals(1, found.size());
    }

    @Test
    void findAgentsInRange_agentOutsideRange_excluded() {
        Environment env = new Environment(100, 100);
        NodeAgent agent = new NodeAgent(5000, 10, 20, 0, 3);
        env.deployAgent(agent);

        // dist = 10 > range 5 → excluded
        List<NodeAgent> found = env.findAgentsInRange(10, 10, 5);

        assertTrue(found.isEmpty());
    }

    @Test
    void findAgentsInRange_noAgentsDeployed_returnsEmpty() {
        Environment env = new Environment(50, 50);

        List<NodeAgent> found = env.findAgentsInRange(25, 25, 10);

        assertTrue(found.isEmpty());
    }

    @Test
    void findAgentsInRange_multipleAgents_onlyInRangeReturned() {
        Environment env = new Environment(100, 100);
        // Agent A: dist = 3 (within range 5)
        env.deployAgent(new NodeAgent(5000, 10, 13, 0, 3));
        // Agent B: dist = 10 (outside range 5)
        env.deployAgent(new NodeAgent(5001, 10, 20, 0, 3));

        List<NodeAgent> found = env.findAgentsInRange(10, 10, 5);

        assertEquals(1, found.size());
        assertEquals(5000, found.get(0).getId());
    }

    // ── Ordering ──────────────────────────────────────────────────────────────

    @Test
    void allAgents_preservesDeploymentOrder() {
        Environment env = new Environment(100, 100);
        env.deployAgent(new NodeAgent(5000, 0,  0, 0, 3));
        env.deployAgent(new NodeAgent(5001, 1,  1, 0, 3));
        env.deployAgent(new NodeAgent(5002, 2,  2, 0, 3));

        NodeAgent[] agents = env.allAgents().toArray(new NodeAgent[0]);

        assertEquals(5000, agents[0].getId());
        assertEquals(5001, agents[1].getId());
        assertEquals(5002, agents[2].getId());
    }

    // ── Grid access ───────────────────────────────────────────────────────────

    @Test
    void setCell_toEmptyCell_returnsTrue() {
        Environment env = new Environment(10, 10);

        assertTrue(env.setCell(3, 4, 747));
        // Confirm the field carries the value (via deployAgent clash)
        NodeAgent agent = new NodeAgent(5000, 3, 4, 0, 3);
        assertFalse(env.deployAgent(agent), "Cell (3,4) should be occupied after setCell");
    }

    @Test
    void setCell_toOccupiedCell_returnsFalse() {
        Environment env = new Environment(10, 10);
        env.deployAgent(new NodeAgent(5000, 3, 4, 0, 3));

        assertFalse(env.setCell(3, 4, 747));
    }
}
