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
import org.junit.jupiter.api.Test;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link NodeAgent}.
 */
class NodeAgentTest {

    // ── Constructor / initial state ───────────────────────────────────────────

    @Test
    void constructor_storesAllFields() {
        NodeAgent agent = new NodeAgent(5000, 10, 20, 3, 2);
        assertEquals(5000, agent.getId());
        assertEquals(10,   agent.getX());
        assertEquals(20,   agent.getY());
        assertEquals(3,    agent.getDirection());
        assertEquals(2,    agent.getSpeed());
    }

    @Test
    void constructor_neighborCacheIsEmpty() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        assertTrue(agent.getNeighborCache().isEmpty());
    }

    // ── updatePosition ────────────────────────────────────────────────────────

    @Test
    void updatePosition_changesXAndY() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        agent.updatePosition(50, 75);
        assertEquals(50, agent.getX());
        assertEquals(75, agent.getY());
    }

    @Test
    void updatePosition_doesNotChangeIdOrSpeed() {
        NodeAgent agent = new NodeAgent(99, 5, 5, 2, 3);
        agent.updatePosition(0, 0);
        assertEquals(99, agent.getId());
        assertEquals(3,  agent.getSpeed());
    }

    // ── setDirection ─────────────────────────────────────────────────────────

    @Test
    void setDirection_updatesDirection() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        agent.setDirection(7);
        assertEquals(7, agent.getDirection());
    }

    @Test
    void setDirection_allValidDirections() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        for (int d = 0; d < NodeAgent.DIRECTION_COUNT; d++) {
            agent.setDirection(d);
            assertEquals(d, agent.getDirection());
        }
    }

    // ── addNeighbor / hasNeighbor ─────────────────────────────────────────────

    @Test
    void addNeighbor_addsId() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        agent.addNeighbor(5001);
        assertTrue(agent.hasNeighbor(5001));
    }

    @Test
    void hasNeighbor_returnsFalseForUnknownId() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        assertFalse(agent.hasNeighbor(9999));
    }

    @Test
    void addNeighbor_duplicateSilentlyIgnored() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        agent.addNeighbor(5001);
        agent.addNeighbor(5001); // duplicate
        assertEquals(1, agent.getNeighborCache().size());
    }

    @Test
    void addNeighbor_multipleDistinctIds() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        agent.addNeighbor(100);
        agent.addNeighbor(200);
        agent.addNeighbor(300);
        assertEquals(3, agent.getNeighborCache().size());
    }

    // ── getNeighborCache ──────────────────────────────────────────────────────

    @Test
    void getNeighborCache_isUnmodifiable() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        Set<Integer> cache = agent.getNeighborCache();
        assertThrows(UnsupportedOperationException.class, () -> cache.add(999));
    }

    @Test
    void getNeighborCache_preservesInsertionOrder() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 0, 1);
        agent.addNeighbor(10);
        agent.addNeighbor(20);
        agent.addNeighbor(30);
        Integer[] ids = agent.getNeighborCache().toArray(new Integer[0]);
        assertEquals(10, ids[0]);
        assertEquals(20, ids[1]);
        assertEquals(30, ids[2]);
    }

    // ── DIRECTION_COUNT constant ──────────────────────────────────────────────

    @Test
    void directionCount_isEight() {
        assertEquals(8, NodeAgent.DIRECTION_COUNT);
    }

    // ── toSensorNode ─────────────────────────────────────────────────────────

    @Test
    void toSensorNode_capturesCurrentState() {
        NodeAgent agent = new NodeAgent(5005, 30, 40, 6, 2);
        SensorNode snap = agent.toSensorNode();
        assertEquals(5005, snap.getId());
        assertEquals(30,   snap.getX(),  1e-9);
        assertEquals(40,   snap.getY(),  1e-9);
        assertEquals(6,    snap.getDirection());
        assertEquals(2,    snap.getSpeed());
    }

    @Test
    void toSensorNode_snapshotIsIndependentOfSubsequentPositionUpdate() {
        NodeAgent agent = new NodeAgent(1, 10, 20, 0, 1);
        SensorNode snap = agent.toSensorNode();
        agent.updatePosition(999, 999);
        assertEquals(10, snap.getX(), 1e-9); // snapshot unchanged
        assertEquals(20, snap.getY(), 1e-9);
    }

    @Test
    void toSensorNode_hasVelocity() {
        NodeAgent agent = new NodeAgent(1, 0, 0, 2, 3);
        assertTrue(agent.toSensorNode().hasVelocity());
    }

    // ── toString ──────────────────────────────────────────────────────────────

    @Test
    void toString_containsIdAndPosition() {
        NodeAgent agent = new NodeAgent(42, 10, 20, 1, 2);
        String s = agent.toString();
        assertTrue(s.contains("42"));
        assertTrue(s.contains("10"));
        assertTrue(s.contains("20"));
    }
}
