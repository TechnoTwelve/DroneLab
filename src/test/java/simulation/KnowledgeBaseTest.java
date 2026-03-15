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
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link KnowledgeBase}.
 */
class KnowledgeBaseTest {

    private KnowledgeBase kb;

    @BeforeEach
    void setUp() {
        kb = new KnowledgeBase();
    }

    // ── Initial state ─────────────────────────────────────────────────────────

    @Test
    void newKnowledgeBase_sizeIsZero() {
        assertEquals(0, kb.size());
    }

    @Test
    void newKnowledgeBase_knowsReturnsFalse() {
        assertFalse(kb.knows(5000));
    }

    @Test
    void newKnowledgeBase_snapshotIsEmpty() {
        assertTrue(kb.snapshot().isEmpty());
    }

    // ── observe(SensorNode) ───────────────────────────────────────────────────

    @Test
    void observeSensorNode_addsSingleNode() {
        kb.observe(new SensorNode(1, 10, 20));
        assertEquals(1, kb.size());
        assertTrue(kb.knows(1));
    }

    @Test
    void observeSensorNode_updatePositionOnReobservation() {
        kb.observe(new SensorNode(1, 10, 20));
        kb.observe(new SensorNode(1, 99, 88)); // same id, new position
        assertEquals(1, kb.size());
        SensorNode latest = kb.snapshot().get(0);
        assertEquals(99, latest.getX(), 1e-9);
        assertEquals(88, latest.getY(), 1e-9);
    }

    @Test
    void observeSensorNode_multipleDistinctNodes() {
        kb.observe(new SensorNode(1, 0, 0));
        kb.observe(new SensorNode(2, 1, 0));
        kb.observe(new SensorNode(3, 2, 0));
        assertEquals(3, kb.size());
    }

    // ── observe(NodeAgent) ────────────────────────────────────────────────────

    @Test
    void observeNodeAgent_addsNodeFromAgent() {
        NodeAgent agent = new NodeAgent(5000, 10, 20, 1, 2);
        kb.observe(agent);
        assertEquals(1, kb.size());
        assertTrue(kb.knows(5000));
    }

    @Test
    void observeNodeAgent_snapshotHasCorrectPosition() {
        NodeAgent agent = new NodeAgent(5001, 30, 40, 3, 1);
        kb.observe(agent);
        SensorNode snap = kb.snapshot().get(0);
        assertEquals(30, snap.getX(), 1e-9);
        assertEquals(40, snap.getY(), 1e-9);
    }

    // ── observeWithTransitiveCache ────────────────────────────────────────────

    @Test
    void observeWithTransitiveCache_addsDirectAndTransitiveNodes() {
        NodeAgent direct  = new NodeAgent(5000, 0, 0, 0, 1);
        NodeAgent neighbor = new NodeAgent(5001, 100, 100, 2, 1);
        direct.addNeighbor(5001);

        Map<Integer, NodeAgent> index = Map.of(5000, direct, 5001, neighbor);
        kb.observeWithTransitiveCache(List.of(direct), index);

        assertTrue(kb.knows(5000));
        assertTrue(kb.knows(5001));
        assertEquals(2, kb.size());
    }

    @Test
    void observeWithTransitiveCache_doesNotOverwriteExistingDirectObservation() {
        // If a node is already in the KB (from a direct observation), the transitive
        // copy must not overwrite it.
        NodeAgent direct     = new NodeAgent(5000, 0, 0, 0, 1);
        NodeAgent alreadyKnown = new NodeAgent(5001, 99, 99, 1, 1);

        // Observe 5001 directly first
        kb.observe(new SensorNode(5001, 50, 50));

        direct.addNeighbor(5001);
        Map<Integer, NodeAgent> index = Map.of(5000, direct, 5001, alreadyKnown);
        kb.observeWithTransitiveCache(List.of(direct), index);

        // 5001 should still have original direct observation (x=50, y=50)
        SensorNode snap = kb.snapshot().stream()
                .filter(n -> n.getId() == 5001)
                .findFirst()
                .orElseThrow();
        assertEquals(50, snap.getX(), 1e-9);
    }

    @Test
    void observeWithTransitiveCache_missingNeighborInIndex_skipsGracefully() {
        NodeAgent direct = new NodeAgent(5000, 0, 0, 0, 1);
        direct.addNeighbor(9999); // 9999 not in index

        Map<Integer, NodeAgent> index = Map.of(5000, direct);
        assertDoesNotThrow(() ->
                kb.observeWithTransitiveCache(List.of(direct), index));
        assertEquals(1, kb.size()); // only the direct node recorded
    }

    // ── snapshot ─────────────────────────────────────────────────────────────

    @Test
    void snapshot_isUnmodifiable() {
        kb.observe(new SensorNode(1, 0, 0));
        assertThrows(UnsupportedOperationException.class,
                () -> kb.snapshot().add(new SensorNode(2, 0, 0)));
    }

    @Test
    void snapshot_preservesInsertionOrder() {
        kb.observe(new SensorNode(10, 0, 0));
        kb.observe(new SensorNode(20, 0, 0));
        kb.observe(new SensorNode(30, 0, 0));
        List<SensorNode> snap = kb.snapshot();
        assertEquals(10, snap.get(0).getId());
        assertEquals(20, snap.get(1).getId());
        assertEquals(30, snap.get(2).getId());
    }

    // ── clear ─────────────────────────────────────────────────────────────────

    @Test
    void clear_removesAllNodes() {
        kb.observe(new SensorNode(1, 0, 0));
        kb.observe(new SensorNode(2, 1, 0));
        kb.clear();
        assertEquals(0, kb.size());
        assertFalse(kb.knows(1));
    }

    @Test
    void clear_snapshotEmptyAfterClear() {
        kb.observe(new SensorNode(1, 0, 0));
        kb.clear();
        assertTrue(kb.snapshot().isEmpty());
    }

    // ── toString ──────────────────────────────────────────────────────────────

    @Test
    void toString_containsKnownNodeCount() {
        kb.observe(new SensorNode(1, 0, 0));
        kb.observe(new SensorNode(2, 0, 0));
        assertTrue(kb.toString().contains("2"));
    }
}
