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

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Per-UAV accumulator of sensor-node knowledge.
 *
 * <p>The {@code KnowledgeBase} records every sensor node that the UAV has
 * directly detected (within its own detection range) or indirectly discovered
 * (via neighbour caches transferred from detected nodes).  It keeps the most
 * recent position snapshot for each known node, de-duplicating by node ID.
 *
 * <h3>Bug E fix — no static state</h3>
 * The legacy {@code Nodes.Node} class held two {@code static ArrayList}
 * fields for drone caches, meaning every node instance shared a single list.
 * The {@code KnowledgeBase} is an instance owned by one specific {@link UAV};
 * it has no static fields and never leaks state across UAV instances.
 *
 * <h3>Snapshot contract</h3>
 * Calling {@link #snapshot()} returns an ordered, unmodifiable
 * {@code List<SensorNode>} safe to pass to the algorithm layer.  The list
 * reflects positions at the time of last observation and is not affected by
 * subsequent movement of sensor nodes.  A fresh snapshot should be taken
 * immediately before each path-planning invocation.
 */
public final class KnowledgeBase {

    /**
     * Maps node ID → most recent {@link SensorNode} snapshot.
     * {@link LinkedHashMap} preserves insertion order (discovery order),
     * which makes {@link #snapshot()} deterministic.
     */
    private final Map<Integer, SensorNode> latestPositions;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Constructs an empty knowledge base.
     */
    public KnowledgeBase() {
        this.latestPositions = new LinkedHashMap<>();
    }

    // ── Observation ───────────────────────────────────────────────────────────

    /**
     * Records a direct observation of a sensor node.
     *
     * <p>If the node has been seen before, its position snapshot is updated
     * to reflect its current location.  If the node is new, it is appended
     * to the discovery record (insertion order is preserved).
     *
     * @param snapshot immutable position snapshot of the observed node
     */
    public void observe(SensorNode snapshot) {
        latestPositions.put(snapshot.getId(), snapshot);
    }

    /**
     * Records a direct observation from a live {@link NodeAgent}.
     *
     * <p>Convenience overload that converts the agent to a snapshot before
     * storing.
     *
     * @param agent the observed node agent; must not be null
     */
    public void observe(NodeAgent agent) {
        observe(agent.toSensorNode());
    }

    /**
     * Records direct observations for a batch of agents and, for each,
     * also ingests the agent's neighbour cache (transitive discovery).
     *
     * <p>This implements the same effect as the original code's "direct
     * caching with node caching" mode: when the UAV detects a node, it also
     * learns about the nodes that node has previously encountered.
     *
     * @param agents     agents directly detected by the UAV
     * @param agentIndex a map from node ID to {@link NodeAgent} used to look
     *                   up neighbours (pass {@link Environment#agentById()})
     */
    public void observeWithTransitiveCache(List<NodeAgent> agents,
                                           Map<Integer, NodeAgent> agentIndex) {
        for (NodeAgent agent : agents) {
            observe(agent);
            // Ingest the neighbour's cache (transitive knowledge)
            for (int neighborId : agent.getNeighborCache()) {
                NodeAgent neighbor = agentIndex.get(neighborId);
                if (neighbor != null && !latestPositions.containsKey(neighborId)) {
                    observe(neighbor);
                }
            }
        }
    }

    // ── Query ─────────────────────────────────────────────────────────────────

    /**
     * Returns {@code true} if the node with the given ID has been observed.
     *
     * @param nodeId node ID to check
     * @return {@code true} if the node is known
     */
    public boolean knows(int nodeId) {
        return latestPositions.containsKey(nodeId);
    }

    /**
     * Returns the number of distinct nodes currently in the knowledge base.
     *
     * @return known-node count
     */
    public int size() {
        return latestPositions.size();
    }

    /**
     * Returns an ordered, unmodifiable snapshot of all known nodes, suitable
     * for passing to the algorithm layer.
     *
     * <p>The ordering is discovery order (the order in which nodes were first
     * added to the knowledge base).  Position data reflects the most recent
     * observation of each node.
     *
     * @return unmodifiable list of known {@link SensorNode} snapshots
     */
    public List<SensorNode> snapshot() {
        return Collections.unmodifiableList(new ArrayList<>(latestPositions.values()));
    }

    /**
     * Clears all accumulated knowledge.  Used when the UAV resets between
     * experimental runs.
     */
    public void clear() {
        latestPositions.clear();
    }

    // ── Object overrides ──────────────────────────────────────────────────────

    @Override
    public String toString() {
        return "KnowledgeBase{knownNodes=" + latestPositions.size() + "}";
    }
}
