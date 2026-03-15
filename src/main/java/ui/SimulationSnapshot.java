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
package ui;

import simulation.UAV;
import java.util.List;

/**
 * Immutable snapshot of the simulation state at one drone-event tick.
 * Passed from {@link simulation.SimulationRunner} to
 * {@link VisualizationListener} for rendering.
 */
public final class SimulationSnapshot {

    // ── Meta ──────────────────────────────────────────────────────────────────
    public final long   tick;
    /** 1 = predefined patrol pass, 2 = autonomous pass. */
    public final int    pass;
    /** Human-readable label for the current pass. */
    public final String passLabel;
    /** Total number of simulation ticks configured. */
    public final long   duration;
    /** Total sensor nodes deployed. */
    public final int    totalNodes;
    /** Simulation field dimensions (cells). */
    public final int    fieldWidth, fieldHeight;

    // ── UAV ───────────────────────────────────────────────────────────────────
    public final int           uavX, uavY;
    /** Home/deployment position of the UAV. */
    public final int           homeX, homeY;
    public final UAV.DriveMode uavMode;
    public final int           droneRange;
    public final int           tokens;
    public final int           kbSize;

    // ── Nodes (one per sensor node) ───────────────────────────────────────────
    public final List<NodeSnap> nodes;

    // ── Route (non-empty only while UAV is in EXECUTE mode) ───────────────────
    /** Ordered list of [x, y] pairs for the active route's waypoints. */
    public final List<double[]> routeWaypoints;

    // ── Frontier exploration target (autonomous pass, PATROL mode only) ───────
    /** [x, y] of the nearest unvisited sector centre, or {@code null}. */
    public final int[] frontierTarget;

    // ── Coverage grid (autonomous pass only; null for patrol pass) ────────────
    /** Deep-copied visited-sector array, or {@code null} for the patrol pass. */
    public final boolean[][] visitedSectors;
    public final int         sectorSize;
    public final double      coverageFraction;

    // ── Constructor ───────────────────────────────────────────────────────────

    public SimulationSnapshot(long tick, int pass, String passLabel,
                               long duration, int totalNodes,
                               int fieldWidth, int fieldHeight,
                               int uavX, int uavY, int homeX, int homeY,
                               UAV.DriveMode uavMode,
                               int droneRange, int tokens, int kbSize,
                               List<NodeSnap> nodes,
                               List<double[]> routeWaypoints,
                               int[] frontierTarget,
                               boolean[][] visitedSectors, int sectorSize,
                               double coverageFraction) {
        this.tick             = tick;
        this.pass             = pass;
        this.passLabel        = passLabel;
        this.duration         = duration;
        this.totalNodes       = totalNodes;
        this.fieldWidth       = fieldWidth;
        this.fieldHeight      = fieldHeight;
        this.uavX             = uavX;
        this.uavY             = uavY;
        this.homeX            = homeX;
        this.homeY            = homeY;
        this.uavMode          = uavMode;
        this.droneRange       = droneRange;
        this.tokens           = tokens;
        this.kbSize           = kbSize;
        this.nodes            = nodes;
        this.routeWaypoints   = routeWaypoints;
        this.frontierTarget   = frontierTarget;
        this.visitedSectors   = visitedSectors;
        this.sectorSize       = sectorSize;
        this.coverageFraction = coverageFraction;
    }

    // ── Nested node state ─────────────────────────────────────────────────────

    /** Immutable snapshot of a single sensor node's state at one tick. */
    public static final class NodeSnap {
        public final int     id;
        public final double  x, y;
        /** True if the UAV has ever been within scan range of this node. */
        public final boolean tokenized;
        /** True if the node is in the UAV's knowledge base. */
        public final boolean knownToUav;

        public NodeSnap(int id, double x, double y,
                        boolean tokenized, boolean knownToUav) {
            this.id         = id;
            this.x          = x;
            this.y          = y;
            this.tokenized  = tokenized;
            this.knownToUav = knownToUav;
        }
    }
}
