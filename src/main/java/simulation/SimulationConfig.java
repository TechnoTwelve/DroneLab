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

import config.AppConfig;

/**
 * Immutable configuration for the discrete-event UAV/WSN simulator.
 *
 * <p>This class holds only environment parameters — field dimensions, node
 * mobility, and drone physical properties.  Algorithm-specific parameters
 * (planning thresholds, route waypoint limits, etc.) live in
 * own config class (e.g. {@link algorithm.gaaco.GaAcoConfig}) and are owned by the intelligence
 * that uses them.
 *
 * <p>Use {@link #builder()} to construct from a clean baseline, or
 * {@link #defaults()} for the optimised WSN data-collection defaults, then
 * use the {@code withXxx} copy-and-override methods for single-field variants.
 */
public final class SimulationConfig {

    private final int  fieldWidth;
    private final int  fieldHeight;
    private final int  nodeCount;
    private final int  nodeIdStart;
    private final int  nodeRange;
    private final int  nodeMinSpeed;
    private final int  nodeMaxSpeed;
    private final int  droneRange;
    private final int  dronePlacement;
    private final int  droneSpeed;
    private final long duration;

    public SimulationConfig(int  fieldWidth,
                            int  fieldHeight,
                            int  nodeCount,
                            int  nodeIdStart,
                            int  nodeRange,
                            int  nodeMinSpeed,
                            int  nodeMaxSpeed,
                            int  droneRange,
                            int  dronePlacement,
                            int  droneSpeed,
                            long duration) {
        this.fieldWidth     = fieldWidth;
        this.fieldHeight    = fieldHeight;
        this.nodeCount      = nodeCount;
        this.nodeIdStart    = nodeIdStart;
        this.nodeRange      = nodeRange;
        this.nodeMinSpeed   = nodeMinSpeed;
        this.nodeMaxSpeed   = nodeMaxSpeed;
        this.droneRange     = droneRange;
        this.dronePlacement = dronePlacement;
        this.droneSpeed     = droneSpeed;
        this.duration       = duration;
    }

    /**
     * Returns the default simulation configuration derived from
     * {@link AppConfig}.
     *
     * <p>The authoritative default values live in
     * {@link AppConfig#defaultProperties()} (mirrored in
     * {@code config.properties}), so updating either of those is the only
     * place that needs changing when defaults change project-wide.
     *
     * <h3>Field and timing notes</h3>
     * <pre>
     *   dronePlacement (dp) = 72.  Patrol corners at (dp,dp), (dp,F-dp),
     *   (F-dp,F-dp), (F-dp,dp).  Each leg = F - 2*dp = 1356 cells.
     *   one_lap = 4 * 1356 = 5424 ticks.
     *
     *   duration = 5434 = one_lap + 10.  The +10 ensures return-home fires
     *   exactly at W4 so all four sides of the rectangle are drawn cleanly.
     * </pre>
     *
     * <h3>Drone range</h3>
     * droneRange 140: parameter sweep over 20 seeds found this is the sweet
     * spot (+17.84% vs patrol, 16/20 wins).  A larger arrival radius causes
     * the drone to declare waypoints "reached" too early; 140 balances scan
     * coverage with precise waypoint traversal.
     */
    public static SimulationConfig defaults() {
        return AppConfig.load().buildSimulationConfig();
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    public int  getFieldWidth()      { return fieldWidth; }
    public int  getFieldHeight()     { return fieldHeight; }
    public int  getNodeCount()       { return nodeCount; }
    public int  getNodeIdStart()     { return nodeIdStart; }
    public int  getNodeRange()       { return nodeRange; }
    public int  getNodeMinSpeed()    { return nodeMinSpeed; }
    public int  getNodeMaxSpeed()    { return nodeMaxSpeed; }
    public int  getDroneRange()      { return droneRange; }
    public int  getDronePlacement()  { return dronePlacement; }
    public int  getDroneSpeed()      { return droneSpeed; }
    public long getDuration()        { return duration; }

    // ── Immutable copy-and-override ───────────────────────────────────────────

    public SimulationConfig withDroneRange(int v) {
        return new SimulationConfig(fieldWidth, fieldHeight, nodeCount, nodeIdStart,
                nodeRange, nodeMinSpeed, nodeMaxSpeed, v, dronePlacement,
                droneSpeed, duration);
    }

    public SimulationConfig withNodeCount(int v) {
        return new SimulationConfig(fieldWidth, fieldHeight, v, nodeIdStart,
                nodeRange, nodeMinSpeed, nodeMaxSpeed, droneRange, dronePlacement,
                droneSpeed, duration);
    }

    public SimulationConfig withDuration(long v) {
        return new SimulationConfig(fieldWidth, fieldHeight, nodeCount, nodeIdStart,
                nodeRange, nodeMinSpeed, nodeMaxSpeed, droneRange, dronePlacement,
                droneSpeed, v);
    }

    @Override
    public String toString() {
        return String.format(
                "SimulationConfig{field=%dx%d, nodes=%d, nodeRange=%d, " +
                "nodeSpeed=[%d,%d], droneRange=%d, placement=%d, speed=%d, duration=%d}",
                fieldWidth, fieldHeight, nodeCount, nodeRange,
                nodeMinSpeed, nodeMaxSpeed, droneRange, dronePlacement,
                droneSpeed, duration);
    }

    // ── Builder ───────────────────────────────────────────────────────────────

    /**
     * Returns a new {@link Builder} pre-populated with the same defaults as
     * {@link #defaults()}.  Override only the fields relevant to your scenario.
     *
     * <pre>
     *   SimulationConfig config = SimulationConfig.builder()
     *       .fieldSize(2000, 2000)
     *       .nodeCount(400)
     *       .droneRange(200)
     *       .duration(8000L)
     *       .build();
     * </pre>
     */
    /**
     * Returns a new {@link Builder} pre-populated from {@link #defaults()}.
     * Only call setters for the fields you want to change.
     */
    public static Builder builder() {
        SimulationConfig d = defaults();
        return new Builder()
                .fieldSize(d.getFieldWidth(), d.getFieldHeight())
                .nodeCount(d.getNodeCount())
                .nodeIdStart(d.getNodeIdStart())
                .nodeRange(d.getNodeRange())
                .nodeSpeed(d.getNodeMinSpeed(), d.getNodeMaxSpeed())
                .droneRange(d.getDroneRange())
                .dronePlacement(d.getDronePlacement())
                .droneSpeed(d.getDroneSpeed())
                .duration(d.getDuration());
    }

    public static final class Builder {

        // Fields have no initializers — builder() pre-populates them from defaults().
        // Construct via SimulationConfig.builder(), not new Builder() directly.
        private int  fieldWidth;
        private int  fieldHeight;
        private int  nodeCount;
        private int  nodeIdStart;
        private int  nodeRange;
        private int  nodeMinSpeed;
        private int  nodeMaxSpeed;
        private int  droneRange;
        private int  dronePlacement;
        private int  droneSpeed;
        private long duration;

        private Builder() {}

        public Builder fieldWidth(int v)           { fieldWidth = v;                        return this; }
        public Builder fieldHeight(int v)          { fieldHeight = v;                       return this; }
        /** Convenience: set both width and height in one call. */
        public Builder fieldSize(int w, int h)     { fieldWidth = w; fieldHeight = h;       return this; }
        public Builder nodeCount(int v)            { nodeCount = v;                         return this; }
        public Builder nodeIdStart(int v)          { nodeIdStart = v;                       return this; }
        public Builder nodeRange(int v)            { nodeRange = v;                         return this; }
        /** Convenience: set both min and max node speed in one call. */
        public Builder nodeSpeed(int min, int max) { nodeMinSpeed = min; nodeMaxSpeed = max; return this; }
        public Builder droneRange(int v)           { droneRange = v;                        return this; }
        public Builder dronePlacement(int v)       { dronePlacement = v;                    return this; }
        public Builder droneSpeed(int v)           { droneSpeed = v;                        return this; }
        public Builder duration(long v)            { duration = v;                          return this; }

        public SimulationConfig build() {
            return new SimulationConfig(fieldWidth, fieldHeight, nodeCount, nodeIdStart,
                    nodeRange, nodeMinSpeed, nodeMaxSpeed, droneRange, dronePlacement,
                    droneSpeed, duration);
        }
    }
}
