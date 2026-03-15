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

import algorithm.Route;
import domain.SensorNode;

/**
 * Strategy interface for a UAV's autonomous decision-making per tick.
 *
 * <p>An implementation is called once per drone tick, <em>after</em> the UAV
 * has scanned its surroundings and moved one step.  It may update a coverage
 * tracker, steer the UAV toward unexplored regions, trigger replanning, or
 * do nothing at all.
 *
 * <h3>Built-in implementations</h3>
 * <ul>
 *   <li>{@link intelligence.PredefinedPatrolIntelligence} — no replanning; the UAV follows
 *       the fixed four-corner perimeter loop for the entire run.  Serves as
 *       the A/B baseline.</li>
 *   <li>{@link intelligence.GaAcoIntelligence} — frontier-based coverage tracking,
 *       coverage-gated replanning, and the GA→ACO→LocalSearch path-planning
 *       pipeline.</li>
 * </ul>
 *
 * <h3>Adding a new intelligence model</h3>
 * Implement this interface and pass an instance to
 * {@link SimulationRunner#run(java.util.List)}.  No changes to {@link UAV},
 * the event loop, or any other class are required.
 *
 * <h3>Threading</h3>
 * Each call to {@link SimulationRunner#runOnce} creates a fresh
 * {@code UAVIntelligence} instance; implementations are therefore never
 * shared between threads and do not need to be thread-safe.
 */
public interface UAVIntelligence {

    /**
     * Invoked once per drone tick, after the UAV has scanned its surroundings
     * and moved one step.
     *
     * <p>Implementations may:
     * <ul>
     *   <li>Update a coverage tracker ({@link CoverageGrid#markScanned}).</li>
     *   <li>Set the UAV's frontier exploration target
     *       ({@link UAV#setExplorationTarget}).</li>
     *   <li>Trigger replanning ({@link UAV#replan}) and return the resulting
     *       {@link Route}.</li>
     *   <li>Do nothing and return {@code null}.</li>
     * </ul>
     *
     * @param tick simulation tick at which this event occurred
     * @param uav  the drone entity; may call any public method
     * @param env  the simulation environment (read-only for intelligence use)
     * @return the new {@link Route} if replanning fired, {@code null} otherwise
     */
    Route onTick(long tick, UAV uav, Environment env);

    /**
     * Human-readable label shown in run reports and the GUI sidebar.
     *
     * @return display label, e.g. {@code "Predefined Patrol"} or
     *         {@code "Autonomous GA→ACO"}
     */
    String getLabel();

    /**
     * Returns the coverage grid managed by this intelligence, or {@code null}
     * if this intelligence does not track field coverage.
     *
     * <p>Used by {@link SimulationRunner} to include sector data in GUI
     * snapshots and in the final coverage-fraction statistic.
     *
     * @return {@link CoverageGrid} instance, or {@code null}
     */
    default CoverageGrid getCoverageGrid() {
        return null;
    }

    /**
     * Computes the effective intercept point for a waypoint node during
     * route execution (EXECUTE mode).
     *
     * <p>The default implementation returns the node's current position —
     * the UAV simply moves toward where the node is now.
     *
     * <p>Override to implement predictive interception of moving targets.
     * See {@link intelligence.GaAcoIntelligence} for a quadratic ballistic
     * intercept example that leads moving nodes to minimise travel time.
     *
     * @param target   the current waypoint node
     * @param uavX     UAV's current x position
     * @param uavY     UAV's current y position
     * @param uavSpeed UAV's speed in cells per tick
     * @return [x, y] of the point to steer toward; clamping to field bounds
     *         is handled by the caller
     */
    default int[] interceptPoint(SensorNode target, int uavX, int uavY, int uavSpeed) {
        return new int[]{ (int) Math.round(target.getX()),
                          (int) Math.round(target.getY()) };
    }
}
