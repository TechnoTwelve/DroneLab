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
package intelligence;

import algorithm.Route;
import algorithm.gaaco.GaAcoConfig;
import domain.SensorNode;
import simulation.*;

import java.util.ArrayList;
import java.util.List;

/**
 * {@link UAVIntelligence} implementation that combines frontier-based
 * coverage exploration with coverage-gated GA→ACO→LocalSearch path planning.
 *
 * <h3>Per-tick behaviour ({@link #onTick})</h3>
 * <ol>
 *   <li><b>Coverage update</b> — marks sectors within drone range as visited
 *       so that sectors traversed during {@code EXECUTE} routes are correctly
 *       recorded and not re-targeted during the next {@code PATROL} phase.</li>
 *   <li><b>Frontier steering</b> — during {@code PATROL} mode, computes the
 *       nearest unvisited sector centre and passes it to
 *       {@link UAV#setExplorationTarget}, driving the UAV toward unexplored
 *       areas.  Cleared when the entire field is covered.</li>
 *   <li><b>Coverage-gated replanning</b> — triggers {@link UAV#replan(List)}
 *       only when all four conditions hold:
 *       <ul>
 *         <li>UAV is in {@code PATROL} mode (not mid-route).</li>
 *         <li>KB size ≥ {@link GaAcoConfig#getPlanningThreshold()}.</li>
 *         <li>Tick is divisible by {@link GaAcoConfig#getPlanningInterval()}.</li>
 *         <li>Coverage fraction ≥ {@link GaAcoConfig#getMinCoverageBeforePlan()}.</li>
 *       </ul>
 *   </li>
 * </ol>
 *
 * <h3>Target selection</h3>
 * Before calling {@link UAV#replan(List)}, this intelligence:
 * <ol>
 *   <li>Filters the KB snapshot to untokenised nodes only.</li>
 *   <li>Sorts them by squared Euclidean distance from the UAV.</li>
 *   <li>Takes the nearest {@link GaAcoConfig#getMaxRouteWaypoints()} entries.</li>
 * </ol>
 * This compact-disc selection produces short routes (~500–800 ticks), enabling
 * multiple replanning cycles per session and higher overall field coverage.
 *
 * <h3>Ownership</h3>
 * This class owns a {@link CoverageGrid} instance that is exposed via
 * {@link #getCoverageGrid()} for GUI rendering and final statistics.
 */
public final class GaAcoIntelligence implements UAVIntelligence {

    private final SimulationConfig config;
    private final GaAcoConfig  algConfig;
    private final CoverageGrid     coverage;

    /**
     * Constructs a {@code GaAcoIntelligence} from simulation and algorithm
     * configurations.
     *
     * @param config    simulation parameters (field dimensions, drone range); must not be null
     * @param algConfig GA-ACO and planning-schedule parameters; must not be null
     */
    public GaAcoIntelligence(SimulationConfig config, GaAcoConfig algConfig) {
        if (config    == null) throw new IllegalArgumentException("config must not be null");
        if (algConfig == null) throw new IllegalArgumentException("algConfig must not be null");
        this.config    = config;
        this.algConfig = algConfig;
        this.coverage  = new CoverageGrid(config.getFieldWidth(),
                                          config.getFieldHeight(),
                                          config.getDroneRange());
    }

    // ── UAVIntelligence ───────────────────────────────────────────────────────

    /**
     * Updates the coverage grid, steers the UAV toward the nearest unvisited
     * sector during {@code PATROL}, and triggers replanning when all
     * conditions are met.
     *
     * @param tick simulation tick
     * @param uav  the drone entity
     * @param env  the simulation environment (available for extensions)
     * @return new {@link Route} if replanning fired, {@code null} otherwise
     */
    @Override
    public Route onTick(long tick, UAV uav, Environment env) {

        // Step 1: mark sectors within scan radius as visited.
        // Done every tick (including EXECUTE mode) so sectors covered during
        // route execution are not re-targeted when the UAV returns to PATROL.
        coverage.markScanned(uav.getX(), uav.getY(), config.getDroneRange());

        // Step 2: frontier steering — redirect the UAV toward the nearest
        // unvisited sector during PATROL.  Has no effect in EXECUTE mode
        // (the UAV follows planned waypoints instead).
        if (uav.getDriveMode() == UAV.DriveMode.PATROL) {
            int[] frontier = coverage.nearestUnvisitedCenter(uav.getX(), uav.getY());
            if (frontier != null) {
                uav.setExplorationTarget(frontier[0], frontier[1]);
            } else {
                uav.clearExplorationTarget(); // all sectors visited
            }
        }

        // Step 3: coverage-gated replanning.
        // All four conditions must hold simultaneously before we plan:
        //   - PATROL mode guard prevents re-routing mid-execute and also
        //     naturally excludes RETURN_HOME.
        //   - KB threshold ensures enough nodes are known.
        //   - Interval gate prevents replanning every tick (expensive).
        //   - Coverage gate ensures the UAV has explored enough of the field
        //     before locking onto a local cluster.
        if (uav.getDriveMode()          == UAV.DriveMode.PATROL
                && uav.getKnowledgeBase().size() >= algConfig.getPlanningThreshold()
                && tick % algConfig.getPlanningInterval() == 0
                && coverage.coverageFraction()   >= algConfig.getMinCoverageBeforePlan()) {

            // Select the nearest N untokenised nodes from the knowledge base.
            // Compact-disc selection: all waypoints lie inside a tight circle
            // centred on the UAV, eliminating the "chaining" artefact where
            // nearest-neighbour chains drift far from the drone.
            List<SensorNode> snapshot = uav.getKnowledgeBase().snapshot();
            List<SensorNode> targets  = new ArrayList<>(snapshot.size());
            for (SensorNode node : snapshot) {
                if (!uav.isTokenized(node.getId())) targets.add(node);
            }

            if (targets.size() < 3) return null;

            final double uavX = uav.getX();
            final double uavY = uav.getY();
            if (targets.size() > algConfig.getMaxRouteWaypoints()) {
                targets.sort((a, b) -> {
                    double da = (a.getX() - uavX) * (a.getX() - uavX)
                              + (a.getY() - uavY) * (a.getY() - uavY);
                    double db = (b.getX() - uavX) * (b.getX() - uavX)
                              + (b.getY() - uavY) * (b.getY() - uavY);
                    return Double.compare(da, db);
                });
                targets = new ArrayList<>(targets.subList(0, algConfig.getMaxRouteWaypoints()));
            }

            return uav.replan(targets);
        }

        return null;
    }

    @Override
    public String getLabel() {
        return "Autonomous GA\u2192ACO";
    }

    @Override
    public CoverageGrid getCoverageGrid() {
        return coverage;
    }

    /**
     * Predictive ballistic intercept for moving sensor nodes.
     *
     * <p>Solves the quadratic |p₀ + v·t − uav|² = (uavSpeed·t)² for the
     * smallest positive t, then returns the predicted node position at that
     * time.  If the node is stationary or no real positive solution exists,
     * falls back to the node's current position.
     */
    @Override
    public int[] interceptPoint(SensorNode target, int uavX, int uavY, int uavSpeed) {
        if (!target.hasVelocity()) {
            return new int[]{ (int) Math.round(target.getX()),
                              (int) Math.round(target.getY()) };
        }

        double vx = NodeMover.deltaX(target.getDirection()) * (double) target.getSpeed();
        double vy = NodeMover.deltaY(target.getDirection()) * (double) target.getSpeed();

        double dx = target.getX() - uavX;
        double dy = target.getY() - uavY;

        double a = vx*vx + vy*vy - (double) uavSpeed * uavSpeed;
        double b = 2.0 * (dx*vx + dy*vy);
        double c = dx*dx + dy*dy;

        double t = -1.0;
        if (Math.abs(a) < 1e-9) {
            if (Math.abs(b) > 1e-9) t = -c / b;
        } else {
            double disc = b*b - 4.0*a*c;
            if (disc >= 0.0) {
                double sq = Math.sqrt(disc);
                double t1 = (-b - sq) / (2.0 * a);
                double t2 = (-b + sq) / (2.0 * a);
                if      (t1 > 1e-9 && t2 > 1e-9) t = Math.min(t1, t2);
                else if (t1 > 1e-9)               t = t1;
                else if (t2 > 1e-9)               t = t2;
            }
        }

        if (t > 0) {
            return new int[]{ (int) Math.round(target.getX() + vx * t),
                              (int) Math.round(target.getY() + vy * t) };
        }
        return new int[]{ (int) Math.round(target.getX()),
                          (int) Math.round(target.getY()) };
    }
}
