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

import algorithm.PathPlanner;
import algorithm.Route;
import domain.SensorNode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * The UAV (drone) entity in the discrete-event simulation.
 */
public final class UAV {

    public static final int UAV_ID = 747;

    public enum DriveMode {
        PATROL,
        EXECUTE,
        RETURN_HOME
    }

    // ── Patrol waypoints (immutable) ──────────────────────────────────────────
    private final int w1x, w1y;
    private final int w2x, w2y;
    private final int w3x, w3y;
    private final int w4x, w4y;

    // ── Mutable position ──────────────────────────────────────────────────────
    private int x;
    private int y;
    private final int speed;

    // ── Route-following state ─────────────────────────────────────────────────
    private DriveMode        mode           = DriveMode.PATROL;
    private List<SensorNode> routeWaypoints = Collections.emptyList();
    private int              waypointIdx    = 0;
    private final int        arrivalRadius;

    // ── Frontier exploration target ───────────────────────────────────────────
    private boolean hasExplorationTarget = false;
    private int     explorationTargetX;
    private int     explorationTargetY;

    // ── Token ledger ──────────────────────────────────────────────────────────
    private final Set<Integer> tokenNodeIds  = new HashSet<>();
    private int                patrolTokens  = 0;
    private int                executeTokens = 0;

    // ── Algorithm layer ───────────────────────────────────────────────────────
    private final KnowledgeBase knowledgeBase;
    private final PathPlanner   planner;

    // ── Constructor ───────────────────────────────────────────────────────────

    public UAV(SimulationConfig config, PathPlanner planner) {
        if (planner == null) throw new IllegalArgumentException("planner must not be null");

        int dp = config.getDronePlacement();
        int fw = config.getFieldWidth();
        int fh = config.getFieldHeight();

        this.x             = dp;
        this.y             = dp;
        this.speed         = config.getDroneSpeed();
        this.arrivalRadius = config.getDroneRange();

        this.w1x = dp;      this.w1y = dp;
        this.w2x = dp;      this.w2y = fh - dp;
        this.w3x = fw - dp; this.w3y = fh - dp;
        this.w4x = fw - dp; this.w4y = dp;

        this.knowledgeBase = new KnowledgeBase();
        this.planner       = planner;
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    public int getX() { return x; }
    public int getY() { return y; }
    public KnowledgeBase getKnowledgeBase() { return knowledgeBase; }
    public DriveMode getDriveMode() { return mode; }
    public int getPatrolTokens()  { return patrolTokens; }
    public int getExecuteTokens() { return executeTokens; }
    public int getTotalTokens()   { return patrolTokens + executeTokens; }
    public boolean isTokenized(int nodeId) { return tokenNodeIds.contains(nodeId); }

    public List<SensorNode> getRouteWaypoints() {
        return Collections.unmodifiableList(routeWaypoints);
    }

    public int[] getExplorationTarget() {
        if (!hasExplorationTarget) return null;
        return new int[]{ explorationTargetX, explorationTargetY };
    }

    // ── Frontier exploration target control ───────────────────────────────────

    public void setExplorationTarget(int targetX, int targetY) {
        this.explorationTargetX = targetX;
        this.explorationTargetY = targetY;
        this.hasExplorationTarget = true;
    }

    public void clearExplorationTarget() {
        this.hasExplorationTarget = false;
    }

    public void returnHome() {
        this.mode = DriveMode.RETURN_HOME;
    }

    // ── Sensing ───────────────────────────────────────────────────────────────

    public void scan(Environment env, int droneRange) {
        List<NodeAgent> detected = env.findAgentsInRange(x, y, droneRange);

        for (NodeAgent agent : detected) {
            if (tokenNodeIds.add(agent.getId())) {
                if (mode == DriveMode.EXECUTE) {
                    executeTokens++;
                } else {
                    patrolTokens++;
                }
            }
        }

        knowledgeBase.observeWithTransitiveCache(detected, env.agentById());
    }

    // ── Movement ──────────────────────────────────────────────────────────────

    public void moveStep(int[][] field, int fieldWidth, int fieldHeight,
                         UAVIntelligence intelligence) {
        if (mode == DriveMode.EXECUTE) {
            doExecuteStep(field, fieldWidth, fieldHeight, intelligence);
        } else if (mode == DriveMode.RETURN_HOME) {
            doReturnHomeStep(field, fieldWidth, fieldHeight);
        } else {
            doPatrolStep(field, fieldWidth, fieldHeight);
        }
    }

    // ── Path planning ─────────────────────────────────────────────────────────

    /**
     * Triggers replanning through all currently known, untokenised nodes.
     *
     * <p>A convenience entry point for intelligences that want to delegate
     * target selection entirely to the knowledge base.  For intelligences
     * that need to control which nodes to include (distance-filtered,
     * coverage-filtered, etc.), use {@link #replan(List)} directly.
     *
     * @return the new {@link Route} if at least 3 untokenised nodes are
     *         known, {@code null} otherwise
     */
    public Route replan() {
        List<SensorNode> snapshot = knowledgeBase.snapshot();
        List<SensorNode> targets  = new ArrayList<>(snapshot.size());
        for (SensorNode node : snapshot) {
            if (!tokenNodeIds.contains(node.getId())) targets.add(node);
        }
        return replan(targets);
    }

    /**
     * Plans an optimised route through the given target nodes, orients it
     * toward the UAV's current position, and activates it immediately.
     *
     * <p>The caller is responsible for selecting and pre-filtering the
     * {@code targets} list.  Common patterns:
     * <ul>
     *   <li>Filter to untokenised nodes only.</li>
     *   <li>Sort by distance from the UAV and take the nearest N.</li>
     *   <li>Exclude nodes in already-visited coverage sectors.</li>
     * </ul>
     *
     * @param targets pre-selected waypoints; requires &ge; 3 entries
     * @return the new {@link Route}, or {@code null} if fewer than 3 targets
     *         are provided
     */
    public Route replan(List<SensorNode> targets) {
        if (targets == null || targets.size() < 3) return null;

        Route route = planner.planRoute(targets);
        route = orientRouteTowardUAV(route);
        activateRoute(route);
        return route;
    }

    /**
     * Returns the route reversed if the last waypoint is closer to the UAV
     * than the first, ensuring minimal initial travel overhead.
     */
    private Route orientRouteTowardUAV(Route route) {
        List<SensorNode> nodes = route.getNodes();
        if (nodes.size() < 2) return route;

        SensorNode first = nodes.get(0);
        SensorNode last  = nodes.get(nodes.size() - 1);

        double distToFirst = Math.hypot(first.getX() - x, first.getY() - y);
        double distToLast  = Math.hypot(last.getX()  - x, last.getY()  - y);

        if (distToLast < distToFirst) {
            List<SensorNode> reversed = new ArrayList<>(nodes);
            Collections.reverse(reversed);
            return new Route(reversed);
        }
        return route;
    }

    public void activateRoute(Route route) {
        this.routeWaypoints = new ArrayList<>(route.getNodes());
        this.waypointIdx    = 0;
        this.mode           = DriveMode.EXECUTE;
    }

    // ── Private movement helpers ──────────────────────────────────────────────

    private void doExecuteStep(int[][] field, int fw, int fh, UAVIntelligence intelligence) {
        long radSq = (long) arrivalRadius * arrivalRadius;

        while (waypointIdx < routeWaypoints.size()) {
            SensorNode wp = routeWaypoints.get(waypointIdx);
            long dx = (long)(x - (int) Math.round(wp.getX()));
            long dy = (long)(y - (int) Math.round(wp.getY()));
            if (dx * dx + dy * dy <= radSq) {
                waypointIdx++;
            } else {
                break;
            }
        }

        if (waypointIdx >= routeWaypoints.size()) {
            mode = DriveMode.PATROL;
            doPatrolStep(field, fw, fh);
            return;
        }

        SensorNode target = routeWaypoints.get(waypointIdx);
        int[] intercept = intelligence.interceptPoint(target, x, y, speed);
        int tx = Math.max(0, Math.min(fw - 1, intercept[0]));
        int ty = Math.max(0, Math.min(fh - 1, intercept[1]));
        moveToward(tx, ty, field, fw, fh);
    }

    private void doPatrolStep(int[][] field, int fieldWidth, int fieldHeight) {
        if (hasExplorationTarget) {
            long dx = (long)(x - explorationTargetX);
            long dy = (long)(y - explorationTargetY);
            if (dx * dx + dy * dy <= (long) arrivalRadius * arrivalRadius) {
                hasExplorationTarget = false;
            }
        }

        int targetX, targetY;
        if (hasExplorationTarget) {
            targetX = explorationTargetX;
            targetY = explorationTargetY;
        } else {
            targetX = computeTargetX();
            targetY = computeTargetY();
        }
        moveToward(targetX, targetY, field, fieldWidth, fieldHeight);
    }

    private void doReturnHomeStep(int[][] field, int fieldWidth, int fieldHeight) {
        moveToward(w1x, w1y, field, fieldWidth, fieldHeight);
    }

    private void moveToward(int targetX, int targetY,
                            int[][] field, int fieldWidth, int fieldHeight) {
        if (x == targetX && y == targetY) return;

        int nx = x;
        int ny = y;

        if      (y < targetY && x == targetX) { ny += speed;              }
        else if (y > targetY && x == targetX) { ny -= speed;              }
        else if (x < targetX && y == targetY) { nx += speed;              }
        else if (x > targetX && y == targetY) { nx -= speed;              }
        else if (y < targetY && x < targetX)  { nx += speed; ny += speed; }
        else if (y < targetY && x > targetX)  { nx -= speed; ny += speed; }
        else if (y > targetY && x < targetX)  { nx += speed; ny -= speed; }
        else if (y > targetY && x > targetX)  { nx -= speed; ny -= speed; }

        if (nx >= 0 && nx < fieldWidth && ny >= 0 && ny < fieldHeight
                && field[nx][ny] == 0) {
            field[x][y]   = 0;
            field[nx][ny] = UAV_ID;
            x = nx;
            y = ny;
        }
    }

    private int computeTargetX() {
        if (x == w2x && y  < w2y) return w2x;
        if (x  < w3x && y == w3y) return w3x;
        if (x == w4x && y  > w4y) return w4x;
        if (x  > w1x && y == w1y) return w1x;
        return w1x;
    }

    private int computeTargetY() {
        if (x == w2x && y  < w2y) return w2y;
        if (x  < w3x && y == w3y) return w3y;
        if (x == w4x && y  > w4y) return w4y;
        if (x  > w1x && y == w1y) return w1y;
        return w1y;
    }

    @Override
    public String toString() {
        return String.format(
                "UAV{pos=(%d,%d), mode=%s, knownNodes=%d, tokens=%d(patrol)/%d(execute)}",
                x, y, mode, knowledgeBase.size(), patrolTokens, executeTokens);
    }
}