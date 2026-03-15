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

import java.util.Random;

/**
 * 8-directional grid movement for sensor nodes.
 *
 * <p>Replaces the {@code Movement} class from the legacy code.
 *
 * <h3>Performance improvement over the legacy implementation</h3>
 * The original {@code Movement.move_north()} (and all seven sibling methods)
 * scanned the entire {@code field[x][y]} grid — O(width × height) — to find
 * the node before moving it.  Because every {@link NodeAgent} now tracks its
 * own {@code (x, y)} grid position, this class reads the position directly
 * (O(1)) and only checks the target cell.
 *
 * <h3>Direction encoding</h3>
 * Directions are integers in [0, 7] with the following deltas on the grid
 * (field is indexed as {@code field[row][col]}, i.e. first index = x, second
 * index = y):
 *
 * <table border="1">
 *   <caption>Direction → delta mapping</caption>
 *   <tr><th>ID</th><th>Name</th><th>Δrow</th><th>Δcol</th></tr>
 *   <tr><td>0</td><td>N</td><td>-1</td><td>0</td></tr>
 *   <tr><td>1</td><td>S</td><td>+1</td><td>0</td></tr>
 *   <tr><td>2</td><td>E</td><td>0</td><td>+1</td></tr>
 *   <tr><td>3</td><td>W</td><td>0</td><td>-1</td></tr>
 *   <tr><td>4</td><td>NE</td><td>-1</td><td>+1</td></tr>
 *   <tr><td>5</td><td>NW</td><td>-1</td><td>-1</td></tr>
 *   <tr><td>6</td><td>SE</td><td>+1</td><td>+1</td></tr>
 *   <tr><td>7</td><td>SW</td><td>+1</td><td>-1</td></tr>
 * </table>
 */
public final class NodeMover {

    /** Direction constant: North (decreasing row). */
    public static final int N  = 0;
    /** Direction constant: South (increasing row). */
    public static final int S  = 1;
    /** Direction constant: East (increasing column). */
    public static final int E  = 2;
    /** Direction constant: West (decreasing column). */
    public static final int W  = 3;
    /** Direction constant: North-East. */
    public static final int NE = 4;
    /** Direction constant: North-West. */
    public static final int NW = 5;
    /** Direction constant: South-East. */
    public static final int SE = 6;
    /** Direction constant: South-West. */
    public static final int SW = 7;

    /** Row deltas indexed by direction. */
    private static final int[] DX = { -1, +1,  0,  0, -1, -1, +1, +1 };

    /** Column deltas indexed by direction. */
    private static final int[] DY = {  0,  0, +1, -1, +1, -1, +1, -1 };

    // ── Static direction helpers ──────────────────────────────────────────────

    /**
     * Returns the row delta (Δrow) for the given direction.
     *
     * <p>Exposed so that other simulation classes (e.g. {@code UAV}) can
     * convert a direction integer to a velocity vector without duplicating
     * the encoding table.
     *
     * @param direction direction integer in [0, 7]
     * @return row delta: −1 (northward), 0 (east/west), or +1 (southward)
     * @throws ArrayIndexOutOfBoundsException if direction is outside [0, 7]
     */
    public static int deltaX(int direction) { return DX[direction]; }

    /**
     * Returns the column delta (Δcol) for the given direction.
     *
     * @param direction direction integer in [0, 7]
     * @return column delta: −1 (westward), 0 (north/south), or +1 (eastward)
     * @throws ArrayIndexOutOfBoundsException if direction is outside [0, 7]
     */
    public static int deltaY(int direction) { return DY[direction]; }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Attempts to move {@code agent} one step in its current direction on the
     * given {@code field}.
     *
     * <p>If the target cell is within the field bounds <em>and</em> empty
     * ({@code field[targetRow][targetCol] == 0}), the agent is moved: the
     * field is updated and {@link NodeAgent#updatePosition} is called.  If
     * the target is out of bounds or already occupied, the move is aborted and
     * a new random direction is returned instead.
     *
     * @param field      the simulation grid ({@code field[row][col]} holds
     *                   the node ID occupying that cell, or 0 if empty)
     * @param agent      the sensor-node agent to move
     * @param fieldWidth number of rows in the field
     * @param fieldHeight number of columns in the field
     * @param rng        random source for fallback direction selection
     * @return the direction actually used (may differ from
     *         {@code agent.getDirection()} if the intended move failed)
     */
    public int move(int[][] field, NodeAgent agent,
                    int fieldWidth, int fieldHeight, Random rng) {
        int dir    = agent.getDirection();
        int currX  = agent.getX();
        int currY  = agent.getY();
        int targetX = currX + DX[dir];
        int targetY = currY + DY[dir];

        if (isValid(targetX, targetY, fieldWidth, fieldHeight)
                && field[targetX][targetY] == 0) {
            // Perform the move
            field[currX][currY]     = 0;
            field[targetX][targetY] = agent.getId();
            agent.updatePosition(targetX, targetY);
            return dir;
        } else {
            // Cannot move in this direction; select a new random direction
            return rng.nextInt(NodeAgent.DIRECTION_COUNT);
        }
    }

    // ── Helper ────────────────────────────────────────────────────────────────

    private static boolean isValid(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
}
