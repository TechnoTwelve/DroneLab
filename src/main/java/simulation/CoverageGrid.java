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

/**
 * Coarse coverage tracker that divides the simulation field into sectors and
 * records which sectors have been scanned by the UAV.
 *
 * <p>Used exclusively in the <em>autonomous</em> UAV pass of the A/B
 * comparison to implement <b>frontier-based exploration</b>: instead of
 * following a fixed four-corner perimeter patrol, the autonomous UAV always
 * moves toward the nearest sector whose centre it has not yet scanned,
 * achieving systematic full-field coverage rather than concentrating on the
 * boundary edges.
 *
 * <h3>Sector model</h3>
 * The field is partitioned into a uniform grid of square sectors of size
 * {@code sectorSize × sectorSize} cells.  A sector is marked <em>visited</em>
 * when the UAV's scan circle (radius {@code droneRange}) contains the sector's
 * centre — a conservative but clean definition that avoids partial-coverage
 * ambiguity.
 *
 * <h3>Frontier selection</h3>
 * {@link #nearestUnvisitedCenter(int, int)} returns the centre of the closest
 * unvisited sector, measured as Euclidean distance from the UAV's current
 * position.  Greedy-nearest selection minimises unnecessary travel and
 * naturally produces an outward-expanding coverage path from the UAV's
 * starting location.
 *
 * <h3>Why this beats fixed-corner patrol</h3>
 * The predefined four-corner patrol concentrates the UAV's scan budget along
 * the field perimeter, leaving the interior largely unexplored.  Frontier
 * exploration adaptively targets unscanned regions regardless of where they
 * are, ensuring that the autonomous UAV builds a fuller knowledge base before
 * each GA→ACO planning invocation.
 */
public final class CoverageGrid {

    private final int       sectorSize;
    private final int       numRows;
    private final int       numCols;
    private final boolean[][] visited;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Creates a coverage grid for the given field dimensions and sector size.
     *
     * @param fieldWidth  field width  in cells (≥ 1)
     * @param fieldHeight field height in cells (≥ 1)
     * @param sectorSize  width and height of each sector in cells (≥ 1);
     *                    setting this equal to {@code droneRange} gives one
     *                    sector per scan-circle diameter — a good default
     */
    public CoverageGrid(int fieldWidth, int fieldHeight, int sectorSize) {
        if (sectorSize < 1) throw new IllegalArgumentException("sectorSize must be ≥ 1");
        this.sectorSize = sectorSize;
        this.numRows    = (fieldWidth  + sectorSize - 1) / sectorSize;
        this.numCols    = (fieldHeight + sectorSize - 1) / sectorSize;
        this.visited    = new boolean[numRows][numCols];
    }

    // ── Marking ───────────────────────────────────────────────────────────────

    /**
     * Marks every sector whose centre lies within {@code droneRange} of
     * position {@code (x, y)} as visited.
     *
     * <p>Only sectors for which the Euclidean distance from {@code (x, y)} to
     * the sector centre is ≤ {@code droneRange} are marked, matching the UAV's
     * actual scan footprint.
     *
     * @param x          UAV x (row) position
     * @param y          UAV y (column) position
     * @param droneRange UAV scan radius in cells
     */
    public void markScanned(int x, int y, int droneRange) {
        long rangeSq = (long) droneRange * droneRange;

        int rMin = Math.max(0,          (x - droneRange) / sectorSize);
        int rMax = Math.min(numRows - 1, (x + droneRange) / sectorSize);
        int cMin = Math.max(0,          (y - droneRange) / sectorSize);
        int cMax = Math.min(numCols - 1, (y + droneRange) / sectorSize);

        for (int r = rMin; r <= rMax; r++) {
            for (int c = cMin; c <= cMax; c++) {
                if (!visited[r][c]) {
                    int  cx = r * sectorSize + sectorSize / 2;
                    int  cy = c * sectorSize + sectorSize / 2;
                    long dx = cx - x;
                    long dy = cy - y;
                    if (dx * dx + dy * dy <= rangeSq) {
                        visited[r][c] = true;
                    }
                }
            }
        }
    }

    // ── Frontier query ────────────────────────────────────────────────────────

    /**
     * Returns the {@code (x, y)} centre of the nearest unvisited sector from
     * position {@code (fromX, fromY)}, or {@code null} if every sector has
     * already been visited.
     *
     * <p>Greedy-nearest selection minimises unnecessary travel and naturally
     * produces a spiral-like outward coverage path from the UAV's current
     * location.
     *
     * @param fromX UAV x (row) position
     * @param fromY UAV y (column) position
     * @return {@code int[]{centerX, centerY}} of the nearest unvisited sector,
     *         or {@code null} if all sectors are covered
     */
    public int[] nearestUnvisitedCenter(int fromX, int fromY) {
        int  bestR    = -1;
        int  bestC    = -1;
        long bestDist = Long.MAX_VALUE;

        for (int r = 0; r < numRows; r++) {
            for (int c = 0; c < numCols; c++) {
                if (!visited[r][c]) {
                    int  cx   = r * sectorSize + sectorSize / 2;
                    int  cy   = c * sectorSize + sectorSize / 2;
                    long dx   = cx - fromX;
                    long dy   = cy - fromY;
                    long dist = dx * dx + dy * dy;
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestR    = r;
                        bestC    = c;
                    }
                }
            }
        }

        if (bestR == -1) return null;
        return new int[]{ bestR * sectorSize + sectorSize / 2,
                          bestC * sectorSize + sectorSize / 2 };
    }

    // ── Status ────────────────────────────────────────────────────────────────

    /**
     * Returns the fraction of sectors that have been visited (0.0 = none,
     * 1.0 = all).
     *
     * @return coverage fraction in [0.0, 1.0]
     */
    public double coverageFraction() {
        int covered = 0;
        for (boolean[] row : visited)
            for (boolean v : row)
                if (v) covered++;
        return (double) covered / (numRows * numCols);
    }

    /**
     * Returns {@code true} if the sector containing grid cell {@code (x, y)}
     * has already been marked as visited.
     *
     * <p>Used by {@link UAV#replan(CoverageGrid)} to exclude waypoint candidates
     * whose sector has been fully scanned — routing to those nodes wastes
     * travel time and causes the planner to revisit already-explored regions
     * (local minima).
     *
     * @param x x (row) coordinate of the node's last-known position
     * @param y y (column) coordinate of the node's last-known position
     * @return {@code true} if the containing sector is already visited
     */
    public boolean isSectorVisited(double x, double y) {
        int r = Math.min((int)(x / sectorSize), numRows - 1);
        int c = Math.min((int)(y / sectorSize), numCols - 1);
        if (r < 0 || c < 0) return false;
        return visited[r][c];
    }

    /** @return {@code true} if every sector has been visited */
    public boolean allCovered() {
        for (boolean[] row : visited)
            for (boolean v : row)
                if (!v) return false;
        return true;
    }

    /** @return total number of sectors in the grid */
    public int totalSectors() {
        return numRows * numCols;
    }

    // ── Snapshot export (for GUI rendering) ───────────────────────────────────

    /** @return sector width/height in cells */
    public int getSectorSize() { return sectorSize; }

    /** @return number of sector rows */
    public int getNumRows() { return numRows; }

    /** @return number of sector columns */
    public int getNumCols() { return numCols; }

    /**
     * Returns a deep copy of the visited-sector array for GUI rendering.
     * Callers may freely modify the returned array without affecting this grid.
     *
     * @return deep copy of the visited array
     */
    public boolean[][] visitedCopy() {
        boolean[][] copy = new boolean[numRows][numCols];
        for (int r = 0; r < numRows; r++) {
            System.arraycopy(visited[r], 0, copy[r], 0, numCols);
        }
        return copy;
    }
}
