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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link CoverageGrid}.
 */
class CoverageGridTest {

    // ── Constructor precondition ──────────────────────────────────────────────

    @Test
    void sectorSizeZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> new CoverageGrid(100, 100, 0));
    }

    @Test
    void sectorSizeNegative_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> new CoverageGrid(100, 100, -1));
    }

    // ── totalSectors ─────────────────────────────────────────────────────────

    @Test
    void totalSectors_exactDivision() {
        CoverageGrid g = new CoverageGrid(100, 200, 50);
        // rows = 100/50 = 2, cols = 200/50 = 4 → 8 sectors
        assertEquals(8, g.totalSectors());
    }

    @Test
    void totalSectors_ceilingDivision() {
        CoverageGrid g = new CoverageGrid(110, 110, 100);
        // 110/100 = ceil(1.1) = 2, so 2×2 = 4 sectors
        assertEquals(4, g.totalSectors());
    }

    @Test
    void totalSectors_singleSector() {
        CoverageGrid g = new CoverageGrid(50, 50, 100);
        assertEquals(1, g.totalSectors());
    }

    // ── coverageFraction on fresh grid ───────────────────────────────────────

    @Test
    void freshGrid_coverageFractionIsZero() {
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        assertEquals(0.0, g.coverageFraction(), 1e-9);
    }

    @Test
    void freshGrid_allCoveredIsFalse() {
        assertFalse(new CoverageGrid(100, 100, 50).allCovered());
    }

    // ── markScanned ──────────────────────────────────────────────────────────

    @Test
    void markScanned_scanCenter_marksContainingSector() {
        // Field: 100×100, sector: 50 → 2×2 grid; sector centres at (25,25),(25,75),(75,25),(75,75)
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        g.markScanned(25, 25, 30); // scan circle around (25,25) with range 30
        assertTrue(g.isSectorVisited(25, 25));
    }

    @Test
    void markScanned_doesNotMarkDistantSectors() {
        // Sector 0,0 centre is at (25,25); sector 1,1 centre is at (75,75)
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        g.markScanned(25, 25, 30);
        assertFalse(g.isSectorVisited(75, 75));
    }

    @Test
    void markScanned_largeRange_marksMultipleSectors() {
        CoverageGrid g = new CoverageGrid(200, 200, 50);
        // Scan from centre (100,100) with range 120 — should reach all sector centres
        g.markScanned(100, 100, 120);
        assertTrue(g.coverageFraction() > 0.0);
    }

    @Test
    void markScanned_fullCoverage_allCoveredReturnsTrue() {
        // Single sector: field 50×50, sectorSize 100 → 1×1 grid, centre at (50,50)
        CoverageGrid g = new CoverageGrid(50, 50, 100);
        g.markScanned(50, 50, 60); // reaches centre (50,50)
        assertTrue(g.allCovered());
        assertEquals(1.0, g.coverageFraction(), 1e-9);
    }

    // ── isSectorVisited ───────────────────────────────────────────────────────

    @Test
    void isSectorVisited_negativeCoords_returnsFalse() {
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        assertFalse(g.isSectorVisited(-1.0, -1.0));
    }

    @Test
    void isSectorVisited_coordsBeyondField_clampsToLastSector() {
        // With 100×100 field and sector 50, valid rows/cols are 0..1.
        // Coordinate 999 is clamped to row/col 1 (numRows-1).
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        // Mark the last sector
        g.markScanned(75, 75, 5); // covers sector (1,1)
        assertTrue(g.isSectorVisited(999.0, 999.0)); // clamped to last sector
    }

    // ── nearestUnvisitedCenter ────────────────────────────────────────────────

    @Test
    void nearestUnvisitedCenter_freshGrid_returnsNonNull() {
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        assertNotNull(g.nearestUnvisitedCenter(0, 0));
    }

    @Test
    void nearestUnvisitedCenter_allVisited_returnsNull() {
        CoverageGrid g = new CoverageGrid(50, 50, 100);
        g.markScanned(50, 50, 60);  // covers the single sector
        assertNull(g.nearestUnvisitedCenter(0, 0));
    }

    @Test
    void nearestUnvisitedCenter_returnsTwoElementArray() {
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        int[] center = g.nearestUnvisitedCenter(0, 0);
        assertNotNull(center);
        assertEquals(2, center.length);
    }

    @Test
    void nearestUnvisitedCenter_closestToQueryPoint() {
        // 4×4 grid of 100-cell sectors in a 400×400 field
        CoverageGrid g = new CoverageGrid(400, 400, 100);
        // Query from (50,50) — nearest unvisited should be the first sector centre (50,50)
        int[] nearest = g.nearestUnvisitedCenter(50, 50);
        assertNotNull(nearest);
        assertEquals(50, nearest[0]);
        assertEquals(50, nearest[1]);
    }

    // ── coverageFraction ─────────────────────────────────────────────────────

    @Test
    void coverageFraction_halfSectorsVisited() {
        // 2-sector 1D grid: field 100×50, sector 50 → 2 rows × 1 col
        CoverageGrid g = new CoverageGrid(100, 50, 50);
        g.markScanned(25, 25, 5); // mark row 0 sector centre
        assertEquals(0.5, g.coverageFraction(), 1e-9);
    }

    // ── visitedCopy ───────────────────────────────────────────────────────────

    @Test
    void visitedCopy_isIndependent() {
        CoverageGrid g = new CoverageGrid(100, 100, 50);
        boolean[][] copy = g.visitedCopy();
        copy[0][0] = true;
        assertFalse(g.isSectorVisited(25, 25)); // underlying grid unchanged
    }

    @Test
    void visitedCopy_dimensionsMatchGetters() {
        CoverageGrid g = new CoverageGrid(200, 300, 100);
        boolean[][] copy = g.visitedCopy();
        assertEquals(g.getNumRows(), copy.length);
        assertEquals(g.getNumCols(), copy[0].length);
    }

    // ── getSectorSize ─────────────────────────────────────────────────────────

    @Test
    void getSectorSize_returnsConfiguredSize() {
        CoverageGrid g = new CoverageGrid(300, 300, 75);
        assertEquals(75, g.getSectorSize());
    }
}
