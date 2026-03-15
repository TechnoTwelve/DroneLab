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
package algorithm;

import algorithm.gaaco.PheromoneMatrix;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link PheromoneMatrix}.
 */
class PheromoneMatrixTest {

    // ── Constructor preconditions ─────────────────────────────────────────────

    @Test
    void nodeCountZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> new PheromoneMatrix(0, 0.1));
    }

    @Test
    void initialValueZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> new PheromoneMatrix(3, 0.0));
    }

    @Test
    void initialValueNegative_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> new PheromoneMatrix(3, -0.1));
    }

    // ── Initialisation ────────────────────────────────────────────────────────

    @Test
    void construction_offDiagonalSetToInitialValue() {
        double tau = 0.2;
        PheromoneMatrix m = new PheromoneMatrix(3, tau);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (i != j) {
                    assertEquals(tau, m.get(i, j), 1e-9,
                            "off-diagonal [" + i + "][" + j + "] should be τ₀");
                }
            }
        }
    }

    @Test
    void construction_diagonalIsZero() {
        PheromoneMatrix m = new PheromoneMatrix(4, 0.5);
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, m.get(i, i), 1e-9,
                    "diagonal [" + i + "][" + i + "] should be 0");
        }
    }

    @Test
    void size_returnsNodeCount() {
        assertEquals(5, new PheromoneMatrix(5, 0.1).size());
    }

    // ── deposit ───────────────────────────────────────────────────────────────

    @Test
    void deposit_incrementsEdgePheromone() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.1);
        double before = m.get(0, 1);
        m.deposit(0, 1, 0.05);
        assertEquals(before + 0.05, m.get(0, 1), 1e-9);
    }

    @Test
    void deposit_doesNotAffectOtherEdges() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.2);
        m.deposit(0, 1, 0.5);
        assertEquals(0.2, m.get(0, 2), 1e-9); // other edge unchanged
        assertEquals(0.2, m.get(1, 0), 1e-9); // reverse edge unchanged
    }

    @Test
    void deposit_multipleDeposits_accumulate() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.1);
        m.deposit(1, 2, 0.1);
        m.deposit(1, 2, 0.2);
        assertEquals(0.1 + 0.1 + 0.2, m.get(1, 2), 1e-9);
    }

    // ── evaporateAll ──────────────────────────────────────────────────────────

    @Test
    void evaporateAll_reducesOffDiagonalByRho() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.4);
        m.evaporateAll(0.5); // retain = 0.5
        assertEquals(0.4 * 0.5, m.get(0, 1), 1e-9);
        assertEquals(0.4 * 0.5, m.get(1, 0), 1e-9);
    }

    @Test
    void evaporateAll_diagonalRemainsZero() {
        PheromoneMatrix m = new PheromoneMatrix(4, 0.3);
        m.evaporateAll(0.2);
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, m.get(i, i), 1e-9);
        }
    }

    @Test
    void evaporateAll_rhoOne_zeroesPheromone() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.9);
        m.evaporateAll(1.0);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                assertEquals(0.0, m.get(i, j), 1e-9);
    }

    @Test
    void evaporateAll_rhoZero_leavesValuesUnchanged() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.5);
        m.evaporateAll(0.0);
        assertEquals(0.5, m.get(0, 1), 1e-9);
    }

    // ── reset ─────────────────────────────────────────────────────────────────

    @Test
    void reset_restoresInitialValue() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.3);
        m.deposit(0, 1, 1.0);
        m.evaporateAll(0.5);
        m.reset(0.3);
        assertEquals(0.3, m.get(0, 1), 1e-9);
        assertEquals(0.3, m.get(1, 0), 1e-9);
    }

    @Test
    void reset_diagonalRemainsZero() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.2);
        m.reset(0.7);
        for (int i = 0; i < 3; i++) {
            assertEquals(0.0, m.get(i, i), 1e-9);
        }
    }

    // ── snapshot ──────────────────────────────────────────────────────────────

    @Test
    void snapshot_isDefensiveCopy_writeDoesNotAffectMatrix() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.2);
        double[][] copy = m.snapshot();
        copy[0][1] = 999.0;
        assertEquals(0.2, m.get(0, 1), 1e-9);
    }

    @Test
    void snapshot_matrixChangesDoNotAffectCopy() {
        PheromoneMatrix m = new PheromoneMatrix(3, 0.2);
        double[][] copy = m.snapshot();
        m.deposit(0, 1, 5.0);
        assertEquals(0.2, copy[0][1], 1e-9);
    }

    @Test
    void snapshot_dimensionMatchesNodeCount() {
        PheromoneMatrix m = new PheromoneMatrix(4, 0.1);
        double[][] s = m.snapshot();
        assertEquals(4, s.length);
        assertEquals(4, s[0].length);
    }
}
