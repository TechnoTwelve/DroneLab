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

import movement.MarkovChain;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link MarkovChain}.
 *
 * <p>The Lévy-walk transition matrix is:
 * <pre>
 *   from IDLE       : { 0,    1.0,  0,    0    }  → always MOVE
 *   from MOVE       : { 0,    0.95, 0.05, 0    }  → 95 % MOVE, 5 % CHANGE_DIR
 *   from CHANGE_DIR : { 0,    0.01, 0,    0.99 }  → 1 % MOVE, 99 % CONTINUE
 *   from CONTINUE   : { 0,    0,    1,    0    }  → always CHANGE_DIR
 * </pre>
 *
 * <h3>IntelliJ IDEA setup</h3>
 * See {@code test/algorithm/RouteTest.java} for instructions on adding JUnit 5
 * and marking the test source root.
 */
class MarkovChainTest {

    // ── Deterministic transitions ─────────────────────────────────────────────

    @Test
    void levyWalk_fromIdle_alwaysTransitionsToMove() {
        // Row 0: { 0, 1.0, 0, 0 } — probability 1.0 on MOVE; deterministic.
        MarkovChain chain = MarkovChain.levyWalk();
        Random rng = new Random(42L);

        for (int i = 0; i < 1_000; i++) {
            assertEquals(MarkovChain.MOVE,
                    chain.nextState(MarkovChain.IDLE, rng),
                    "IDLE must always transition to MOVE in Lévy-walk");
        }
    }

    @Test
    void levyWalk_fromContinue_alwaysTransitionsToChangeDir() {
        // Row 3: { 0, 0, 1, 0 } — probability 1.0 on CHANGE_DIR; deterministic.
        MarkovChain chain = MarkovChain.levyWalk();
        Random rng = new Random(42L);

        for (int i = 0; i < 1_000; i++) {
            assertEquals(MarkovChain.CHANGE_DIR,
                    chain.nextState(MarkovChain.CONTINUE, rng),
                    "CONTINUE must always transition to CHANGE_DIR in Lévy-walk");
        }
    }

    // ── Statistical transitions ───────────────────────────────────────────────

    @Test
    void levyWalk_fromMove_returnsMoveApproximately95Percent() {
        // Row 1: { 0, 0.95, 0.05, 0 } — 95 % MOVE, 5 % CHANGE_DIR.
        MarkovChain chain = MarkovChain.levyWalk();
        Random rng = new Random(0L);
        int N = 10_000;
        int moveCount = 0;

        for (int i = 0; i < N; i++) {
            if (chain.nextState(MarkovChain.MOVE, rng) == MarkovChain.MOVE) {
                moveCount++;
            }
        }

        double actualRate = (double) moveCount / N;
        assertEquals(0.95, actualRate, 0.02,
                "MOVE→MOVE rate should be ~95 % (±2 %)");
    }

    @Test
    void levyWalk_fromChangeDir_returnsContinueApproximately99Percent() {
        // Row 2: { 0, 0.01, 0, 0.99 } — 99 % CONTINUE.
        MarkovChain chain = MarkovChain.levyWalk();
        Random rng = new Random(1L);
        int N = 10_000;
        int continueCount = 0;

        for (int i = 0; i < N; i++) {
            if (chain.nextState(MarkovChain.CHANGE_DIR, rng) == MarkovChain.CONTINUE) {
                continueCount++;
            }
        }

        double actualRate = (double) continueCount / N;
        assertEquals(0.99, actualRate, 0.02,
                "CHANGE_DIR→CONTINUE rate should be ~99 % (±2 %)");
    }

    // ── Output range ──────────────────────────────────────────────────────────

    @Test
    void allStates_nextStateIsAlwaysInValidRange() {
        MarkovChain chain = MarkovChain.levyWalk();
        Random rng = new Random(12345L);
        int[] states = {
            MarkovChain.IDLE, MarkovChain.MOVE,
            MarkovChain.CHANGE_DIR, MarkovChain.CONTINUE
        };

        for (int state : states) {
            for (int i = 0; i < 200; i++) {
                int next = chain.nextState(state, rng);
                assertTrue(next >= 0 && next <= 3,
                        "Expected next state in [0, 3] but got " + next
                        + " from state " + state);
            }
        }
    }

    // ── Preconditions ─────────────────────────────────────────────────────────

    @Test
    void invalidState_belowZero_throwsIllegalArgumentException() {
        MarkovChain chain = MarkovChain.levyWalk();
        Random rng = new Random(0L);

        assertThrows(IllegalArgumentException.class,
                () -> chain.nextState(-1, rng));
    }

    @Test
    void invalidState_atOrAboveN_throwsIllegalArgumentException() {
        MarkovChain chain = MarkovChain.levyWalk(); // n = 4; valid range is [0, 4)
        Random rng = new Random(0L);

        assertThrows(IllegalArgumentException.class,
                () -> chain.nextState(4, rng));
    }

    @Test
    void nullMatrix_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () -> new MarkovChain(null));
    }

    @Test
    void emptyMatrix_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class,
                () -> new MarkovChain(new double[0][0]));
    }
}
