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
package movement;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link LevyWalkStrategy}.
 */
class LevyWalkStrategyTest {

    // ── Construction ──────────────────────────────────────────────────────────

    @Test
    void defaultConstructor_doesNotThrow() {
        // Cast to Executable to resolve the ambiguity between the two assertDoesNotThrow overloads
        assertDoesNotThrow((org.junit.jupiter.api.function.Executable) LevyWalkStrategy::new);
    }

    @Test
    void chainConstructor_nullChain_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> new LevyWalkStrategy(null));
    }

    // ── nextState delegates to chain ─────────────────────────────────────────

    @Test
    void nextState_fromIdle_alwaysReturnsMove() {
        LevyWalkStrategy s = new LevyWalkStrategy();
        Random rng = new Random(0L);
        for (int i = 0; i < 500; i++) {
            assertEquals(MarkovChain.MOVE,
                    s.nextState(MarkovChain.IDLE, rng),
                    "IDLE → MOVE always in Lévy-walk");
        }
    }

    @Test
    void nextState_fromContinue_alwaysReturnsChangeDir() {
        LevyWalkStrategy s = new LevyWalkStrategy();
        Random rng = new Random(0L);
        for (int i = 0; i < 500; i++) {
            assertEquals(MarkovChain.CHANGE_DIR,
                    s.nextState(MarkovChain.CONTINUE, rng),
                    "CONTINUE → CHANGE_DIR always in Lévy-walk");
        }
    }

    @Test
    void nextState_fromMove_returnsMoveApproximately95Percent() {
        LevyWalkStrategy s = new LevyWalkStrategy();
        Random rng = new Random(42L);
        int N = 10_000, moveCount = 0;
        for (int i = 0; i < N; i++) {
            if (s.nextState(MarkovChain.MOVE, rng) == MarkovChain.MOVE) moveCount++;
        }
        assertEquals(0.95, (double) moveCount / N, 0.02);
    }

    @Test
    void nextState_returnValueIsValidState() {
        LevyWalkStrategy s = new LevyWalkStrategy();
        Random rng = new Random(1L);
        int[] states = { MarkovChain.IDLE, MarkovChain.MOVE, MarkovChain.CHANGE_DIR, MarkovChain.CONTINUE };
        for (int state : states) {
            for (int i = 0; i < 100; i++) {
                int next = s.nextState(state, rng);
                assertTrue(next >= 0 && next <= 3,
                        "nextState returned out-of-range value: " + next);
            }
        }
    }

    // ── Custom chain ─────────────────────────────────────────────────────────

    @Test
    void customChain_behaviourMatchesChainOutput() {
        // Use brownian motion chain — IDLE always → MOVE (same as levy in row 0)
        MarkovChain brownian = MarkovChain.brownianMotion();
        LevyWalkStrategy s   = new LevyWalkStrategy(brownian);
        Random rng = new Random(7L);

        // Both strategies agree on IDLE → MOVE (deterministic in both matrices)
        assertEquals(MarkovChain.MOVE, s.nextState(MarkovChain.IDLE, rng));
    }
}
