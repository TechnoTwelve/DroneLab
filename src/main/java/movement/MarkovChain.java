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

import java.util.Random;

/**
 * Markov-chain state machine for sensor-node mobility models.
 *
 * <p>Encapsulates a 4×4 row-stochastic transition matrix and advances the
 * chain one step by roulette-wheel selection.  Replacing {@code MarkOvChain}
 * from the legacy code.
 *
 * <h3>State semantics</h3>
 * <table border="1">
 *   <caption>Motion states</caption>
 *   <tr><th>Constant</th><th>Value</th><th>Meaning</th></tr>
 *   <tr><td>{@link #IDLE}</td><td>0</td><td>Node stays in place</td></tr>
 *   <tr><td>{@link #MOVE}</td><td>1</td><td>Node advances in current direction</td></tr>
 *   <tr><td>{@link #CHANGE_DIR}</td><td>2</td><td>Node picks a new random direction</td></tr>
 *   <tr><td>{@link #CONTINUE}</td><td>3</td><td>Node continues in current direction</td></tr>
 * </table>
 *
 * <h3>Factory methods</h3>
 * <ul>
 *   <li>{@link #levyWalk()} — the active transition matrix from the original
 *       {@code MarkOvChain.java}.</li>
 *   <li>{@link #brownianMotion()} — the commented-out matrix in the original,
 *       provided here for experimental comparison.</li>
 * </ul>
 */
public final class MarkovChain {

    // ── State constants ───────────────────────────────────────────────────────

    /** Motion state 0: node does not move this tick. */
    public static final int IDLE       = 0;

    /** Motion state 1: node advances one step in its current direction. */
    public static final int MOVE       = 1;

    /** Motion state 2: node selects a new random direction (0–7). */
    public static final int CHANGE_DIR = 2;

    /** Motion state 3: node continues in its current direction. */
    public static final int CONTINUE   = 3;

    // ── Transition matrix ─────────────────────────────────────────────────────

    private final double[][] transition;
    private final int        n;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Constructs a Markov chain with the given transition matrix.
     *
     * @param transition row-stochastic matrix; {@code transition[i][j]} is the
     *                   probability of transitioning from state {@code i} to
     *                   state {@code j}; must be n×n with rows summing to 1
     * @throws IllegalArgumentException if the matrix is null or empty
     */
    public MarkovChain(double[][] transition) {
        if (transition == null || transition.length == 0) {
            throw new IllegalArgumentException("transition matrix must not be null or empty");
        }
        this.n = transition.length;
        this.transition = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(transition[i], 0, this.transition[i], 0, n);
        }
    }

    // ── Factory methods ───────────────────────────────────────────────────────

    /**
     * Returns a Markov chain using the Lévy-walk transition matrix from the
     * original {@code MarkOvChain.java}.
     *
     * <pre>
     *   { { 0,    1.0,  0,    0    },   // from IDLE
     *     { 0,    0.95, 0.05, 0    },   // from MOVE: 95 % keep moving, 5 % change dir
     *     { 0,    0.01, 0,    0.99 },   // from CHANGE_DIR: 1 % move, 99 % continue
     *     { 0,    0,    1,    0    } }   // from CONTINUE: always change dir next
     * </pre>
     *
     * @return Lévy-walk chain
     */
    public static MarkovChain levyWalk() {
        return new MarkovChain(new double[][] {
            { 0,    1.0,  0,    0    },
            { 0,    0.95, 0.05, 0    },
            { 0,    0.01, 0,    0.99 },
            { 0,    0,    1,    0    }
        });
    }

    /**
     * Returns a Markov chain designed for wide-area exploration.
     *
     * <p>Compared with the Lévy walk, direction changes occur roughly six
     * times more often:
     *
     * <pre>
     *   { { 0,    1.0,  0,    0    },   // from IDLE:       always start moving
     *     { 0,    0.70, 0.30, 0    },   // from MOVE:       70 % keep going, 30 % turn
     *     { 0,    0.75, 0,    0.25 },   // from CHANGE_DIR: 75 % move, 25 % continue
     *     { 0,    0.50, 0.50, 0    } }  // from CONTINUE:   50 % move, 50 % turn again
     * </pre>
     *
     * <p>The resulting motion pattern:
     * <ul>
     *   <li>Average straight run ≈ 3–4 steps (vs. ≈20 for Lévy walk)</li>
     *   <li>After a direction change the node usually takes a few steps before
     *       turning again — no oscillation in place</li>
     *   <li>Nodes diffuse through the interior of the field rather than
     *       rushing to the boundary and clustering there</li>
     * </ul>
     *
     * @return exploratory-walk chain
     */
    public static MarkovChain exploratoryWalk() {
        return new MarkovChain(new double[][] {
            { 0,    1.0,  0,    0    },
            { 0,    0.70, 0.30, 0    },
            { 0,    0.75, 0,    0.25 },
            { 0,    0.50, 0.50, 0    }
        });
    }

    /**
     * Returns a Markov chain using the Brownian-motion transition matrix that
     * appears commented-out in the original {@code MarkOvChain.java}.
     *
     * <pre>
     *   { { 0, 1, 0, 0 },
     *     { 0, 0, 1, 0 },
     *     { 0, 0, 0, 1 },
     *     { 0, 0, 1, 0 } }
     * </pre>
     *
     * @return Brownian-motion chain
     */
    public static MarkovChain brownianMotion() {
        return new MarkovChain(new double[][] {
            { 0, 1, 0, 0 },
            { 0, 0, 1, 0 },
            { 0, 0, 0, 1 },
            { 0, 0, 1, 0 }
        });
    }

    // ── State transition ──────────────────────────────────────────────────────

    /**
     * Advances the chain one step from {@code currentState} and returns the
     * next state, selected by roulette-wheel sampling over the row of the
     * transition matrix that corresponds to {@code currentState}.
     *
     * <p>This is the direct equivalent of {@code MarkOvChain.state0()},
     * {@code state1()}, {@code state2()}, {@code state3()} merged into a
     * single parameterised method.
     *
     * @param currentState current motion state (must be in [0, n))
     * @param rng          shared random source
     * @return next motion state
     * @throws IllegalArgumentException if {@code currentState} is out of range
     */
    public int nextState(int currentState, Random rng) {
        if (currentState < 0 || currentState >= n) {
            throw new IllegalArgumentException(
                "currentState must be in [0, " + n + "), got: " + currentState);
        }
        double r   = rng.nextDouble();
        double sum = 0.0;
        for (int j = 0; j < n; j++) {
            sum += transition[currentState][j];
            if (r <= sum) return j;
        }
        return n - 1; // numerical safety fallback
    }
}
