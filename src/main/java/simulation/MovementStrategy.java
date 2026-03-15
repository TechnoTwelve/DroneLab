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
 * Strategy interface for sensor-node mobility models.
 *
 * <p>A {@code MovementStrategy} encapsulates the Markov-chain state-transition
 * logic that governs how sensor nodes change their motion state each simulation
 * tick.  The two concrete mobility models supported by the original thesis
 * simulator are:
 * <ul>
 *   <li><b>Lévy walk</b> — long-range ballistic motion interrupted by pauses,
 *       implemented by {@link LevyWalkStrategy}.  This is the active model in
 *       the original {@code MarkOvChain.java}.</li>
 *   <li><b>Brownian motion</b> — random isotropic diffusion (commented-out
 *       transition matrix in the original {@code MarkOvChain.java}).</li>
 * </ul>
 *
 * <h3>State semantics</h3>
 * The four integer states used by the simulator (and exposed as constants on
 * {@link MarkovChain}) are:
 * <ul>
 *   <li>{@code 0} — IDLE: node stays in place this tick.</li>
 *   <li>{@code 1} — MOVE: node advances one step in its current direction.</li>
 *   <li>{@code 2} — CHANGE_DIR: node picks a new random direction (0–7).</li>
 *   <li>{@code 3} — CONTINUE: node continues in its current direction.</li>
 * </ul>
 */
public interface MovementStrategy {

    /**
     * Returns the next motion state given the current state.
     *
     * <p>The implementation samples a random value and uses the transition
     * probability matrix to select the successor state.
     *
     * @param currentState current motion state (0–3)
     * @param rng          shared random source
     * @return next motion state (0–3)
     */
    int nextState(int currentState, Random rng);
}
