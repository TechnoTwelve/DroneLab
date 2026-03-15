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

import simulation.MovementStrategy;

import java.util.Random;

/**
 * Lévy-walk movement strategy — the default mobility model for sensor nodes.
 *
 * <p>Delegates to a {@link MarkovChain} initialised with the Lévy-walk
 * transition matrix (see {@link MarkovChain#levyWalk()}).  This is the
 * active model from the original {@code MarkOvChain.java}; the Brownian-
 * motion alternative can be obtained by constructing an instance with
 * {@code new LevyWalkStrategy(MarkovChain.brownianMotion())}.
 *
 * <h3>Lévy-walk characteristics</h3>
 * The transition matrix is designed to produce long ballistic flights
 * (many consecutive MOVE steps) interspersed with occasional direction
 * changes and rare idle pauses — the signature of a Lévy-walk process.
 */
public final class LevyWalkStrategy implements MovementStrategy {

    private final MarkovChain chain;

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * Constructs a Lévy-walk strategy using the default Lévy-walk transition
     * matrix from {@link MarkovChain#levyWalk()}.
     */
    public LevyWalkStrategy() {
        this(MarkovChain.levyWalk());
    }

    /**
     * Constructs a strategy backed by the given {@link MarkovChain}, allowing
     * the caller to supply an alternative mobility model (e.g.
     * {@link MarkovChain#brownianMotion()}).
     *
     * @param chain Markov chain to use for state transitions; must not be null
     */
    public LevyWalkStrategy(MarkovChain chain) {
        if (chain == null) throw new IllegalArgumentException("chain must not be null");
        this.chain = chain;
    }

    // ── MovementStrategy ──────────────────────────────────────────────────────

    /**
     * Returns the next motion state for a sensor node currently in
     * {@code currentState}.
     *
     * @param currentState current motion state (0–3); see
     *                     {@link MarkovChain} for state semantics
     * @param rng          shared random source
     * @return next motion state
     */
    @Override
    public int nextState(int currentState, Random rng) {
        return chain.nextState(currentState, rng);
    }
}
