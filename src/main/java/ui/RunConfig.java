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
package ui;

import config.AppConfig;
import simulation.SimulationConfig;

/**
 * Immutable data class capturing all parameters chosen in {@link LaunchDialog}
 * for one simulation run.
 *
 * <p>Passed from the launch dialog through {@code Main} to the
 * {@link simulation.SimulationRunner} and {@link SimulationGui} so every
 * component shares the same run parameters.
 */
public final class RunConfig {

    /** Field / drone / node configuration for this run. */
    public final SimulationConfig simConfig;

    /**
     * When {@code true} all parallel passes are synchronised tick-by-tick via
     * a {@link java.util.concurrent.CyclicBarrier} so the GUI shows them
     * frame-for-frame in lock-step.  Useful for direct research comparison.
     */
    public final boolean syncMode;

    /**
     * Random seed for this run.
     * {@link AppConfig#NO_SEED} ({@code Long.MIN_VALUE}) means generate a
     * fresh seed automatically.
     */
    public final long seed;

    public RunConfig(SimulationConfig simConfig, boolean syncMode, long seed) {
        if (simConfig == null) throw new IllegalArgumentException("simConfig must not be null");
        this.simConfig = simConfig;
        this.syncMode  = syncMode;
        this.seed      = seed;
    }

    @Override
    public String toString() {
        return String.format("RunConfig{syncMode=%b, seed=%s, config=%s}",
                syncMode,
                seed == AppConfig.NO_SEED ? "auto" : Long.toString(seed),
                simConfig);
    }
}
