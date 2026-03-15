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
package scenarios;

import algorithm.PathPlanner;
import algorithm.gaaco.GaAcoConfig;
import algorithm.gaaco.GaAcoPlanner;
import config.AppConfig;
import simulation.SimulationConfig;
import simulation.SimulationScenario;
import simulation.UAVIntelligence;

import java.util.List;
import java.util.function.Supplier;

/**
 * The original MSc thesis scenario: a UAV collecting data tokens from a
 * wireless sensor network deployed across a field configured via
 * {@code config.properties}.
 *
 * <p>Runs two strategies side-by-side for comparison as specified in
 * {@code run.intelligences}:
 * <ol>
 *   <li>Predefined four-corner patrol (baseline)</li>
 *   <li>GA→ACO→LocalSearch adaptive path planner</li>
 * </ol>
 *
 * <p>Environment parameters (field size, node count, drone range, …) are
 * loaded from {@code config.properties} in the working directory.  Edit that
 * file to change settings without recompiling.  Falls back to sensible
 * defaults when the file is absent.
 *
 * <h3>Adding a new strategy</h3>
 * <ol>
 *   <li>Implement {@link UAVIntelligence}.</li>
 *   <li>Register it in {@link AppConfig} REGISTRY.</li>
 *   <li>Add its key to {@code run.intelligences} in {@code config.properties}.</li>
 * </ol>
 *
 * <h3>Usage</h3>
 * <pre>
 *   SimulationRunner.fromScenario(new WSNDataCollectionScenario())
 *       .withSeed(17L)
 *       .run();
 * </pre>
 */
public final class WSNDataCollectionScenario implements SimulationScenario {

    private final AppConfig        appConfig;
    private final SimulationConfig simConfig;

    /**
     * Loads configuration from {@code config.properties} and caches the
     * resulting {@link SimulationConfig}.
     */
    public WSNDataCollectionScenario() {
        this.appConfig = AppConfig.load();
        this.simConfig = appConfig.buildSimulationConfig();
    }

    @Override
    public String getName() {
        return "WSN Data Collection";
    }

    @Override
    public String getDescription() {
        return "UAVs collecting data from mobile sensor nodes in a disaster-zone WSN. "
                + "Compares predefined patrol vs GA\u2192ACO adaptive path planning.";
    }

    @Override
    public SimulationConfig getConfig() {
        return simConfig;
    }

    /**
     * Returns the intelligence strategies listed in {@code run.intelligences}.
     * Register new strategies in {@link AppConfig} REGISTRY and add their key
     * to {@code config.properties} — no changes needed here.
     */
    @Override
    public List<UAVIntelligence> getIntelligences() {
        return appConfig.buildIntelligences();
    }

    /**
     * Uses the GA→ACO planner with research-calibrated defaults.
     * Override {@link GaAcoConfig#defaults()} via the {@code withXxx}
     * copy-and-override methods if you need a custom configuration.
     */
    @Override
    public Supplier<PathPlanner> getPlannerFactory() {
        return () -> new GaAcoPlanner(GaAcoConfig.defaults());
    }
}
