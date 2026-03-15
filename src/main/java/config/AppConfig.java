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
package config;

import algorithm.gaaco.GaAcoConfig;
import intelligence.GaAcoIntelligence;
import intelligence.PredefinedPatrolIntelligence;
import simulation.SimulationConfig;
import simulation.UAVIntelligence;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * File-backed configuration for the UAV/WSN simulator.
 *
 * <p>Reads {@code config.properties} from the working directory so users can
 * tweak simulation parameters and swap strategies without touching — or even
 * opening — any Java source file.
 *
 * <p>Falls back to built-in defaults when the file is absent, so the
 * application always starts cleanly.
 *
 * <h3>What belongs here</h3>
 * <ul>
 *   <li><b>Environment parameters</b> — field size, node count, drone range,
 *       etc. (all {@code simulation.*} keys in {@code config.properties}).</li>
 *   <li><b>Intelligence registry</b> — the mapping from short string key to
 *       a factory that creates a fresh {@link UAVIntelligence} instance.</li>
 * </ul>
 *
 * <h3>What does NOT belong here</h3>
 * Algorithm-specific parameters (GA population size, ACO evaporation rate,
 * planning thresholds, …) live in their own config class (e.g.
 * {@link GaAcoConfig}).  Each algorithm owns its configuration; this class
 * only knows about the simulation environment.
 *
 * <h3>Adding a new intelligence strategy — three steps</h3>
 * <ol>
 *   <li>Implement {@link UAVIntelligence}.</li>
 *   <li>Add one line to {@link #REGISTRY}:
 *       <pre>REGISTRY.put("mykey", sim -&gt; new MyIntelligence(sim));</pre></li>
 *   <li>Add {@code "mykey"} to {@code run.intelligences} in
 *       {@code config.properties}.</li>
 * </ol>
 * No other code needs to change.
 *
 * <h3>Typical usage</h3>
 * <pre>
 *   AppConfig cfg = AppConfig.load();
 *   SimulationRunner runner = new SimulationRunner(cfg.buildSimulationConfig());
 *   if (cfg.getSeed() != AppConfig.NO_SEED) runner.withSeed(cfg.getSeed());
 *   List&lt;UAVIntelligence&gt; intelligences = cfg.buildIntelligences();
 * </pre>
 */
public final class AppConfig {

    /** Sentinel value meaning "no fixed seed — generate a fresh one per run". */
    public static final long NO_SEED = Long.MIN_VALUE;

    /** Default config file name, looked up relative to the working directory. */
    public static final String CONFIG_FILE = "config.properties";

    // ── Intelligence registry ──────────────────────────────────────────────────
    //
    // Map key  : short string used in run.intelligences (e.g. "patrol")
    // Map value: factory that receives SimulationConfig and returns a fresh
    //            UAVIntelligence.  Each factory is responsible for creating
    //            its own algorithm config — this class stays algorithm-agnostic.
    //
    // *** ADD YOUR OWN INTELLIGENCE HERE — one line per strategy ***
    //
    //   Example:
    //     REGISTRY.put("mine", sim -> new MyIntelligence(sim));
    //
    //   Then add "mine" to run.intelligences in config.properties.
    //
    public static final Map<String, Function<SimulationConfig, UAVIntelligence>>
            REGISTRY = new LinkedHashMap<>();

    static {
        REGISTRY.put("patrol", sim -> new PredefinedPatrolIntelligence());
        REGISTRY.put("gaaco",  sim -> new GaAcoIntelligence(sim, GaAcoConfig.defaults()));
    }

    // ── Internal state ────────────────────────────────────────────────────────

    private final Properties props;

    private AppConfig(Properties props) {
        this.props = props;
    }

    // ── Static factories ──────────────────────────────────────────────────────

    /**
     * Loads configuration from {@code config.properties} in the working
     * directory.  Falls back to built-in defaults if the file is absent.
     *
     * @return a fully initialised {@code AppConfig} instance
     */
    public static AppConfig load() {
        return load(CONFIG_FILE);
    }

    /**
     * Loads configuration from the specified file path.
     * Useful for testing or for specifying an alternate config location.
     *
     * @param filePath path to the {@code .properties} file
     * @return a fully initialised {@code AppConfig} instance
     */
    public static AppConfig load(String filePath) {
        Properties props = defaultProperties();
        try (InputStream in = new FileInputStream(filePath)) {
            props.load(in);
            System.out.println("[AppConfig] Loaded: " + filePath);
        } catch (IOException e) {
            System.out.println("[AppConfig] '" + filePath
                    + "' not found — using built-in defaults.");
        }
        return new AppConfig(props);
    }

    // ── Config builders ───────────────────────────────────────────────────────

    /**
     * Builds a {@link SimulationConfig} from the {@code simulation.*} properties.
     *
     * <p>Users adjust these values in {@code config.properties}; no Java changes
     * needed to change field dimensions, node counts, drone parameters, etc.
     */
    public SimulationConfig buildSimulationConfig() {
        return new SimulationConfig(
                getInt ("simulation.fieldWidth"),
                getInt ("simulation.fieldHeight"),
                getInt ("simulation.nodeCount"),
                getInt ("simulation.nodeIdStart"),
                getInt ("simulation.nodeRange"),
                getInt ("simulation.nodeMinSpeed"),
                getInt ("simulation.nodeMaxSpeed"),
                getInt ("simulation.droneRange"),
                getInt ("simulation.dronePlacement"),
                getInt ("simulation.droneSpeed"),
                getLong("simulation.duration")
        );
    }

    /**
     * Builds the list of {@link UAVIntelligence} instances specified by
     * {@code run.intelligences}.
     *
     * <p>Each comma-separated key is looked up in {@link #REGISTRY}.
     * Unknown keys are logged and skipped rather than throwing, so a typo
     * doesn't crash the whole application.
     *
     * @return ordered list of fresh intelligence instances
     * @throws IllegalStateException if no valid intelligence keys are found
     */
    public List<UAVIntelligence> buildIntelligences() {
        return buildIntelligences(buildSimulationConfig());
    }

    /**
     * Builds intelligence instances using the provided {@link SimulationConfig}
     * instead of the one read from {@code config.properties}.
     *
     * <p>Use this overload when the user has configured a custom run via
     * {@link ui.LaunchDialog} and you want to apply those settings to the
     * intelligence strategies without modifying the properties file.
     *
     * @param simCfg the simulation configuration to pass to each intelligence
     * @return ordered list of fresh intelligence instances
     * @throws IllegalStateException if no valid intelligence keys are found
     */
    public List<UAVIntelligence> buildIntelligences(SimulationConfig simCfg) {
        String   raw  = props.getProperty("run.intelligences", "patrol,gaaco").trim();
        String[] keys = Arrays.stream(raw.split(","))
                              .map(String::trim)
                              .filter(s -> !s.isEmpty())
                              .toArray(String[]::new);

        List<UAVIntelligence> list = new ArrayList<>(keys.length);
        for (String key : keys) {
            Function<SimulationConfig, UAVIntelligence> factory =
                    REGISTRY.get(key.toLowerCase());
            if (factory == null) {
                System.err.println("[AppConfig] Unknown intelligence key: '" + key
                        + "'.  Registered keys: " + REGISTRY.keySet());
            } else {
                list.add(factory.apply(simCfg));
            }
        }

        if (list.isEmpty()) {
            throw new IllegalStateException(
                    "[AppConfig] No valid intelligence strategies found in "
                    + "run.intelligences='" + raw
                    + "'.  Registered keys: " + REGISTRY.keySet());
        }
        return list;
    }

    // ── Run-mode accessors ────────────────────────────────────────────────────

    /**
     * Returns {@code true} if {@code run.headless=true} in the config.
     * When headless, no GUI window is opened and output goes to stdout only.
     */
    public boolean isHeadless() {
        return Boolean.parseBoolean(props.getProperty("run.headless", "false").trim());
    }

    /**
     * Returns the fixed random seed, or {@link #NO_SEED} if none is set.
     * A blank or missing {@code run.seed} means "generate a fresh seed per run".
     */
    public long getSeed() {
        String val = props.getProperty("run.seed", "").trim();
        if (val.isEmpty()) return NO_SEED;
        try {
            return Long.parseLong(val);
        } catch (NumberFormatException e) {
            System.err.println("[AppConfig] Invalid run.seed value '" + val
                    + "' — ignoring, will use a random seed.");
            return NO_SEED;
        }
    }

    /**
     * Returns the configured intelligence keys from {@code run.intelligences},
     * filtered to only keys that are actually present in {@link #REGISTRY}.
     *
     * <p>Unknown keys are skipped (with a warning) so the panel count in the
     * GUI always matches the number of simulations that will actually run.
     */
    public List<String> getIntelligenceKeys() {
        return Arrays.stream(
                props.getProperty("run.intelligences", "patrol,gaaco").split(","))
                .map(String::trim)
                .filter(s -> !s.isEmpty())
                .filter(s -> {
                    boolean known = REGISTRY.containsKey(s.toLowerCase());
                    if (!known) System.err.println(
                            "[AppConfig] Ignoring unknown intelligence key: '" + s + "'");
                    return known;
                })
                .collect(Collectors.toList());
    }

    /**
     * Returns all keys currently registered in the intelligence registry.
     * These are the values you can put in {@code run.intelligences}.
     */
    public static List<String> registeredKeys() {
        return new ArrayList<>(REGISTRY.keySet());
    }

    // ── Property helpers ──────────────────────────────────────────────────────

    private int getInt(String key) {
        String val = require(key);
        try {
            return Integer.parseInt(val);
        } catch (NumberFormatException e) {
            throw new IllegalStateException(
                    "[AppConfig] Property '" + key + "' must be an integer, got: '" + val + "'");
        }
    }

    private long getLong(String key) {
        String val = require(key);
        try {
            return Long.parseLong(val);
        } catch (NumberFormatException e) {
            throw new IllegalStateException(
                    "[AppConfig] Property '" + key + "' must be a long integer, got: '" + val + "'");
        }
    }

    private String require(String key) {
        String val = props.getProperty(key);
        if (val == null)
            throw new IllegalStateException("[AppConfig] Missing required property: " + key);
        return val.trim();
    }

    // ── Built-in defaults ─────────────────────────────────────────────────────

    /**
     * Returns a {@link Properties} pre-populated with built-in defaults.
     * This is the single source of truth for all default simulation values
     * and the safe fallback when {@code config.properties} is absent.
     *
     * <p>{@link simulation.SimulationConfig#defaults()} derives from this method,
     * so these hardcoded values (which mirror {@code config.properties}) are the
     * only place that needs updating when defaults change.
     */
    private static Properties defaultProperties() {
        Properties p = new Properties();

        // Run settings
        p.setProperty("run.headless",      "false");
        p.setProperty("run.seed",          "");
        p.setProperty("run.intelligences", "patrol,gaaco");

        // Simulation environment (single source of truth for all default values).
        //
        // droneRange 140: parameter sweep over 20 seeds (sweet spot: +17.84% vs
        //   patrol, 16/20 wins). Larger values declare waypoints "reached" too
        //   early; 140 balances scan coverage with precise traversal.
        //
        // dronePlacement 72: patrol corners at (72,72),(72,1428),(1428,1428),(1428,72).
        //   Each leg = 1500 - 2*72 = 1356 cells; one_lap = 4*1356 = 5424 ticks.
        //
        // duration 5434 = one_lap + 10; the +10 ensures return-home fires exactly
        //   at waypoint W4 so all four sides of the rectangle are drawn cleanly.
        p.setProperty("simulation.fieldWidth",     "1500");
        p.setProperty("simulation.fieldHeight",    "1500");
        p.setProperty("simulation.nodeCount",      "50");
        p.setProperty("simulation.nodeIdStart",    "5000");
        p.setProperty("simulation.nodeRange",      "100");
        p.setProperty("simulation.nodeMinSpeed",   "1");
        p.setProperty("simulation.nodeMaxSpeed",   "1");
        p.setProperty("simulation.droneRange",     "140"); // sweep-optimised
        p.setProperty("simulation.dronePlacement", "72");
        p.setProperty("simulation.droneSpeed",     "1");
        p.setProperty("simulation.duration",       "5434"); // one_lap + 10

        return p;
    }
}
