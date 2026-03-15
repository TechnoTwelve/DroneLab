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

import org.junit.jupiter.api.Test;
import simulation.SimulationConfig;
import simulation.UAVIntelligence;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link AppConfig}.
 */
class AppConfigTest {

    // ── load (built-in defaults when file missing) ────────────────────────────

    @Test
    void load_missingFile_usesBuiltInDefaults() {
        AppConfig cfg = AppConfig.load("nonexistent-file-xyz.properties");
        assertNotNull(cfg);
    }

    @Test
    void load_noArg_returnsNonNull() {
        // May load actual config.properties from working dir, or fall back to defaults — both OK
        assertNotNull(AppConfig.load());
    }

    // ── buildSimulationConfig ─────────────────────────────────────────────────

    @Test
    void buildSimulationConfig_defaultsMatchKnownValues() {
        AppConfig cfg = AppConfig.load("nonexistent.properties");
        SimulationConfig sc = cfg.buildSimulationConfig();
        assertEquals(1500, sc.getFieldWidth());
        assertEquals(1500, sc.getFieldHeight());
        assertEquals(50,   sc.getNodeCount());
        assertEquals(5000, sc.getNodeIdStart());
        assertEquals(100,  sc.getNodeRange());
        assertEquals(1,    sc.getNodeMinSpeed());
        assertEquals(1,    sc.getNodeMaxSpeed());
        assertEquals(140,  sc.getDroneRange());
        assertEquals(72,   sc.getDronePlacement());
        assertEquals(1,    sc.getDroneSpeed());
        assertEquals(5434L, sc.getDuration());
    }

    // ── buildIntelligences ────────────────────────────────────────────────────

    @Test
    void buildIntelligences_defaultKeys_returnsPatrolAndGaAco() {
        AppConfig cfg = AppConfig.load("nonexistent.properties");
        List<UAVIntelligence> list = cfg.buildIntelligences();
        assertEquals(2, list.size());
    }

    @Test
    void buildIntelligences_unknownKeyOnly_throwsIllegalStateException()
            throws IOException {
        Path tmp = createTempConfig("run.intelligences=unknown_key");
        AppConfig cfg = AppConfig.load(tmp.toString());
        assertThrows(IllegalStateException.class, cfg::buildIntelligences);
    }

    @Test
    void buildIntelligences_patrolOnly_returnsOneIntelligence()
            throws IOException {
        Path tmp = createTempConfig("run.intelligences=patrol");
        AppConfig cfg = AppConfig.load(tmp.toString());
        assertEquals(1, cfg.buildIntelligences().size());
    }

    @Test
    void buildIntelligences_labelsAreNotNull() {
        AppConfig cfg = AppConfig.load("nonexistent.properties");
        for (UAVIntelligence ui : cfg.buildIntelligences()) {
            assertNotNull(ui.getLabel());
        }
    }

    // ── isHeadless ────────────────────────────────────────────────────────────

    @Test
    void isHeadless_defaultIsFalse() {
        AppConfig cfg = AppConfig.load("nonexistent.properties");
        assertFalse(cfg.isHeadless());
    }

    @Test
    void isHeadless_trueWhenConfigured() throws IOException {
        Path tmp = createTempConfig("run.headless=true");
        assertTrue(AppConfig.load(tmp.toString()).isHeadless());
    }

    // ── getSeed ───────────────────────────────────────────────────────────────

    @Test
    void getSeed_defaultIsNoSeed() {
        AppConfig cfg = AppConfig.load("nonexistent.properties");
        assertEquals(AppConfig.NO_SEED, cfg.getSeed());
    }

    @Test
    void getSeed_fixedSeed_returnsParsedValue() throws IOException {
        Path tmp = createTempConfig("run.seed=12345");
        assertEquals(12345L, AppConfig.load(tmp.toString()).getSeed());
    }

    @Test
    void getSeed_invalidValue_returnsNoSeed() throws IOException {
        Path tmp = createTempConfig("run.seed=not_a_number");
        assertEquals(AppConfig.NO_SEED, AppConfig.load(tmp.toString()).getSeed());
    }

    // ── getIntelligenceKeys ───────────────────────────────────────────────────

    @Test
    void getIntelligenceKeys_defaultReturnsTwoKeys() {
        AppConfig cfg = AppConfig.load("nonexistent.properties");
        List<String> keys = cfg.getIntelligenceKeys();
        assertTrue(keys.contains("patrol"));
        assertTrue(keys.contains("gaaco"));
    }

    @Test
    void getIntelligenceKeys_customValue() throws IOException {
        Path tmp = createTempConfig("run.intelligences=patrol");
        List<String> keys = AppConfig.load(tmp.toString()).getIntelligenceKeys();
        assertEquals(List.of("patrol"), keys);
    }

    // ── registeredKeys ────────────────────────────────────────────────────────

    @Test
    void registeredKeys_containsPatrolAndGaAco() {
        List<String> keys = AppConfig.registeredKeys();
        assertTrue(keys.contains("patrol"));
        assertTrue(keys.contains("gaaco"));
    }

    // ── REGISTRY ─────────────────────────────────────────────────────────────

    @Test
    void registry_containsTwoEntries() {
        assertEquals(2, AppConfig.REGISTRY.size());
    }

    // ── NO_SEED constant ──────────────────────────────────────────────────────

    @Test
    void noSeed_isLongMinValue() {
        assertEquals(Long.MIN_VALUE, AppConfig.NO_SEED);
    }

    // ── load from file ────────────────────────────────────────────────────────

    @Test
    void load_fromFileWithOverrides_appliesOverride() throws IOException {
        Path tmp = createTempConfig("simulation.nodeCount=99");
        AppConfig cfg = AppConfig.load(tmp.toString());
        assertEquals(99, cfg.buildSimulationConfig().getNodeCount());
    }

    // ── helper ────────────────────────────────────────────────────────────────

    /**
     * Creates a temporary .properties file with the given content appended to
     * the default properties (so all required keys are present).
     */
    private static Path createTempConfig(String extraLine) throws IOException {
        Path tmp = Files.createTempFile("appconfig-test-", ".properties");
        tmp.toFile().deleteOnExit();
        try (PrintWriter pw = new PrintWriter(Files.newBufferedWriter(tmp))) {
            // Write required defaults
            pw.println("run.headless=false");
            pw.println("run.seed=");
            pw.println("run.intelligences=patrol,gaaco");
            pw.println("simulation.fieldWidth=1500");
            pw.println("simulation.fieldHeight=1500");
            pw.println("simulation.nodeCount=50");
            pw.println("simulation.nodeIdStart=5000");
            pw.println("simulation.nodeRange=100");
            pw.println("simulation.nodeMinSpeed=1");
            pw.println("simulation.nodeMaxSpeed=1");
            pw.println("simulation.droneRange=140");
            pw.println("simulation.dronePlacement=72");
            pw.println("simulation.droneSpeed=1");
            pw.println("simulation.duration=5434");
            // Override with the caller's extra line (comes last, so it wins)
            pw.println(extraLine);
        }
        return tmp;
    }
}
