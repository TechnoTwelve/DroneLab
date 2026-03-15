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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link SimulationConfig}.
 */
class SimulationConfigTest {

    // ── Direct constructor ────────────────────────────────────────────────────

    @Test
    void constructor_storesAllFields() {
        SimulationConfig c = new SimulationConfig(
                100, 200, 10, 5000,
                50, 1, 3,
                30, 10, 2,
                1000L);
        assertEquals(100,   c.getFieldWidth());
        assertEquals(200,   c.getFieldHeight());
        assertEquals(10,    c.getNodeCount());
        assertEquals(5000,  c.getNodeIdStart());
        assertEquals(50,    c.getNodeRange());
        assertEquals(1,     c.getNodeMinSpeed());
        assertEquals(3,     c.getNodeMaxSpeed());
        assertEquals(30,    c.getDroneRange());
        assertEquals(10,    c.getDronePlacement());
        assertEquals(2,     c.getDroneSpeed());
        assertEquals(1000L, c.getDuration());
    }

    // ── withXxx copy-and-override ─────────────────────────────────────────────

    @Test
    void withDroneRange_returnsNewInstanceWithUpdatedValue() {
        SimulationConfig original = base();
        SimulationConfig updated  = original.withDroneRange(999);

        assertEquals(999,              updated.getDroneRange());
        assertEquals(original.getFieldWidth(),    updated.getFieldWidth());
        assertEquals(original.getNodeCount(),     updated.getNodeCount());
        assertEquals(original.getDuration(),      updated.getDuration());
    }

    @Test
    void withDroneRange_doesNotMutateOriginal() {
        SimulationConfig original = base();
        int originalRange = original.getDroneRange();
        original.withDroneRange(999);
        assertEquals(originalRange, original.getDroneRange());
    }

    @Test
    void withNodeCount_updatesNodeCount() {
        SimulationConfig updated = base().withNodeCount(42);
        assertEquals(42, updated.getNodeCount());
    }

    @Test
    void withDuration_updatesDuration() {
        SimulationConfig updated = base().withDuration(9999L);
        assertEquals(9999L, updated.getDuration());
    }

    // ── Builder ───────────────────────────────────────────────────────────────

    @Test
    void builder_fieldSize_setsWidthAndHeight() {
        SimulationConfig c = SimulationConfig.builder()
                .fieldSize(800, 600)
                .build();
        assertEquals(800, c.getFieldWidth());
        assertEquals(600, c.getFieldHeight());
    }

    @Test
    void builder_fieldWidth_fieldHeight_individually() {
        SimulationConfig c = SimulationConfig.builder()
                .fieldWidth(1234)
                .fieldHeight(567)
                .build();
        assertEquals(1234, c.getFieldWidth());
        assertEquals(567,  c.getFieldHeight());
    }

    @Test
    void builder_nodeCount_overridesDefault() {
        SimulationConfig c = SimulationConfig.builder().nodeCount(77).build();
        assertEquals(77, c.getNodeCount());
    }

    @Test
    void builder_nodeSpeed_setsMinAndMax() {
        SimulationConfig c = SimulationConfig.builder()
                .nodeSpeed(2, 5)
                .build();
        assertEquals(2, c.getNodeMinSpeed());
        assertEquals(5, c.getNodeMaxSpeed());
    }

    @Test
    void builder_droneRange_overridesDefault() {
        SimulationConfig c = SimulationConfig.builder().droneRange(250).build();
        assertEquals(250, c.getDroneRange());
    }

    @Test
    void builder_dronePlacement_overridesDefault() {
        SimulationConfig c = SimulationConfig.builder().dronePlacement(50).build();
        assertEquals(50, c.getDronePlacement());
    }

    @Test
    void builder_droneSpeed_overridesDefault() {
        SimulationConfig c = SimulationConfig.builder().droneSpeed(3).build();
        assertEquals(3, c.getDroneSpeed());
    }

    @Test
    void builder_duration_overridesDefault() {
        SimulationConfig c = SimulationConfig.builder().duration(12345L).build();
        assertEquals(12345L, c.getDuration());
    }

    @Test
    void builder_nodeIdStart_overridesDefault() {
        SimulationConfig c = SimulationConfig.builder().nodeIdStart(9000).build();
        assertEquals(9000, c.getNodeIdStart());
    }

    @Test
    void builder_nodeRange_overridesDefault() {
        SimulationConfig c = SimulationConfig.builder().nodeRange(75).build();
        assertEquals(75, c.getNodeRange());
    }

    // ── toString ──────────────────────────────────────────────────────────────

    @Test
    void toString_containsKeyFields() {
        SimulationConfig c = base();
        String s = c.toString();
        assertTrue(s.contains("field="));
        assertTrue(s.contains("nodes="));
        assertTrue(s.contains("duration="));
    }

    // ── defaults ──────────────────────────────────────────────────────────────

    @Test
    void defaults_returnsNonNullConfig() {
        assertNotNull(SimulationConfig.defaults());
    }

    @Test
    void defaults_fieldWidthIs1500() {
        assertEquals(1500, SimulationConfig.defaults().getFieldWidth());
    }

    // ── helper ────────────────────────────────────────────────────────────────

    private static SimulationConfig base() {
        return new SimulationConfig(1500, 1500, 50, 5000, 100, 1, 1, 140, 72, 1, 5434L);
    }
}
