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
package algorithm;

import algorithm.gaaco.GaAcoConfig;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link GaAcoConfig}.
 */
class GaAcoConfigTest {

    // ── defaults factory ──────────────────────────────────────────────────────

    @Test
    void defaults_returnsNonNull() {
        assertNotNull(GaAcoConfig.defaults());
    }

    @Test
    void defaults_populationSizeIs20() {
        assertEquals(20, GaAcoConfig.defaults().getPopulationSize());
    }

    @Test
    void defaults_mutationRateIs0_09() {
        assertEquals(0.09, GaAcoConfig.defaults().getMutationRate(), 1e-9);
    }

    @Test
    void defaults_tournamentSizeIs2() {
        assertEquals(2, GaAcoConfig.defaults().getTournamentSize());
    }

    @Test
    void defaults_eliteCountIs1() {
        assertEquals(1, GaAcoConfig.defaults().getEliteCount());
    }

    @Test
    void defaults_generationCountIs15() {
        assertEquals(15, GaAcoConfig.defaults().getGenerationCount());
    }

    @Test
    void defaults_antCountIs10() {
        assertEquals(10, GaAcoConfig.defaults().getAntCount());
    }

    @Test
    void defaults_pheromoneInitIs0_2() {
        assertEquals(0.2, GaAcoConfig.defaults().getPheromoneInit(), 1e-9);
    }

    @Test
    void defaults_qIs0_08() {
        assertEquals(0.08, GaAcoConfig.defaults().getQ(), 1e-9);
    }

    @Test
    void defaults_rhoIs0_2() {
        assertEquals(0.2, GaAcoConfig.defaults().getRho(), 1e-9);
    }

    @Test
    void defaults_alphaIs0_1() {
        assertEquals(0.1, GaAcoConfig.defaults().getAlpha(), 1e-9);
    }

    @Test
    void defaults_betaIs11() {
        assertEquals(11.0, GaAcoConfig.defaults().getBeta(), 1e-9);
    }

    @Test
    void defaults_acoIterationCountIs5() {
        assertEquals(5, GaAcoConfig.defaults().getAcoIterationCount());
    }

    @Test
    void defaults_planningThresholdIs15() {
        assertEquals(15, GaAcoConfig.defaults().getPlanningThreshold());
    }

    @Test
    void defaults_planningIntervalIs25() {
        assertEquals(25, GaAcoConfig.defaults().getPlanningInterval());
    }

    @Test
    void defaults_minCoverageBeforePlanIs0_05() {
        assertEquals(0.05, GaAcoConfig.defaults().getMinCoverageBeforePlan(), 1e-9);
    }

    @Test
    void defaults_maxRouteWaypointsIs12() {
        assertEquals(12, GaAcoConfig.defaults().getMaxRouteWaypoints());
    }

    // ── withXxx copy-and-override ─────────────────────────────────────────────

    @Test
    void withPopulationSize_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withPopulationSize(50);
        assertEquals(50, c.getPopulationSize());
    }

    @Test
    void withPopulationSize_doesNotMutateOriginal() {
        GaAcoConfig d = GaAcoConfig.defaults();
        d.withPopulationSize(50);
        assertEquals(20, d.getPopulationSize());
    }

    @Test
    void withGenerationCount_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withGenerationCount(30);
        assertEquals(30, c.getGenerationCount());
    }

    @Test
    void withAntCount_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withAntCount(5);
        assertEquals(5, c.getAntCount());
    }

    @Test
    void withMutationRate_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withMutationRate(0.5);
        assertEquals(0.5, c.getMutationRate(), 1e-9);
    }

    @Test
    void withAcoIterationCount_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withAcoIterationCount(3);
        assertEquals(3, c.getAcoIterationCount());
    }

    @Test
    void withPlanningThreshold_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withPlanningThreshold(10);
        assertEquals(10, c.getPlanningThreshold());
    }

    @Test
    void withPlanningInterval_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withPlanningInterval(50);
        assertEquals(50, c.getPlanningInterval());
    }

    @Test
    void withMinCoverageBeforePlan_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withMinCoverageBeforePlan(0.1);
        assertEquals(0.1, c.getMinCoverageBeforePlan(), 1e-9);
    }

    @Test
    void withMaxRouteWaypoints_updatesValue() {
        GaAcoConfig c = GaAcoConfig.defaults().withMaxRouteWaypoints(20);
        assertEquals(20, c.getMaxRouteWaypoints());
    }

    // ── Validation ────────────────────────────────────────────────────────────

    @Test
    void populationSizeLessThanOne_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withPopulationSize(0));
    }

    @Test
    void mutationRateBelowZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withMutationRate(-0.1));
    }

    @Test
    void mutationRateAboveOne_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withMutationRate(1.1));
    }

    @Test
    void antCountZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withAntCount(0));
    }

    @Test
    void generationCountZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withGenerationCount(0));
    }

    @Test
    void acoIterationCountZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withAcoIterationCount(0));
    }

    @Test
    void planningThresholdZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withPlanningThreshold(0));
    }

    @Test
    void planningIntervalZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withPlanningInterval(0));
    }

    @Test
    void minCoverageBeforePlanNegative_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withMinCoverageBeforePlan(-0.1));
    }

    @Test
    void minCoverageBeforePlanAboveOne_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withMinCoverageBeforePlan(1.1));
    }

    @Test
    void maxRouteWaypointsZero_throws() {
        assertThrows(IllegalArgumentException.class,
                () -> GaAcoConfig.defaults().withMaxRouteWaypoints(0));
    }

    // Constructor-level validation
    @Test
    void eliteCount_equalToPopulationSize_throws() {
        // eliteCount must be < populationSize
        assertThrows(IllegalArgumentException.class, () ->
                new GaAcoConfig(5, 0.09, 2, 5, 15, 10, 0.2, 0.08, 0.2, 0.1, 11.0,
                        5, 15, 25, 0.05, 12));
    }

    @Test
    void tournamentSizeLessThanTwo_throws() {
        assertThrows(IllegalArgumentException.class, () ->
                new GaAcoConfig(20, 0.09, 1, 1, 15, 10, 0.2, 0.08, 0.2, 0.1, 11.0,
                        5, 15, 25, 0.05, 12));
    }

    @Test
    void pheromoneInitZero_throws() {
        assertThrows(IllegalArgumentException.class, () ->
                new GaAcoConfig(20, 0.09, 2, 1, 15, 10, 0.0, 0.08, 0.2, 0.1, 11.0,
                        5, 15, 25, 0.05, 12));
    }

    @Test
    void pheromoneInitOne_throws() {
        assertThrows(IllegalArgumentException.class, () ->
                new GaAcoConfig(20, 0.09, 2, 1, 15, 10, 1.0, 0.08, 0.2, 0.1, 11.0,
                        5, 15, 25, 0.05, 12));
    }

    @Test
    void betaBelowOne_throws() {
        assertThrows(IllegalArgumentException.class, () ->
                new GaAcoConfig(20, 0.09, 2, 1, 15, 10, 0.2, 0.08, 0.2, 0.1, 0.9,
                        5, 15, 25, 0.05, 12));
    }

    @Test
    void alphaNegative_throws() {
        assertThrows(IllegalArgumentException.class, () ->
                new GaAcoConfig(20, 0.09, 2, 1, 15, 10, 0.2, 0.08, 0.2, -0.1, 11.0,
                        5, 15, 25, 0.05, 12));
    }

    // ── toString ──────────────────────────────────────────────────────────────

    @Test
    void toString_containsKeyParameters() {
        String s = GaAcoConfig.defaults().toString();
        assertTrue(s.contains("pop=20"));
        assertTrue(s.contains("ants=10"));
        assertTrue(s.contains("gens=15"));
    }
}
