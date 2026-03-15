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

import algorithm.gaaco.LocalSearch;
import domain.SensorNode;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link LocalSearch}.
 *
 * <p>Both {@code twoOpt} and {@code orOpt} are pure functions — same input
 * always yields equivalent (or better) output, so we can test deterministically.
 */
class LocalSearchTest {

    // ── helpers ───────────────────────────────────────────────────────────────

    private static SensorNode n(int id, double x, double y) {
        return new SensorNode(id, x, y);
    }

    private static Route route(SensorNode... nodes) {
        return new Route(List.of(nodes));
    }

    // ── twoOpt — short routes (passthrough) ───────────────────────────────────

    @Test
    void twoOpt_oneNode_returnsEquivalentRoute() {
        Route r = route(n(1, 0, 0));
        Route result = LocalSearch.twoOpt(r);
        assertEquals(r.getTotalDistance(), result.getTotalDistance(), 1e-9);
    }

    @Test
    void twoOpt_twoNodes_returnsEquivalentRoute() {
        Route r = route(n(1, 0, 0), n(2, 1, 0));
        Route result = LocalSearch.twoOpt(r);
        assertEquals(r.getTotalDistance(), result.getTotalDistance(), 1e-9);
    }

    @Test
    void twoOpt_threeNodes_returnsEquivalentRoute() {
        Route r = route(n(1, 0, 0), n(2, 1, 0), n(3, 2, 0));
        Route result = LocalSearch.twoOpt(r);
        assertEquals(r.getTotalDistance(), result.getTotalDistance(), 1e-9);
    }

    // ── twoOpt — improvement ──────────────────────────────────────────────────

    @Test
    void twoOpt_crossingRoute_improvesOrMaintainsDistance() {
        // A crossing 4-node route: A(0,0) → C(10,10) → B(10,0) → D(0,10)
        // Optimal non-crossing: A → B → D → C or similar
        Route crossing = route(
                n(1,  0,  0),
                n(3, 10, 10),
                n(2, 10,  0),
                n(4,  0, 10));
        Route improved = LocalSearch.twoOpt(crossing);
        assertTrue(improved.getTotalDistance() <= crossing.getTotalDistance() + 1e-9);
    }

    @Test
    void twoOpt_alreadyOptimal_returnsSameDistance() {
        // Collinear route along x-axis: already optimal open path
        Route r = route(n(1, 0, 0), n(2, 1, 0), n(3, 2, 0), n(4, 3, 0));
        Route result = LocalSearch.twoOpt(r);
        assertEquals(r.getTotalDistance(), result.getTotalDistance(), 1e-9);
    }

    @Test
    void twoOpt_doesNotMutateInput() {
        Route input = route(n(1, 0, 0), n(2, 5, 5), n(3, 10, 0), n(4, 15, 5));
        double originalDist = input.getTotalDistance();
        LocalSearch.twoOpt(input);
        assertEquals(originalDist, input.getTotalDistance(), 1e-9);
    }

    // ── twoOpt — result size ─────────────────────────────────────────────────

    @Test
    void twoOpt_resultHasSameNumberOfNodes() {
        Route r = route(n(1, 0, 0), n(2, 3, 4), n(3, 6, 0), n(4, 9, 4));
        assertEquals(4, LocalSearch.twoOpt(r).size());
    }

    // ── orOpt — short routes (passthrough) ───────────────────────────────────

    @Test
    void orOpt_oneNode_returnsEquivalentRoute() {
        Route r = route(n(1, 0, 0));
        Route result = LocalSearch.orOpt(r);
        assertEquals(r.getTotalDistance(), result.getTotalDistance(), 1e-9);
    }

    @Test
    void orOpt_twoNodes_returnsEquivalentRoute() {
        Route r = route(n(1, 0, 0), n(2, 10, 0));
        Route result = LocalSearch.orOpt(r);
        assertEquals(r.getTotalDistance(), result.getTotalDistance(), 1e-9);
    }

    // ── orOpt — improvement ───────────────────────────────────────────────────

    @Test
    void orOpt_relocatableMidpoint_improvesOrMaintainsDistance() {
        // 4 nodes in a suboptimal order: A(0,0) → B(5,5) → C(10,0) → D(15,5)
        Route r = route(n(1, 0, 0), n(2, 5, 5), n(3, 10, 0), n(4, 15, 5));
        Route result = LocalSearch.orOpt(r);
        assertTrue(result.getTotalDistance() <= r.getTotalDistance() + 1e-9);
    }

    @Test
    void orOpt_alreadyOptimal_returnsSameDistance() {
        // Collinear route: already optimal
        Route r = route(n(1, 0, 0), n(2, 1, 0), n(3, 2, 0), n(4, 3, 0));
        Route result = LocalSearch.orOpt(r);
        assertEquals(r.getTotalDistance(), result.getTotalDistance(), 1e-9);
    }

    @Test
    void orOpt_doesNotMutateInput() {
        Route input = route(n(1, 0, 0), n(2, 5, 5), n(3, 10, 0));
        double originalDist = input.getTotalDistance();
        LocalSearch.orOpt(input);
        assertEquals(originalDist, input.getTotalDistance(), 1e-9);
    }

    // ── orOpt — result size ───────────────────────────────────────────────────

    @Test
    void orOpt_resultHasSameNumberOfNodes() {
        Route r = route(n(1, 0, 0), n(2, 3, 4), n(3, 6, 0), n(4, 9, 4));
        assertEquals(4, LocalSearch.orOpt(r).size());
    }

    // ── Chained application ───────────────────────────────────────────────────

    @Test
    void twoOptThenOrOpt_neverWorsens() {
        Route r = route(
                n(1,  0,  0),
                n(2, 10,  5),
                n(3,  5, 10),
                n(4, 15,  0),
                n(5,  0, 15));
        double original = r.getTotalDistance();
        Route after = LocalSearch.orOpt(LocalSearch.twoOpt(r));
        assertTrue(after.getTotalDistance() <= original + 1e-9);
    }

    // ── known optimal ─────────────────────────────────────────────────────────

    @Test
    void twoOpt_reverseCollinear_producesOptimalDistance() {
        // Reverse of a collinear route should be corrected back to monotone order
        // because reversing any segment of a non-crossed path doesn't improve distance.
        // Distance either stays equal or improves.
        Route r = route(n(1, 3, 0), n(2, 2, 0), n(3, 1, 0), n(4, 0, 0));
        Route result = LocalSearch.twoOpt(r);
        assertEquals(3.0, result.getTotalDistance(), 1e-9);
    }
}
