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

import algorithm.gaaco.Population;
import domain.SensorNode;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link Population}.
 */
class PopulationTest {

    // ── helpers ───────────────────────────────────────────────────────────────

    private static SensorNode node(int id, double x, double y) {
        return new SensorNode(id, x, y);
    }

    /** Route of a single node at the given distance from origin. */
    private static Route routeOfLength(double length) {
        return new Route(List.of(node(1, 0, 0), node(2, length, 0)));
    }

    // ── Constructor preconditions ─────────────────────────────────────────────

    @Test
    void nullList_throws() {
        assertThrows(IllegalArgumentException.class, () -> new Population(null));
    }

    @Test
    void emptyList_throws() {
        assertThrows(IllegalArgumentException.class, () -> new Population(List.of()));
    }

    // ── Basic access ─────────────────────────────────────────────────────────

    @Test
    void size_reflectsRouteCount() {
        Population p = new Population(List.of(routeOfLength(10), routeOfLength(20)));
        assertEquals(2, p.size());
    }

    @Test
    void get_returnsCorrectRoute() {
        Route r0 = routeOfLength(5);
        Route r1 = routeOfLength(10);
        Population p = new Population(List.of(r0, r1));
        assertSame(r0, p.get(0));
        assertSame(r1, p.get(1));
    }

    @Test
    void get_outOfBounds_throws() {
        Population p = new Population(List.of(routeOfLength(1)));
        assertThrows(IndexOutOfBoundsException.class, () -> p.get(5));
    }

    // ── Defensive copy ────────────────────────────────────────────────────────

    @Test
    void constructor_defensiveCopy_externalListMutationDoesNotAffectPopulation() {
        List<Route> mutable = new ArrayList<>(List.of(routeOfLength(5), routeOfLength(10)));
        Population p = new Population(mutable);
        mutable.clear();
        assertEquals(2, p.size());
    }

    // ── set ───────────────────────────────────────────────────────────────────

    @Test
    void set_replacesRouteAtIndex() {
        Population p = new Population(List.of(routeOfLength(5), routeOfLength(10)));
        Route replacement = routeOfLength(3);
        p.set(0, replacement);
        assertSame(replacement, p.get(0));
    }

    @Test
    void set_nullRoute_throws() {
        Population p = new Population(List.of(routeOfLength(5)));
        assertThrows(NullPointerException.class, () -> p.set(0, null));
    }

    // ── getRoutes unmodifiable view ───────────────────────────────────────────

    @Test
    void getRoutes_returnsUnmodifiableList() {
        Population p = new Population(List.of(routeOfLength(1)));
        assertThrows(UnsupportedOperationException.class,
                () -> p.getRoutes().add(routeOfLength(2)));
    }

    // ── sortByFitness ─────────────────────────────────────────────────────────

    @Test
    void sortByFitness_ordersDescendingByFitness() {
        // Shorter route = higher fitness
        Route short_  = routeOfLength(1);   // fitness = 1/1 = 1.0
        Route medium  = routeOfLength(5);   // fitness = 0.2
        Route long_   = routeOfLength(10);  // fitness = 0.1
        Population p = new Population(List.of(long_, short_, medium));

        p.sortByFitness();

        // After sort: index 0 = highest fitness (shortest route)
        assertEquals(short_.getTotalDistance(),  p.get(0).getTotalDistance(), 1e-9);
        assertEquals(medium.getTotalDistance(),  p.get(1).getTotalDistance(), 1e-9);
        assertEquals(long_.getTotalDistance(),   p.get(2).getTotalDistance(), 1e-9);
    }

    // ── getBest ───────────────────────────────────────────────────────────────

    @Test
    void getBest_returnsRouteWithHighestFitness() {
        Route best   = routeOfLength(1);
        Route worst  = routeOfLength(100);
        Population p = new Population(List.of(worst, best));

        Route found = p.getBest();
        assertEquals(best.getTotalDistance(), found.getTotalDistance(), 1e-9);
    }

    @Test
    void getBest_worksRegardlessOfSortOrder() {
        // Deliberately unsorted
        Population p = new Population(List.of(
                routeOfLength(50), routeOfLength(2), routeOfLength(30)));
        assertEquals(2.0, p.getBest().getTotalDistance(), 1e-9);
    }

    @Test
    void getBest_afterSortByFitness_isSameAsIndex0() {
        Population p = new Population(List.of(
                routeOfLength(10), routeOfLength(5), routeOfLength(20)));
        p.sortByFitness();
        assertSame(p.get(0), p.getBest());
    }

    // ── toString ──────────────────────────────────────────────────────────────

    @Test
    void toString_containsSizeAndBest() {
        Population p = new Population(List.of(routeOfLength(5)));
        String s = p.toString();
        assertTrue(s.contains("size=1"));
        assertTrue(s.contains("best="));
    }
}
