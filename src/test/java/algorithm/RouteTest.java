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

import domain.SensorNode;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link Route}.
 *
 * <h3>IntelliJ IDEA setup</h3>
 * Add the JUnit 5 (junit-jupiter) jar to the module's test-scope dependencies:
 * File → Project Structure → Modules → Dependencies → "+" → JARs/Directories.
 * Alternatively, download via Maven central: org.junit.jupiter:junit-jupiter:5.10.x.
 *
 * Mark the {@code test/} directory as a "Test Sources Root":
 * File → Project Structure → Modules → Sources → right-click test/ → "Test Sources".
 */
class RouteTest {

    // ── Distance and fitness ──────────────────────────────────────────────────

    @Test
    void singleNode_distanceIsZero_fitnessIsMaxValue() {
        Route route = new Route(Collections.singletonList(new SensorNode(1, 0, 0)));

        assertEquals(0.0, route.getTotalDistance(), 1e-10);
        assertEquals(Double.MAX_VALUE, route.getFitness());
        assertEquals(1, route.size());
    }

    @Test
    void twoNodes_345Triangle_distanceIsFive() {
        // (0,0) → (3,4): Euclidean = √(9 + 16) = 5.0  (Pythagorean triple)
        List<SensorNode> nodes = Arrays.asList(
                new SensorNode(1, 0.0, 0.0),
                new SensorNode(2, 3.0, 4.0)
        );
        Route route = new Route(nodes);

        assertEquals(5.0, route.getTotalDistance(), 1e-10);
        assertEquals(1.0 / 5.0, route.getFitness(), 1e-10);
    }

    @Test
    void threeNodes_openPath_distanceIsSumOfTwoSegments() {
        // (0,0) → (0,3): distance = 3
        // (0,3) → (4,3): distance = 4
        // Open path total = 7 (no return leg)
        List<SensorNode> nodes = Arrays.asList(
                new SensorNode(1, 0.0, 0.0),
                new SensorNode(2, 0.0, 3.0),
                new SensorNode(3, 4.0, 3.0)
        );
        Route route = new Route(nodes);

        assertEquals(7.0, route.getTotalDistance(), 1e-10);
        assertEquals(1.0 / 7.0, route.getFitness(), 1e-10);
    }

    @Test
    void fitness_isReciprocalOfDistance() {
        List<SensorNode> nodes = Arrays.asList(
                new SensorNode(1, 0.0, 0.0),
                new SensorNode(2, 0.0, 10.0)   // distance = 10
        );
        Route route = new Route(nodes);

        // fitness = 1 / 10 = 0.1
        assertEquals(0.1, route.getFitness(), 1e-10);
    }

    @Test
    void size_returnsNodeCount() {
        List<SensorNode> nodes = Arrays.asList(
                new SensorNode(1, 0, 0),
                new SensorNode(2, 1, 0),
                new SensorNode(3, 2, 0),
                new SensorNode(4, 3, 0)
        );
        assertEquals(4, new Route(nodes).size());
    }

    // ── Immutability ──────────────────────────────────────────────────────────

    @Test
    void defensiveCopy_mutatingInputListDoesNotAffectRoute() {
        SensorNode a = new SensorNode(1, 0, 0);
        SensorNode b = new SensorNode(2, 1, 0);
        List<SensorNode> mutable = new ArrayList<>(Arrays.asList(a, b));

        Route route = new Route(mutable);

        // Add an extra node to the original list after construction
        mutable.add(new SensorNode(3, 99, 99));

        // The route must still contain only the original two nodes
        assertEquals(2, route.size());
    }

    @Test
    void getNodes_returnsUnmodifiableList() {
        Route route = new Route(Arrays.asList(
                new SensorNode(1, 0, 0),
                new SensorNode(2, 5, 0)
        ));

        assertThrows(UnsupportedOperationException.class,
                () -> route.getNodes().add(new SensorNode(99, 0, 0)));
    }

    // ── Preconditions ─────────────────────────────────────────────────────────

    @Test
    void nullInput_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () -> new Route(null));
    }

    @Test
    void emptyInput_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () -> new Route(new ArrayList<>()));
    }
}
