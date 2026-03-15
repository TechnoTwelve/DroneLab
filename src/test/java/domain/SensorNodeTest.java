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
package domain;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link SensorNode}.
 */
class SensorNodeTest {

    // ── Constructors ──────────────────────────────────────────────────────────

    @Test
    void positionOnlyConstructor_setsFieldsCorrectly() {
        SensorNode n = new SensorNode(1, 10.0, 20.0);
        assertEquals(1,    n.getId());
        assertEquals(10.0, n.getX(), 1e-9);
        assertEquals(20.0, n.getY(), 1e-9);
        assertEquals(-1,   n.getDirection()); // sentinel: no velocity
        assertEquals(0,    n.getSpeed());
    }

    @Test
    void fullConstructor_setsAllFields() {
        SensorNode n = new SensorNode(42, 5.5, 7.7, 3, 2);
        assertEquals(42,  n.getId());
        assertEquals(5.5, n.getX(),  1e-9);
        assertEquals(7.7, n.getY(),  1e-9);
        assertEquals(3,   n.getDirection());
        assertEquals(2,   n.getSpeed());
    }

    // ── hasVelocity ───────────────────────────────────────────────────────────

    @Test
    void hasVelocity_falseWhenPositionOnlyConstructor() {
        assertFalse(new SensorNode(1, 0, 0).hasVelocity());
    }

    @Test
    void hasVelocity_falseWhenDirectionNegative() {
        assertFalse(new SensorNode(1, 0, 0, -1, 2).hasVelocity());
    }

    @Test
    void hasVelocity_falseWhenSpeedZero() {
        assertFalse(new SensorNode(1, 0, 0, 2, 0).hasVelocity());
    }

    @Test
    void hasVelocity_trueWhenDirectionGeZeroAndSpeedPositive() {
        assertTrue(new SensorNode(1, 0, 0, 0, 1).hasVelocity());
    }

    @Test
    void hasVelocity_trueAllDirections() {
        for (int dir = 0; dir <= 7; dir++) {
            assertTrue(new SensorNode(1, 0, 0, dir, 1).hasVelocity(),
                    "should have velocity for direction " + dir);
        }
    }

    // ── withPosition ──────────────────────────────────────────────────────────

    @Test
    void withPosition_returnsNewInstanceWithUpdatedCoordinates() {
        SensorNode original = new SensorNode(7, 1.0, 2.0, 5, 3);
        SensorNode updated  = original.withPosition(100.0, 200.0);

        // Same ID and velocity
        assertEquals(7,     updated.getId());
        assertEquals(5,     updated.getDirection());
        assertEquals(3,     updated.getSpeed());
        // New position
        assertEquals(100.0, updated.getX(), 1e-9);
        assertEquals(200.0, updated.getY(), 1e-9);
    }

    @Test
    void withPosition_doesNotMutateOriginal() {
        SensorNode original = new SensorNode(7, 1.0, 2.0);
        original.withPosition(999.0, 999.0);

        assertEquals(1.0, original.getX(), 1e-9);
        assertEquals(2.0, original.getY(), 1e-9);
    }

    @Test
    void withPosition_preservesVelocityInNewInstance() {
        SensorNode n = new SensorNode(1, 0, 0, 4, 2).withPosition(50.0, 75.0);
        assertTrue(n.hasVelocity());
        assertEquals(4, n.getDirection());
        assertEquals(2, n.getSpeed());
    }

    // ── calculateDistance ─────────────────────────────────────────────────────

    @Test
    void calculateDistance_samePoint_returnsZero() {
        SensorNode a = new SensorNode(1, 3.0, 4.0);
        assertEquals(0.0, a.calculateDistance(a), 1e-9);
    }

    @Test
    void calculateDistance_axisAlignedX() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        SensorNode b = new SensorNode(2, 5.0, 0.0);
        assertEquals(5.0, a.calculateDistance(b), 1e-9);
    }

    @Test
    void calculateDistance_axisAlignedY() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        SensorNode b = new SensorNode(2, 0.0, 12.0);
        assertEquals(12.0, a.calculateDistance(b), 1e-9);
    }

    @Test
    void calculateDistance_pythagoreanTriple3_4_5() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        SensorNode b = new SensorNode(2, 3.0, 4.0);
        assertEquals(5.0, a.calculateDistance(b), 1e-9);
    }

    @Test
    void calculateDistance_isSymmetric() {
        SensorNode a = new SensorNode(1, 1.0, 2.0);
        SensorNode b = new SensorNode(2, 4.0, 6.0);
        assertEquals(a.calculateDistance(b), b.calculateDistance(a), 1e-9);
    }

    @Test
    void calculateDistance_nullThrowsIllegalArgumentException() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        assertThrows(IllegalArgumentException.class, () -> a.calculateDistance(null));
    }

    // ── equals / hashCode ─────────────────────────────────────────────────────

    @Test
    void equals_sameId_equalRegardlessOfPosition() {
        SensorNode a = new SensorNode(5, 1.0, 1.0);
        SensorNode b = new SensorNode(5, 9.0, 9.0);
        assertEquals(a, b);
    }

    @Test
    void equals_differentId_notEqual() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        SensorNode b = new SensorNode(2, 0.0, 0.0);
        assertNotEquals(a, b);
    }

    @Test
    void equals_sameReference_isEqual() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        assertEquals(a, a);
    }

    @Test
    void equals_nullReturnsFalse() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        assertNotEquals(null, a);
    }

    @Test
    void equals_differentTypeReturnsFalse() {
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        assertNotEquals("not a SensorNode", a);
    }

    @Test
    void hashCode_consistentWithEquals() {
        SensorNode a = new SensorNode(5, 1.0, 1.0);
        SensorNode b = new SensorNode(5, 9.0, 9.0);
        assertEquals(a.hashCode(), b.hashCode());
    }

    @Test
    void hashCode_differentIdsDifferentHashCodes() {
        // Not guaranteed by contract, but holds for Integer.hashCode
        SensorNode a = new SensorNode(1, 0.0, 0.0);
        SensorNode b = new SensorNode(2, 0.0, 0.0);
        assertNotEquals(a.hashCode(), b.hashCode());
    }

    // ── toString ──────────────────────────────────────────────────────────────

    @Test
    void toString_noVelocity_containsIdAndCoordinates() {
        SensorNode n = new SensorNode(42, 3.0, 7.0);
        String s = n.toString();
        assertTrue(s.contains("42"));
        assertTrue(s.contains("3.0"));
        assertTrue(s.contains("7.0"));
        assertFalse(s.contains("dir="));
    }

    @Test
    void toString_withVelocity_containsDirectionAndSpeed() {
        SensorNode n = new SensorNode(1, 0.0, 0.0, 3, 2);
        String s = n.toString();
        assertTrue(s.contains("dir=3"));
        assertTrue(s.contains("spd=2"));
    }
}
