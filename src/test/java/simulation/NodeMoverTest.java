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

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link NodeMover}.
 *
 * <p>Key invariants verified:
 * <ul>
 *   <li>A successful move: both the grid cells and the agent's position are
 *       updated atomically; the original cell is cleared and the target cell
 *       carries the agent's ID.</li>
 *   <li>A blocked move (occupied target or out-of-bounds): the agent does not
 *       move; a random fallback direction is returned.</li>
 * </ul>
 *
 * <h3>IntelliJ IDEA setup</h3>
 * See {@code test/algorithm/RouteTest.java} for instructions on adding JUnit 5
 * and marking the test source root.
 */
class NodeMoverTest {

    // ── Successful moves ──────────────────────────────────────────────────────

    @Test
    void move_northFromMiddle_agentMovedOneRowUp() {
        int[][] field = new int[10][10];
        NodeAgent agent = new NodeAgent(5000, 5, 5, NodeMover.N, 3);
        field[5][5] = 5000;
        NodeMover mover = new NodeMover();

        int dir = mover.move(field, agent, 10, 10, new Random(0L));

        assertEquals(NodeMover.N, dir,         "Direction returned must be the intended direction N");
        assertEquals(4, agent.getX(),          "Agent x must decrement by 1 (row decreases going north)");
        assertEquals(5, agent.getY(),          "Agent y must be unchanged");
        assertEquals(0, field[5][5],           "Old cell must be cleared after move");
        assertEquals(5000, field[4][5],        "New cell must carry the agent's ID");
    }

    @Test
    void move_southEastDiagonal_agentMovedCorrectly() {
        int[][] field = new int[10][10];
        NodeAgent agent = new NodeAgent(5001, 3, 3, NodeMover.SE, 3);
        field[3][3] = 5001;
        NodeMover mover = new NodeMover();

        int dir = mover.move(field, agent, 10, 10, new Random(0L));

        assertEquals(NodeMover.SE, dir);
        assertEquals(4, agent.getX());  // row + 1
        assertEquals(4, agent.getY());  // col + 1
        assertEquals(0,    field[3][3]);
        assertEquals(5001, field[4][4]);
    }

    // ── Blocked moves — occupied target cell ──────────────────────────────────

    @Test
    void move_southCellOccupied_agentDoesNotMove_randomDirReturned() {
        int[][] field = new int[10][10];
        NodeAgent agent = new NodeAgent(5000, 5, 5, NodeMover.S, 3);
        field[5][5] = 5000;
        field[6][5] = 9999;  // south cell is occupied
        NodeMover mover = new NodeMover();

        int dir = mover.move(field, agent, 10, 10, new Random(42L));

        // Agent position unchanged
        assertEquals(5, agent.getX());
        assertEquals(5, agent.getY());
        // Grid unchanged
        assertEquals(5000, field[5][5]);
        assertEquals(9999, field[6][5]);
        // Returned direction is a random fallback, must be in [0, DIRECTION_COUNT)
        assertTrue(dir >= 0 && dir < NodeAgent.DIRECTION_COUNT,
                "Fallback direction must be in [0, 8) but got: " + dir);
    }

    // ── Blocked moves — out-of-bounds target ──────────────────────────────────

    @Test
    void move_northAtTopRow_agentDoesNotMove_randomDirReturned() {
        int[][] field = new int[10][10];
        NodeAgent agent = new NodeAgent(5000, 0, 5, NodeMover.N, 3);  // row 0, heading north
        field[0][5] = 5000;
        NodeMover mover = new NodeMover();

        int dir = mover.move(field, agent, 10, 10, new Random(7L));

        assertEquals(0, agent.getX(),          "Agent must stay at row 0");
        assertEquals(5, agent.getY());
        assertEquals(5000, field[0][5],        "Cell must remain occupied");
        assertTrue(dir >= 0 && dir < NodeAgent.DIRECTION_COUNT,
                "Fallback direction must be in [0, 8) but got: " + dir);
    }

    @Test
    void move_westAtLeftColumn_agentDoesNotMove_randomDirReturned() {
        int[][] field = new int[10][10];
        NodeAgent agent = new NodeAgent(5000, 5, 0, NodeMover.W, 3);  // col 0, heading west
        field[5][0] = 5000;
        NodeMover mover = new NodeMover();

        int dir = mover.move(field, agent, 10, 10, new Random(3L));

        assertEquals(5, agent.getX());
        assertEquals(0, agent.getY(),          "Agent must stay at col 0");
        assertEquals(5000, field[5][0]);
        assertTrue(dir >= 0 && dir < NodeAgent.DIRECTION_COUNT);
    }

    @Test
    void move_southAtBottomRow_agentDoesNotMove_randomDirReturned() {
        int[][] field = new int[10][10];
        // fieldHeight = 10, valid rows 0..9; row 9 heading S → row 10 (out of bounds)
        NodeAgent agent = new NodeAgent(5000, 9, 5, NodeMover.S, 3);
        field[9][5] = 5000;
        NodeMover mover = new NodeMover();

        int dir = mover.move(field, agent, 10, 10, new Random(1L));

        assertEquals(9, agent.getX());
        assertEquals(5000, field[9][5]);
        assertTrue(dir >= 0 && dir < NodeAgent.DIRECTION_COUNT);
    }
}
