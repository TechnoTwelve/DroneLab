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

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link EventScheduler} and its inner {@link EventScheduler.Event}.
 */
class EventSchedulerTest {

    private EventScheduler scheduler;

    @BeforeEach
    void setUp() {
        scheduler = new EventScheduler();
    }

    // ── Empty scheduler ───────────────────────────────────────────────────────

    @Test
    void newScheduler_isEmpty() {
        assertTrue(scheduler.isEmpty());
    }

    @Test
    void newScheduler_sizeIsZero() {
        assertEquals(0, scheduler.size());
    }

    @Test
    void newScheduler_peekReturnsNull() {
        assertNull(scheduler.peek());
    }

    @Test
    void newScheduler_pollReturnsNull() {
        assertNull(scheduler.poll());
    }

    // ── schedule ──────────────────────────────────────────────────────────────

    @Test
    void schedule_addsEvent_sizeIncreases() {
        scheduler.schedule(10L, 5000, 1);
        assertEquals(1, scheduler.size());
        assertFalse(scheduler.isEmpty());
    }

    @Test
    void schedule_multipleEvents_sizeReflectsCount() {
        scheduler.schedule(10L, 1, 0);
        scheduler.schedule(20L, 2, 1);
        scheduler.schedule(30L, 3, 2);
        assertEquals(3, scheduler.size());
    }

    // ── poll ordering ─────────────────────────────────────────────────────────

    @Test
    void poll_returnsEarliestEvent() {
        scheduler.schedule(30L, 3, 2);
        scheduler.schedule(10L, 1, 0);
        scheduler.schedule(20L, 2, 1);

        EventScheduler.Event e = scheduler.poll();
        assertNotNull(e);
        assertEquals(10L, e.time);
        assertEquals(1,   e.nodeId);
        assertEquals(0,   e.motionState);
    }

    @Test
    void poll_removesEventFromQueue() {
        scheduler.schedule(5L, 1, 0);
        scheduler.poll();
        assertEquals(0, scheduler.size());
        assertTrue(scheduler.isEmpty());
    }

    @Test
    void poll_drainInAscendingTimeOrder() {
        scheduler.schedule(50L, 5, 0);
        scheduler.schedule(10L, 1, 0);
        scheduler.schedule(30L, 3, 0);
        scheduler.schedule(20L, 2, 0);
        scheduler.schedule(40L, 4, 0);

        long prev = Long.MIN_VALUE;
        while (!scheduler.isEmpty()) {
            EventScheduler.Event e = scheduler.poll();
            assertTrue(e.time >= prev, "Events should arrive in non-decreasing time order");
            prev = e.time;
        }
    }

    // ── peek ──────────────────────────────────────────────────────────────────

    @Test
    void peek_returnsEarliestWithoutRemoving() {
        scheduler.schedule(15L, 7, 1);
        scheduler.schedule(5L,  8, 2);

        EventScheduler.Event e = scheduler.peek();
        assertNotNull(e);
        assertEquals(5L, e.time);
        assertEquals(2, scheduler.size()); // peek does not remove
    }

    @Test
    void peek_sameResultAsFirstPoll() {
        scheduler.schedule(99L, 9, 3);
        EventScheduler.Event peeked  = scheduler.peek();
        EventScheduler.Event polled  = scheduler.poll();
        assertEquals(peeked.time,        polled.time);
        assertEquals(peeked.nodeId,      polled.nodeId);
        assertEquals(peeked.motionState, polled.motionState);
    }

    // ── Event record ──────────────────────────────────────────────────────────

    @Test
    void event_storesFields() {
        EventScheduler.Event e = new EventScheduler.Event(42L, 7, 3);
        assertEquals(42L, e.time);
        assertEquals(7,   e.nodeId);
        assertEquals(3,   e.motionState);
    }

    @Test
    void event_compareToAscendingByTime() {
        EventScheduler.Event earlier = new EventScheduler.Event(1L, 0, 0);
        EventScheduler.Event later   = new EventScheduler.Event(2L, 0, 0);
        assertTrue(earlier.compareTo(later) < 0);
        assertTrue(later.compareTo(earlier) > 0);
    }

    @Test
    void event_compareToEqualTimes() {
        EventScheduler.Event a = new EventScheduler.Event(5L, 1, 0);
        EventScheduler.Event b = new EventScheduler.Event(5L, 2, 0);
        assertEquals(0, a.compareTo(b));
    }

    @Test
    void event_toString_containsFieldValues() {
        EventScheduler.Event e = new EventScheduler.Event(10L, 747, 1);
        String s = e.toString();
        assertTrue(s.contains("10"));
        assertTrue(s.contains("747"));
        assertTrue(s.contains("1"));
    }

    // ── Large-scale ordering ──────────────────────────────────────────────────

    @Test
    void schedule_manyEvents_allDrainedInOrder() {
        for (int i = 100; i >= 1; i--) {
            scheduler.schedule(i, i, 0);
        }
        long prev = Long.MIN_VALUE;
        int count = 0;
        while (!scheduler.isEmpty()) {
            EventScheduler.Event e = scheduler.poll();
            assertTrue(e.time >= prev);
            prev = e.time;
            count++;
        }
        assertEquals(100, count);
    }
}
