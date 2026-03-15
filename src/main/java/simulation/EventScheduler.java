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

import java.util.PriorityQueue;

/**
 * Priority-queue–based event scheduler for the discrete-event simulation.
 *
 * <p>Replaces the hand-rolled sorted linked list in the legacy {@code Events}
 * class.  Events are dequeued in ascending time order; ties are broken by
 * insertion order (natural FIFO for equal timestamps, since
 * {@link java.util.PriorityQueue} is a min-heap).
 *
 * <h3>Event fields</h3>
 * Each {@link Event} carries:
 * <ul>
 *   <li>{@code time} — the simulation tick at which this event fires.</li>
 *   <li>{@code nodeId} — the entity to act on (sensor-node ID or {@code 747}
 *       for the UAV/drone).</li>
 *   <li>{@code motionState} — the Markov-chain state at the time of the event
 *       (0 = IDLE, 1 = MOVE, 2 = CHANGE_DIR, 3 = CONTINUE).</li>
 * </ul>
 */
public final class EventScheduler {

    // ── Inner event record ────────────────────────────────────────────────────

    /**
     * An immutable simulation event.
     */
    public static final class Event implements Comparable<Event> {

        /** Simulation tick at which this event fires. */
        public final long time;

        /** ID of the entity this event targets (node ID or 747 for UAV). */
        public final int nodeId;

        /** Markov-chain motion state when the event was scheduled. */
        public final int motionState;

        /**
         * Constructs an event.
         *
         * @param time        simulation tick (≥ 1)
         * @param nodeId      entity ID
         * @param motionState Markov state (0–3)
         */
        public Event(long time, int nodeId, int motionState) {
            this.time        = time;
            this.nodeId      = nodeId;
            this.motionState = motionState;
        }

        /** Events are ordered by ascending time. */
        @Override
        public int compareTo(Event other) {
            return Long.compare(this.time, other.time);
        }

        @Override
        public String toString() {
            return String.format("Event{t=%d, node=%d, state=%d}",
                    time, nodeId, motionState);
        }
    }

    // ── Scheduler implementation ──────────────────────────────────────────────

    private final PriorityQueue<Event> queue;

    /**
     * Constructs an empty scheduler.
     */
    public EventScheduler() {
        this.queue = new PriorityQueue<>();
    }

    // ── Mutation ──────────────────────────────────────────────────────────────

    /**
     * Schedules a new event.
     *
     * @param time        simulation tick at which the event should fire
     * @param nodeId      entity ID targeted by the event
     * @param motionState Markov-chain motion state (0–3)
     */
    public void schedule(long time, int nodeId, int motionState) {
        queue.offer(new Event(time, nodeId, motionState));
    }

    // ── Query ─────────────────────────────────────────────────────────────────

    /**
     * Returns the earliest-scheduled event without removing it, or
     * {@code null} if the queue is empty.
     *
     * @return next event, or {@code null}
     */
    public Event peek() {
        return queue.peek();
    }

    /**
     * Removes and returns the earliest-scheduled event, or {@code null} if
     * the queue is empty.
     *
     * @return next event, or {@code null}
     */
    public Event poll() {
        return queue.poll();
    }

    /**
     * Returns {@code true} if no events are currently scheduled.
     *
     * @return {@code true} when the queue is empty
     */
    public boolean isEmpty() {
        return queue.isEmpty();
    }

    /**
     * Returns the number of pending events.
     *
     * @return pending event count
     */
    public int size() {
        return queue.size();
    }
}
