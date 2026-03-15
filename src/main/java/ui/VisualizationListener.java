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
package ui;

/**
 * Callback interface invoked by {@link simulation.SimulationRunner} after
 * every drone-event tick when a visualization listener is registered.
 *
 * <p>Implementations are responsible for updating the display and, if desired,
 * sleeping between ticks to pace the simulation to a human-visible speed.
 * The callback runs on the simulation thread — all Swing operations must be
 * dispatched to the EDT via {@code SwingUtilities.invokeLater}.
 */
@FunctionalInterface
public interface VisualizationListener {
    /**
     * Called once per drone-event tick with a snapshot of the current
     * simulation state.
     *
     * @param snap immutable state snapshot; never null
     */
    void onTick(SimulationSnapshot snap);
}
