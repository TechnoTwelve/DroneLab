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
package intelligence;

import algorithm.Route;
import simulation.*;

/**
 * No-op {@link UAVIntelligence} implementation that represents the predefined
 * four-corner patrol baseline.
 *
 * <p>This intelligence never triggers replanning and never touches the
 * coverage tracker.  The UAV stays in {@link UAV.DriveMode#PATROL} for the
 * entire simulation duration, following the fixed W1→W2→W3→W4 corner
 * waypoints built into {@link UAV}.
 *
 * <p>Used as the A/B baseline in {@link SimulationRunner#run()}.
 */
public final class PredefinedPatrolIntelligence implements UAVIntelligence {

    /**
     * Does nothing.  The UAV will use its own internal patrol-corner logic
     * without any intelligence intervention.
     *
     * @return always {@code null} (no replanning)
     */
    @Override
    public Route onTick(long tick, UAV uav, Environment env) {
        return null;
    }

    @Override
    public String getLabel() {
        return "Predefined Patrol";
    }
}
