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
package algorithmTest;

import algorithm.PathPlanner;
import algorithm.Route;
import algorithm.gaaco.GaAcoConfig;
import algorithm.gaaco.GaAcoPlanner;
import domain.SensorNode;

import java.util.ArrayList;
import java.util.List;

/**
 * Self-contained demonstration of the GA→ACO fusion path planner.
 *
 * <p>No simulation infrastructure is needed — the planner works directly on
 * a list of {@link SensorNode} objects.  Run this class via IntelliJ IDEA's
 * "Run" action to see the algorithm output.
 *
 * <h3>What this example shows</h3>
 * <ol>
 *   <li>Create sensor nodes at known positions (no UAV simulation required).</li>
 *   <li>Configure the planner using {@link GaAcoConfig#defaults()} (or a
 *       custom config via the {@code withXxx} copy-and-override methods).</li>
 *   <li>Instantiate {@link GaAcoPlanner} (or any other {@link PathPlanner})
 *       and call {@link PathPlanner#planRoute(List)} — the GA runs first,
 *       then ACO refines the result.</li>
 *   <li>Read the best {@link Route}: node count, total distance, fitness.</li>
 * </ol>
 */
public final class GaAcoTest {

    public static void main(String[] args) {

        // ── 1. Create sensor nodes ─────────────────────────────────────────
        List<SensorNode> nodes = createGridNodes(3, 3, 100.0, 5000);

        System.out.printf("Planning route through %d sensor nodes:%n", nodes.size());
        for (SensorNode n : nodes) {
            System.out.printf("  %s%n", n);
        }

        // ── 2. Configure the planner ───────────────────────────────────────
        GaAcoConfig config = GaAcoConfig.defaults();
        System.out.printf("%nAlgorithm config: %s%n%n", config);

        // ── 3. Run the planner ─────────────────────────────────────────────
        // Program against PathPlanner interface — swap GaAcoPlanner for
        // any other implementation without changing the rest of this code.
        PathPlanner planner = new GaAcoPlanner(config);
        long startMs = System.currentTimeMillis();
        Route route = planner.planRoute(nodes);
        long elapsedMs = System.currentTimeMillis() - startMs;

        // ── 4. Print results ───────────────────────────────────────────────
        System.out.printf("Best route found (in %d ms):%n", elapsedMs);
        System.out.printf("  Nodes visited : %d%n", route.size());
        System.out.printf("  Total distance: %.4f%n", route.getTotalDistance());
        System.out.printf("  Fitness (1/d) : %.6f%n", route.getFitness());
        System.out.printf("  Route order   : %s%n", route);
    }

    private static List<SensorNode> createGridNodes(int rows, int cols,
                                                    double spacing, int idStart) {
        List<SensorNode> nodes = new ArrayList<>(rows * cols);
        int id = idStart;
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                nodes.add(new SensorNode(id++, r * spacing, c * spacing));
            }
        }
        return nodes;
    }
}
