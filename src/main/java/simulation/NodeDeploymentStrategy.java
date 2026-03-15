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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * Strategy for deploying sensor nodes (or other targets) into the simulation
 * environment at the start of each pass.
 *
 * <h3>Built-in implementations</h3>
 * <ul>
 *   <li>{@link #random()} — fully random placement; any position in the field
 *       is equally likely.  Can produce clusters and sparse regions by chance,
 *       making results sensitive to which seed is used.</li>
 *   <li>{@link #stratified()} — stratified random sampling (jittered grid);
 *       divides the field into a grid of equal cells and places exactly one
 *       node at a random position within each cell.  Guarantees spatial
 *       uniformity regardless of seed while retaining per-run randomness.
 *       <em>Recommended for scientifically credible experiments.</em></li>
 * </ul>
 *
 * <h3>Custom strategies</h3>
 * <pre>
 *   // Cluster nodes in the bottom-left quadrant
 *   NodeDeploymentStrategy clustered = (env, config, rng) -&gt; {
 *       int halfW = config.getFieldWidth()  / 2;
 *       int halfH = config.getFieldHeight() / 2;
 *       int placed = 0;
 *       while (placed &lt; config.getNodeCount()) {
 *           int x = rng.nextInt(halfW);
 *           int y = rng.nextInt(halfH);
 *           NodeAgent a = new NodeAgent(config.getNodeIdStart() + placed,
 *                                       x, y,
 *                                       rng.nextInt(NodeAgent.DIRECTION_COUNT),
 *                                       config.getNodeMinSpeed());
 *           if (env.deployAgent(a)) placed++;
 *       }
 *   };
 * </pre>
 */
@FunctionalInterface
public interface NodeDeploymentStrategy {

    /**
     * Deploys exactly {@link SimulationConfig#getNodeCount()} agents into
     * {@code env} using {@code rng} for all random decisions.
     *
     * @param env    environment to deploy into
     * @param config simulation parameters (node count, field size, speeds, etc.)
     * @param rng    seeded random generator shared with the rest of the pass;
     *               all random decisions must go through this instance for
     *               reproducibility
     */
    void deploy(Environment env, SimulationConfig config, Random rng);

    // ── Built-in strategies ───────────────────────────────────────────────────

    /**
     * Fully random placement: each node is placed at a uniformly random
     * position anywhere in the field.
     *
     * <p>Simple but can produce clusters and sparse regions by chance, making
     * results sensitive to the choice of seed.  Use {@link #stratified()}
     * for experiments that require seed-independent spatial uniformity.
     */
    static NodeDeploymentStrategy random() {
        return (env, config, rng) -> {
            int fw    = config.getFieldWidth();
            int fh    = config.getFieldHeight();
            int count = 0;
            while (count < config.getNodeCount()) {
                int x         = rng.nextInt(fw);
                int y         = rng.nextInt(fh);
                int id        = config.getNodeIdStart() + count;
                int direction = rng.nextInt(NodeAgent.DIRECTION_COUNT);
                int speedRange = config.getNodeMaxSpeed() - config.getNodeMinSpeed() + 1;
                int speed      = config.getNodeMinSpeed() + rng.nextInt(speedRange);
                NodeAgent agent = new NodeAgent(id, x, y, direction, speed);
                if (env.deployAgent(agent)) count++;
            }
        };
    }

    /**
     * Stratified random sampling (jittered grid): divides the field into a
     * near-square grid of cells and places exactly one node at a random
     * position within each cell.
     *
     * <h3>Why this matters scientifically</h3>
     * Pure random placement is spatially uniform in expectation but produces
     * clusters and voids in any individual run.  Strategy A might happen to
     * cover a dense cluster that Strategy B misses purely by chance, skewing
     * the comparison.  Stratified sampling guarantees every region of the
     * field receives exactly one node, making results independent of which
     * seed is used and removing the need to cherry-pick "good" seeds.
     *
     * <h3>Example — 225 nodes, 1500×1500 field</h3>
     * Grid: 15 columns × 15 rows, each cell 100×100 units.  Every 100×100
     * region gets exactly one randomly-positioned node.  The seed still
     * controls <em>where within each cell</em> the node lands, plus initial
     * direction and speed, so results vary across seeds while remaining
     * spatially fair.
     */
    static NodeDeploymentStrategy stratified() {
        return (env, config, rng) -> {
            int n  = config.getNodeCount();
            int fw = config.getFieldWidth();
            int fh = config.getFieldHeight();

            // Build a near-square grid with at least n cells
            int cols = (int) Math.ceil(Math.sqrt(n));
            int rows = (int) Math.ceil((double) n / cols);

            double cellW = (double) fw / cols;
            double cellH = (double) fh / rows;

            // Shuffle all cell indices so the n chosen cells are spatially random
            // (matters when rows*cols > n, e.g. 10 nodes in a 4×3 grid)
            List<Integer> cellIndices = new ArrayList<>(rows * cols);
            for (int i = 0; i < rows * cols; i++) cellIndices.add(i);
            Collections.shuffle(cellIndices, rng);

            int placed = 0;
            for (int ci : cellIndices) {
                if (placed >= n) break;
                int col = ci % cols;
                int row = ci / cols;

                // Random position within this cell, clamped to field bounds
                int x = Math.min((int)(col * cellW + rng.nextDouble() * cellW), fw - 1);
                int y = Math.min((int)(row * cellH + rng.nextDouble() * cellH), fh - 1);

                int id         = config.getNodeIdStart() + placed;
                int direction  = rng.nextInt(NodeAgent.DIRECTION_COUNT);
                int speedRange = config.getNodeMaxSpeed() - config.getNodeMinSpeed() + 1;
                int speed      = config.getNodeMinSpeed() + rng.nextInt(speedRange);

                NodeAgent agent = new NodeAgent(id, x, y, direction, speed);
                if (env.deployAgent(agent)) {
                    placed++;
                }
                // If the cell's chosen position was already occupied (extremely rare),
                // this cell is skipped; the while-loop below covers any shortfall.
            }

            // Fallback for any unplaced nodes (collision edge-case only)
            while (placed < n) {
                int x         = rng.nextInt(fw);
                int y         = rng.nextInt(fh);
                int id        = config.getNodeIdStart() + placed;
                int direction = rng.nextInt(NodeAgent.DIRECTION_COUNT);
                int speedRange = config.getNodeMaxSpeed() - config.getNodeMinSpeed() + 1;
                int speed      = config.getNodeMinSpeed() + rng.nextInt(speedRange);
                NodeAgent agent = new NodeAgent(id, x, y, direction, speed);
                if (env.deployAgent(agent)) placed++;
            }
        };
    }
}
