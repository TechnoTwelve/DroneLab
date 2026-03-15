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
package algorithm.gaaco;

import algorithm.Route;
import domain.SensorNode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Stateless local-search passes for TSP-style open-path optimisation.
 *
 * <p>All methods are pure functions: they accept a {@link Route} and return a
 * (possibly improved) {@link Route}.  The input route is never modified.
 * Both passes are called as the final "phase 3" of the GA→ACO→LocalSearch
 * fusion pipeline in {@link GaAcoPlanner}, and the 2-opt pass is also
 * applied to the GA best route before it seeds the ACO phase.
 *
 * <h3>Open-path semantics</h3>
 * The UAV collects data in a single sweep and does not return to its origin,
 * so the path is treated as an open Hamiltonian path (not a TSP cycle).
 * Edge costs involve only consecutive pairs in the visit order; there is no
 * return edge from the last node back to the first.
 *
 * <h3>Typical usage</h3>
 * <pre>
 *   Route r = aco.refine(gaRoute, nodes);
 *   r = LocalSearch.twoOpt(r);    // eliminate crossings
 *   r = LocalSearch.orOpt(r);     // relocate stray waypoints
 * </pre>
 */
public final class LocalSearch {

    /** Utility class — no instances. */
    private LocalSearch() {}

    // ── 2-opt ─────────────────────────────────────────────────────────────────

    /**
     * 2-opt local search: iteratively reverses path segments to eliminate
     * crossing edges until no further improvement is possible.
     *
     * <h3>Move semantics (open path)</h3>
     * For each pair {@code (i, j)} with {@code i < j}, reversing the segment
     * {@code [i+1 … j]} changes two "connecting" edges:
     * <ul>
     *   <li>Removed: {@code (i → i+1)} and {@code (j → j+1)} if {@code j < n-1}</li>
     *   <li>Added:   {@code (i → j)}   and {@code (i+1 → j+1)} if {@code j < n-1}</li>
     * </ul>
     * When {@code j = n-1} only the single edge {@code (i → i+1)} is removed and
     * replaced by {@code (i → n-1)}; the internal segment distances cancel
     * symmetrically because distances are Euclidean (symmetric).
     *
     * <p>The move is applied whenever total added distance is less than total
     * removed distance by more than a numerical tolerance (1e-10).
     *
     * <p>Complexity: O(n²) per pass; converges in at most O(n²) passes.
     * For the node counts typical in this simulation (≤ 50) this is
     * near-instant.
     *
     * @param route route to improve; must not be null
     * @return improved route (may be identical to input if already locally optimal)
     */
    public static Route twoOpt(Route route) {
        List<SensorNode> nodes = new ArrayList<>(route.getNodes());
        int n = nodes.size();
        if (n < 4) return route; // 2-opt needs at least 4 nodes to offer a meaningful swap

        boolean improved = true;
        while (improved) {
            improved = false;
            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 2; j < n; j++) {
                    // Edges removed by the reversal
                    double removed = nodes.get(i).calculateDistance(nodes.get(i + 1))
                                   + (j < n - 1
                                      ? nodes.get(j).calculateDistance(nodes.get(j + 1))
                                      : 0.0);
                    // Edges added after reversing [i+1..j]
                    double added   = nodes.get(i).calculateDistance(nodes.get(j))
                                   + (j < n - 1
                                      ? nodes.get(i + 1).calculateDistance(nodes.get(j + 1))
                                      : 0.0);
                    if (added < removed - 1e-10) {
                        Collections.reverse(nodes.subList(i + 1, j + 1));
                        improved = true;
                    }
                }
            }
        }
        return new Route(nodes);
    }

    // ── Or-opt ────────────────────────────────────────────────────────────────

    /**
     * Or-opt local search: relocates single nodes to cheaper positions in the
     * path until no further improvement is possible.
     *
     * <h3>Move semantics</h3>
     * For each node at position {@code i}, the algorithm computes the cost
     * saved by removing it from its current position (the "gap" is closed with
     * a direct edge between its neighbours), then evaluates the cost of
     * reinserting it at every other position in the path.  The best net
     * improvement across all candidate positions is applied if positive, then
     * the search restarts from the beginning of the (now-modified) path.
     *
     * <h3>Complementary to 2-opt</h3>
     * While 2-opt eliminates crossing edges by reversing segments, Or-opt moves
     * individual waypoints.  The two passes address different structural defects
     * and are strongest when applied in sequence: 2-opt first, then Or-opt.
     *
     * <h3>Open-path boundary handling</h3>
     * The first and last nodes have only one neighbouring edge.  The removal
     * saving for a path-end node is simply the cost of that single edge
     * (removing the endpoint shortens the path by that amount with no repair
     * edge needed).
     *
     * <p>Complexity: O(n²) per pass.  For the small node counts used in this
     * simulation (≤ 50) this is negligible.
     *
     * @param route route to improve; must not be null
     * @return improved route (may be identical to input if already locally optimal)
     */
    public static Route orOpt(Route route) {
        List<SensorNode> nodes = new ArrayList<>(route.getNodes());
        int n = nodes.size();
        if (n < 3) return route; // need at least 3 nodes to relocate one meaningfully

        boolean improved = true;
        while (improved) {
            improved = false;
            for (int i = 0; i < nodes.size() && !improved; i++) {
                int sz = nodes.size();

                // ── Cost saved by removing node i from its current position ───
                // For 0 < i < sz-1: remove edges (i-1→i) and (i→i+1), add repair (i-1→i+1).
                // For endpoints: only one edge is removed; no repair edge needed.
                double removeSaving;
                if (i == 0) {
                    removeSaving = nodes.get(0).calculateDistance(nodes.get(1));
                } else if (i == sz - 1) {
                    removeSaving = nodes.get(sz - 2).calculateDistance(nodes.get(sz - 1));
                } else {
                    removeSaving = nodes.get(i - 1).calculateDistance(nodes.get(i))
                                 + nodes.get(i).calculateDistance(nodes.get(i + 1))
                                 - nodes.get(i - 1).calculateDistance(nodes.get(i + 1));
                }

                SensorNode moved    = nodes.get(i);
                int        remaining = sz - 1; // size of path after hypothetical removal

                double bestNet = 1e-10; // minimum threshold (avoid floating-point no-ops)
                int    bestJ   = -1;

                // ── Try every insertion position in the post-removal path ─────
                // Post-removal list: original[0..i-1, i+1..sz-1], indexed [0..remaining-1].
                // j is a gap index: j=0 means "before post-removal[0]",
                // j=remaining means "after post-removal[remaining-1]".
                // Mapping to original indices:
                //   postRemoval[k] = nodes[k]   if k < i
                //                  = nodes[k+1] if k ≥ i
                for (int j = 0; j <= remaining; j++) {
                    if (j == i) continue; // j==i restores the original position — no gain

                    double insertCost;
                    if (j == 0) {
                        // Insert before the first element of the post-removal list
                        int firstOrigIdx = (i == 0) ? 1 : 0;
                        insertCost = moved.calculateDistance(nodes.get(firstOrigIdx));

                    } else if (j == remaining) {
                        // Insert after the last element of the post-removal list
                        int lastOrigIdx = (i == sz - 1) ? sz - 2 : sz - 1;
                        insertCost = nodes.get(lastOrigIdx).calculateDistance(moved);

                    } else {
                        // Insert between post-removal positions j-1 and j.
                        // Map to original indices (skip over i):
                        int prevOrigIdx = (j - 1 < i) ? j - 1 : j;
                        int nextOrigIdx = (j     < i) ? j     : j + 1;
                        insertCost = nodes.get(prevOrigIdx).calculateDistance(moved)
                                   + moved.calculateDistance(nodes.get(nextOrigIdx))
                                   - nodes.get(prevOrigIdx).calculateDistance(nodes.get(nextOrigIdx));
                    }

                    double net = removeSaving - insertCost;
                    if (net > bestNet) {
                        bestNet = net;
                        bestJ   = j;
                    }
                }

                // ── Apply the best relocation found ───────────────────────────
                if (bestJ != -1) {
                    nodes.remove(i);
                    // After remove(i), nodes IS the post-removal list, so bestJ is valid directly.
                    nodes.add(bestJ, moved);
                    improved = true;
                }
            }
        }
        return new Route(nodes);
    }
}
