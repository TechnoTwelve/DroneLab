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
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * A mutable, ordered collection of {@link Route} objects representing one
 * generation in the Genetic Algorithm.
 *
 * <h3>Mutability contract</h3>
 * Individual routes within the population may be replaced via {@link #set},
 * allowing the GA to build each new generation in-place without allocating a
 * new population object for every internal step.  The backing list is exposed
 * as an unmodifiable view through {@link #getRoutes()}; callers that need to
 * iterate routes safely can use that accessor.
 *
 * <h3>Sorting</h3>
 * {@link #sortByFitness()} orders routes from highest fitness (best route) to
 * lowest, placing the elite candidates at the front of the list — exactly where
 * the GA expects to find them when it copies elite routes into the next
 * generation unchanged.
 */
public final class Population {

    private final List<Route> routes;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Constructs a population from the given route list.
     *
     * <p>A defensive copy of {@code routes} is made; modifications to the
     * caller's list after construction have no effect on this population.
     *
     * @param routes initial route list; must not be null or empty
     * @throws IllegalArgumentException if {@code routes} is null or empty
     */
    public Population(List<Route> routes) {
        if (routes == null || routes.isEmpty()) {
            throw new IllegalArgumentException("Population must contain at least one route.");
        }
        this.routes = new ArrayList<>(routes);
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /**
     * Returns an unmodifiable view of the route list.
     *
     * <p>Use {@link #get} and {@link #set} to access individual slots when
     * a mutable reference to an element is required.
     *
     * @return unmodifiable ordered list of routes
     */
    public List<Route> getRoutes() {
        return Collections.unmodifiableList(routes);
    }

    /**
     * Returns the route at the given index.
     *
     * @param index zero-based position in the population
     * @return route at that position
     * @throws IndexOutOfBoundsException if index is out of range
     */
    public Route get(int index) {
        return routes.get(index);
    }

    /**
     * Replaces the route at the given index.
     *
     * <p>Used by the GA to install crossed-over and mutated routes into the
     * population without creating a new population object.
     *
     * @param index zero-based position to replace
     * @param route replacement route; must not be null
     * @throws IndexOutOfBoundsException if index is out of range
     * @throws NullPointerException      if route is null
     */
    public void set(int index, Route route) {
        if (route == null) throw new NullPointerException("route must not be null");
        routes.set(index, route);
    }

    /**
     * Returns the number of routes in this population.
     *
     * @return population size (≥ 1)
     */
    public int size() {
        return routes.size();
    }

    // ── Fitness operations ────────────────────────────────────────────────────

    /**
     * Sorts the population in descending fitness order (best route first).
     *
     * <p>After this call, the elite routes occupy the lowest indices and the
     * weakest routes occupy the highest indices — the layout that
     * {@link GeneticAlgorithm} relies on when it preserves elites and replaces
     * the tail of the population by crossover.
     */
    public void sortByFitness() {
        routes.sort(Comparator.comparingDouble(Route::getFitness).reversed());
    }

    /**
     * Returns the route with the highest fitness value in this population.
     *
     * <p>Works correctly regardless of sort state: iterates the entire list
     * and picks the global maximum.  For a sorted population the result is
     * always {@code get(0)}.
     *
     * @return best route in the population
     */
    public Route getBest() {
        return routes.stream()
                .max(Comparator.comparingDouble(Route::getFitness))
                .orElseThrow(() -> new IllegalStateException("Population is empty"));
    }

    // ── Object overrides ──────────────────────────────────────────────────────

    @Override
    public String toString() {
        return String.format("Population{size=%d, best=%s}", routes.size(), getBest());
    }
}
