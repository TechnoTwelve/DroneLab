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
import config.AppConfig;
import scenarios.WSNDataCollectionScenario;
import simulation.SimulationRunner;
import simulation.SimulationScenario;
import ui.DashboardGui;
import javax.swing.*;

/**
 * Application entry point for the UAV/WSN discrete-event simulation.
 *
 * <h3>GUI mode (default)</h3>
 * Opens the {@link DashboardGui} — a full-screen, IntelliJ-style dashboard.
 * Settings (scan range, field size, node count, duration, seed …) live in a
 * permanent bottom bar.  A toggleable console panel at the bottom captures all
 * simulation output.
 *
 * <h3>Headless / batch mode</h3>
 * Pass {@code --headless} to skip the GUI and run on stdout.  Add
 * {@code --runs N} to repeat the simulation N times (useful for parameter
 * sweeps and result averaging).
 *
 * <pre>
 *   java -jar drone-lab.jar --headless
 *   java -jar drone-lab.jar --headless --seed 12345
 *   java -jar drone-lab.jar --headless --runs 20
 *   java -jar drone-lab.jar --headless --runs 20 --seed 42
 * </pre>
 *
 * <h3>Swapping scenarios</h3>
 * Replace {@code new WSNDataCollectionScenario()} with any class that
 * implements {@link SimulationScenario} to run a different experiment without
 * touching the runner or GUI code.
 */
public class Main {

    public static void main(String[] args) throws InterruptedException {

        // ── Parse CLI flags ───────────────────────────────────────────────────
        boolean headless  = false;
        long    forceSeed = AppConfig.NO_SEED;
        int     runs      = 1;

        for (int i = 0; i < args.length; i++) {
            if ("--headless".equalsIgnoreCase(args[i])) headless = true;
            if ("--seed".equalsIgnoreCase(args[i]) && i + 1 < args.length) {
                try { forceSeed = Long.parseLong(args[++i]); }
                catch (NumberFormatException ignored) { }
            }
            if ("--runs".equalsIgnoreCase(args[i]) && i + 1 < args.length) {
                try { runs = Math.max(1, Integer.parseInt(args[++i])); }
                catch (NumberFormatException ignored) { }
            }
        }

        // ── Scenario selection ────────────────────────────────────────────────
        SimulationScenario scenario  = new WSNDataCollectionScenario();
        AppConfig          appConfig = AppConfig.load();

        if (headless) {
            // ── Headless batch mode ───────────────────────────────────────────
            System.out.printf("=== DroneLab headless — %d run%s ===%n%n",
                    runs, runs == 1 ? "" : "s");
            for (int r = 1; r <= runs; r++) {
                if (runs > 1) System.out.printf("--- Run %d / %d ---%n", r, runs);
                SimulationRunner runner = SimulationRunner.fromScenario(scenario);
                if (forceSeed != AppConfig.NO_SEED) runner.withSeed(forceSeed);
                runner.run(scenario);
                if (runs > 1) System.out.println();
            }
            System.out.printf("=== Done ===%n");
            return;
        }

        // ── GUI mode: install FlatLaf Darcula (IntelliJ IDEA Darcula theme) ──────
        try {
            com.formdev.flatlaf.FlatDarculaLaf.setup();
        } catch (Exception e) {
            try { UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName()); }
            catch (Exception ignored) { }
        }

        // ── Open the unified dashboard ────────────────────────────────────────
        DashboardGui dashboard = new DashboardGui(scenario, appConfig);
        dashboard.show();
        // Note: --seed CLI override only applies to headless mode.
        // In GUI mode use the Seed section in the bottom bar.
    }
}
