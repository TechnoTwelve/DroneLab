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

import simulation.SimulationConfig;
import simulation.UAV;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import java.awt.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

/**
 * Professional Swing GUI for comparing N UAV intelligence strategies side-by-side.
 *
 * <p>Pass the number of strategies to the constructor — the layout, stat rows,
 * footer results, and listener slots all scale automatically.
 *
 * <h3>Layout (example: 2 passes, sync mode off)</h3>
 * <pre>
 * +--------------------------------------------------------------------+
 * |  🚁  UAV Coverage — Strategy Comparison           [header]         |
 * |  [field: 1500×1500]  [nodes: 50]  [range: 140]  [ticks: 5434]     |
 * +----------------------------------+---------------------------------+
 * |  PASS 1 LABEL                    |  PASS 2 LABEL                   |
 * |  +----------------------------+  |  +---------------------------+  |
 * |  |       SimPanel             |  |  |       SimPanel            |  |
 * |  +----------------------------+  |  +---------------------------+  |
 * |  Tick | Tokens | Mode | Coverage |  Tick | Tokens | Mode | Coverage|
 * +----------------------------------+---------------------------------+
 * |  Pass 1: X/100  ⎥  Pass 2: X/100  ⎥  ●Running  [Speed ←→] [⏸]   |
 * |                  [New Run]                                          |
 * +--------------------------------------------------------------------+
 * </pre>
 *
 * <h3>Threading model</h3>
 * All simulation passes run on separate background threads.  Each pass calls
 * its dedicated {@link VisualizationListener}, which posts Swing updates to
 * the EDT via {@link SwingUtilities#invokeLater}.
 */
public final class SimulationGui {

    // ── Palette ───────────────────────────────────────────────────────────────
    private static final Color COL_BG        = new Color(10,  10,  22);
    private static final Color COL_HEADER    = new Color(14,  14,  30);
    private static final Color COL_FOOTER    = new Color(12,  12,  26);
    private static final Color COL_PANEL_BG  = new Color(13,  13,  27);
    private static final Color COL_DIVIDER   = new Color(38,  38,  68);
    private static final Color COL_TEXT      = new Color(200, 200, 225);
    private static final Color COL_LABEL     = new Color(110, 110, 155);
    private static final Color COL_ACCENT    = new Color(110, 185, 255);
    private static final Color COL_STAT_VAL  = new Color(220, 220, 255);
    private static final Color COL_TAG_BG    = new Color(22,  22,  48);
    private static final Color COL_SYNC_BG   = new Color(  0, 100,  50);
    private static final Color COL_SYNC_FG   = new Color(  0, 230, 120);

    /** Per-pass accent colours — extend to support more than 5 passes. */
    private static final Color[] PASS_COLORS = {
        new Color( 60, 135, 255),   // blue
        new Color(  0, 210, 130),   // green
        new Color(255, 165,  50),   // orange
        new Color(200,  80, 255),   // purple
        new Color(255, 220,  50),   // yellow
    };

    // ── Dynamic per-pass state ────────────────────────────────────────────────
    private final int             passCount;
    private final SimulationPanel[] panels;
    private final JLabel[]          panelTitleLabels;
    private final JLabel[]          lblTick;
    private final JLabel[]          lblTokens;
    private final JLabel[]          lblMode;
    private final JLabel[]          lblCoverage;
    private final JLabel[]          lblResults;

    // Footer status
    private final JLabel lblStatus    = makeStatusVal("Running\u2026");
    private final JLabel lblSyncBadge;

    // Controls
    private final JSlider       speedSlider;
    private final JButton       pauseBtn;
    private final JButton       newRunBtn;
    private final AtomicBoolean paused    = new AtomicBoolean(false);
    private final AtomicInteger tickDelay = new AtomicInteger(20);

    /** Called when the user clicks "New Run". Main.java supplies the action. */
    private volatile Runnable newRunAction = null;

    private final JFrame frame;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * Creates a GUI for {@code passCount} parallel strategy passes with a
     * full configuration summary displayed in the header.
     *
     * @param passCount number of intelligence strategies to display (&ge; 1)
     * @param config    active simulation configuration shown in the header
     * @param syncMode  {@code true} shows a SYNC badge and adjusts labels
     * @throws IllegalArgumentException if {@code passCount} &lt; 1
     */
    public SimulationGui(int passCount, SimulationConfig config, boolean syncMode) {
        if (passCount < 1)
            throw new IllegalArgumentException("passCount must be >= 1, got: " + passCount);

        this.passCount       = passCount;
        panels           = new SimulationPanel[passCount];
        panelTitleLabels = new JLabel[passCount];
        lblTick          = new JLabel[passCount];
        lblTokens        = new JLabel[passCount];
        lblMode          = new JLabel[passCount];
        lblCoverage      = new JLabel[passCount];
        lblResults       = new JLabel[passCount];

        for (int i = 0; i < passCount; i++) {
            panels[i]           = new SimulationPanel();
            Color accent        = passColor(i);
            panelTitleLabels[i] = makePanelTitle("  PASS " + (i + 1), accent);
            lblTick[i]          = makeStatVal("\u2013");
            lblTokens[i]        = makeStatVal("\u2013");
            lblMode[i]          = makeStatVal("\u2013");
            lblCoverage[i]      = makeStatVal("\u2013");
            lblResults[i]       = makeResultVal("\u2013");
        }

        // Sync badge (only visible when syncMode=true)
        if (syncMode) {
            lblSyncBadge = makeChip("\uD83D\uDD04  SYNC", COL_SYNC_FG, COL_SYNC_BG);
        } else {
            lblSyncBadge = null;
        }

        // Speed slider
        speedSlider = new JSlider(0, 100, 20);
        speedSlider.setBackground(COL_FOOTER);
        speedSlider.setForeground(COL_TEXT);
        speedSlider.setPaintTicks(false);
        speedSlider.setPreferredSize(new Dimension(180, 22));
        speedSlider.addChangeListener(e -> {
            int v = speedSlider.getValue();
            tickDelay.set((v * v) / 50);   // quadratic 0-200 ms
        });

        // Pause button
        pauseBtn = makeControlButton("\u23F8  Pause", new Color(35, 55, 90));
        pauseBtn.addActionListener(e -> togglePause());

        // New Run button (hidden until all passes complete)
        newRunBtn = makeControlButton("\uD83D\uDD04  New Run", new Color(30, 55, 35));
        newRunBtn.setVisible(false);
        newRunBtn.addActionListener(e -> onNewRun());

        // Root layout
        JPanel root = new JPanel(new BorderLayout(0, 0));
        root.setBackground(COL_BG);
        root.add(buildHeader(config, syncMode), BorderLayout.NORTH);
        root.add(buildCenter(),                 BorderLayout.CENTER);
        root.add(buildFooter(),                 BorderLayout.SOUTH);

        frame = new JFrame("UAV/WSN Strategy Comparison");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().setBackground(COL_BG);
        frame.getContentPane().add(root);
        frame.pack();
        frame.setResizable(true);
        frame.setLocationRelativeTo(null);
    }

    /**
     * Convenience overload — no config summary, sync mode off.
     * Equivalent to {@code SimulationGui(passCount, SimulationConfig.defaults(), false)}.
     *
     * @param passCount number of intelligence strategies to display (&ge; 1)
     */
    public SimulationGui(int passCount) {
        this(passCount, SimulationConfig.defaults(), false);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Makes the window visible (safe to call from any thread). */
    public void show() {
        SwingUtilities.invokeLater(() -> frame.setVisible(true));
    }

    /**
     * Registers an action that is invoked when the user clicks "New Run".
     * Called on the EDT.  Set before calling {@link #show()}.
     *
     * @param action action to perform; pass {@code null} to clear
     */
    public void setNewRunAction(Runnable action) {
        this.newRunAction = action;
    }

    /**
     * Returns a {@link VisualizationListener} that feeds simulation snapshots
     * into the panel at {@code panelIndex}.
     *
     * @param panelIndex 0-based index; must be in range [0, passCount)
     * @return listener that paces the simulation and posts updates to the EDT
     */
    public VisualizationListener createListener(int panelIndex) {
        if (panelIndex < 0 || panelIndex >= passCount)
            throw new IllegalArgumentException(
                    "panelIndex must be 0.." + (passCount - 1) + ", got: " + panelIndex);

        return snap -> {
            paceAndPause();
            SwingUtilities.invokeLater(() -> {
                // Update panel title on first real label
                if (panelTitleLabels[panelIndex].getText().startsWith("  PASS ")) {
                    panelTitleLabels[panelIndex].setText("  " + snap.passLabel.toUpperCase());
                }
                panels[panelIndex].setSnapshot(snap);
                lblTick[panelIndex]  .setText(snap.tick + " / " + snap.duration);
                lblTokens[panelIndex].setText(snap.tokens + " / " + snap.totalNodes);
                lblMode[panelIndex]  .setText(snap.uavMode.name());
                lblMode[panelIndex]  .setForeground(modeColor(snap.uavMode));
                if (!Double.isNaN(snap.coverageFraction) && snap.coverageFraction > 0) {
                    lblCoverage[panelIndex].setText(
                            String.format("%.1f%%", snap.coverageFraction * 100));
                }
            });
        };
    }

    /**
     * Notifies the GUI that a pass has finished.  Safe to call from any thread.
     *
     * @param pass     1-based pass index
     * @param tokens   tokens collected
     * @param total    total nodes
     * @param coverage coverage fraction [0,1] (NaN if not tracked)
     */
    public void notifyPassComplete(int pass, int tokens, int total, double coverage) {
        String result = String.format("%d / %d  (%.0f%%)",
                tokens, total, total > 0 ? 100.0 * tokens / total : 0.0);
        SwingUtilities.invokeLater(() -> {
            int idx = pass - 1;
            if (idx >= 0 && idx < passCount) {
                lblResults[idx].setText(result);
                lblResults[idx].setForeground(passColor(idx));
            }
            int done = countCompletedPasses();
            if (done == passCount) {
                pauseBtn.setText("\u23F9  Done");
                pauseBtn.setEnabled(false);
                lblStatus.setText("All " + passCount + " passes complete.");
                newRunBtn.setVisible(true);   // reveal "New Run" when done
            } else {
                lblStatus.setText(done + " / " + passCount + " passes done\u2026");
            }
        });
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private int countCompletedPasses() {
        int done = 0;
        for (JLabel l : lblResults) {
            if (!l.getText().equals("\u2013")) done++;
        }
        return done;
    }

    private void paceAndPause() {
        int d = tickDelay.get();
        if (d > 0) {
            try { Thread.sleep(d); } catch (InterruptedException ignored) {}
        }
        while (paused.get()) {
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        }
    }

    private void togglePause() {
        boolean nowPaused = !paused.get();
        paused.set(nowPaused);
        pauseBtn.setText(nowPaused ? "\u25B6  Resume" : "\u23F8  Pause");
    }

    private void onNewRun() {
        Runnable action = newRunAction;
        if (action != null) {
            frame.dispose();
            action.run();
        }
    }

    private static Color modeColor(UAV.DriveMode mode) {
        switch (mode) {
            case EXECUTE:     return new Color(  0, 220, 255);
            case RETURN_HOME: return new Color(255, 100,  80);
            default:          return new Color(170, 170, 215);
        }
    }

    private static Color passColor(int index) {
        return PASS_COLORS[index % PASS_COLORS.length];
    }

    // ── Layout builders ───────────────────────────────────────────────────────

    /** Header: title + config summary chips + optional SYNC badge. */
    private JPanel buildHeader(SimulationConfig config, boolean syncMode) {
        JPanel p = new JPanel();
        p.setBackground(COL_HEADER);
        p.setLayout(new BoxLayout(p, BoxLayout.Y_AXIS));
        p.setBorder(new EmptyBorder(10, 24, 8, 24));

        // ── Title row ──────────────────────────────────────────────────────
        JPanel titleRow = new JPanel(new FlowLayout(FlowLayout.CENTER, 10, 0));
        titleRow.setOpaque(false);

        JLabel title = new JLabel("\uD83D\uDE81  UAV Coverage \u2014 Strategy Comparison");
        title.setForeground(COL_ACCENT);
        title.setFont(new Font("SansSerif", Font.BOLD, 17));
        titleRow.add(title);

        if (lblSyncBadge != null) titleRow.add(lblSyncBadge);

        p.add(titleRow);
        p.add(Box.createVerticalStrut(5));

        // ── Config-summary chips row ───────────────────────────────────────
        JPanel chips = new JPanel(new FlowLayout(FlowLayout.CENTER, 6, 0));
        chips.setOpaque(false);
        chips.add(makeChip(
                "\u25A0 " + config.getFieldWidth() + "\u00D7" + config.getFieldHeight(),
                COL_LABEL, COL_TAG_BG));
        chips.add(makeChip(
                "\u25CF " + config.getNodeCount() + " nodes",
                COL_LABEL, COL_TAG_BG));
        chips.add(makeChip(
                "\u25CE range: " + config.getDroneRange(),
                COL_LABEL, COL_TAG_BG));
        chips.add(makeChip(
                "\u29D7 " + config.getDuration() + " ticks",
                COL_LABEL, COL_TAG_BG));
        chips.add(makeChip(
                "\u2694 " + passCount + (passCount == 1 ? " pass" : " passes"),
                COL_LABEL, COL_TAG_BG));
        p.add(chips);

        p.add(Box.createVerticalStrut(8));
        p.add(hLine());
        return p;
    }

    /** Centre area: N simulation panels side-by-side. */
    private JPanel buildCenter() {
        JPanel p = new JPanel(new GridLayout(1, passCount, 10, 0));
        p.setBackground(COL_BG);
        p.setBorder(new EmptyBorder(8, 8, 8, 8));
        for (int i = 0; i < passCount; i++) {
            p.add(buildPassWrapper(panelTitleLabels[i], panels[i],
                    lblTick[i], lblTokens[i], lblMode[i], lblCoverage[i]));
        }
        return p;
    }

    /**
     * Wraps a {@link SimulationPanel} with a coloured title bar above and
     * a live-stats row below.
     */
    private JPanel buildPassWrapper(JLabel titleLabel, SimulationPanel simPanel,
                                     JLabel tickLbl, JLabel tokensLbl,
                                     JLabel modeLbl, JLabel coverageLbl) {
        Color accent = titleLabel.getForeground();

        JPanel wrapper = new JPanel(new BorderLayout(0, 3));
        wrapper.setBackground(COL_PANEL_BG);
        wrapper.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(accent.darker().darker(), 1),
                new EmptyBorder(0, 0, 0, 0)));

        wrapper.add(titleLabel, BorderLayout.NORTH);
        wrapper.add(simPanel,   BorderLayout.CENTER);

        JPanel stats = new JPanel(new FlowLayout(FlowLayout.LEFT, 14, 3));
        stats.setBackground(new Color(12, 12, 24));
        stats.setBorder(new EmptyBorder(1, 4, 1, 4));
        stats.add(statPair("Tick",     tickLbl));
        stats.add(statPair("Tokens",   tokensLbl));
        stats.add(statPair("Mode",     modeLbl));
        stats.add(statPair("Coverage", coverageLbl));
        wrapper.add(stats, BorderLayout.SOUTH);

        return wrapper;
    }

    /** Bottom footer: per-pass results + speed + controls. */
    private JPanel buildFooter() {
        JPanel p = new JPanel();
        p.setBackground(COL_FOOTER);
        p.setLayout(new BoxLayout(p, BoxLayout.Y_AXIS));
        p.setBorder(new EmptyBorder(5, 16, 9, 16));

        p.add(hLine());
        p.add(Box.createVerticalStrut(5));

        // Results row
        JPanel resultsRow = new JPanel(new FlowLayout(FlowLayout.CENTER, 20, 0));
        resultsRow.setBackground(COL_FOOTER);
        for (int i = 0; i < passCount; i++) {
            if (i > 0) resultsRow.add(vBar());
            resultsRow.add(makeKey("Pass " + (i + 1) + ":"));
            resultsRow.add(lblResults[i]);
        }
        resultsRow.add(vBar());
        resultsRow.add(lblStatus);
        p.add(resultsRow);

        p.add(Box.createVerticalStrut(3));

        // Controls row
        JPanel ctrlRow = new JPanel(new FlowLayout(FlowLayout.CENTER, 12, 0));
        ctrlRow.setBackground(COL_FOOTER);
        ctrlRow.add(makeKey("Speed:"));
        ctrlRow.add(makeSmall("Fast"));
        ctrlRow.add(speedSlider);
        ctrlRow.add(makeSmall("Slow"));
        ctrlRow.add(pauseBtn);
        ctrlRow.add(newRunBtn);
        p.add(ctrlRow);

        return p;
    }

    // ── Widget helpers ────────────────────────────────────────────────────────

    private static JLabel makePanelTitle(String text, Color accent) {
        JLabel l = new JLabel(text);
        l.setForeground(accent);
        l.setFont(new Font("SansSerif", Font.BOLD, 13));
        l.setOpaque(true);
        l.setBackground(new Color(
                accent.getRed()   / 9,
                accent.getGreen() / 9,
                accent.getBlue()  / 9));
        l.setBorder(new EmptyBorder(5, 8, 5, 8));
        return l;
    }

    /** Small pill-shaped chip label for the config summary row. */
    private static JLabel makeChip(String text, Color fg, Color bg) {
        JLabel l = new JLabel("  " + text + "  ") {
            @Override protected void paintComponent(Graphics g) {
                Graphics2D g2 = (Graphics2D) g.create();
                g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                        RenderingHints.VALUE_ANTIALIAS_ON);
                g2.setColor(getBackground());
                g2.fillRoundRect(0, 0, getWidth(), getHeight(), 10, 10);
                g2.dispose();
                super.paintComponent(g);
            }
        };
        l.setForeground(fg);
        l.setBackground(bg);
        l.setFont(new Font("SansSerif", Font.PLAIN, 10));
        l.setOpaque(false);
        l.setBorder(new EmptyBorder(2, 2, 2, 2));
        return l;
    }

    private JPanel statPair(String key, JLabel val) {
        JPanel row = new JPanel(new FlowLayout(FlowLayout.LEFT, 3, 0));
        row.setBackground(new Color(12, 12, 24));
        JLabel k = new JLabel(key + ":");
        k.setForeground(COL_LABEL);
        k.setFont(new Font("SansSerif", Font.PLAIN, 10));
        row.add(k);
        row.add(val);
        return row;
    }

    private JSeparator hLine() {
        JSeparator sep = new JSeparator();
        sep.setForeground(COL_DIVIDER);
        sep.setBackground(COL_DIVIDER);
        sep.setMaximumSize(new Dimension(Integer.MAX_VALUE, 1));
        return sep;
    }

    private static JLabel vBar() {
        JLabel l = new JLabel("\u2502");
        l.setForeground(COL_DIVIDER);
        return l;
    }

    private static JButton makeControlButton(String text, Color bg) {
        JButton b = new JButton(text);
        b.setBackground(bg);
        b.setForeground(COL_TEXT);
        b.setFocusPainted(false);
        b.setBorderPainted(false);
        b.setFont(new Font("SansSerif", Font.PLAIN, 12));
        b.setPreferredSize(new Dimension(110, 28));
        b.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        return b;
    }

    private static JLabel makeStatVal(String t) {
        JLabel l = new JLabel(t);
        l.setForeground(COL_STAT_VAL);
        l.setFont(new Font("Monospaced", Font.PLAIN, 10));
        return l;
    }

    private static JLabel makeResultVal(String t) {
        JLabel l = new JLabel(t);
        l.setForeground(new Color(180, 180, 225));
        l.setFont(new Font("Monospaced", Font.BOLD, 12));
        return l;
    }

    private static JLabel makeStatusVal(String t) {
        JLabel l = new JLabel(t);
        l.setForeground(COL_LABEL);
        l.setFont(new Font("SansSerif", Font.ITALIC, 11));
        return l;
    }

    private static JLabel makeKey(String t) {
        JLabel l = new JLabel(t);
        l.setForeground(COL_LABEL);
        l.setFont(new Font("SansSerif", Font.PLAIN, 11));
        return l;
    }

    private static JLabel makeSmall(String t) {
        JLabel l = new JLabel(t);
        l.setForeground(COL_LABEL);
        l.setFont(new Font("SansSerif", Font.PLAIN, 9));
        return l;
    }
}
