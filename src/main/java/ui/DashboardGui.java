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

import config.AppConfig;
import simulation.SimulationConfig;
import simulation.SimulationRunner;
import simulation.SimulationScenario;
import simulation.UAV;
import simulation.UAVIntelligence;

import javax.swing.*;
import javax.swing.border.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

/**
 * Unified single-window dashboard styled after IntelliJ IDEA Darcula.
 *
 * <h3>Layout</h3>
 * <pre>
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │  UAV/WSN Coverage Dashboard                ● IDLE  ▶ Start  ⏸  ↺       │  toolbar
 * ├────┬─────────────────────────────────────────────────────────────────────┤
 * │    │  PASS 1                    PASS 2                                    │
 * │ ⚙  │  ┌──────────────────────┐  ┌──────────────────────┐                 │  sim
 * │    │  │   SimulationPanel    │  │   SimulationPanel    │                 │
 * │ ≡  │  └──────────────────────┘  └──────────────────────┘                 │
 * │    │  Tick|Tokens|Mode|Cov      Tick|Tokens|Mode|Cov                     │
 * ├────┴──────────────────────────────────────────────────────────────────────┤
 * │  Pass 1: – | Pass 2: – | Ready…                                          │  status
 * ├──────────────────────────────────────────────────────────────────────────┤
 * │  Settings  (animates up/down independently)                               │  tool win
 * │  Console   (animates up/down independently, stacks with settings)         │
 * └──────────────────────────────────────────────────────────────────────────┘
 * </pre>
 */
public final class DashboardGui {

    // ── Tool window heights ───────────────────────────────────────────────────
    private static final int SETTINGS_H = 235;
    private static final int CONSOLE_H  = 200;

    // ── Custom colours ────────────────────────────────────────────────────────
    // COL_SIM_BG: neutral dark for simulation canvas — enough contrast for dots
    private static final Color COL_SIM_BG = new Color(30, 31, 34);   // Darcula editor bg
    private static final Color COL_CON_BG = new Color(30, 31, 34);
    private static final Color COL_CON_FG = new Color(170, 210, 160);
    private static final Color COL_ACCENT = new Color(75,  140, 250);

    private static final Color[] PASS_COLORS = {
        new Color( 76, 160, 255),
        new Color(  0, 200, 120),
        new Color(255, 150,  40),
        new Color(185,  70, 240),
    };

    // ── Run state ─────────────────────────────────────────────────────────────
    private enum RunState { IDLE, RUNNING, PAUSED, COMPLETE }
    private volatile RunState      runState   = RunState.IDLE;
    private final AtomicInteger    generation = new AtomicInteger(0);
    private volatile SimulationRunner activeRunner = null;
    private final AtomicBoolean    paused    = new AtomicBoolean(false);
    private final AtomicInteger    tickDelay = new AtomicInteger(20);

    // ── Settings controls ─────────────────────────────────────────────────────
    private JSlider  sliderRange, sliderNodes, sliderDuration;
    private JLabel   lblRangeVal, lblNodesVal, lblDurVal;
    private JSpinner spinDroneSpeed, spinPlacement;
    private JSpinner spinFieldW, spinFieldH;
    private JSpinner spinNodeMin, spinNodeMax;
    private JRadioButton rbAuto, rbFixed;
    private JTextField   tfSeed;
    private JSlider      speedSlider;

    // ── Toolbar buttons ───────────────────────────────────────────────────────
    private JButton btnStart, btnPause, btnRestart;
    private JLabel  lblState;

    // ── Icon sidebar ──────────────────────────────────────────────────────────
    private JButton btnIconSettings, btnIconConsole;

    // ── Tool windows (stacked, independently toggled) ─────────────────────────
    private JPanel  toolWindow;          // BoxLayout Y_AXIS container
    private JPanel  settingsWrap;        // wrapper for settings panel
    private JPanel  consoleWrap;         // wrapper for console panel
    private boolean settingsOpen = false;
    private boolean consoleOpen  = false;

    // ── Console ───────────────────────────────────────────────────────────────
    private JTextArea consoleArea;

    // ── Simulation panels ─────────────────────────────────────────────────────
    private final int          passCount;
    private SimulationPanel[]  panels;
    private JLabel[]           panelTitles;
    private JLabel[]           lblResults;
    private JLabel[]           lblTick;
    private JLabel[]           lblTokens;
    private JLabel[]           lblMode;
    private JLabel[]           lblCoverage;
    private JPanel             simAreaCards;
    private CardLayout         simCards;
    private JLabel             lblStatus;

    // ── Infrastructure ────────────────────────────────────────────────────────
    private final SimulationScenario scenario;
    private final AppConfig          appConfig;
    private final JFrame             frame;

    // ── Constructor ───────────────────────────────────────────────────────────

    public DashboardGui(SimulationScenario scenario, AppConfig appConfig) {
        this.scenario  = scenario;
        this.appConfig = appConfig;
        this.passCount = Math.max(1, appConfig.getIntelligenceKeys().size());

        initPanelArrays();
        initConsole();
        initToolWindow();   // must come after initConsole()

        JPanel simColumn = new JPanel(new BorderLayout(0, 0));
        simColumn.add(buildSimArea(),   BorderLayout.CENTER);
        simColumn.add(buildStatusBar(), BorderLayout.SOUTH);

        JPanel workArea = new JPanel(new BorderLayout(0, 0));
        workArea.add(buildIconSidebar(), BorderLayout.WEST);
        workArea.add(simColumn,          BorderLayout.CENTER);

        JPanel root = new JPanel(new BorderLayout(0, 0));
        root.add(buildToolbar(), BorderLayout.NORTH);
        root.add(workArea,       BorderLayout.CENTER);
        root.add(toolWindow,     BorderLayout.SOUTH);

        frame = new JFrame("DroneLab");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setContentPane(root);
        frame.pack();
        frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
    }

    public void show() {
        SwingUtilities.invokeLater(() -> {
            frame.setVisible(true);
            // Show both tool windows immediately on startup
            SwingUtilities.invokeLater(() -> {
                settingsOpen = true;
                consoleOpen  = true;
                settingsWrap.setVisible(true);
                consoleWrap.setVisible(true);
                refreshSidebarHighlight();
            });
        });
    }

    // ── Panel-array initialisation ────────────────────────────────────────────

    private void initPanelArrays() {
        panels      = new SimulationPanel[passCount];
        panelTitles = new JLabel[passCount];
        lblResults  = new JLabel[passCount];
        lblTick     = new JLabel[passCount];
        lblTokens   = new JLabel[passCount];
        lblMode     = new JLabel[passCount];
        lblCoverage = new JLabel[passCount];

        List<String> keys = appConfig.getIntelligenceKeys();
        for (int i = 0; i < passCount; i++) {
            panels[i]      = new SimulationPanel();
            panels[i].setPreferredSize(new Dimension(passCount <= 2 ? 480 : 360, 430));
            String algoLabel = (i < keys.size()) ? keys.get(i) : "Pass " + (i + 1);
            panelTitles[i] = makePanelTitle(algoLabel.toUpperCase(), passColor(i));
            lblResults[i]  = makeResultVal("\u2013");
            lblTick[i]     = makeStatVal("\u2013");
            lblTokens[i]   = makeStatVal("\u2013");
            lblMode[i]     = makeStatVal("\u2013");
            lblCoverage[i] = makeStatVal("\u2013");
        }

        lblStatus = new JLabel("Ready \u2014 click \u25B6 Start to begin");
        lblStatus.setFont(new Font("SansSerif", Font.ITALIC, 11));
    }

    // ── Console ───────────────────────────────────────────────────────────────

    private void initConsole() {
        consoleArea = new JTextArea();
        consoleArea.setEditable(false);
        consoleArea.setBackground(COL_CON_BG);
        consoleArea.setForeground(COL_CON_FG);
        consoleArea.setFont(new Font("Monospaced", Font.PLAIN, 11));
        consoleArea.setBorder(new EmptyBorder(4, 8, 4, 8));

        PrintStream teeOut = new PrintStream(new TeeOutputStream(System.out, consoleArea), true);
        PrintStream teeErr = new PrintStream(new TeeOutputStream(System.err, consoleArea), true);
        System.setOut(teeOut);
        System.setErr(teeErr);
    }

    // ── Tool window init ──────────────────────────────────────────────────────

    private void initToolWindow() {
        settingsWrap = fixedHeightWrap(buildSettingsCard(), SETTINGS_H);
        consoleWrap  = fixedHeightWrap(buildConsoleCard(),  CONSOLE_H);

        toolWindow = new JPanel();
        toolWindow.setLayout(new BoxLayout(toolWindow, BoxLayout.Y_AXIS));
        toolWindow.add(settingsWrap);
        toolWindow.add(consoleWrap);
    }

    /** Wraps {@code content} in a panel clamped to {@code h} pixels; starts hidden. */
    private static JPanel fixedHeightWrap(JComponent content, int h) {
        JPanel wrap = new JPanel(new BorderLayout(0, 0));
        wrap.add(content, BorderLayout.CENTER);
        Dimension d = new Dimension(Integer.MAX_VALUE, h);
        wrap.setPreferredSize(new Dimension(0, h));
        wrap.setMinimumSize(new Dimension(0, h));
        wrap.setMaximumSize(d);
        wrap.setVisible(false);
        return wrap;
    }

    // ── Toolbar ───────────────────────────────────────────────────────────────

    private JPanel buildToolbar() {
        JPanel bar = new JPanel(new BorderLayout(0, 0));
        bar.setBorder(new MatteBorder(0, 0, 1, 0, separatorColor()));

        JLabel title = new JLabel("  DroneLab");
        title.setFont(new Font("SansSerif", Font.BOLD, 14));
        title.setBorder(new EmptyBorder(0, 4, 0, 0));

        lblState = new JLabel("\u25CF IDLE");
        lblState.setFont(new Font("SansSerif", Font.BOLD, 11));

        btnStart   = toolbarBtn("\u25B6  Start");
        btnPause   = toolbarBtn("\u23F8  Pause");
        btnRestart = toolbarBtn("\u21BA  Restart");

        btnStart.setForeground(new Color(98, 188, 78));
        btnPause.setEnabled(false);
        btnRestart.setEnabled(false);

        btnStart.addActionListener(e   -> onStart());
        btnPause.addActionListener(e   -> onPause());
        btnRestart.addActionListener(e -> onRestart());

        JSeparator sep = new JSeparator(SwingConstants.VERTICAL);
        sep.setPreferredSize(new Dimension(1, 22));

        JPanel right = new JPanel(new FlowLayout(FlowLayout.RIGHT, 6, 5));
        right.setOpaque(false);
        right.add(lblState);
        right.add(sep);
        right.add(btnStart);
        right.add(btnPause);
        right.add(btnRestart);
        right.add(Box.createHorizontalStrut(6));

        bar.add(title, BorderLayout.WEST);
        bar.add(right, BorderLayout.EAST);
        return bar;
    }

    private static JButton toolbarBtn(String text) {
        JButton b = new JButton(text);
        b.putClientProperty("JButton.buttonType", "toolBarButton");
        b.setFocusPainted(false);
        b.setFont(new Font("SansSerif", Font.BOLD, 12));
        b.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        return b;
    }

    // ── Icon sidebar ──────────────────────────────────────────────────────────

    private JPanel buildIconSidebar() {
        JPanel sidebar = new JPanel();
        sidebar.setLayout(new BoxLayout(sidebar, BoxLayout.Y_AXIS));
        sidebar.setPreferredSize(new Dimension(40, 0));
        sidebar.setBorder(new MatteBorder(0, 0, 0, 1, separatorColor()));

        sidebar.add(Box.createVerticalStrut(6));

        btnIconSettings = sidebarBtn("\u2699", "Settings");
        // Use MouseListener for reliable event delivery on macOS full-screen
        // mousePressed fires on press regardless of mouse movement — matches IntelliJ toolbar behaviour
        btnIconSettings.addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1) toggleSettings();
            }
        });
        sidebar.add(btnIconSettings);

        sidebar.add(Box.createVerticalStrut(2));

        btnIconConsole = sidebarBtn("\u2261", "Run Log");
        btnIconConsole.addMouseListener(new MouseAdapter() {
            @Override public void mousePressed(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1) toggleConsole();
            }
        });
        sidebar.add(btnIconConsole);

        sidebar.add(Box.createVerticalGlue());
        return sidebar;
    }

    private static JButton sidebarBtn(String icon, String tooltip) {
        JButton b = new JButton(icon);
        b.setToolTipText(tooltip);
        b.putClientProperty("JButton.buttonType", "borderless");
        b.setFont(new Font("SansSerif", Font.PLAIN, 17));
        b.setFocusPainted(false);
        b.setPreferredSize(new Dimension(40, 40));
        b.setMaximumSize(new Dimension(40, 40));
        b.setAlignmentX(Component.CENTER_ALIGNMENT);
        b.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        return b;
    }

    // ── Tool window toggles ───────────────────────────────────────────────────
    // IntelliJ-style: instant setVisible — no timers, no preferredSize tricks.
    // Timers + revalidate are inherently racy and break on macOS full-screen.

    private void toggleSettings() {
        settingsOpen = !settingsOpen;
        settingsWrap.setVisible(settingsOpen);
        refreshSidebarHighlight();
    }

    private void toggleConsole() {
        consoleOpen = !consoleOpen;
        consoleWrap.setVisible(consoleOpen);
        refreshSidebarHighlight();
    }

    private void refreshSidebarHighlight() {
        Color inactive = UIManager.getColor("Label.foreground");
        if (inactive == null) inactive = new Color(187, 187, 187);
        btnIconSettings.setForeground(settingsOpen ? COL_ACCENT : inactive);
        btnIconConsole .setForeground(consoleOpen  ? COL_ACCENT : inactive);
    }

    // ── Settings card ─────────────────────────────────────────────────────────

    private JPanel buildSettingsCard() {
        JPanel card = new JPanel(new BorderLayout(0, 0));
        card.add(toolWindowTitleBar("Settings", true, this::toggleSettings), BorderLayout.NORTH);

        JPanel bar = new JPanel();
        bar.setLayout(new BoxLayout(bar, BoxLayout.X_AXIS));
        bar.setBorder(new EmptyBorder(6, 10, 6, 10));

        SimulationConfig d = SimulationConfig.defaults();

        // DRONE
        bar.add(buildSection("DRONE", form -> {
            sliderRange = compactSlider(20, 500, d.getDroneRange());
            lblRangeVal = compactValLbl(d.getDroneRange() + "");
            sliderRange.addChangeListener(e -> {
                int v = sliderRange.getValue();
                lblRangeVal.setText(v + "");
                if (activeRunner != null) activeRunner.setLiveRange(v);
            });
            addRow(form, "Range", sliderRange, lblRangeVal);

            spinDroneSpeed = compactSpinner(1, 10, d.getDroneSpeed(), 1);
            addRow(form, "Speed", spinDroneSpeed, mini("c/t"));

            spinPlacement = compactSpinner(1, 300, d.getDronePlacement(), 1);
            addRow(form, "Place", spinPlacement, null);
        }));

        bar.add(vSep());

        // ENVIRONMENT
        bar.add(buildSection("ENVIRONMENT", form -> {
            spinFieldW = wideSpinner(300, 10000, d.getFieldWidth(),  100);
            spinFieldH = wideSpinner(300, 10000, d.getFieldHeight(), 100);
            addRow(form, "Field W", spinFieldW, mini("cells"));
            addRow(form, "Field H", spinFieldH, mini("cells"));

            sliderNodes = compactSlider(5, 300, d.getNodeCount());
            lblNodesVal = compactValLbl(d.getNodeCount() + "");
            sliderNodes.addChangeListener(e -> lblNodesVal.setText(sliderNodes.getValue() + ""));
            addRow(form, "Nodes", sliderNodes, lblNodesVal);

            spinNodeMin = compactSpinner(1, 10, d.getNodeMinSpeed(), 1);
            spinNodeMin.setPreferredSize(new Dimension(52, 24));
            spinNodeMax = compactSpinner(1, 10, d.getNodeMaxSpeed(), 1);
            spinNodeMax.setPreferredSize(new Dimension(52, 24));
            JPanel sp = new JPanel(new FlowLayout(FlowLayout.LEFT, 2, 0));
            sp.setOpaque(false);
            JLabel dash = new JLabel(" \u2013 ");
            sp.add(spinNodeMin); sp.add(dash); sp.add(spinNodeMax);
            addRow(form, "Nd Speed", sp, null);
        }));

        bar.add(vSep());

        // TIMING
        bar.add(buildSection("TIMING", form -> {
            sliderDuration = compactSlider(500, 30000, (int) d.getDuration());
            lblDurVal      = compactValLbl((int) d.getDuration() + "");
            sliderDuration.addChangeListener(e -> lblDurVal.setText(sliderDuration.getValue() + ""));
            addRow(form, "Duration", sliderDuration, lblDurVal);
        }));

        bar.add(vSep());

        // SEED
        bar.add(buildSection("SEED", form -> {
            rbAuto  = new JRadioButton("Auto");
            rbFixed = new JRadioButton("Fixed:");
            ButtonGroup bg = new ButtonGroup();
            bg.add(rbAuto); bg.add(rbFixed);
            rbAuto.setSelected(true);
            rbAuto.setOpaque(false);
            rbFixed.setOpaque(false);

            tfSeed = new JTextField("42", 5);
            tfSeed.setEnabled(false);

            JButton roll = new JButton("Rnd");
            roll.setFont(new Font("SansSerif", Font.PLAIN, 10));
            roll.setFocusPainted(false);
            roll.addActionListener(e -> tfSeed.setText(Long.toString(Math.abs(new Random().nextLong()))));
            rbAuto.addActionListener(e  -> tfSeed.setEnabled(false));
            rbFixed.addActionListener(e -> { tfSeed.setEnabled(true); tfSeed.requestFocus(); });

            JPanel r1 = new JPanel(new FlowLayout(FlowLayout.LEFT, 3, 1));
            r1.setOpaque(false);
            r1.add(rbAuto); r1.add(rbFixed);

            JPanel r2 = new JPanel(new FlowLayout(FlowLayout.LEFT, 3, 1));
            r2.setOpaque(false);
            r2.add(tfSeed); r2.add(roll);

            JPanel inner = new JPanel();
            inner.setLayout(new BoxLayout(inner, BoxLayout.Y_AXIS));
            inner.setOpaque(false);
            inner.setBorder(new EmptyBorder(2, 4, 4, 4));
            r1.setAlignmentX(Component.LEFT_ALIGNMENT);
            r2.setAlignmentX(Component.LEFT_ALIGNMENT);
            inner.add(r1);
            inner.add(r2);
            replaceContent(form, inner);
        }));

        bar.add(vSep());

        // VIZ SPEED
        bar.add(buildSection("VIZ SPEED", form -> {
            speedSlider = compactSlider(0, 100, 20);
            speedSlider.addChangeListener(e -> {
                int v = speedSlider.getValue();
                tickDelay.set((v * v) / 50);
            });
            JPanel row = new JPanel(new BorderLayout(4, 0));
            row.setOpaque(false);
            row.setBorder(new EmptyBorder(8, 4, 6, 4));
            row.add(mini("Fast"), BorderLayout.WEST);
            row.add(speedSlider,  BorderLayout.CENTER);
            row.add(mini("Slow"), BorderLayout.EAST);
            replaceContent(form, row);
        }));

        bar.add(Box.createHorizontalGlue());

        JScrollPane scroll = new JScrollPane(bar,
                JScrollPane.VERTICAL_SCROLLBAR_NEVER,
                JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
        scroll.setBorder(null);
        card.add(scroll, BorderLayout.CENTER);
        return card;
    }

    // ── Console card ──────────────────────────────────────────────────────────

    private JPanel buildConsoleCard() {
        JPanel card = new JPanel(new BorderLayout(0, 0));
        card.add(toolWindowTitleBar("Run Log", false, this::toggleConsole), BorderLayout.NORTH);

        JScrollPane scroll = new JScrollPane(consoleArea);
        scroll.setBorder(null);
        card.add(scroll, BorderLayout.CENTER);
        return card;
    }

    // ── Tool-window title bar ─────────────────────────────────────────────────

    private JPanel toolWindowTitleBar(String title, boolean isSettings, Runnable onClose) {
        JPanel bar = new JPanel(new BorderLayout(0, 0));
        bar.setBorder(new MatteBorder(1, 0, 1, 0, separatorColor()));

        JLabel lbl = new JLabel("  " + title);
        lbl.setFont(new Font("SansSerif", Font.BOLD, 11));

        JPanel right = new JPanel(new FlowLayout(FlowLayout.RIGHT, 4, 2));
        right.setOpaque(false);

        if (!isSettings) {
            JButton clear = new JButton("Clear");
            clear.putClientProperty("JButton.buttonType", "borderless");
            clear.setFont(new Font("SansSerif", Font.PLAIN, 10));
            clear.setFocusPainted(false);
            clear.addActionListener(e -> consoleArea.setText(""));
            right.add(clear);
        }

        JButton close = new JButton("\u2715");
        close.putClientProperty("JButton.buttonType", "borderless");
        close.setFont(new Font("SansSerif", Font.PLAIN, 11));
        close.setFocusPainted(false);
        close.addActionListener(e -> onClose.run());
        right.add(close);

        bar.add(lbl,   BorderLayout.WEST);
        bar.add(right, BorderLayout.EAST);
        return bar;
    }

    // ── Sim area ──────────────────────────────────────────────────────────────

    private JPanel buildSimArea() {
        JPanel idle = new JPanel(new GridBagLayout());

        // Custom-painted play button: no FlatLaf hover box — just colour change on rollover
        final Color playNormal = new Color(100, 100, 110);
        final Color playHover  = COL_ACCENT;
        JButton idlePlay = new JButton() {
            @Override protected void paintComponent(Graphics g) {
                Graphics2D g2 = (Graphics2D) g.create();
                g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
                                    RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
                g2.setFont(getFont());
                g2.setColor(getModel().isRollover() ? playHover : playNormal);
                String s = "\u25B6";
                FontMetrics fm = g2.getFontMetrics();
                g2.drawString(s,
                        (getWidth()  - fm.stringWidth(s)) / 2,
                        (getHeight() + fm.getAscent() - fm.getDescent()) / 2);
                g2.dispose();
            }
        };
        idlePlay.setOpaque(false);
        idlePlay.setContentAreaFilled(false);
        idlePlay.setBorderPainted(false);
        idlePlay.setFocusPainted(false);
        idlePlay.setFont(new Font("SansSerif", Font.BOLD, 64));
        idlePlay.setPreferredSize(new Dimension(100, 100));
        idlePlay.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        idlePlay.setToolTipText("Start simulation");
        // Repaint on rollover state changes so the colour updates
        idlePlay.getModel().addChangeListener(e -> idlePlay.repaint());
        idlePlay.addActionListener(e -> onStart());
        idle.add(idlePlay);

        JPanel running = buildRunningPanel();

        simCards     = new CardLayout();
        simAreaCards = new JPanel(simCards);
        simAreaCards.add(idle,    "idle");
        simAreaCards.add(running, "sim");
        simCards.show(simAreaCards, "idle");

        JPanel area = new JPanel(new BorderLayout(0, 0));
        area.add(simAreaCards, BorderLayout.CENTER);
        return area;
    }

    private JPanel buildRunningPanel() {
        JPanel wrap = new JPanel(new BorderLayout(0, 0));
        wrap.setBackground(COL_SIM_BG);
        wrap.setBorder(new EmptyBorder(8, 8, 4, 8));

        JPanel grid = new JPanel(new GridLayout(1, passCount, 8, 0));
        grid.setBackground(COL_SIM_BG);
        for (int i = 0; i < passCount; i++) grid.add(buildPassCard(i));
        wrap.add(grid, BorderLayout.CENTER);
        return wrap;
    }

    private JPanel buildPassCard(int idx) {
        JPanel card = new JPanel(new BorderLayout(0, 0));
        card.setBorder(BorderFactory.createLineBorder(separatorColor(), 1));
        card.add(panelTitles[idx], BorderLayout.NORTH);
        card.add(panels[idx],      BorderLayout.CENTER);

        JPanel stats = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 3));
        stats.setOpaque(false);
        stats.setBorder(new MatteBorder(1, 0, 0, 0, separatorColor()));
        stats.add(statPair("Tick",     lblTick[idx]));
        stats.add(statPair("Tokens",   lblTokens[idx]));
        stats.add(statPair("Mode",     lblMode[idx]));
        stats.add(statPair("Coverage", lblCoverage[idx]));
        card.add(stats, BorderLayout.SOUTH);
        return card;
    }

    // ── Status bar ────────────────────────────────────────────────────────────

    private JPanel buildStatusBar() {
        JPanel p = new JPanel(new FlowLayout(FlowLayout.LEFT, 12, 4));
        p.setBorder(new MatteBorder(1, 0, 0, 0, separatorColor()));

        for (int i = 0; i < passCount; i++) {
            if (i > 0) {
                JLabel sep = new JLabel("|");
                sep.setForeground(separatorColor());
                p.add(sep);
            }
            JLabel key = new JLabel("Pass " + (i + 1) + ":");
            key.setFont(new Font("SansSerif", Font.PLAIN, 11));
            p.add(key);
            p.add(lblResults[i]);
        }
        JLabel sep = new JLabel("|");
        sep.setForeground(separatorColor());
        p.add(sep);
        p.add(lblStatus);
        return p;
    }

    // ── Actions ───────────────────────────────────────────────────────────────

    /** Slide both tool windows closed — called when simulation starts/restarts. */
    private void collapsePanels() {
        if (settingsOpen) toggleSettings();
        if (consoleOpen)  toggleConsole();
    }

    private void onStart() {
        if (runState == RunState.RUNNING || runState == RunState.PAUSED) return;
        paused.set(false);
        btnPause.setText("\u23F8  Pause");
        collapsePanels();
        startSimulation();
    }

    private void onPause() {
        if (runState == RunState.IDLE || runState == RunState.COMPLETE) return;
        boolean nowPaused = !paused.get();
        paused.set(nowPaused);
        btnPause.setText(nowPaused ? "\u25B6  Resume" : "\u23F8  Pause");
        applyState(nowPaused ? RunState.PAUSED : RunState.RUNNING);
    }

    private void onRestart() {
        paused.set(false);
        btnPause.setText("\u23F8  Pause");
        collapsePanels();
        resetPanelLabels();
        startSimulation();
    }

    // ── Simulation ────────────────────────────────────────────────────────────

    private void startSimulation() {
        SimulationConfig cfg;
        long seed;
        try {
            cfg  = buildSimConfig();
            seed = resolveSeed();
        } catch (IllegalArgumentException ex) {
            JOptionPane.showMessageDialog(frame, ex.getMessage(),
                    "Invalid Configuration", JOptionPane.WARNING_MESSAGE);
            return;
        }

        final int myGen = generation.incrementAndGet();
        applyState(RunState.RUNNING);

        rebuildPassCards();
        simCards.show(simAreaCards, "sim");
        frame.revalidate();

        final List<String> keys = appConfig.getIntelligenceKeys();
        final int n = keys.size();
        final List<UAVIntelligence> intelligences = appConfig.buildIntelligences(cfg);

        final long simStartMs = System.currentTimeMillis();
        System.out.println();
        System.out.println("  \u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550");
        System.out.println("  DroneLab  \u00B7  Simulation run");
        System.out.printf("  Algorithms : %s%n", String.join("  \u00B7  ", keys));
        System.out.printf("  Field      : %d \u00D7 %d cells%n",
                cfg.getFieldWidth(), cfg.getFieldHeight());
        System.out.printf("  Nodes      : %d  (speed %d \u2013 %d c/t)%n",
                cfg.getNodeCount(), cfg.getNodeMinSpeed(), cfg.getNodeMaxSpeed());
        System.out.printf("  UAV        : range %d  |  speed %d c/t  |  placement %d%n",
                cfg.getDroneRange(), cfg.getDroneSpeed(), cfg.getDronePlacement());
        System.out.printf("  Duration   : %d ticks  |  seed: %s%n",
                cfg.getDuration(), seed == AppConfig.NO_SEED ? "auto" : seed);
        System.out.println("  \u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550");

        SwingUtilities.invokeLater(() ->
                lblStatus.setText("Running \u2014 " + n + " pass" + (n == 1 ? "" : "es")
                        + " in progress\u2026"));

        final SimulationRunner runner = new SimulationRunner(cfg);
        runner.withPlannerFactory(scenario.getPlannerFactory());
        runner.withMovementStrategyFactory(scenario::getMovementStrategy);
        runner.withDeploymentStrategy(scenario.getDeploymentStrategy());
        runner.setLiveRange(cfg.getDroneRange());
        if (seed != AppConfig.NO_SEED) runner.withSeed(seed);
        activeRunner = runner;

        final List<VisualizationListener> listeners = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            final int idx = i;
            listeners.add(snap -> {
                int d = tickDelay.get();
                if (d > 0) try { Thread.sleep(d); } catch (InterruptedException ignored) {}
                while (paused.get()) try { Thread.sleep(50); } catch (InterruptedException ignored) {}
                if (generation.get() != myGen) return;
                SwingUtilities.invokeLater(() -> {
                    if (generation.get() != myGen) return;
                    if (idx < panelTitles.length)
                        panelTitles[idx].setText(snap.passLabel.toUpperCase());
                    if (idx < panels.length) panels[idx].setSnapshot(snap);
                    if (idx < lblTick.length) {
                        long pct = snap.duration > 0 ? (long)(100.0 * snap.tick / snap.duration) : 0;
                        lblTick[idx].setText(snap.tick + " / " + snap.duration + "  (" + pct + "%)");
                        int tokPct = snap.totalNodes > 0 ? (int)(100.0 * snap.tokens / snap.totalNodes) : 0;
                        lblTokens[idx].setText(snap.tokens + " / " + snap.totalNodes + "  (" + tokPct + "%)");
                        lblMode[idx].setText(snap.uavMode.name());
                        lblMode[idx].setForeground(modeColor(snap.uavMode));
                        if (!Double.isNaN(snap.coverageFraction))
                            lblCoverage[idx].setText(String.format("%.1f%%", snap.coverageFraction * 100));
                    }
                });
            });
        }

        // Accumulate per-pass results for the final summary table
        final String[][] passData = new String[n][];
        final AtomicInteger completedCount = new AtomicInteger(0);

        final SimulationRunner.PassCompleteCallback onComplete = (pass, tok, total, cov) -> {
            if (generation.get() != myGen) return;
            int    idx        = pass - 1;
            String algo       = (idx >= 0 && idx < keys.size()) ? keys.get(idx).toUpperCase() : "PASS " + pass;
            int    pct        = total > 0 ? (int)(100.0 * tok / total) : 0;
            double efficiency = cfg.getDuration() > 0 ? (double) tok / cfg.getDuration() : 0;
            String covStr     = Double.isNaN(cov) ? "\u2013" : String.format("%.1f%%", cov * 100);

            System.out.printf("  [%-10s]  tokens %d/%d (%d%%)%s  \u00B7  %.5f tok/tick%n",
                    algo, tok, total, pct,
                    Double.isNaN(cov) ? "" : "  \u00B7  coverage " + covStr,
                    efficiency);

            if (idx >= 0 && idx < n)
                passData[idx] = new String[]{ algo,
                        tok + " / " + total, pct + "%", covStr,
                        String.format("%.5f", efficiency) };

            int done = completedCount.incrementAndGet();
            if (done >= n) {
                long elapsed = System.currentTimeMillis() - simStartMs;
                System.out.println();
                System.out.println("  \u2500\u2500 Results " + "\u2500".repeat(47));
                System.out.printf("  %-14s %-14s %-8s %-12s %s%n",
                        "Algorithm", "Tokens", "Pct", "Coverage", "Efficiency");
                System.out.println("  " + "\u2500".repeat(55));
                for (String[] row : passData)
                    if (row != null)
                        System.out.printf("  %-14s %-14s %-8s %-12s %s tok/tick%n",
                                row[0], row[1], row[2], row[3], row[4]);
                System.out.println("  " + "\u2500".repeat(55));
                System.out.printf("  Completed in %.1f s%n%n", elapsed / 1000.0);
            }

            String res = String.format("%d / %d  (%d%%)", tok, total, pct);
            SwingUtilities.invokeLater(() -> {
                if (generation.get() != myGen) return;
                if (idx >= 0 && idx < passCount) {
                    lblResults[idx].setText(res);
                    lblResults[idx].setForeground(passColor(idx));
                }
                int uiDone = countDone();
                if (uiDone >= n) {
                    applyState(RunState.COMPLETE);
                    lblStatus.setText("All " + n + " passes complete. "
                            + "Adjust settings and click \u21BA Restart.");
                } else {
                    lblStatus.setText(uiDone + " / " + n + " passes complete\u2026");
                }
            });
        };

        Thread t = new Thread(() ->
                runner.runInParallel(intelligences, listeners, onComplete, false),
                "sim-runner");
        t.setDaemon(true);
        t.start();
    }

    // ── State management ──────────────────────────────────────────────────────

    private void applyState(RunState s) {
        runState = s;
        SwingUtilities.invokeLater(() -> {
            switch (s) {
                case IDLE:
                    btnStart.setEnabled(true);
                    btnStart.setForeground(new Color(98, 188, 78));
                    btnPause.setEnabled(false);
                    btnRestart.setEnabled(false);
                    lblState.setText("\u25CF IDLE");
                    lblState.setForeground(UIManager.getColor("Label.disabledForeground"));
                    break;
                case RUNNING:
                    btnStart.setEnabled(false);
                    btnPause.setEnabled(true);
                    btnRestart.setEnabled(true);
                    lblState.setText("\u25CF RUNNING");
                    lblState.setForeground(new Color(98, 188, 78));
                    break;
                case PAUSED:
                    btnStart.setEnabled(false);
                    btnPause.setEnabled(true);
                    btnRestart.setEnabled(true);
                    lblState.setText("\u23F8 PAUSED");
                    lblState.setForeground(new Color(230, 145, 50));
                    break;
                case COMPLETE:
                    btnStart.setEnabled(true);
                    btnStart.setForeground(new Color(98, 188, 78));
                    btnPause.setEnabled(false);
                    btnRestart.setEnabled(true);
                    lblState.setText("\u2713 COMPLETE");
                    lblState.setForeground(new Color(100, 200, 255));
                    break;
            }
        });
    }

    private void resetPanelLabels() {
        List<String> keys = appConfig.getIntelligenceKeys();
        for (int i = 0; i < passCount; i++) {
            panels[i].clearTrail();
            panels[i].setSnapshot(null);
            String algoLabel = (i < keys.size()) ? keys.get(i) : "Pass " + (i + 1);
            panelTitles[i].setText(algoLabel.toUpperCase());
            lblResults[i].setText("\u2013");
            lblTick[i].setText("\u2013");
            lblTokens[i].setText("\u2013");
            lblMode[i].setText("\u2013");
            lblCoverage[i].setText("\u2013");
        }
        lblStatus.setText("Restarting\u2026");
    }

    private void rebuildPassCards() {
        for (SimulationPanel panel : panels) {
            panel.clearTrail();
            panel.setSnapshot(null);
        }
    }

    private int countDone() {
        int c = 0;
        for (JLabel l : lblResults) if (!"\u2013".equals(l.getText())) c++;
        return c;
    }

    // ── Config / seed ─────────────────────────────────────────────────────────

    private SimulationConfig buildSimConfig() {
        int  fw  = (Integer) spinFieldW.getValue();
        int  fh  = (Integer) spinFieldH.getValue();
        int  nc  = sliderNodes.getValue();
        int  nms = (Integer) spinNodeMin.getValue();
        int  nmx = (Integer) spinNodeMax.getValue();
        long dur = sliderDuration.getValue();
        int  dr  = sliderRange.getValue();
        int  ds  = (Integer) spinDroneSpeed.getValue();
        int  dp  = (Integer) spinPlacement.getValue();

        if (nms > nmx)
            throw new IllegalArgumentException(
                    "Node min speed (" + nms + ") must be \u2264 max speed (" + nmx + ").");
        if (dp * 2 >= Math.min(fw, fh))
            throw new IllegalArgumentException(
                    "Placement (" + dp + ") too large for "
                    + fw + "\u00D7" + fh + " field.");

        SimulationConfig base = SimulationConfig.defaults();
        return new SimulationConfig(fw, fh, nc, base.getNodeIdStart(),
                base.getNodeRange(), nms, nmx, dr, dp, ds, dur);
    }

    private long resolveSeed() {
        if (rbAuto.isSelected()) return AppConfig.NO_SEED;
        String raw = tfSeed.getText().trim();
        if (raw.isEmpty()) return AppConfig.NO_SEED;
        try {
            return Long.parseLong(raw);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(
                    "Seed must be a valid integer, got: \"" + raw + "\"");
        }
    }

    // ── Utility ───────────────────────────────────────────────────────────────

    private static Color separatorColor() {
        Color c = UIManager.getColor("Separator.foreground");
        return c != null ? c : new Color(80, 82, 85);
    }

    private static Color modeColor(UAV.DriveMode mode) {
        switch (mode) {
            case EXECUTE:     return new Color(  0, 220, 255);
            case RETURN_HOME: return new Color(255, 100,  80);
            default:          return new Color(170, 170, 215);
        }
    }

    private static Color passColor(int i) {
        return PASS_COLORS[i % PASS_COLORS.length];
    }

    // ── Section builder ───────────────────────────────────────────────────────

    private static JPanel buildSection(String title, Consumer<JPanel> populate) {
        JPanel outer = new JPanel(new BorderLayout(0, 0));
        outer.setOpaque(false);

        JLabel hdr = new JLabel("  " + title);
        hdr.setFont(new Font("SansSerif", Font.BOLD, 9));
        Color dis = UIManager.getColor("Label.disabledForeground");
        hdr.setForeground(dis != null ? dis : new Color(140, 140, 150));
        hdr.setBorder(new EmptyBorder(4, 4, 2, 4));

        JPanel form = new JPanel();
        form.setLayout(new BoxLayout(form, BoxLayout.Y_AXIS));
        form.setOpaque(false);
        form.setName("form");
        form.setBorder(new EmptyBorder(2, 2, 2, 14));

        outer.add(hdr,  BorderLayout.NORTH);
        outer.add(form, BorderLayout.CENTER);

        populate.accept(outer);
        return outer;
    }

    private static void addRow(JPanel section, String label, JComponent ctrl, JComponent unit) {
        JPanel form = findForm(section);
        if (form == null) return;

        JPanel row = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 2));
        row.setOpaque(false);
        row.setAlignmentX(Component.LEFT_ALIGNMENT);

        JLabel lbl = new JLabel(label + ":");
        lbl.setFont(new Font("SansSerif", Font.PLAIN, 10));
        lbl.setPreferredSize(new Dimension(58, 16));

        row.add(lbl);
        row.add(ctrl);
        if (unit != null) row.add(unit);
        form.add(row);
    }

    private static void replaceContent(JPanel section, JComponent child) {
        JPanel form = findForm(section);
        if (form != null) section.remove(form);
        child.setOpaque(false);
        section.add(child, BorderLayout.CENTER);
    }

    private static JPanel findForm(JPanel container) {
        for (Component c : container.getComponents())
            if (c instanceof JPanel && "form".equals(((JPanel) c).getName()))
                return (JPanel) c;
        return null;
    }

    // ── Widget factories ──────────────────────────────────────────────────────

    private static JSlider compactSlider(int min, int max, int value) {
        JSlider s = new JSlider(min, max, Math.min(Math.max(value, min), max));
        s.setOpaque(false);
        s.setPaintTicks(false);
        s.setPaintLabels(false);
        s.setPreferredSize(new Dimension(95, 20));
        return s;
    }

    private static JSpinner compactSpinner(int min, int max, int value, int step) {
        return new JSpinner(new SpinnerNumberModel(
                Math.min(Math.max(value, min), max), min, max, step)) {{
            setPreferredSize(new Dimension(62, 24));
        }};
    }

    private static JSpinner wideSpinner(int min, int max, int value, int step) {
        return new JSpinner(new SpinnerNumberModel(
                Math.min(Math.max(value, min), max), min, max, step)) {{
            setPreferredSize(new Dimension(78, 24));
        }};
    }

    private static JLabel compactValLbl(String text) {
        JLabel l = new JLabel(text);
        l.setFont(new Font("Monospaced", Font.BOLD, 10));
        l.setPreferredSize(new Dimension(42, 16));
        return l;
    }

    private static JLabel mini(String t) {
        JLabel l = new JLabel(t);
        l.setFont(new Font("SansSerif", Font.PLAIN, 9));
        return l;
    }

    private static Component vSep() {
        JSeparator s = new JSeparator(SwingConstants.VERTICAL);
        s.setMaximumSize(new Dimension(1, Integer.MAX_VALUE));
        s.setPreferredSize(new Dimension(1, 40));
        return s;
    }

    private static JLabel makePanelTitle(String text, Color accent) {
        JLabel l = new JLabel("  " + text);
        l.setForeground(accent);
        l.setFont(new Font("SansSerif", Font.BOLD, 11));
        l.setOpaque(true);
        // Derive title bg from Darcula panel colour, blended subtly with the pass accent
        Color base = UIManager.getColor("Panel.background");
        if (base == null) base = new Color(60, 63, 65);
        l.setBackground(new Color(
                (base.getRed()   * 3 + accent.getRed())   / 4,
                (base.getGreen() * 3 + accent.getGreen()) / 4,
                (base.getBlue()  * 3 + accent.getBlue())  / 4));
        l.setBorder(new EmptyBorder(5, 8, 5, 8));
        return l;
    }

    private static JPanel statPair(String key, JLabel val) {
        JPanel row = new JPanel(new FlowLayout(FlowLayout.LEFT, 3, 0));
        row.setOpaque(false);
        JLabel k = new JLabel(key + ":");
        Color dim = UIManager.getColor("Label.disabledForeground");
        k.setForeground(dim != null ? dim : new Color(128, 128, 128));
        k.setFont(new Font("SansSerif", Font.PLAIN, 9));
        row.add(k);
        row.add(val);
        return row;
    }

    private static JLabel makeStatVal(String t) {
        JLabel l = new JLabel(t);
        l.setFont(new Font("Monospaced", Font.PLAIN, 9));
        return l;
    }

    private static JLabel makeResultVal(String t) {
        JLabel l = new JLabel(t);
        l.setFont(new Font("Monospaced", Font.BOLD, 11));
        return l;
    }

    // ── TeeOutputStream ───────────────────────────────────────────────────────

    private static final class TeeOutputStream extends OutputStream {
        private final PrintStream original;
        private final JTextArea   area;

        TeeOutputStream(PrintStream original, JTextArea area) {
            this.original = original;
            this.area     = area;
        }

        @Override public void write(int b) {
            original.write(b);
            String s = String.valueOf((char) b);
            SwingUtilities.invokeLater(() -> {
                area.append(s);
                area.setCaretPosition(area.getDocument().getLength());
            });
        }

        @Override public void write(byte[] buf, int off, int len) {
            original.write(buf, off, len);
            String s = new String(buf, off, len);
            SwingUtilities.invokeLater(() -> {
                area.append(s);
                area.setCaretPosition(area.getDocument().getLength());
            });
        }
    }
}
