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

import javax.swing.*;
import javax.swing.border.*;
import java.awt.*;
import java.util.Random;

/**
 * Professional modal launch dialog shown before every simulation run.
 *
 * <p>Provides sliders and spinners for all key simulation parameters so users
 * can configure each run without editing {@code config.properties}.  Includes
 * a <em>Sync Mode</em> toggle for frame-by-frame research comparison.
 *
 * <h3>Usage (from any thread)</h3>
 * <pre>
 *   RunConfig cfg = LaunchDialog.show(SimulationConfig.defaults());
 *   if (cfg == null) System.exit(0);   // user cancelled
 * </pre>
 */
public final class LaunchDialog extends JDialog {

    // ── Palette (matches SimulationGui) ───────────────────────────────────────
    private static final Color COL_BG      = new Color(10,  10,  22);
    private static final Color COL_CARD    = new Color(16,  16,  34);
    private static final Color COL_BORDER  = new Color(38,  38,  68);
    private static final Color COL_TEXT    = new Color(200, 200, 225);
    private static final Color COL_LABEL   = new Color(110, 110, 155);
    private static final Color COL_ACCENT  = new Color(110, 185, 255);
    private static final Color COL_VAL     = new Color(220, 220, 255);
    private static final Color COL_GREEN   = new Color(  0, 200, 110);

    // ── Result ────────────────────────────────────────────────────────────────
    private RunConfig result = null;

    // ── Form controls ─────────────────────────────────────────────────────────
    private JSlider  sliderDroneRange;
    private JSlider  sliderNodeCount;
    private JSlider  sliderDuration;
    private JSpinner spinnerFieldW;
    private JSpinner spinnerFieldH;
    private JSpinner spinnerDronePlacement;
    private JSpinner spinnerDroneSpeed;
    private JSpinner spinnerNodeMinSpeed;
    private JSpinner spinnerNodeMaxSpeed;

    // ── Live value labels ─────────────────────────────────────────────────────
    private JLabel lblDroneRangeVal;
    private JLabel lblNodeCountVal;
    private JLabel lblDurationVal;

    // ── Mode controls ─────────────────────────────────────────────────────────
    private JCheckBox    cbSyncMode;
    private JRadioButton rbSeedAuto;
    private JRadioButton rbSeedFixed;
    private JTextField   tfSeed;

    private final SimulationConfig defaults;

    // ── Constructor ───────────────────────────────────────────────────────────

    private LaunchDialog(Frame owner, SimulationConfig defaults) {
        super(owner, "\uD83D\uDE81  Launch Simulation", true);
        this.defaults = defaults;
        setDefaultCloseOperation(DISPOSE_ON_CLOSE);
        buildUI();
        pack();
        setMinimumSize(new Dimension(580, getHeight()));
        setResizable(false);
        setLocationRelativeTo(owner);
    }

    // ── Static factory (thread-safe) ──────────────────────────────────────────

    /**
     * Shows the launch dialog on the EDT and blocks until the user dismisses it.
     * Safe to call from any thread.
     *
     * @param defaults pre-populate controls; {@code null} uses {@link SimulationConfig#defaults()}
     * @return the chosen {@link RunConfig}, or {@code null} if cancelled
     */
    public static RunConfig show(SimulationConfig defaults) {
        RunConfig[] ref = { null };
        Runnable doShow = () -> {
            SimulationConfig cfg = (defaults != null) ? defaults : SimulationConfig.defaults();
            LaunchDialog dlg = new LaunchDialog(null, cfg);
            dlg.setVisible(true);   // blocks until disposed (modal)
            ref[0] = dlg.result;
        };
        if (SwingUtilities.isEventDispatchThread()) {
            doShow.run();
        } else {
            try {
                SwingUtilities.invokeAndWait(doShow);
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            }
        }
        return ref[0];
    }

    // ── UI Construction ───────────────────────────────────────────────────────

    private void buildUI() {
        JPanel root = new JPanel();
        root.setBackground(COL_BG);
        root.setLayout(new BoxLayout(root, BoxLayout.Y_AXIS));

        root.add(buildTitleBar());
        root.add(wrap(buildDroneSection(),       12, 18, 4,  18));
        root.add(wrap(buildEnvironmentSection(), 0,  18, 4,  18));
        root.add(wrap(buildTimingSection(),      0,  18, 4,  18));
        root.add(wrap(buildSyncSection(),        0,  18, 4,  18));
        root.add(wrap(buildSeedSection(),        0,  18, 12, 18));
        root.add(buildButtonBar());

        setContentPane(root);
        getContentPane().setBackground(COL_BG);
    }

    /** Wraps a component with empty border padding. */
    private static JPanel wrap(JComponent c, int top, int left, int bot, int right) {
        JPanel p = new JPanel(new BorderLayout());
        p.setBackground(COL_BG);
        p.setBorder(new EmptyBorder(top, left, bot, right));
        p.add(c, BorderLayout.CENTER);
        return p;
    }

    // ── Section builders ──────────────────────────────────────────────────────

    private JPanel buildTitleBar() {
        JPanel p = new JPanel();
        p.setBackground(new Color(14, 14, 30));
        p.setLayout(new BoxLayout(p, BoxLayout.Y_AXIS));
        p.setBorder(new EmptyBorder(18, 24, 14, 24));

        JLabel title = new JLabel("\uD83D\uDE81  Launch Simulation");
        title.setForeground(COL_ACCENT);
        title.setFont(new Font("SansSerif", Font.BOLD, 20));
        title.setAlignmentX(Component.CENTER_ALIGNMENT);
        p.add(title);

        p.add(Box.createVerticalStrut(4));

        JLabel sub = new JLabel("Configure drone and environment parameters for this run");
        sub.setForeground(COL_LABEL);
        sub.setFont(new Font("SansSerif", Font.PLAIN, 12));
        sub.setAlignmentX(Component.CENTER_ALIGNMENT);
        p.add(sub);

        p.add(Box.createVerticalStrut(12));
        p.add(hRule());
        return p;
    }

    private JPanel buildDroneSection() {
        JPanel card = makeCard("DRONE SETTINGS", new Color(60, 130, 210));

        // Scan range
        sliderDroneRange = makeSlider(20, 500, defaults.getDroneRange());
        lblDroneRangeVal = makeValLabel(defaults.getDroneRange() + " cells");
        sliderDroneRange.addChangeListener(e ->
                lblDroneRangeVal.setText(sliderDroneRange.getValue() + " cells"));
        addRow(card, "Scan Range",     sliderDroneRange,     lblDroneRangeVal);

        // Drone speed
        spinnerDroneSpeed = makeIntSpinner(1, 10, defaults.getDroneSpeed(), 1);
        addRow(card, "Speed",          spinnerDroneSpeed,    makeUnitLabel("cells / tick"));

        // Home placement
        spinnerDronePlacement = makeIntSpinner(1, 300, defaults.getDronePlacement(), 1);
        addRow(card, "Home Placement", spinnerDronePlacement, makeUnitLabel("cells from corner"));

        return card;
    }

    private JPanel buildEnvironmentSection() {
        JPanel card = makeCard("ENVIRONMENT", new Color(0, 160, 90));

        spinnerFieldW = makeIntSpinner(300, 10000, defaults.getFieldWidth(),  100);
        spinnerFieldH = makeIntSpinner(300, 10000, defaults.getFieldHeight(), 100);
        addRow(card, "Field Width",  spinnerFieldW, makeUnitLabel("cells"));
        addRow(card, "Field Height", spinnerFieldH, makeUnitLabel("cells"));

        sliderNodeCount = makeSlider(5, 300, defaults.getNodeCount());
        lblNodeCountVal = makeValLabel(defaults.getNodeCount() + " nodes");
        sliderNodeCount.addChangeListener(e ->
                lblNodeCountVal.setText(sliderNodeCount.getValue() + " nodes"));
        addRow(card, "Node Count", sliderNodeCount, lblNodeCountVal);

        spinnerNodeMinSpeed = makeIntSpinner(1, 10, defaults.getNodeMinSpeed(), 1);
        spinnerNodeMaxSpeed = makeIntSpinner(1, 10, defaults.getNodeMaxSpeed(), 1);
        JPanel speedRange = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 0));
        speedRange.setBackground(COL_CARD);
        JLabel dash = new JLabel(" \u2013 ");
        dash.setForeground(COL_LABEL);
        speedRange.add(spinnerNodeMinSpeed);
        speedRange.add(dash);
        speedRange.add(spinnerNodeMaxSpeed);
        speedRange.add(makeUnitLabel(" cells / tick"));
        addRow(card, "Node Speed", speedRange, null);

        return card;
    }

    private JPanel buildTimingSection() {
        JPanel card = makeCard("TIMING", new Color(200, 140, 30));
        sliderDuration = makeSlider(500, 30000, (int) defaults.getDuration());
        lblDurationVal = makeValLabel(defaults.getDuration() + " ticks");
        sliderDuration.addChangeListener(e ->
                lblDurationVal.setText(sliderDuration.getValue() + " ticks"));
        addRow(card, "Duration", sliderDuration, lblDurationVal);
        return card;
    }

    private JPanel buildSyncSection() {
        JPanel card = makeCard("COMPARISON MODE", new Color(160, 70, 220));

        cbSyncMode = new JCheckBox(
                " Sync Mode \u2014 passes advance tick-by-tick in lock-step");
        cbSyncMode.setBackground(COL_CARD);
        cbSyncMode.setForeground(COL_VAL);
        cbSyncMode.setFont(new Font("SansSerif", Font.BOLD, 12));
        cbSyncMode.setFocusPainted(false);
        cbSyncMode.setSelected(false);

        JLabel desc = new JLabel(
                "<html><span style='color:#6060a0; font-size:10px'>" +
                "When enabled, every pass waits for all others before advancing to the next tick, " +
                "enabling direct frame-by-frame comparison in the visualiser.</span></html>");
        desc.setBorder(new EmptyBorder(2, 24, 0, 0));

        JPanel inner = new JPanel();
        inner.setBackground(COL_CARD);
        inner.setLayout(new BoxLayout(inner, BoxLayout.Y_AXIS));
        inner.setBorder(new EmptyBorder(4, 8, 6, 8));
        inner.add(cbSyncMode);
        inner.add(desc);

        JPanel wrap = new JPanel(new BorderLayout());
        wrap.setBackground(COL_CARD);
        wrap.add(inner, BorderLayout.CENTER);
        // Append to card's form area
        appendToCard(card, wrap);

        return card;
    }

    private JPanel buildSeedSection() {
        JPanel card = makeCard("RANDOM SEED", new Color(170, 90, 40));

        rbSeedAuto  = makeRadio("Auto (random each run)");
        rbSeedFixed = makeRadio("Fixed seed:");
        ButtonGroup bg = new ButtonGroup();
        bg.add(rbSeedAuto);
        bg.add(rbSeedFixed);
        rbSeedAuto.setSelected(true);

        tfSeed = new JTextField("42", 10);
        styleTf(tfSeed);
        tfSeed.setEnabled(false);

        JButton btnRoll = makeSmallBtn("\uD83C\uDFB2 Roll");
        btnRoll.addActionListener(e -> tfSeed.setText(
                Long.toString(Math.abs(new Random().nextLong()))));

        rbSeedAuto.addActionListener(e  -> tfSeed.setEnabled(false));
        rbSeedFixed.addActionListener(e -> { tfSeed.setEnabled(true); tfSeed.requestFocus(); });

        JPanel inner = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 2));
        inner.setBackground(COL_CARD);
        inner.setBorder(new EmptyBorder(4, 6, 6, 6));
        inner.add(rbSeedAuto);
        inner.add(rbSeedFixed);
        inner.add(tfSeed);
        inner.add(btnRoll);

        appendToCard(card, inner);
        return card;
    }

    private JPanel buildButtonBar() {
        JPanel bar = new JPanel(new BorderLayout());
        bar.setBackground(new Color(12, 12, 26));
        bar.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createMatteBorder(1, 0, 0, 0, COL_BORDER),
                new EmptyBorder(10, 18, 14, 18)));

        JButton btnReset  = makeSmallBtn("Reset to Defaults");
        JButton btnCancel = makeSmallBtn("Cancel");
        JButton btnLaunch = makePrimaryBtn("\uD83D\uDE80  Launch");

        btnReset.addActionListener(e  -> resetToDefaults());
        btnCancel.addActionListener(e -> { result = null; dispose(); });
        btnLaunch.addActionListener(e -> onLaunch());

        JPanel left  = new JPanel(new FlowLayout(FlowLayout.LEFT,  0, 0));
        left.setOpaque(false);
        left.add(btnReset);

        JPanel right = new JPanel(new FlowLayout(FlowLayout.RIGHT, 6, 0));
        right.setOpaque(false);
        right.add(btnCancel);
        right.add(btnLaunch);

        bar.add(left,  BorderLayout.WEST);
        bar.add(right, BorderLayout.EAST);
        return bar;
    }

    // ── Actions ───────────────────────────────────────────────────────────────

    private void onLaunch() {
        try {
            result = new RunConfig(buildConfig(), cbSyncMode.isSelected(), resolveSeed());
            dispose();
        } catch (IllegalArgumentException ex) {
            JOptionPane.showMessageDialog(this, ex.getMessage(),
                    "Invalid Configuration", JOptionPane.WARNING_MESSAGE);
        }
    }

    private void resetToDefaults() {
        SimulationConfig d = SimulationConfig.defaults();
        sliderDroneRange.setValue(d.getDroneRange());
        spinnerDroneSpeed.setValue(d.getDroneSpeed());
        spinnerDronePlacement.setValue(d.getDronePlacement());
        spinnerFieldW.setValue(d.getFieldWidth());
        spinnerFieldH.setValue(d.getFieldHeight());
        sliderNodeCount.setValue(d.getNodeCount());
        spinnerNodeMinSpeed.setValue(d.getNodeMinSpeed());
        spinnerNodeMaxSpeed.setValue(d.getNodeMaxSpeed());
        sliderDuration.setValue((int) d.getDuration());
        cbSyncMode.setSelected(false);
        rbSeedAuto.setSelected(true);
        tfSeed.setEnabled(false);
        tfSeed.setText("42");
    }

    private SimulationConfig buildConfig() {
        int  fw  = intOf(spinnerFieldW);
        int  fh  = intOf(spinnerFieldH);
        int  nc  = sliderNodeCount.getValue();
        int  dr  = sliderDroneRange.getValue();
        int  ds  = intOf(spinnerDroneSpeed);
        int  dp  = intOf(spinnerDronePlacement);
        int  nms = intOf(spinnerNodeMinSpeed);
        int  nmx = intOf(spinnerNodeMaxSpeed);
        long dur = sliderDuration.getValue();

        if (dp * 2 >= Math.min(fw, fh))
            throw new IllegalArgumentException(
                    "Home placement (" + dp + ") is too large for the chosen field size (" +
                    fw + " \u00D7 " + fh + "). Must be less than half of each dimension.");
        if (nms > nmx)
            throw new IllegalArgumentException(
                    "Node min speed (" + nms + ") must be \u2264 max speed (" + nmx + ").");

        SimulationConfig base = SimulationConfig.defaults();
        return new SimulationConfig(fw, fh, nc,
                base.getNodeIdStart(),
                base.getNodeRange(),
                nms, nmx, dr, dp, ds, dur);
    }

    private long resolveSeed() {
        if (rbSeedAuto.isSelected()) return AppConfig.NO_SEED;
        String raw = tfSeed.getText().trim();
        if (raw.isEmpty()) return AppConfig.NO_SEED;
        try {
            return Long.parseLong(raw);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(
                    "Seed must be a valid integer, got: \"" + raw + "\"");
        }
    }

    // ── Card helpers ──────────────────────────────────────────────────────────

    /**
     * Creates a titled card panel.  Use {@link #addRow} to append
     * label/control rows, or {@link #appendToCard} for free-form content.
     */
    private static JPanel makeCard(String title, Color accent) {
        JPanel card = new JPanel(new BorderLayout(0, 0));
        card.setBackground(COL_CARD);
        card.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(new Color(
                        clamp(accent.getRed()   / 4),
                        clamp(accent.getGreen() / 4),
                        clamp(accent.getBlue()  / 4)), 1),
                BorderFactory.createEmptyBorder(0, 0, 0, 0)));

        JLabel header = new JLabel("  " + title);
        header.setFont(new Font("SansSerif", Font.BOLD, 10));
        header.setForeground(accent.brighter());
        header.setOpaque(true);
        header.setBackground(new Color(
                clamp(accent.getRed()   / 6 + 8),
                clamp(accent.getGreen() / 6 + 8),
                clamp(accent.getBlue()  / 6 + 8)));
        header.setBorder(new EmptyBorder(4, 8, 4, 8));
        card.add(header, BorderLayout.NORTH);

        // A form panel at CENTER (appended rows go here)
        JPanel form = new JPanel(new GridBagLayout());
        form.setBackground(COL_CARD);
        form.setBorder(new EmptyBorder(6, 12, 8, 12));
        form.setName("form");
        card.add(form, BorderLayout.CENTER);

        return card;
    }

    /** Appends a label + control + optional value/unit label row to a card. */
    private static void addRow(JPanel card, String labelText,
                                JComponent control, JComponent valueLabel) {
        JPanel form = findForm(card);
        int row = form.getComponentCount() == 0 ? 0 : maxGridY(form) + 1;

        GridBagConstraints gbc = new GridBagConstraints();
        gbc.insets  = new Insets(3, 0, 3, 6);
        gbc.anchor  = GridBagConstraints.WEST;

        JLabel lbl = new JLabel(labelText + ":");
        lbl.setForeground(COL_LABEL);
        lbl.setFont(new Font("SansSerif", Font.PLAIN, 11));
        lbl.setPreferredSize(new Dimension(130, 22));

        gbc.gridx = 0; gbc.gridy = row; gbc.weightx = 0; gbc.fill = GridBagConstraints.NONE;
        form.add(lbl, gbc);

        gbc.gridx = 1; gbc.weightx = 1.0; gbc.fill = GridBagConstraints.HORIZONTAL;
        form.add(control, gbc);

        if (valueLabel != null) {
            gbc.gridx   = 2; gbc.weightx = 0; gbc.fill = GridBagConstraints.NONE;
            gbc.insets  = new Insets(3, 8, 3, 0);
            form.add(valueLabel, gbc);
        }
    }

    /** Replaces the card's CENTER component with {@code component}. */
    private static void appendToCard(JPanel card, JComponent component) {
        card.remove(findForm(card));
        component.setBackground(COL_CARD);
        card.add(component, BorderLayout.CENTER);
    }

    private static JPanel findForm(JPanel card) {
        for (Component c : card.getComponents()) {
            if (c instanceof JPanel && "form".equals(((JPanel) c).getName()))
                return (JPanel) c;
        }
        return (JPanel) card.getComponent(card.getComponentCount() - 1);
    }

    private static int maxGridY(JPanel form) {
        int max = -1;
        GridBagLayout gbl = (GridBagLayout) form.getLayout();
        for (Component c : form.getComponents()) {
            int gy = gbl.getConstraints(c).gridy;
            if (gy > max) max = gy;
        }
        return max;
    }

    // ── Widget factories ──────────────────────────────────────────────────────

    private static JSlider makeSlider(int min, int max, int value) {
        JSlider s = new JSlider(min, max, Math.min(Math.max(value, min), max));
        s.setBackground(COL_CARD);
        s.setForeground(COL_TEXT);
        s.setPaintTicks(false);
        s.setPaintLabels(false);
        s.setPreferredSize(new Dimension(220, 24));
        return s;
    }

    private static JSpinner makeIntSpinner(int min, int max, int value, int step) {
        JSpinner s = new JSpinner(new SpinnerNumberModel(
                Math.min(Math.max(value, min), max), min, max, step));
        s.setPreferredSize(new Dimension(90, 26));
        // Style the embedded text field
        JComponent ed = s.getEditor();
        if (ed instanceof JSpinner.DefaultEditor) {
            JTextField tf = ((JSpinner.DefaultEditor) ed).getTextField();
            tf.setBackground(new Color(20, 20, 40));
            tf.setForeground(COL_VAL);
            tf.setCaretColor(COL_ACCENT);
            tf.setFont(new Font("Monospaced", Font.PLAIN, 12));
        }
        return s;
    }

    private static JLabel makeValLabel(String text) {
        JLabel l = new JLabel(text);
        l.setForeground(COL_VAL);
        l.setFont(new Font("Monospaced", Font.BOLD, 11));
        l.setPreferredSize(new Dimension(110, 20));
        return l;
    }

    private static JLabel makeUnitLabel(String text) {
        JLabel l = new JLabel(text);
        l.setForeground(COL_LABEL);
        l.setFont(new Font("SansSerif", Font.PLAIN, 10));
        return l;
    }

    private static JRadioButton makeRadio(String text) {
        JRadioButton r = new JRadioButton(text);
        r.setBackground(COL_CARD);
        r.setForeground(COL_TEXT);
        r.setFont(new Font("SansSerif", Font.PLAIN, 12));
        r.setFocusPainted(false);
        return r;
    }

    private static JButton makeSmallBtn(String text) {
        JButton b = new JButton(text);
        b.setBackground(new Color(28, 28, 52));
        b.setForeground(COL_TEXT);
        b.setFont(new Font("SansSerif", Font.PLAIN, 11));
        b.setFocusPainted(false);
        b.setBorderPainted(true);
        b.setOpaque(true);
        b.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(COL_BORDER),
                new EmptyBorder(4, 10, 4, 10)));
        b.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        return b;
    }

    private static JButton makePrimaryBtn(String text) {
        JButton b = new JButton(text) {
            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = (Graphics2D) g.create();
                g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                        RenderingHints.VALUE_ANTIALIAS_ON);
                g2.setColor(getModel().isRollover()
                        ? new Color(50, 150, 75) : new Color(30, 120, 55));
                g2.fillRoundRect(0, 0, getWidth(), getHeight(), 8, 8);
                g2.dispose();
                super.paintComponent(g);
            }
        };
        b.setContentAreaFilled(false);
        b.setOpaque(false);
        b.setForeground(Color.WHITE);
        b.setFont(new Font("SansSerif", Font.BOLD, 13));
        b.setFocusPainted(false);
        b.setBorderPainted(false);
        b.setBorder(new EmptyBorder(8, 24, 8, 24));
        b.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        return b;
    }

    private static void styleTf(JTextField tf) {
        tf.setBackground(new Color(20, 20, 40));
        tf.setForeground(COL_VAL);
        tf.setCaretColor(COL_ACCENT);
        tf.setFont(new Font("Monospaced", Font.PLAIN, 12));
        tf.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(COL_BORDER),
                new EmptyBorder(2, 6, 2, 6)));
    }

    private static JSeparator hRule() {
        JSeparator s = new JSeparator();
        s.setForeground(COL_BORDER);
        s.setBackground(COL_BORDER);
        s.setMaximumSize(new Dimension(Integer.MAX_VALUE, 1));
        return s;
    }

    // ── Utilities ─────────────────────────────────────────────────────────────

    private static int intOf(JSpinner s) { return (Integer) s.getValue(); }

    private static int clamp(int v) { return Math.max(0, Math.min(255, v)); }
}
