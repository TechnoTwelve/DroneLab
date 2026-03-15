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

import simulation.UAV;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

/**
 * Custom {@link JPanel} that renders one tick of the UAV/WSN simulation.
 *
 * <h3>Visual encoding</h3>
 * <ul>
 *   <li><b>Dark grey dot</b>   — sensor node unknown to the UAV</li>
 *   <li><b>Amber dot</b>       — node in the UAV knowledge base (untokenized)</li>
 *   <li><b>Green dot + glow</b>— tokenized node (UAV has been within scan range)</li>
 *   <li><b>Quadcopter icon</b> — UAV: blue (patrol), cyan (executing route),
 *       orange-red (returning home)</li>
 *   <li><b>Dashed blue ring</b>— UAV scan radius</li>
 *   <li><b>Fading blue trail</b>— recent UAV path history</li>
 *   <li><b>Red arrowed lines</b>— active planned route</li>
 *   <li><b>Orange ×</b>        — frontier exploration target</li>
 *   <li><b>Gold beacon</b>     — UAV home / deployment position</li>
 *   <li><b>Sector overlay</b>  — green/red tint for visited/unvisited coverage sectors</li>
 *   <li><b>HUD panel</b>       — live stats rendered directly onto the field</li>
 * </ul>
 */
public final class SimulationPanel extends JPanel {

    private static final int MARGIN    = 12;
    private static final int TRAIL_MAX = 80;

    // ── Colours ───────────────────────────────────────────────────────────────
    private static final Color COL_BG         = new Color(30,  31,  34);   // Darcula editor bg
    private static final Color COL_GRID       = new Color(42,  43,  46);   // subtle grid
    private static final Color COL_BORDER     = new Color(70,  73,  75);   // Darcula separator
    private static final Color COL_NODE_UNK   = new Color(72,  74,  78);   // neutral unknown node
    private static final Color COL_NODE_KNOW  = new Color(220, 170,   0);
    private static final Color COL_NODE_TOK   = new Color(50,  210,  80);
    private static final Color COL_UAV_PAT    = new Color(40,  145, 255);
    private static final Color COL_UAV_EXEC   = new Color(0,   220, 255);
    private static final Color COL_UAV_HOME   = new Color(255, 105,  80);
    private static final Color COL_SCAN_FILL  = new Color(0,   110, 210,  12);
    private static final Color COL_SCAN_RING  = new Color(0,   150, 255,  55);
    private static final Color COL_ROUTE      = new Color(255,  75,  75, 210);
    private static final Color COL_WP         = new Color(255, 145, 145);
    private static final Color COL_FRONTIER   = new Color(255, 162,   0);
    private static final Color COL_SEC_VISIT  = new Color(28,   95,  28,  22);
    private static final Color COL_SEC_UNVIS  = new Color(95,   28,  28,  22);
    private static final Color COL_HOME_MARK  = new Color(255, 205,  60);
    private static final Color COL_TRAIL      = new Color(70,  150, 255);

    // ── State ─────────────────────────────────────────────────────────────────
    private volatile SimulationSnapshot snap  = null;
    private final Deque<int[]>          trail = new ArrayDeque<>();

    // ── Constructor ───────────────────────────────────────────────────────────

    public SimulationPanel() {
        setBackground(COL_BG);
        setPreferredSize(new Dimension(600, 600));
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Thread-safe snapshot update; schedules a repaint on the EDT. */
    public void setSnapshot(SimulationSnapshot s) {
        this.snap = s;
        if (s != null) {
            trail.addLast(new int[]{ s.uavX, s.uavY });
            while (trail.size() > TRAIL_MAX) trail.removeFirst();
        }
        repaint();
    }

    /** Clears the UAV trail (call between passes). */
    public void clearTrail() { trail.clear(); }

    // ── Rendering ─────────────────────────────────────────────────────────────

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        SimulationSnapshot s = this.snap;

        Graphics2D g2 = (Graphics2D) g.create();
        applyHints(g2);

        int pw       = getWidth();
        int ph       = getHeight();
        int fieldPx  = Math.min(pw, ph) - 2 * MARGIN;
        // Center the square field within the panel (removes dead space on tall panels)
        int offX = (pw - fieldPx) / 2;
        int offY = (ph - fieldPx) / 2;
        int fieldSize = (s != null) ? Math.max(s.fieldWidth, s.fieldHeight) : 1000;
        double scale  = (double) fieldPx / fieldSize;

        // Fill background
        g2.setColor(COL_BG);
        g2.fillRect(0, 0, pw, ph);

        // ── Subtle background grid ──────────────────────────────────────────
        g2.setColor(COL_GRID);
        g2.setStroke(new BasicStroke(0.4f));
        int gridStep = Math.max(1, (int)(100 * scale));
        for (int gx = offX; gx <= offX + fieldPx; gx += gridStep)
            g2.drawLine(gx, offY, gx, offY + fieldPx);
        for (int gy = offY; gy <= offY + fieldPx; gy += gridStep)
            g2.drawLine(offX, gy, offX + fieldPx, gy);

        if (s == null) {
            g2.setColor(new Color(100, 100, 150));
            g2.setFont(new Font("SansSerif", Font.ITALIC, 13));
            g2.drawString("Initialising simulation\u2026", offX + 14, offY + 36);
            drawBorder(g2, fieldPx, offX, offY);
            g2.dispose();
            return;
        }

        // Clip to field — prevents node glows / scan rings bleeding past the border
        java.awt.Shape origClip = g2.getClip();
        g2.setClip(offX, offY, fieldPx + 1, fieldPx + 1);

        // ── Coverage-sector overlay (autonomous pass only) ──────────────────
        if (s.visitedSectors != null && s.sectorSize > 0) {
            int nr = s.visitedSectors.length;
            int nc = nr > 0 ? s.visitedSectors[0].length : 0;
            for (int r = 0; r < nr; r++) {
                for (int c = 0; c < nc; c++) {
                    int sx = offX + (int)(r * s.sectorSize * scale);
                    int sy = offY + (int)(c * s.sectorSize * scale);
                    int sw = (int)(s.sectorSize * scale);
                    g2.setColor(s.visitedSectors[r][c] ? COL_SEC_VISIT : COL_SEC_UNVIS);
                    g2.fillRect(sx, sy, sw, sw);
                }
            }
        }

        // ── Home beacon ─────────────────────────────────────────────────────
        int hmx = offX + (int)(s.homeX * scale);
        int hmy = offY + (int)(s.homeY * scale);
        drawHomeBeacon(g2, hmx, hmy);

        // ── Active route edges (drawn behind nodes) ─────────────────────────
        List<double[]> wps = s.routeWaypoints;
        if (wps != null && wps.size() > 1) {
            g2.setStroke(new BasicStroke(1.7f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            for (int i = 0; i < wps.size() - 1; i++) {
                int ax = offX + (int)(wps.get(i    )[0] * scale);
                int ay = offY + (int)(wps.get(i    )[1] * scale);
                int bx = offX + (int)(wps.get(i + 1)[0] * scale);
                int by = offY + (int)(wps.get(i + 1)[1] * scale);
                g2.setColor(COL_ROUTE);
                g2.drawLine(ax, ay, bx, by);
                drawArrowHead(g2, ax, ay, bx, by, COL_ROUTE);
            }
            // Waypoint circles
            for (double[] wp : wps) {
                int wx = offX + (int)(wp[0] * scale);
                int wy = offY + (int)(wp[1] * scale);
                g2.setColor(COL_WP);
                g2.fillOval(wx - 3, wy - 3, 6, 6);
            }
        }

        // ── Sensor nodes ────────────────────────────────────────────────────
        for (SimulationSnapshot.NodeSnap n : s.nodes) {
            int nx = offX + (int)(n.x * scale);
            int ny = offY + (int)(n.y * scale);
            if (n.tokenized) {
                // Green glow halo
                g2.setColor(new Color(50, 210, 80, 45));
                g2.fillOval(nx - 7, ny - 7, 14, 14);
                // Bright core
                g2.setColor(COL_NODE_TOK);
                g2.fillOval(nx - 4, ny - 4, 8, 8);
                // Subtle highlight ring
                g2.setColor(new Color(80, 255, 100, 70));
                g2.setStroke(new BasicStroke(0.7f));
                g2.drawOval(nx - 5, ny - 5, 10, 10);
            } else if (n.knownToUav) {
                g2.setColor(COL_NODE_KNOW);
                g2.fillOval(nx - 3, ny - 3, 6, 6);
            } else {
                g2.setColor(COL_NODE_UNK);
                g2.fillOval(nx - 2, ny - 2, 4, 4);
            }
        }

        // ── Forward prediction trail (UAV → frontier, mirror of history trail) ─
        if (s.frontierTarget != null && s.uavMode == UAV.DriveMode.PATROL) {
            int ux = offX + (int)(s.uavX              * scale);
            int uy = offY + (int)(s.uavY              * scale);
            int fx = offX + (int)(s.frontierTarget[0] * scale);
            int fy = offY + (int)(s.frontierTarget[1] * scale);
            int steps = 30;
            for (int i = 0; i < steps - 1; i++) {
                float t0    = (float)  i      / steps;
                float t1    = (float)(i + 1)  / steps;
                float fade  = 1f - t0;
                int   alpha = Math.max(5, (int)(fade * 120));
                float width = Math.max(0.5f, 1.8f * fade);
                int ax = ux + (int)((fx - ux) * t0);
                int ay = uy + (int)((fy - uy) * t0);
                int bx = ux + (int)((fx - ux) * t1);
                int by = uy + (int)((fy - uy) * t1);
                g2.setColor(new Color(COL_FRONTIER.getRed(),
                                      COL_FRONTIER.getGreen(),
                                      COL_FRONTIER.getBlue(), alpha));
                g2.setStroke(new BasicStroke(width, BasicStroke.CAP_ROUND,
                                              BasicStroke.JOIN_ROUND));
                g2.drawLine(ax, ay, bx, by);
            }
        }

        // ── UAV scan range ──────────────────────────────────────────────────
        int usx    = offX + (int)(s.uavX * scale);
        int usy    = offY + (int)(s.uavY * scale);
        int rangeR = (int)(s.droneRange * scale);

        g2.setColor(COL_SCAN_FILL);
        g2.fillOval(usx - rangeR, usy - rangeR, rangeR * 2, rangeR * 2);

        float[] scanDash = { 6f, 3f };
        g2.setColor(COL_SCAN_RING);
        g2.setStroke(new BasicStroke(1.2f, BasicStroke.CAP_BUTT,
                BasicStroke.JOIN_MITER, 10f, scanDash, 0f));
        g2.drawOval(usx - rangeR, usy - rangeR, rangeR * 2, rangeR * 2);

        // ── UAV flight trail ────────────────────────────────────────────────
        drawTrail(g2, scale, offX, offY);

        // ── UAV quadcopter body ─────────────────────────────────────────────
        Color uavCol = (s.uavMode == UAV.DriveMode.RETURN_HOME) ? COL_UAV_HOME
                     : (s.uavMode == UAV.DriveMode.EXECUTE)     ? COL_UAV_EXEC
                     :                                             COL_UAV_PAT;
        drawQuadcopter(g2, usx, usy, uavCol, s.uavMode);

        // ── Restore clip, draw border on top ────────────────────────────────
        g2.setClip(origClip);
        drawBorder(g2, fieldPx, offX, offY);

        g2.dispose();
    }

    // ── Quadcopter drone ──────────────────────────────────────────────────────

    /**
     * Draws a quadcopter icon: 4 diagonal arms with rotor discs at the tips
     * and a coloured central body.
     */
    private static void drawQuadcopter(Graphics2D g2, int cx, int cy,
                                        Color col, UAV.DriveMode mode) {
        int armLen = 12;
        int bodyR  =  5;
        int rotorR =  5;

        // 4 arms at 45°, 135°, 225°, 315°
        int[][] arms = {{ armLen, -armLen }, { armLen,  armLen },
                        {-armLen,  armLen }, {-armLen, -armLen }};

        // ── Arms ─────────────────────────────────────────────────────────────
        g2.setColor(col.darker());
        g2.setStroke(new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        for (int[] arm : arms)
            g2.drawLine(cx, cy, cx + arm[0], cy + arm[1]);

        // ── Rotors ───────────────────────────────────────────────────────────
        for (int[] arm : arms) {
            int rx = cx + arm[0];
            int ry = cy + arm[1];
            // Outer glow
            g2.setColor(new Color(col.getRed(), col.getGreen(), col.getBlue(), 35));
            g2.fillOval(rx - rotorR - 2, ry - rotorR - 2,
                        (rotorR + 2) * 2, (rotorR + 2) * 2);
            // Disc
            g2.setColor(new Color(col.getRed(), col.getGreen(), col.getBlue(), 155));
            g2.fillOval(rx - rotorR, ry - rotorR, rotorR * 2, rotorR * 2);
            // Ring
            g2.setColor(col.brighter());
            g2.setStroke(new BasicStroke(0.9f));
            g2.drawOval(rx - rotorR, ry - rotorR, rotorR * 2, rotorR * 2);
        }

        // ── Central body ─────────────────────────────────────────────────────
        // Dark fill
        g2.setColor(new Color(30, 31, 34));
        g2.fillOval(cx - bodyR, cy - bodyR, bodyR * 2, bodyR * 2);
        // Coloured face
        g2.setColor(col);
        g2.fillOval(cx - bodyR + 1, cy - bodyR + 1, bodyR * 2 - 2, bodyR * 2 - 2);
        // White outline
        g2.setColor(Color.WHITE);
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawOval(cx - bodyR, cy - bodyR, bodyR * 2, bodyR * 2);

        // ── Mode indicator dot (small coloured pip above body) ────────────────
        int pipY = cy - bodyR - 7;
        if (mode == UAV.DriveMode.EXECUTE) {
            g2.setColor(new Color(0, 255, 180));
            g2.fillOval(cx - 2, pipY, 4, 4);
        } else if (mode == UAV.DriveMode.RETURN_HOME) {
            g2.setColor(new Color(255, 120, 60));
            g2.fillOval(cx - 2, pipY, 4, 4);
        }
    }

    // ── Home beacon ───────────────────────────────────────────────────────────

    private static void drawHomeBeacon(Graphics2D g2, int cx, int cy) {
        // Outer pulsing ring
        g2.setColor(new Color(255, 205, 60, 45));
        g2.fillOval(cx - 13, cy - 13, 26, 26);
        // Inner ring
        g2.setColor(new Color(255, 205, 60, 110));
        g2.fillOval(cx - 7, cy - 7, 14, 14);
        // Ring outline
        g2.setColor(new Color(255, 205, 60, 190));
        g2.setStroke(new BasicStroke(1.2f));
        g2.drawOval(cx - 9, cy - 9, 18, 18);
        // Centre dot
        g2.setColor(COL_HOME_MARK);
        g2.fillOval(cx - 3, cy - 3, 6, 6);
        // "H" label
        g2.setFont(new Font("SansSerif", Font.BOLD, 8));
        g2.setColor(new Color(0, 0, 0, 190));
        g2.drawString("H", cx - 3, cy + 3);
    }

    // ── UAV flight trail ──────────────────────────────────────────────────────

    private void drawTrail(Graphics2D g2, double scale, int offX, int offY) {
        int[][] arr = trail.toArray(new int[0][]);
        int n = arr.length;
        if (n < 2) return;
        for (int i = 0; i < n - 1; i++) {
            float t     = (float)(i + 1) / n;           // 0→1 (oldest→newest)
            int   alpha = Math.max(5, (int)(t * 110));
            float width = Math.max(0.5f, 1.6f * t);
            int ax = offX + (int)(arr[i    ][0] * scale);
            int ay = offY + (int)(arr[i    ][1] * scale);
            int bx = offX + (int)(arr[i + 1][0] * scale);
            int by = offY + (int)(arr[i + 1][1] * scale);
            g2.setColor(new Color(COL_TRAIL.getRed(),
                                  COL_TRAIL.getGreen(),
                                  COL_TRAIL.getBlue(), alpha));
            g2.setStroke(new BasicStroke(width, BasicStroke.CAP_ROUND,
                                          BasicStroke.JOIN_ROUND));
            g2.drawLine(ax, ay, bx, by);
        }
    }

    // ── Route arrow heads ─────────────────────────────────────────────────────

    private static void drawArrowHead(Graphics2D g2,
                                       int x1, int y1, int x2, int y2, Color col) {
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int sz = 7;
        int[] xp = { x2,
                (int)(x2 - sz * Math.cos(angle - 0.4)),
                (int)(x2 - sz * Math.cos(angle + 0.4)) };
        int[] yp = { y2,
                (int)(y2 - sz * Math.sin(angle - 0.4)),
                (int)(y2 - sz * Math.sin(angle + 0.4)) };
        g2.setColor(col);
        g2.setStroke(new BasicStroke(1f));
        g2.fillPolygon(xp, yp, 3);
    }

    // ── Field border ──────────────────────────────────────────────────────────

    private void drawBorder(Graphics2D g2, int fieldPx, int offX, int offY) {
        g2.setColor(COL_BORDER);
        g2.setStroke(new BasicStroke(2f));
        g2.drawRect(offX, offY, fieldPx, fieldPx);
    }

    // ── Rendering hints ───────────────────────────────────────────────────────

    private static void applyHints(Graphics2D g2) {
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                            RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
                            RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
                            RenderingHints.VALUE_STROKE_PURE);
        g2.setRenderingHint(RenderingHints.KEY_RENDERING,
                            RenderingHints.VALUE_RENDER_QUALITY);
    }
}
