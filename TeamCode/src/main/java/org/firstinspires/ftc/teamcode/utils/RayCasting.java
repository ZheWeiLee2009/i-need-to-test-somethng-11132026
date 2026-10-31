package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * Tracks a fixed set of polygons over a square coordinate system and answers,
 * on demand, whether a given (x, y) point currently falls inside any of them.
 *
 * <p>Intended usage pattern:
 * <ol>
 *   <li>Construct once, at the start of the program, with the full set of
 *       polygons to track (they cannot be changed afterward).</li>
 *   <li>Call {@link #update(double, double)} every time a new position is
 *       available (e.g. every frame or tick).</li>
 *   <li>Read {@link #isInside()} / {@link #getZoneIds()} / {@link #getFirstZoneId()}
 *       to find out whether the point is inside anything, and which polygon(s)
 *       matched — no need to capture update()'s return value.</li>
 * </ol>
 *
 * <p>Internally this uses the ray-casting (even-odd rule) algorithm for the
 * actual containment test, a uniform spatial grid to avoid checking every
 * polygon on every call, and a "temporal coherence" shortcut that re-checks
 * last update's matches first, since a continuously moving point is likely
 * to still be inside (or near) the same polygon it was in a moment ago.
 */
public class RayCasting {

    /**
     * A single (x, y) coordinate within the tracked coordinate system.
     * Mutable fields are used purely to avoid unnecessary object churn
     * internally; polygon vertices themselves are not expected to change
     * after a {@link Polygon} is built.
     */
    public static class Point {
        public double x;
        public double y;

        /**
         * @param x horizontal coordinate
         * @param y vertical coordinate
         */
        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * An immutable polygon: an ID plus an ordered list of vertices.
     * Vertices should be given in order (clockwise or counter-clockwise,
     * either works) and should NOT repeat the first point at the end —
     * the class closes the loop automatically.
     *
     * <p>On construction, the polygon's axis-aligned bounding box is
     * computed once and cached, so containment checks can cheaply reject
     * a point before running the full ray-casting test.
     */
    public static class Polygon {

        /** Identifier for this polygon, returned via getZoneIds()/getFirstZoneId() when a point matches it. */
        public final String id;

        /** Ordered vertices defining the polygon's boundary. */
        public final List<Point> points;

        /** Cached axis-aligned bounding box, used for fast rejection before ray-casting. */
        final double minX, minY, maxX, maxY;

        /**
         * @param id     identifier for this polygon (e.g. "SafeZone", "A")
         * @param points ordered vertices, at least 3, not closing back to the first point
         * @throws IllegalArgumentException if fewer than 3 points are given
         */
        public Polygon(String id, List<Point> points) {
            if (points == null || points.size() < 3) {
                throw new IllegalArgumentException("A polygon needs at least 3 points.");
            }
            this.id = id;
            this.points = points;

            double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
            double maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
            for (Point p : points) {
                minX = Math.min(minX, p.x);
                minY = Math.min(minY, p.y);
                maxX = Math.max(maxX, p.x);
                maxY = Math.max(maxY, p.y);
            }
            this.minX = minX;
            this.minY = minY;
            this.maxX = maxX;
            this.maxY = maxY;
        }

        /**
         * Convenience factory that builds a {@link Polygon} directly from raw
         * coordinate pairs, so callers don't need to construct {@link Point}
         * objects by hand.
         *
         * <p>Example:
         * <pre>{@code
         * Polygon.fromCoords("Zone1", new double[][]{ {20,20}, {70,15}, {80,60} });
         * }</pre>
         *
         * @param id     identifier for this polygon
         * @param coords array of [x, y] pairs, at least 3
         * @return a new Polygon built from the given coordinates
         */
        public static Polygon fromCoords(String id, double[][] coords) {
            List<Point> pts = new ArrayList<>();
            for (double[] c : coords) {
                pts.add(new Point(c[0], c[1]));
            }
            return new Polygon(id, pts);
        }

        /**
         * Starts a fluent builder for constructing a polygon one vertex at a
         * time via {@link PolygonBuilder#addPoint(double, double)}. Preferred
         * over {@link #fromCoords(String, double[][])} when vertices are
         * expected to be tuned/edited often, since each point sits on its own
         * line — easy to add, remove, reorder, comment, or spot in a diff,
         * without needing to recount array brackets.
         *
         * <p>Example:
         * <pre>{@code
         * Polygon.builder("redGarden")
         *     .addPoint(0, 0)
         *     .addPoint(0, 70.75)
         *     .addPoint(141.5, 70.75)
         *     .addPoint(141.5, 0)
         *     .build();
         * }</pre>
         *
         * @param id identifier for this polygon
         * @return a new builder for this polygon
         */
        public static PolygonBuilder builder(String id) {
            return new PolygonBuilder(id);
        }

        /**
         * Cheap pre-check used before the full ray-casting test: if the point
         * falls outside this polygon's bounding box, it cannot possibly be
         * inside the polygon itself.
         *
         * @param x point's x coordinate
         * @param y point's y coordinate
         * @return true if (x, y) falls within this polygon's bounding box
         */
        boolean isWithinBoundingBox(double x, double y) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }

    /**
     * Fluent builder for a {@link Polygon}, added one vertex at a time.
     * Get one via {@link Polygon#builder(String)}.
     */
    public static class PolygonBuilder {
        private final String id;
        private final List<Point> points = new ArrayList<>();

        private PolygonBuilder(String id) {
            this.id = id;
        }

        /**
         * Adds the next vertex to this polygon, in order.
         *
         * @param x vertex x coordinate
         * @param y vertex y coordinate
         * @return this builder, for chaining
         */
        public PolygonBuilder addPoint(double x, double y) {
            points.add(new Point(x, y));
            return this;
        }

        /**
         * Finalizes the polygon from the vertices added so far.
         *
         * @return the built, immutable Polygon
         * @throws IllegalArgumentException if fewer than 3 points were added
         */
        public Polygon build() {
            return new Polygon(id, points);
        }
    }

    /**
     * The outcome of a single {@link #update(double, double)} call: whether
     * the point was inside at least one tracked polygon, and the IDs of every
     * polygon that matched (a point can be inside more than one if polygons
     * overlap).
     *
     * <p>This is kept internal now — most callers should use
     * {@link #isInside()} / {@link #getZoneIds()} / {@link #getFirstZoneId()}
     * instead of handling a Result directly.
     */
    private static class Result {
        private final boolean inside;
        private final List<String> polygonIds;

        Result(boolean inside, List<String> polygonIds) {
            this.inside = inside;
            this.polygonIds = polygonIds;
        }

        /** @return true if the point was inside at least one polygon */
        boolean isInside() {
            return inside;
        }

        /** @return IDs of every polygon containing the point (empty list if none) */
        List<String> getPolygonIds() {
            return polygonIds;
        }

        /**
         * Convenience accessor for callers who know their polygons never
         * overlap and just want "the" matching ID.
         *
         * @return the first matching polygon ID, or null if none matched
         */
        String getFirstPolygonId() {
            return polygonIds.isEmpty() ? null : polygonIds.get(0);
        }

        @Override
        public String toString() {
            return "Result{inside=" + inside + ", polygonIds=" + polygonIds + "}";
        }
    }

    /** The fixed set of polygons being tracked, set once at construction. */
    private final List<Polygon> polygons;

    /** Side length of the square coordinate system (e.g. 141.5). */
    private final double worldSize;

    /**
     * Uniform spatial grid over the coordinate system. Each cell holds the
     * list of polygons whose bounding box overlaps that cell, so
     * {@link #update(double, double)} only needs to check the polygons in
     * the point's current cell rather than every tracked polygon.
     */
    private final List<Polygon>[][] grid;

    /** Width/height of a single grid cell, in coordinate-system units. */
    private final double cellSize;

    /** Number of grid columns / rows (the grid is always square). */
    private final int gridCols, gridRows;

    /**
     * Polygons that matched on the previous {@link #update(double, double)}
     * call. Checked first on the next call, since a continuously moving
     * point is likely to still be inside (or just left) the same polygon(s).
     */
    private List<Polygon> lastHits = new ArrayList<>();

    /**
     * Cached outcome of the most recent {@link #update(double, double)} call.
     * Read via {@link #isInside()}, {@link #getZoneIds()}, and
     * {@link #getFirstZoneId()} so callers never need to hold onto update()'s
     * return value.
     */
    private Result lastResult = new Result(false, new ArrayList<>());

    /**
     * Builds a tracker for a fixed set of polygons. The polygon set cannot
     * be changed after construction — this class is designed for the case
     * where zones/regions are defined once at startup and only the point
     * being tested changes over time.
     *
     * @param polygons  the polygons to track for the lifetime of this object
     * @param worldSize the side length of the square coordinate system (e.g. 141.5)
     */
    @SuppressWarnings("unchecked")
    public RayCasting(List<Polygon> polygons, double worldSize) {
        this.polygons = new ArrayList<>(polygons);
        this.worldSize = worldSize;

        // Aim for roughly sqrt(N) cells per axis so each cell holds a small,
        // manageable number of polygons on average.
        int cellsPerAxis = Math.max(1, (int) Math.ceil(Math.sqrt(Math.max(1, polygons.size()))));
        this.cellSize = worldSize / cellsPerAxis;
        this.gridCols = cellsPerAxis;
        this.gridRows = cellsPerAxis;

        grid = new List[gridCols][gridRows];
        for (int i = 0; i < gridCols; i++) {
            for (int j = 0; j < gridRows; j++) {
                grid[i][j] = new ArrayList<>();
            }
        }

        // Register each polygon in every grid cell its bounding box touches,
        // so a point in any of those cells will find it during update().
        for (Polygon poly : this.polygons) {
            int minCol = clampCol((int) (poly.minX / cellSize));
            int maxCol = clampCol((int) (poly.maxX / cellSize));
            int minRow = clampRow((int) (poly.minY / cellSize));
            int maxRow = clampRow((int) (poly.maxY / cellSize));

            for (int c = minCol; c <= maxCol; c++) {
                for (int r = minRow; r <= maxRow; r++) {
                    grid[c][r].add(poly);
                }
            }
        }
    }

    /** Clamps a column index into the valid grid range [0, gridCols - 1]. */
    private int clampCol(int c) {
        return Math.max(0, Math.min(gridCols - 1, c));
    }

    /** Clamps a row index into the valid grid range [0, gridRows - 1]. */
    private int clampRow(int r) {
        return Math.max(0, Math.min(gridRows - 1, r));
    }

    /**
     * Read-only view of the polygons this instance is tracking.
     *
     * @return an unmodifiable copy of the tracked polygon list
     */
    public List<Polygon> getPolygons() {
        return List.copyOf(polygons);
    }

    /**
     * Tests a new (x, y) position against all tracked polygons and stores
     * the outcome internally. Call this every time position updates (e.g.
     * every loop cycle), then read the result with {@link #isInside()},
     * {@link #getZoneIds()}, or {@link #getFirstZoneId()} — there is no
     * return value to capture.
     *
     * <p>Internally this checks last call's matches first (temporal
     * coherence), then falls back to scanning only the polygons registered
     * in the point's current grid cell — so cost stays low even with many
     * tracked polygons.
     *
     * @param x the point's current x coordinate
     * @param y the point's current y coordinate
     */
    public void update(double x, double y) {
        List<String> hitIds = new ArrayList<>();
        List<Polygon> newHits = new ArrayList<>();

        // Fast path: re-check whatever matched last time first, since the
        // point likely hasn't moved far since the previous update().
        for (Polygon poly : lastHits) {
            if (poly.isWithinBoundingBox(x, y) && rayCast(x, y, poly)) {
                hitIds.add(poly.id);
                newHits.add(poly);
            }
        }

        // Fall back to the spatial grid: only check polygons registered in
        // the cell the point currently falls in, skipping any already
        // confirmed above.
        int col = clampCol((int) (x / cellSize));
        int row = clampRow((int) (y / cellSize));

        for (Polygon poly : grid[col][row]) {
            if (newHits.contains(poly)) continue;
            if (poly.isWithinBoundingBox(x, y) && rayCast(x, y, poly)) {
                hitIds.add(poly.id);
                newHits.add(poly);
            }
        }

        lastHits = newHits;
        lastResult = new Result(!hitIds.isEmpty(), hitIds);
    }

    /**
     * @return true if the most recent {@link #update(double, double)} call
     *         found the point inside at least one tracked polygon
     */
    public boolean isInside() {
        return lastResult.isInside();
    }

    /**
     * @return IDs of every polygon the most recent {@link #update(double, double)}
     *         call found the point inside (empty list if none)
     */
    public List<String> getZoneIds() {
        return lastResult.getPolygonIds();
    }

    /**
     * Convenience accessor for callers who know their polygons never
     * overlap and just want "the" matching zone.
     *
     * @return the first matching zone ID from the most recent update(), or
     *         null if none matched
     */
    public String getFirstZoneId() {
        return lastResult.getFirstPolygonId();
    }

    /**
     * Core containment test using the ray-casting (even-odd rule) algorithm:
     * counts how many polygon edges a ray from the point crosses. An odd
     * count means the point is inside; even means outside. Works correctly
     * for convex, concave, and self-intersecting polygons.
     *
     * @param x       point's x coordinate
     * @param y       point's y coordinate
     * @param polygon the polygon to test against
     * @return true if the point lies inside the polygon
     */
    private static boolean rayCast(double x, double y, Polygon polygon) {
        List<Point> verts = polygon.points;
        int n = verts.size();
        boolean inside = false;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            Point vi = verts.get(i);
            Point vj = verts.get(j);

            boolean intersects = ((vi.y > y) != (vj.y > y)) &&
                    (x < (vj.x - vi.x) * (y - vi.y) / (vj.y - vi.y) + vi.x);

            if (intersects) {
                inside = !inside;
            }
        }
        return inside;
    }
}