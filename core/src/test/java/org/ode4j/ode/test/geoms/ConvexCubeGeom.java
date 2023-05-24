package org.ode4j.ode.test.geoms;

public class ConvexCubeGeom {
    public static final double[] planes = // planes for a cube
            {1.0f, 0.0f, 0.0f, 0.25f,
                    0.0f, 1.0f, 0.0f, 0.25f,
                    0.0f, 0.0f, 1.0f, 0.25f,
                    0.0f, 0.0f, -1.0f, 0.25f,
                    0.0f, -1.0f, 0.0f, 0.25f,
                    -1.0f, 0.0f, 0.0f, 0.25f};
    public static final int planecount = 6;
    public static final double[] points = // points for a cube
            {0.25f, 0.25f, 0.25f, // point 0
                    -0.25f, 0.25f, 0.25f, // point 1

                    0.25f, -0.25f, 0.25f, // point 2
                    -0.25f, -0.25f, 0.25f, // point 3

                    0.25f, 0.25f, -0.25f, // point 4
                    -0.25f, 0.25f, -0.25f, // point 5

                    0.25f, -0.25f, -0.25f, // point 6
                    -0.25f, -0.25f, -0.25f,// point 7
            };
    public static final int pointcount = 8;
    public static final int[] polygons = // Polygons for a cube (6 squares)
            {4, 0, 2, 6, 4, // positive X
                    4, 1, 0, 4, 5, // positive Y
                    4, 0, 1, 3, 2, // positive Z
                    4, 3, 1, 5, 7, // negative X
                    4, 2, 3, 7, 6, // negative Y
                    4, 5, 4, 6, 7, // negative Z
            };

    /**
     * cube with edge length 1
     */
    public static final double[] planes1 = // planes for a cube
            {1.0, 0.0, 0.0, 0.5,
                    0.0, 1.0, 0.0, 0.5,
                    0.0, 0.0, 1.0, 0.5,
                    0.0, 0.0, -1.0, 0.5,
                    0.0, -1.0, 0.0, 0.5,
                    -1.0, 0.0, 0.0, 0.5};

    /**
     * cube with edge length 1
     */
    public static final double[] points1 = // points for a cube
            {0.5, 0.5, 0.5, // point 0
                    -0.5, 0.5, 0.5, // point 1

                    0.5, -0.5, 0.5, // point 2
                    -0.5, -0.5, 0.5, // point 3

                    0.5, 0.5, -0.5, // point 4
                    -0.5, 0.5, -0.5, // point 5

                    0.5, -0.5, -0.5, // point 6
                    -0.5, -0.5, -0.5,// point 7
            };

    /**
     * cube with edge length 1
     */
    public static final int[] polygons1 = polygons;
}