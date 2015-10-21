package org.ode4j.demo;

class ConvexCubeGeom {
	static final double[] planes = // planes for a cube
	{ 1.0f, 0.0f, 0.0f, 0.25f, 0.0f, 1.0f, 0.0f, 0.25f, 0.0f, 0.0f, 1.0f, 0.25f, 0.0f, 0.0f, -1.0f, 0.25f, 0.0f, -1.0f,
			0.0f, 0.25f, -1.0f, 0.0f, 0.0f, 0.25f };
	static final int planecount = 6;
	static final double points[] = // points for a cube
	{ 0.25f, 0.25f, 0.25f, // point 0
			-0.25f, 0.25f, 0.25f, // point 1

			0.25f, -0.25f, 0.25f, // point 2
			-0.25f, -0.25f, 0.25f, // point 3

			0.25f, 0.25f, -0.25f, // point 4
			-0.25f, 0.25f, -0.25f, // point 5

			0.25f, -0.25f, -0.25f, // point 6
			-0.25f, -0.25f, -0.25f,// point 7
	};
	static final int pointcount = 8;
	static final int polygons[] = // Polygons for a cube (6 squares)
	{ 4, 0, 2, 6, 4, // positive X
			4, 1, 0, 4, 5, // positive Y
			4, 0, 1, 3, 2, // positive Z
			4, 3, 1, 5, 7, // negative X
			4, 2, 3, 7, 6, // negative Y
			4, 5, 4, 6, 7, // negative Z
	};
}