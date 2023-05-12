package org.ode4j.ode.internal;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class DxGimpactDataTest {

	private static final float[] CUBE_POINTS = {
			0.25f, 0.25f, 0.25f, // point 0
			-0.25f, 0.25f, 0.25f, // point 1
			0.25f, -0.25f, 0.25f, // point 2
			-0.25f, -0.25f, 0.25f, // point 3
			0.25f, 0.25f, -0.25f, // point 4
			-0.25f, 0.25f, -0.25f, // point 5
			0.25f, -0.25f, -0.25f, // point 6
			-0.25f, -0.25f, -0.25f,// point 7
	};

	private static final int[] CUBE_INDICES = {
			0, 2, 6, // 0
			0, 6, 4, // 1
			1, 0, 4, // 2
			1, 4, 5, // 3
			0, 1, 3, // 4
			0, 3, 2, // 5
			3, 1, 5, // 6
			3, 5, 7, // 7
			2, 3, 7, // 8
			2, 7, 6, // 9
			5, 4, 6, // 10
			5, 6, 7  // 11
			};

	@Test
	public void testCube() {
		DxGimpactData data = new DxGimpactData();
		data.build(CUBE_POINTS, CUBE_INDICES);
		data.preprocess();
		assertEquals(Math.PI * 0.5, data.getEdgeAngle(0, 0), 0.001);
		assertEquals(Math.PI * 0.5, data.getEdgeAngle(0, 1), 0.001);
		assertEquals(0, data.getEdgeAngle(0, 2), 0.001);
		assertEquals(Math.PI * 0.5, data.getEdgeAngle(6, 0), 0.001);
		assertEquals(Math.PI * 0.5, data.getEdgeAngle(6, 1), 0.001);
		assertEquals(0.0, data.getEdgeAngle(6, 2), 0.001);
	}

	private static final float[] QUAD_POINTS = {
			-1f, 0.0f, 0.0f, // point 0
			0.0f, 0.0f, 1.0f, // point 1
			0.0f, -0.0f, -1.0f, // point 2
			1.0f, -1.0f, 0.0f, // point 3
	};

	private static final int[] QUAD_INDICES = {
			0, 1, 2, // 0
			2, 1, 3, // 1
			};

	@Test
	public void testQuad() {
		DxGimpactData data = new DxGimpactData();
		data.build(QUAD_POINTS, QUAD_INDICES);
		data.preprocess();
		// TZ: I disabled these because this is not how ODE behaves.
		// assertEquals(Math.PI * 2, data.getEdgeAngle(0, 0), 0.001);
		assertEquals(Math.PI * 0.25, data.getEdgeAngle(0, 1), 0.001);
		// assertEquals(Math.PI * 2, data.getEdgeAngle(0, 2), 0.001);
		assertEquals(Math.PI * 0.25, data.getEdgeAngle(1, 0), 0.001);
		// assertEquals(Math.PI * 2, data.getEdgeAngle(1, 1), 0.001);
		// assertEquals(Math.PI * 2, data.getEdgeAngle(1, 2), 0.001);
	}

}
