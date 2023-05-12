package org.ode4j.ode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.internal.DxRay;

public class ConvexRayCollisionTest {

	private static final double EPSILON = Double.MIN_VALUE;
	private final int prism_pointcount = 8;
	private final int prism_planecount = 6;
	private final double prism_points[] = { 10.0, 1.0, -1.0, 10.0, -1.0, -1.0, -10.0, -1.0, -1.0, -10.0, 1.0, -1.0,
			10.0, 1.0, 1.0, 10.0, -1.0, 1.0, -10.0, -1.0, 1.0, -10.0, 1.0, 1.0 };

	private final int prism_polygons[] = { 4, 0, 1, 2, 3, 4, 4, 7, 6, 5, 4, 0, 4, 5, 1, 4, 1, 5, 6, 2, 4, 2, 6, 7, 3, 4,
			4, 0, 3, 7, };

	private final double prism_planes[] = { 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 10.0, 0.0, -1.0,
			0.0, 1.0, -1.0, 0.0, -0.0, 10.0, 0.0, 1.0, 0.0, 1.0, };

	@Test
	public void testConvexAndRayColliderConsidersPositions() {
		OdeHelper.initODE2(0);
		// Create convex
		DGeom convex = OdeHelper.createConvex(prism_planes, prism_planecount, prism_points, prism_pointcount,
				prism_polygons);
		convex.setPosition(0, 0, 0);

		// Create ray
		DxRay ray = (DxRay) OdeHelper.createRay(20);
		ray.set(0, -10, 0, 0, 1, 0);

		DContactGeomBuffer contact = new DContactGeomBuffer(1);
		int count = OdeHelper.collide(ray, convex, 1, contact);

		assertEquals(1, count);
		assertEquals(0.0, contact.get().pos.get0(), EPSILON);
		assertEquals(-1.0, contact.get().pos.get1(), EPSILON);
		assertEquals(0.0, contact.get().pos.get2(), EPSILON);
		assertEquals(0.0, contact.get().normal.get0(), EPSILON);
		assertEquals(-1.0, contact.get().normal.get1(), EPSILON);
		assertEquals(0.0, contact.get().normal.get2(), EPSILON);
		assertEquals(9.0, contact.get().depth, EPSILON);

		// Move Ray
		ray.set(5, -10, 0, 0, 1, 0);
		count = OdeHelper.collide(ray, convex, 1, contact);
		assertEquals(1, count);
		assertEquals(5.0, contact.get().pos.get0(), EPSILON);
		assertEquals(-1.0, contact.get().pos.get1(), EPSILON);
		assertEquals(0.0, contact.get().pos.get2(), EPSILON);
		assertEquals(0.0, contact.get().normal.get0(), EPSILON);
		assertEquals(-1.0, contact.get().normal.get1(), EPSILON);
		assertEquals(0.0, contact.get().normal.get2(), EPSILON);
		assertEquals(9.0, contact.get().depth, EPSILON);

		// Rotate Convex
		DMatrix3 rotate90z = new DMatrix3(0, -1, 0, 1, 0, 0, 0, 0, 1);
		convex.setRotation(rotate90z);
		count = OdeHelper.collide(ray, convex, 1, contact);
		assertEquals(0, count);

		// Move Ray
		ray.set(10, 0, 0, -1, 0, 0);
		count = OdeHelper.collide(ray, convex, 1, contact);
		assertEquals(1, count);
		assertEquals(1.0, contact.get().pos.get0(), EPSILON);
		assertEquals(0.0, contact.get().pos.get1(), EPSILON);
		assertEquals(0.0, contact.get().pos.get2(), EPSILON);
		assertEquals(1.0, contact.get().normal.get0(), EPSILON);
		assertEquals(0.0, contact.get().normal.get1(), EPSILON);
		assertEquals(0.0, contact.get().normal.get2(), EPSILON);
		assertEquals(9.0, contact.get().depth, EPSILON);

		// Move Ray
		ray.set(10, 1000, 1000, -1, 0, 0);
		// Move Geom
		convex.setPosition(0, 1000, 1000);
		count = OdeHelper.collide(ray, convex, 1, contact);
		assertEquals(1, count);
		assertEquals(1.0, contact.get().pos.get0(), EPSILON);
		assertEquals(1000.0, contact.get().pos.get1(), EPSILON);
		assertEquals(1000.0, contact.get().pos.get2(), EPSILON);
		assertEquals(1.0, contact.get().normal.get0(), EPSILON);
		assertEquals(0.0, contact.get().normal.get1(), EPSILON);
		assertEquals(0.0, contact.get().normal.get2(), EPSILON);
		assertEquals(9.0, contact.get().depth, EPSILON);
	}
}