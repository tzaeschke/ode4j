package org.ode4j.ode.internal;

import org.junit.Test;
import org.ode4j.ode.internal.CollisionLibccd.ccd_sphere_t;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

import static org.junit.Assert.*;

public class CollisionLibccdTest {

	@Test
	public void testSphereSupport() {
		ccd_sphere_t sphere = new ccd_sphere_t();
		sphere.radius = 5;
		sphere.pos.set(10, -5, 2);
		ccd_vec3_t v = new ccd_vec3_t();
		ccd_vec3_t dir = new ccd_vec3_t(-1, 0.2, 0.5);
		CollisionLibccd.ccdSupportSphere.run(sphere, dir, v);
		assertEquals(5.597745468371881, v.get0(), 0.00000001f);
		assertEquals(-4.119549093674376, v.get1(), 0.00000001f);
		assertEquals(4.20112726581406, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, 0, 2);
		CollisionLibccd.ccdSupportSphere.run(sphere, dir, v);
		assertEquals(12.23606797749979, v.get0(), 0.00000001f);
		assertEquals(-5.0, v.get1(), 0.00000001f);
		assertEquals(6.47213595499958, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, -0.4, 0.1);
		CollisionLibccd.ccdSupportSphere.run(sphere, dir, v);
		assertEquals(14.622501635210241, v.get0(), 0.00000001f);
		assertEquals(-6.849000654084097, v.get1(), 0.00000001f);
		assertEquals(2.4622501635210243, v.get2(), 0.00000001f);
	}
}
