package org.ode4j.ode.internal;

import org.junit.Test;
import org.ode4j.ode.internal.CollisionLibccd.ccd_cap_t;
import org.ode4j.ode.internal.CollisionLibccd.ccd_cyl_t;
import org.ode4j.ode.internal.CollisionLibccd.ccd_sphere_t;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

import static org.junit.Assert.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.ccdVec3Normalize;

public class CollisionLibccdTest {

	@Test
	public void testSphereSupport() {
		ccd_sphere_t sphere = new ccd_sphere_t();
		DxSphere geom = new DxSphere(null, 5);
		geom.setPosition(10, -5, 2);
		CollisionLibccd.ccdGeomToSphere(geom, sphere);
		ccd_vec3_t v = new ccd_vec3_t();
		ccd_vec3_t dir = new ccd_vec3_t(-1, 0.2, 0.5);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportSphere.run(sphere, dir, v);
		assertEquals(5.597745468371881, v.get0(), 0.00000001f);
		assertEquals(-4.119549093674376, v.get1(), 0.00000001f);
		assertEquals(4.20112726581406, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, 0, 2);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportSphere.run(sphere, dir, v);
		assertEquals(12.23606797749979, v.get0(), 0.00000001f);
		assertEquals(-5.0, v.get1(), 0.00000001f);
		assertEquals(6.47213595499958, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, -0.4, 0.1);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportSphere.run(sphere, dir, v);
		assertEquals(14.622501635210241, v.get0(), 0.00000001f);
		assertEquals(-6.849000654084097, v.get1(), 0.00000001f);
		assertEquals(2.4622501635210243, v.get2(), 0.00000001f);
	}

	@Test
	public void testCapsuleSupport() {
		ccd_cap_t capsule = new ccd_cap_t();
		DxCapsule geom = new DxCapsule(null, 5, 8);
		geom.setPosition(10, -5, 2);
		CollisionLibccd.ccdGeomToCap(geom, capsule);
		ccd_vec3_t v = new ccd_vec3_t();
		ccd_vec3_t dir = new ccd_vec3_t(-1, 0.2, -0.5);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportCap.run(capsule, dir, v);
		assertEquals(5.597745468371881, v.get0(), 0.00000001f);
		assertEquals(-4.119549093674376, v.get1(), 0.00000001f);
		assertEquals(-4.20112726581406, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, 0, 2);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportCap.run(capsule, dir, v);
		assertEquals(12.23606797749979, v.get0(), 0.00000001f);
		assertEquals(-5.0, v.get1(), 0.00000001f);
		assertEquals(10.47213595499958, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, 1, 1);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportCap.run(capsule, dir, v);
		assertEquals(12.886751345948129, v.get0(), 0.00000001f);
		assertEquals(-2.113248654051871, v.get1(), 0.00000001f);
		assertEquals(8.886751345948129, v.get2(), 0.00000001f);
	}

	@Test
	public void testCylinderSupport() {
		ccd_cyl_t cylinder = new ccd_cyl_t();
		DxCylinder geom = new DxCylinder(null, 5, 8);
		geom.setPosition(10, -5, 2);
		CollisionLibccd.ccdGeomToCyl(geom, cylinder);
		ccd_vec3_t v = new ccd_vec3_t();
		ccd_vec3_t dir = new ccd_vec3_t(-1, 0.2, -0.5);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportCyl.run(cylinder, dir, v);
		assertEquals(5.097096621545399, v.get0(), 0.00000001f);
		assertEquals(-4.01941932430908, v.get1(), 0.00000001f);
		assertEquals(-2, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, 0, 2);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportCyl.run(cylinder, dir, v);
		assertEquals(15.0, v.get0(), 0.00000001f);
		assertEquals(-5.0, v.get1(), 0.00000001f);
		assertEquals(6, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(1, 1, 1);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportCyl.run(cylinder, dir, v);
		assertEquals(13.535533905932738, v.get0(), 0.00000001f);
		assertEquals(-1.4644660940672627, v.get1(), 0.00000001f);
		assertEquals(6.0, v.get2(), 0.00000001f);
		dir = new ccd_vec3_t(0, 0, 1);
		ccdVec3Normalize(dir);
		CollisionLibccd.ccdSupportCyl.run(cylinder, dir, v);
		assertEquals(10, v.get0(), 0.00000001f);
		assertEquals(-5, v.get1(), 0.00000001f);
		assertEquals(6.0, v.get2(), 0.00000001f);
	}
}
