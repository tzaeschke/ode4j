package org.ode4j.libccd;

import org.junit.Test;
import org.ode4j.ode.internal.libccd.CCDQuat;
import org.ode4j.ode.internal.libccd.CCDQuat.ccd_quat_t;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

import static org.junit.Assert.*;

public class TestQuat {

	@Test
	public void testRotation() {
		ccd_quat_t q = new ccd_quat_t();
		ccd_vec3_t a = new ccd_vec3_t(0, 1, 0);
		CCDQuat.ccdQuatSetAngleAxis(q, Math.PI / 4, a);
		ccd_vec3_t v = new ccd_vec3_t(1, 0, 0);
		CCDQuat.ccdQuatRotVec(v, q);
		assertEquals(Math.sqrt(2) / 2, v.get0(), 0.00001f);
		assertEquals(-Math.sqrt(2) / 2, v.get2(), 0.00001f);
		CCDQuat.ccdQuatSetAngleAxis(q, Math.PI / 6, a);
		v = new ccd_vec3_t(1, 0, 0);
		CCDQuat.ccdQuatRotVec(v, q);
		assertEquals(Math.sqrt(3) / 2, v.get0(), 0.00001f);
		assertEquals(-0.5, v.get2(), 0.00001f);
		a.set(Math.sqrt(3), Math.sqrt(3), Math.sqrt(3));
		CCDQuat.ccdQuatSetAngleAxis(q, Math.PI / 3, a);
		v = new ccd_vec3_t(1, 1, 1);
		CCDQuat.ccdQuatRotVec(v, q);
		assertEquals(1.0, v.get0(), 0.00001f);
		assertEquals(1.0, v.get1(), 0.00001f);
		assertEquals(1.0, v.get2(), 0.00001f);
		v = new ccd_vec3_t(1, 3, 1);
		CCDQuat.ccdQuatSetAngleAxis(q, Math.PI / 6, a);
		CCDQuat.ccdQuatRotVec(v, q);
		assertEquals(0.511966612, v.get0(), 0.00001f);
		assertEquals(2.821367205, v.get1(), 0.00001f);
		assertEquals(1.666666666, v.get2(), 0.00001f);
	}
}
