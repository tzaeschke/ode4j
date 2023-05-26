/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2009-2014 Tilmann Zaeschke<ode4j@gmx.de>  
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */
package org.ode4j.ode.internal.libccd;

import static org.ode4j.ode.internal.libccd.CCDVec3.*;

public class CCDQuat {

	public static class ccd_quat_t {
		//TZ: fields are much faster than arrays.
	    double q0, q1, q2, q3; //!< x, y, z, w
		public ccd_quat_t() {}
	    public void set(double x, double y, double z, double w) {
	    	q0 = x;
	    	q1 = y;
	    	q2 = z;
	    	q3 = w;
	    }
		public Object get0() {
			return q0;
		}
		public Object get1() {
			return q1;
		}
		public Object get2() {
			return q2;
		}
		public Object get3() {
			return q3;
		}
	};


	/**** INLINES ****/
	static double ccdQuatLen2(final ccd_quat_t q)
	{
	    double len;

	    len  = q.q0 * q.q0;
	    len += q.q1 * q.q1;
	    len += q.q2 * q.q2;
	    len += q.q3 * q.q3;

	    return len;
	}

	static double ccdQuatLen(final ccd_quat_t q)
	{
	    return CCD_SQRT(ccdQuatLen2(q));
	}

	public static void ccdQuatSet(ccd_quat_t q, double x, double y, double z, double w)
	{
	    q.q0 = x;
	    q.q1 = y;
	    q.q2 = z;
	    q.q3 = w;
	}

	static void ccdQuatCopy(ccd_quat_t dest, final ccd_quat_t src)
	{
		//*dest = *src;
	    dest.q0 = src.q0;
	    dest.q1 = src.q1;
	    dest.q2 = src.q2;
	    dest.q3 = src.q3;
	}


	static int ccdQuatNormalize(ccd_quat_t q)
	{
	    double len = ccdQuatLen(q);
	    if (len < CCD_EPS)
	        return 0;

	    ccdQuatScale(q, CCD_ONE / len);
	    return 1;
	}

	public static void ccdQuatSetAngleAxis(ccd_quat_t q,
										   double angle, final ccd_vec3_t axis)
	{
	    double a, x, y, z, n, s;

	    a = angle/2;
	    x = ccdVec3X(axis);
	    y = ccdVec3Y(axis);
	    z = ccdVec3Z(axis);
	    n = CCD_SQRT(x*x + y*y + z*z);

	    // axis==0? (treat this the same as angle==0 with an arbitrary axis)
	    if (n < CCD_EPS){
	        q.q0 = q.q1 = q.q2 = CCD_ZERO;
	        q.q3 = CCD_ONE;
	    }else{
	        s = Math.sin(a)/n;

	        q.q3 = Math.cos(a);
	        q.q0 = x*s;
	        q.q1 = y*s;
	        q.q2 = z*s;

	        ccdQuatNormalize(q);
	    }
	}


	static void ccdQuatScale(ccd_quat_t q, double k)
	{
		q.q0 *= k;
		q.q1 *= k;
		q.q2 *= k;
		q.q3 *= k;
	}

	
	/**
	 * q = q * q2
	 * @param q q
	 * @param q2 q2
	 */
	public static void ccdQuatMul(ccd_quat_t q, final ccd_quat_t q2)
	{
	    ccd_quat_t a = new ccd_quat_t();
	    ccdQuatCopy(a, q);
	    ccdQuatMul2(q, a, q2);
	}

	/**
	 * q = a * b
	 */
	static void ccdQuatMul2(ccd_quat_t q,
							final ccd_quat_t a, final ccd_quat_t b)
	{
	    q.q0 = a.q3 * b.q0
	                + a.q0 * b.q3
	                + a.q1 * b.q2
	                - a.q2 * b.q1;
	    q.q1 = a.q3 * b.q1
	                + a.q1 * b.q3
	                - a.q0 * b.q2
	                + a.q2 * b.q0;
	    q.q2 = a.q3 * b.q2
	                + a.q2 * b.q3
	                + a.q0 * b.q1
	                - a.q1 * b.q0;
	    q.q3 = a.q3 * b.q3
	                - a.q0 * b.q0
	                - a.q1 * b.q1
	                - a.q2 * b.q2;
	}

	
	/**
	 * Inverts quaternion.
	 * Returns 0 on success.
	 */
	static int ccdQuatInvert(ccd_quat_t q)
	{
	    double len2 = ccdQuatLen2(q);
	    if (len2 < CCD_EPS)
	        return -1;

	    len2 = CCD_ONE / len2;

	    q.q0 = -q.q0 * len2;
	    q.q1 = -q.q1 * len2;
	    q.q2 = -q.q2 * len2;
	    q.q3 = q.q3 * len2;

	    return 0;
	}
	
	/**
	 * Inverts quaternion.
	 * @param dest dest 
	 * @param src src
	 * @return 0 on success.
	 */
	public static int ccdQuatInvert2(ccd_quat_t dest, final ccd_quat_t src)
	{
	    ccdQuatCopy(dest, src);
	    return ccdQuatInvert(dest);
	}

	/**
	 * Rotate vector v by quaternion q.
	 * @param v v
	 * @param q q
	 */
	public static void ccdQuatRotVec(ccd_vec3_t v, final ccd_quat_t q) {
		// original version: 31 mul + 21 add
		// optimized version: 18 mul + 12 add
		// formula: v = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
		double cross1_x, cross1_y, cross1_z, cross2_x, cross2_y, cross2_z;
		double x, y, z, w;
		double vx, vy, vz;

		vx = ccdVec3X(v);
		vy = ccdVec3Y(v);
		vz = ccdVec3Z(v);

		w = q.q3;
		x = q.q0;
		y = q.q1;
		z = q.q2;

		cross1_x = y * vz - z * vy + w * vx;
		cross1_y = z * vx - x * vz + w * vy;
		cross1_z = x * vy - y * vx + w * vz;
		cross2_x = y * cross1_z - z * cross1_y;
		cross2_y = z * cross1_x - x * cross1_z;
		cross2_z = x * cross1_y - y * cross1_x;
		ccdVec3Set(v, vx + 2 * cross2_x, vy + 2 * cross2_y, vz + 2 * cross2_z);
	}

	private CCDQuat() {}
}
