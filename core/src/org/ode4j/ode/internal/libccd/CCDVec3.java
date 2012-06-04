/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010,2011 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2007-2012 Tilmann Zäschke <ode4j@gmx.de>  
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

import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.Common;

public class CCDVec3 {
	//TZ
	public static final double M_PI = Math.PI;
	
	//# define CCD_EPS 1E-10
	public static final double CCD_EPS = Common.DBL_EPSILON;

	public static final double CCD_REAL_MAX = Double.MAX_VALUE;//DBL_MAX;

	//# define CCD_REAL(x) (x)       /*!< form a finalant */
	//# define CCD_SQRT(x) (sqrt(x)) /*!< square root */
	public static final double CCD_SQRT(double x) { return Math.sqrt(x); }
	/* absolute value */
	private static final double CCD_FABS(double x) { return Math.abs(x); }
	/* maximum of two floats */
	//private static final double CCD_FMAX(double x, double y) { return x > y ? x : y; }
	/* minimum of two floats */
	static final double CCD_FMIN(double x, double y) { return x < y ? x : y; }
	
	//#define CCD_ONE CCD_REAL(1.)
	public static final double CCD_ONE = 1;
	//#define CCD_ZERO CCD_REAL(0.)
	public static final double CCD_ZERO = 0;

	public static class ccd_vec3_t {
		final double[] v = new double[3];
	    public ccd_vec3_t(double x, double y, double z) {
			v[0] = x;
			v[1] = y;
			v[2] = z;
		}

		public ccd_vec3_t() {
			// TODO Auto-generated constructor stub
		}

		public void set(double x, double y, double z) {
			v[0] = x;
			v[1] = y;
			v[2] = z;
		}
		public double get0() {
			return v[0];
		}
		public double get1() {
			return v[1];
		}
		public double get2() {
			return v[2];
		}

		public void add(int pos, double d) {
			v[pos] += d;
		}
	};


	private static final ccd_vec3_t CCD_VEC3_STATIC(double x, double y, double z) {
		return new ccd_vec3_t(x, y, z);
	}

	private static final ccd_vec3_t CCD_VEC3(double x, double y, double z) { 
		return new ccd_vec3_t(x, y, z);
	}



	/**** INLINES ****/
	/** Returns sign of value. */
	public static final int ccdSign(double val)
	{
	    if (ccdIsZero(val)){
	        return 0;
	    }else if (val < CCD_ZERO){
	        return -1;
	    }
	    return 1;
	}

	/** Returns true if val is zero. **/
	public static final boolean ccdIsZero(double val)
	{
	    return CCD_FABS(val) < CCD_EPS;
	}

	/** Returns true if a and b equal. **/
	public static final boolean ccdEq(double _a, double _b)
	{
	    double ab;
		double a, b;

	    ab = CCD_FABS(_a - _b);
	    if (CCD_FABS(ab) < CCD_EPS)
	        return true;//1;

	    a = CCD_FABS(_a);
	    b = CCD_FABS(_b);
	    if (b > a){
	        return ab < CCD_EPS * b;
	    }else{
	        return ab < CCD_EPS * a;
	    }
	}


	public static final double ccdVec3X(final ccd_vec3_t v)
	{
	    return v.v[0];
	}

	public static final double ccdVec3Y(final ccd_vec3_t v)
	{
	    return v.v[1];
	}

	public static final double ccdVec3Z(final ccd_vec3_t v)
	{
	    return v.v[2];
	}

	/**
	 * Returns true if a and b equal.
	 */
	public static final boolean ccdVec3Eq(final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    return ccdEq(ccdVec3X(a), ccdVec3X(b))
	            && ccdEq(ccdVec3Y(a), ccdVec3Y(b))
	            && ccdEq(ccdVec3Z(a), ccdVec3Z(b));
	}

	/**
	 * Returns squared length of vector.
	 */
	public static final double ccdVec3Len2(final ccd_vec3_t v)
	{
	    return ccdVec3Dot(v, v);
	}

	/**
	 * Returns distance between a and b.
	 */
	public static final double ccdVec3Dist2(final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    ccd_vec3_t ab = new ccd_vec3_t();
	    ccdVec3Sub2(ab, a, b);
	    return ccdVec3Len2(ab);
	}

	public static final void ccdVec3Set(ccd_vec3_t v, double x, double y, double z)
	{
	    v.v[0] = x;
	    v.v[1] = y;
	    v.v[2] = z;
	}

	public static final void ccdVec3Set(ccd_vec3_t v, DVector3C xyz)
	{
	    v.v[0] = xyz.get0();
	    v.v[1] = xyz.get1();
	    v.v[2] = xyz.get2();
	}

	/**
	 * v = w
	 */
	public static final void ccdVec3Copy(ccd_vec3_t v, final ccd_vec3_t w)
	{
	    //*v = *w;
		v.v[0] = w.v[0];
		v.v[1] = w.v[1];
		v.v[2] = w.v[2];
	}

	/**
	 * Substracts coordinates of vector w from vector v. v = v - w
	 */
	static final void ccdVec3Sub(ccd_vec3_t v, final ccd_vec3_t w)
	{
	    v.v[0] -= w.v[0];
	    v.v[1] -= w.v[1];
	    v.v[2] -= w.v[2];
	}
	/**
	 * d = v - w
	 */
	static final void ccdVec3Sub2(ccd_vec3_t d, final ccd_vec3_t v, final ccd_vec3_t w)
	{
	    d.v[0] = v.v[0] - w.v[0];
	    d.v[1] = v.v[1] - w.v[1];
	    d.v[2] = v.v[2] - w.v[2];
	}

	/**
	 * Adds coordinates of vector w to vector v. v = v + w
	 */
	public static final void ccdVec3Add(ccd_vec3_t v, final ccd_vec3_t w)
	{
	    v.v[0] += w.v[0];
	    v.v[1] += w.v[1];
	    v.v[2] += w.v[2];
	}

	/**
	 * d = d * k;
	 */
	public static final void ccdVec3Scale(ccd_vec3_t d, double k)
	{
	    d.v[0] *= k;
	    d.v[1] *= k;
	    d.v[2] *= k;
	}

	/**
	 * Normalizes given vector to unit length.
	 */
	static final void ccdVec3Normalize(ccd_vec3_t d)
	{
	    double k = CCD_ONE / CCD_SQRT(ccdVec3Len2(d));
	    ccdVec3Scale(d, k);
	}

	/**
	 * Dot product of two vectors.
	 */
	public static final double ccdVec3Dot(final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    double dot;

	    dot  = a.v[0] * b.v[0];
	    dot += a.v[1] * b.v[1];
	    dot += a.v[2] * b.v[2];
	    return dot;
	}

	/**
	 * Cross product: d = a x b.
	 */
	static final void ccdVec3Cross(ccd_vec3_t d, final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    d.v[0] = (a.v[1] * b.v[2]) - (a.v[2] * b.v[1]);
	    d.v[1] = (a.v[2] * b.v[0]) - (a.v[0] * b.v[2]);
	    d.v[2] = (a.v[0] * b.v[1]) - (a.v[1] * b.v[0]);
	}
	//static CCD_VEC3(__ccd_vec3_origin, CCD_ZERO, CCD_ZERO, CCD_ZERO);
	/**
	 * Holds origin (0,0,0) - this variable is meant to be read-only!
	 */
	//extern
	static ccd_vec3_t ccd_vec3_origin = CCD_VEC3(CCD_ZERO, CCD_ZERO, CCD_ZERO);
	//ccd_vec3_t *ccd_vec3_origin = &__ccd_vec3_origin;

	/**
	 * Array of points uniformly distributed on unit sphere.
	 */
	static ccd_vec3_t points_on_sphere[] = {
		CCD_VEC3_STATIC(( 0.000000), (-0.000000), (-1.000000)),
		CCD_VEC3_STATIC(( 0.723608), (-0.525725), (-0.447219)),
		CCD_VEC3_STATIC((-0.276388), (-0.850649), (-0.447219)),
		CCD_VEC3_STATIC((-0.894426), (-0.000000), (-0.447216)),
		CCD_VEC3_STATIC((-0.276388), ( 0.850649), (-0.447220)),
		CCD_VEC3_STATIC(( 0.723608), ( 0.525725), (-0.447219)),
		CCD_VEC3_STATIC(( 0.276388), (-0.850649), ( 0.447220)),
		CCD_VEC3_STATIC((-0.723608), (-0.525725), ( 0.447219)),
		CCD_VEC3_STATIC((-0.723608), ( 0.525725), ( 0.447219)),
		CCD_VEC3_STATIC(( 0.276388), ( 0.850649), ( 0.447219)),
		CCD_VEC3_STATIC(( 0.894426), ( 0.000000), ( 0.447216)),
		CCD_VEC3_STATIC((-0.000000), ( 0.000000), ( 1.000000)), 
		CCD_VEC3_STATIC(( 0.425323), (-0.309011), (-0.850654)),
		CCD_VEC3_STATIC((-0.162456), (-0.499995), (-0.850654)),
		CCD_VEC3_STATIC(( 0.262869), (-0.809012), (-0.525738)),
		CCD_VEC3_STATIC(( 0.425323), ( 0.309011), (-0.850654)),
		CCD_VEC3_STATIC(( 0.850648), (-0.000000), (-0.525736)),
		CCD_VEC3_STATIC((-0.525730), (-0.000000), (-0.850652)),
		CCD_VEC3_STATIC((-0.688190), (-0.499997), (-0.525736)),
		CCD_VEC3_STATIC((-0.162456), ( 0.499995), (-0.850654)),
		CCD_VEC3_STATIC((-0.688190), ( 0.499997), (-0.525736)),
		CCD_VEC3_STATIC(( 0.262869), ( 0.809012), (-0.525738)),
		CCD_VEC3_STATIC(( 0.951058), ( 0.309013), ( 0.000000)),
		CCD_VEC3_STATIC(( 0.951058), (-0.309013), ( 0.000000)),
		CCD_VEC3_STATIC(( 0.587786), (-0.809017), ( 0.000000)),
		CCD_VEC3_STATIC(( 0.000000), (-1.000000), ( 0.000000)),
		CCD_VEC3_STATIC((-0.587786), (-0.809017), ( 0.000000)),
		CCD_VEC3_STATIC((-0.951058), (-0.309013), (-0.000000)),
		CCD_VEC3_STATIC((-0.951058), ( 0.309013), (-0.000000)),
		CCD_VEC3_STATIC((-0.587786), ( 0.809017), (-0.000000)),
		CCD_VEC3_STATIC((-0.000000), ( 1.000000), (-0.000000)),
		CCD_VEC3_STATIC(( 0.587786), ( 0.809017), (-0.000000)),
		CCD_VEC3_STATIC(( 0.688190), (-0.499997), ( 0.525736)),
		CCD_VEC3_STATIC((-0.262869), (-0.809012), ( 0.525738)),
		CCD_VEC3_STATIC((-0.850648), ( 0.000000), ( 0.525736)),
		CCD_VEC3_STATIC((-0.262869), ( 0.809012), ( 0.525738)),
		CCD_VEC3_STATIC(( 0.688190), ( 0.499997), ( 0.525736)),
		CCD_VEC3_STATIC(( 0.525730), ( 0.000000), ( 0.850652)),
		CCD_VEC3_STATIC(( 0.162456), (-0.499995), ( 0.850654)),
		CCD_VEC3_STATIC((-0.425323), (-0.309011), ( 0.850654)),
		CCD_VEC3_STATIC((-0.425323), ( 0.309011), ( 0.850654)),
		CCD_VEC3_STATIC(( 0.162456), ( 0.499995), ( 0.850654))
	};
	//ccd_vec3_t *ccd_points_on_sphere = points_on_sphere;
	static final ccd_vec3_t[] ccd_points_on_sphere = points_on_sphere;
	//private static final int ccd_points_on_sphere_len = sizeof(points_on_sphere) / sizeof(ccd_vec3_t);
	static final int ccd_points_on_sphere_len = points_on_sphere.length;


	static final double __ccdVec3PointSegmentDist2(final ccd_vec3_t P,
	                                                  final ccd_vec3_t x0,
	                                                  final ccd_vec3_t b,
	                                                  ccd_vec3_t witness)
	{
	    // The computation comes from solving equation of segment:
	    //      S(t) = x0 + t.d
	    //          where - x0 is initial point of segment
	    //                - d is direction of segment from x0 (|d| > 0)
	    //                - t belongs to <0, 1> interval
	    // 
	    // Than, distance from a segment to some point P can be expressed:
	    //      D(t) = |x0 + t.d - P|^2
	    //          which is distance from any point on segment. Minimization
	    //          of this function brings distance from P to segment.
	    // Minimization of D(t) leads to simple quadratic equation that's
	    // solving is straightforward.
	    //
	    // Bonus of this method is witness point for free.

	    double dist, t;
	    ccd_vec3_t d = new ccd_vec3_t(), a = new ccd_vec3_t();

	    // direction of segment
	    ccdVec3Sub2(d, b, x0);

	    // precompute vector from P to x0
	    ccdVec3Sub2(a, x0, P);

	    t  = -(1.) * ccdVec3Dot(a, d);
	    t /= ccdVec3Len2(d);

	    if (t < CCD_ZERO || ccdIsZero(t)){
	        dist = ccdVec3Dist2(x0, P);
	        if (witness!=null)
	            ccdVec3Copy(witness, x0);
	    }else if (t > CCD_ONE || ccdEq(t, CCD_ONE)){
	        dist = ccdVec3Dist2(b, P);
	        if (witness!=null)
	            ccdVec3Copy(witness, b);
	    }else{
	        if (witness!=null){
	            ccdVec3Copy(witness, d);
	            ccdVec3Scale(witness, t);
	            ccdVec3Add(witness, x0);
	            dist = ccdVec3Dist2(witness, P);
	        }else{
	            // recycling variables
	            ccdVec3Scale(d, t);
	            ccdVec3Add(d, a);
	            dist = ccdVec3Len2(d);
	        }
	    }

	    return dist;
	}

	/**
	 * Returns distance2 of point P to segment ab.
	 * If witness is non-NULL it is filled with coordinates of point from which
	 * was computed distance to point P.
	 */
	public static double ccdVec3PointSegmentDist2(final ccd_vec3_t P,
	                                    final ccd_vec3_t x0, final ccd_vec3_t b,
	                                    ccd_vec3_t witness)
	{
	    return __ccdVec3PointSegmentDist2(P, x0, b, witness);
	}

	/**
	 * Returns distance2 of point P from triangle formed by triplet a, b, c.
	 * If witness vector is provided it is filled with coordinates of point
	 * from which was computed distance to point P.
	 */
	public static double ccdVec3PointTriDist2(final ccd_vec3_t P,
	                                final ccd_vec3_t x0, final ccd_vec3_t B,
	                                final ccd_vec3_t C,
	                                ccd_vec3_t witness)
	{
	    // Computation comes from analytic expression for triangle (x0, B, C)
	    //      T(s, t) = x0 + s.d1 + t.d2, where d1 = B - x0 and d2 = C - x0 and
	    // Then equation for distance is:
	    //      D(s, t) = | T(s, t) - P |^2
	    // This leads to minimization of quadratic function of two variables.
	    // The solution from is taken only if s is between 0 and 1, t is
	    // between 0 and 1 and t + s < 1, otherwise distance from segment is
	    // computed.

	    ccd_vec3_t d1 = new ccd_vec3_t(), d2 = new ccd_vec3_t(), a = new ccd_vec3_t();
	    double u, v, w, p, q, r;
	    double s, t, dist, dist2;
	    ccd_vec3_t witness2 = new ccd_vec3_t();

	    ccdVec3Sub2(d1, B, x0);
	    ccdVec3Sub2(d2, C, x0);
	    ccdVec3Sub2(a, x0, P);

	    u = ccdVec3Dot(a, a);
	    v = ccdVec3Dot(d1, d1);
	    w = ccdVec3Dot(d2, d2);
	    p = ccdVec3Dot(a, d1);
	    q = ccdVec3Dot(a, d2);
	    r = ccdVec3Dot(d1, d2);

	    s = (q * r - w * p) / (w * v - r * r);
	    t = (-s * r - q) / w;

	    if ((ccdIsZero(s) || s > CCD_ZERO)
	            && (ccdEq(s, CCD_ONE) || s < CCD_ONE)
	            && (ccdIsZero(t) || t > CCD_ZERO)
	            && (ccdEq(t, CCD_ONE) || t < CCD_ONE)
	            && (ccdEq(t + s, CCD_ONE) || t + s < CCD_ONE)){

	        if (witness!=null){
	            ccdVec3Scale(d1, s);
	            ccdVec3Scale(d2, t);
	            ccdVec3Copy(witness, x0);
	            ccdVec3Add(witness, d1);
	            ccdVec3Add(witness, d2);

	            dist = ccdVec3Dist2(witness, P);
	        }else{
	            dist  = s * s * v;
	            dist += t * t * w;
	            dist += (2.) * s * t * r;
	            dist += (2.) * s * p;
	            dist += (2.) * t * q;
	            dist += u;
	        }
	    }else{
	        dist = __ccdVec3PointSegmentDist2(P, x0, B, witness);

	        dist2 = __ccdVec3PointSegmentDist2(P, x0, C, witness2);
	        if (dist2 < dist){
	            dist = dist2;
	            if (witness!=null)
	                ccdVec3Copy(witness, witness2);
	        }

	        dist2 = __ccdVec3PointSegmentDist2(P, B, C, witness2);
	        if (dist2 < dist){
	            dist = dist2;
	            if (witness!=null)
	                ccdVec3Copy(witness, witness2);
	        }
	    }

	    return dist;
	}

}
