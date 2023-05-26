/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010,2011 Daniel Fiser <danfis@danfis.cz>
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
	public static double CCD_SQRT(double x) { return Math.sqrt(x); }
	/* absolute value */
	private static double CCD_FABS(double x) { return Math.abs(x); }
	/* maximum of two floats */
	public static double CCD_FMAX(double x, double y) { return Math.max(x, y); }
	/* minimum of two floats */
	static double CCD_FMIN(double x, double y) { return Math.min(x, y); }

	//#define CCD_ONE CCD_REAL(1.)
	public static final double CCD_ONE = 1;
	//#define CCD_ZERO CCD_REAL(0.)
	public static final double CCD_ZERO = 0;

	public static class ccd_vec3_t {
		//TZ: fields are much faster than arrays.
		double v0;
		double v1;
		double v2;
	    public ccd_vec3_t(double x, double y, double z) {
			v0 = x;
			v1 = y;
			v2 = z;
		}

		public ccd_vec3_t() {
			// TODO Auto-generated constructor stub
		}

		public void set(double x, double y, double z) {
			v0 = x;
			v1 = y;
			v2 = z;
		}
		public double get0() {
			return v0;
		}
		public double get1() {
			return v1;
		}
		public double get2() {
			return v2;
		}

		public void add(int pos, double d) {
			switch(pos) {
			case 0: v0+=d; break;
			case 1: v1+=d; break;
			case 2: v2+=d; break;
			default:
				throw new IllegalArgumentException();
			}
		}

		@Override
		public String toString() {
			return "CCDVec3[ " + get0() + ", " + get1() + ", " + get2() + " ]";
		}
	}


	private static ccd_vec3_t CCD_VEC3_STATIC(double x, double y, double z) {
		return new ccd_vec3_t(x, y, z);
	}

	private static ccd_vec3_t CCD_VEC3(double x, double y, double z) {
		return new ccd_vec3_t(x, y, z);
	}



	/* *** INLINES *** */
	/** 
	 * @param val val
	 * @return sign of value. 
	 */
	public static int ccdSign(double val)
	{
	    if (ccdIsZero(val)){
	        return 0;
	    }else if (val < CCD_ZERO){
	        return -1;
	    }
	    return 1;
	}

	/** 
	 * @param val value
	 * @return true if val is zero. 
	 **/
	public static boolean ccdIsZero(double val)
	{
	    return CCD_FABS(val) < CCD_EPS;
	}

	/** 
	 * @param _a a
	 * @param _b b
	 * @return true if a and b equal. 
	 **/
	public static boolean ccdEq(double _a, double _b)
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


	public static double ccdVec3X(final ccd_vec3_t v)
	{
	    return v.v0;
	}

	public static double ccdVec3Y(final ccd_vec3_t v)
	{
	    return v.v1;
	}

	public static double ccdVec3Z(final ccd_vec3_t v)
	{
	    return v.v2;
	}

	/**
	 * @param a a
	 * @param b b
	 * @return true if a and b equal.
	 */
	public static boolean ccdVec3Eq(final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    return ccdEq(ccdVec3X(a), ccdVec3X(b))
	            && ccdEq(ccdVec3Y(a), ccdVec3Y(b))
	            && ccdEq(ccdVec3Z(a), ccdVec3Z(b));
	}

	/**
	 * @param v v
	 * @return squared length of vector.
	 */
	public static double ccdVec3Len2(final ccd_vec3_t v)
	{
	    return ccdVec3Dot(v, v);
	}

	/**
	 * @param a a
	 * @param b b
	 * @return distance between a and b.
	 */
	public static double ccdVec3Dist2(final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    ccd_vec3_t ab = new ccd_vec3_t();
	    ccdVec3Sub2(ab, a, b);
	    return ccdVec3Len2(ab);
	}

	public static void ccdVec3Set(ccd_vec3_t v, double x, double y, double z)
	{
	    v.v0 = x;
	    v.v1 = y;
	    v.v2 = z;
	}

	public static void ccdVec3Set(ccd_vec3_t v, DVector3C xyz)
	{
	    v.v0 = xyz.get0();
	    v.v1 = xyz.get1();
	    v.v2 = xyz.get2();
	}

	/**
	 * v = w
	 * @param v v
	 * @param w w
	 */
	public static void ccdVec3Copy(ccd_vec3_t v, final ccd_vec3_t w)
	{
	    //*v = *w;
		v.v0 = w.v0;
		v.v1 = w.v1;
		v.v2 = w.v2;
	}

	/**
	 * Subtracts coordinates of vector w from vector v. v = v - w
	 * @param v v
	 * @param w w
	 */
	public static void ccdVec3Sub(ccd_vec3_t v, final ccd_vec3_t w)
	{
	    v.v0 -= w.v0;
	    v.v1 -= w.v1;
	    v.v2 -= w.v2;
	}

	/**
	 * Adds coordinates of vector w to vector v. v = v + w
	 * @param v v
	 * @param w w
	 */
	public static void ccdVec3Add(ccd_vec3_t v, final ccd_vec3_t w)
	{
		v.v0 += w.v0;
		v.v1 += w.v1;
		v.v2 += w.v2;
	}

	/**
	 * d = v - w
	 * @param d d
	 * @param v v
	 * @param w w
	 */
	public static void ccdVec3Sub2(ccd_vec3_t d, final ccd_vec3_t v, final ccd_vec3_t w)
	{
	    d.v0 = v.v0 - w.v0;
	    d.v1 = v.v1 - w.v1;
	    d.v2 = v.v2 - w.v2;
	}

	/**
	 * d = d * k;
	 *
	 * @param d d
	 * @param k k
	 */
	public static void ccdVec3Scale(ccd_vec3_t d, double k) {
		d.v0 *= k;
		d.v1 *= k;
		d.v2 *= k;
	}

	/**
	 * Normalizes given vector to unit length.
	 * @param d d
	 */
	public static void ccdVec3Normalize(ccd_vec3_t d)
	{
		double k = CCD_ONE / CCD_SQRT(ccdVec3Len2(d));
		ccdVec3Scale(d, k);
	}

	/**
	 * @param a a
	 * @param b b
	 * @return Dot product of two vectors.
	 */
	public static double ccdVec3Dot(final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    double dot;

	    dot  = a.v0 * b.v0;
	    dot += a.v1 * b.v1;
	    dot += a.v2 * b.v2;
	    return dot;
	}

	/**
	 * Cross product: d = a x b.
	 * @param d d
	 * @param a a
	 * @param b b
	 */
	public static void ccdVec3Cross(ccd_vec3_t d, final ccd_vec3_t a, final ccd_vec3_t b)
	{
	    d.v0 = (a.v1 * b.v2) - (a.v2 * b.v1);
	    d.v1 = (a.v2 * b.v0) - (a.v0 * b.v2);
	    d.v2 = (a.v0 * b.v1) - (a.v1 * b.v0);
	}
	//static CCD_VEC3(__ccd_vec3_origin, CCD_ZERO, CCD_ZERO, CCD_ZERO);
	/**
	 * Holds origin (0,0,0) - this variable is meant to be read-only!
	 */
	//extern
	static final ccd_vec3_t ccd_vec3_origin = CCD_VEC3(CCD_ZERO, CCD_ZERO, CCD_ZERO);
	//ccd_vec3_t *ccd_vec3_origin = &__ccd_vec3_origin;

	/**
	 * Array of points uniformly distributed on unit sphere.
	 */
	private static final ccd_vec3_t[] points_on_sphere = {
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


	static double __ccdVec3PointSegmentDist2(final ccd_vec3_t P,
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
	 * @param P P
	 * @param x0 x0
	 * @param b b
	 * @param witness witness 
	 * @return distance distance
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
	 * @param P PO
	 * @param x0 x0
	 * @param B B
	 * @param C C
	 * @param witness witness 
	 * @return distance
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

	private CCDVec3() {}
}
