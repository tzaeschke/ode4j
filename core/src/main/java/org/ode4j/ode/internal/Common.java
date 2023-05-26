/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;

import static org.ode4j.ode.internal.ErrorHandler.*;


/**
 * configuration stuff.
 */
@SuppressWarnings("unused")
public class Common extends OdeConstants {

	/* configuration stuff */

	/** constants */

	public static final double M_PI = 3.1415926535897932384626433832795029;
	public static final double M_PI_2 = 1.5707963267948966192313216916398;
	public static final double M_SQRT1_2 = 0.7071067811865475244008443621048490;

	/** debugging:
	 *   IASSERT  is an internal assertion, i.e. a consistency check. if it fails
	 *            we want to know where.
	 *   UASSERT  is a user assertion, i.e. if it fails a nice error message
	 *            should be printed for the user.
	 *   AASSERT  is an arguments assertion, i.e. if it fails "bad argument(s)"
	 *            is printed.
	 *   DEBUGMSG just prints out a message
	 */

	public static final boolean dNODEBUG = !OdeConfig.dDEBUG;


	public static final boolean dDOUBLE = true;
	public static final boolean dSINGLE = false;
	public static final double dEpsilon;
	public static final double MAX_FLOAT;
	static {
		if (dDOUBLE == dSINGLE) {
			throw new RuntimeException("dDOUBLE == dSINGLE");
		}
		if (dDOUBLE) {
			// dEpsilon = FLT_EPSILON or DBL_EPSILON
			dEpsilon = Double.MIN_NORMAL;
			MAX_FLOAT = Double.MAX_VALUE;
		} else {
			dEpsilon = Float.MIN_NORMAL;
			MAX_FLOAT = Float.MAX_VALUE;
		}
	}

	// Use the error-checking memory allocation system.  Because this system uses heap
	//  (malloc) instead of stack (alloca), it is slower.  However, it allows you to
	//  simulate larger scenes, as well as handle out-of-memory errors in a somewhat
	//  graceful manner

	//#ifdef dUSE_MALLOC_FOR_ALLOCA
	//enum {
	//  d_MEMORY_OK = 0,              /* no memory errors */
	//  d_MEMORY_OUT_OF_MEMORY        /* malloc failed due to out of memory error */
	//};
	//#endif
	//TODO why check for dUSE_MALLOC_FOR_ALLOCA
	/** no memory errors. */
	public static final int d_MEMORY_OK = 0;
	/** malloc failed due to out of memory error. */
	public static final int d_MEMORY_OUT_OF_MEMORY = 1;


	//From config-defaults.h
	/** @deprecated TZ this can be removed? */
	@Deprecated
    public static final boolean  dATOMICS_ENABLED = false;
	public static final boolean  dTRIMESH_16BIT_INDICES = false;

	public static final boolean  dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER = false;

	//TODO use MACRO
	//#define EPSILON 0.0001   // Define your own tolerance
	//#define FLOAT_EQ(x,v) (((v - EPSILON) < x) && (x <( v + EPSILON)))
	//public static final double DBL_EPSILON = 2.22045e-16;
	public static final double DBL_EPSILON = 2.2204460492503131e-016;

	public static void dIVERIFY(boolean a) {
		dIASSERT(a);
	}

	public static void dUVERIFY(boolean a, String msg) {
		dUASSERT(a, msg);
	}

	/**
	 * Internal assertion
	 * @param b Fail if 'false'
	 */
	public static void dIASSERT(boolean b) {
		if (!b) {
			dDebug(d_ERR_IASSERT, "assertion failed ");
			new RuntimeException().printStackTrace();
		}
	}

	public static void dUASSERT(DxBody  a, String msg) {
		if (a == null) {
			dDebug (d_ERR_UASSERT, msg);
		}
	}

	//TODO remove these checks?
	public static void dUASSERT(DVector3 a, String msg) {
		if (a == null) {
			dDebug (d_ERR_UASSERT, msg);
		}
	}

	public static void dUASSERT(boolean  a, String msg) {
		if (!a) {
			dDebug (d_ERR_UASSERT, msg);
		}
	}

	public static void dUASSERT(int  a, String msg) {
		if (a == 0) {
			dDebug (d_ERR_UASSERT, msg);
		}
	}

	public static void dDEBUGMSG(String msg) {
		dMessage (d_ERR_UASSERT, msg);
	}

//	#ifdef __GNUC__
//#define dUNUSED(Name) Name __attribute__((unused))
//			#else // not __GNUC__
//			#define dUNUSED(Name) Name
//#endif
//
//#if __cplusplus >= 201103L
//			#define dSASSERT(e)  static_assert(e, #e)
//			#define dSMSGASSERT(e, message)  static_assert(e, message)
//			#else
//			#define d_SASSERT_INNER_TOKENPASTE(x, y) x ## y
//#define d_SASSERT_TOKENPASTE(x, y) d_SASSERT_INNER_TOKENPASTE(x, y)
//			#define dSASSERT(e) typedef char dUNUSED(d_SASSERT_TOKENPASTE(d_StaticAssertionFailed_, __LINE__)[(e)?1:-1])
//			#define dSMSGASSERT(e, message)  dSASSERT(e)
//			#endif

	public static void dSASSERT(boolean  a) {
		if (!a) {
			dDebug (d_ERR_SASSERT, "Static assert failed");
		}
	}

	//	#  ifdef __GNUC__
//	#    define dICHECK(a) { if (!(a)) { dDebug (d_ERR_IASSERT, \
//	      "assertion \"" #a "\" failed in %s() [%s:%u]",__FUNCTION__,__FILE__,__LINE__); *(int *)0 = 0; } }
//	#  else // not __GNUC__
//	#    define dICHECK(a) { if (!(a)) { dDebug (d_ERR_IASSERT, \
//	      "assertion \"" #a "\" failed in %s:%u",__FILE__,__LINE__); *(int *)0 = 0; } }
//	#  endif
	public static void dICHECK(boolean a) {
		if (!a) {
			dDebug(d_ERR_IASSERT, "", (Object[])null);
		}
	}

	/**
	 * Assert 'not-null'.
	 * @param aa Object to assert
	 */
	public static void dAASSERT(Object ... aa) {
		for (Object a: aa) {
			if (a == null) {
				dUASSERT(0, "Bad argument(s)");
			}
			if (a instanceof Integer) {
				dUASSERT((Integer)a, "Bad argument(s)");
			}
			if (a instanceof Boolean) {
				dUASSERT((Boolean)a, "Bad argument(s)");
			}
		}
	}

	public static void dAASSERT(DNearCallback a) {
		if (a == null) {
			dUASSERT(0, "Bad argument(s)");
		}
	}

	public static void dAASSERT(DWorld a) {
		if (a == null) {
			dUASSERT(0, "Bad argument(s)");
		}
	}

	/**
	 * Assert 'true'.
	 * @param b Fail if 'false'
	 */
	public static void dAASSERT(boolean b) {
		if (!b)
			dUASSERT(b, "Bad argument(s)");
	}

	public static void dAVERIFY(Object a) {
		dUVERIFY(a != null, "Bad argument(s)");
	}

	/* floating point data type, vector, matrix and quaternion types */

	//#if defined(dSINGLE)
	//typedef float dReal;
	//#ifdef dDOUBLE
	//#error You can only #define dSINGLE or dDOUBLE, not both.
	//#endif // dDOUBLE
	//#elif defined(dDOUBLE)
	//typedef double dReal;
	//#else
	//#error You must #define dSINGLE or dDOUBLE
	//#endif

	// Detect if we've got both trimesh engines enabled.

	public static final boolean dTRIMESH_ENABLED = true;
	public static final boolean dTRIMESH_OPCODE = false;
	public static final boolean dTRIMESH_GIMPACT = true;
	static {
		if (dTRIMESH_ENABLED) {
			if (dTRIMESH_OPCODE && dTRIMESH_GIMPACT) {
				System.err.println("You can only #define dTRIMESH_OPCODE " +
				"or dTRIMESH_GIMPACT, not both.");
			}
		}
	}

	//#if dTRIMESH_ENABLED
	//#if dTRIMESH_OPCODE && dTRIMESH_GIMPACT
	//#error You can only #define dTRIMESH_OPCODE or dTRIMESH_GIMPACT, not both.
	//#endif
	//#endif // dTRIMESH_ENABLED

	// Define a type for indices, either 16 or 32 bit, based on build option
	// TODO: Currently GIMPACT only supports 32 bit indices.
	//#if dTRIMESH_16BIT_INDICES
	//#if dTRIMESH_GIMPACT
	//typedef uint32 dTriIndex;
	//#else // dTRIMESH_GIMPACT
	//typedef uint16 dTriIndex;
	//#endif // dTRIMESH_GIMPACT
	//#else // dTRIMESH_16BIT_INDICES
	//typedef uint32 dTriIndex;
	//#endif // dTRIMESH_16BIT_INDICES
//	public static final Class<?> dTRIMESH = Integer.TYPE;

	/**
	 * Round an integer up to a multiple of 4, except that 0 and 1
	 * are unmodified (used to compute matrix leading dimensions).
	 * TODO Check that returned value is used!! (NOT Call by reference).
	 * deprecated-keep for now (Remove this if possible) (TZ)
	 * @param a a
	 * @return Padded offset
	 */
	public static int dPAD(int a) {
		return (((a) > 1) ? (((a) + 3) & (~3)) : (a));
	}
	//#define dPAD(a) (((a) > 1) ? (((a) + 3) & (int)(~3)) : (a))

	static class dSpaceAxis {
		public static final int dSA__MIN = 0;
		public static final int dSA_X = dSA__MIN;
		public static final int dSA_Y = 1;
		public static final int dSA_Z = 2;
		public static final int dSA__MAX = 3;
	}

	static class dMotionDynamics {
		public static final int dMD__MIN = 0;
		public static final int dMD_LINEAR = dMD__MIN;
		public static final int dMD_ANGULAR = 1;
		public static final int dMD__MAX = 2;
	}

	static class dDynamicsAxis {
		public static final int dDA__MIN = 0;
		public static final int dDA__L_MIN = dDA__MIN + dMotionDynamics.dMD_LINEAR * dSpaceAxis.dSA__MAX;
		public static final int dDA_LX = dDA__L_MIN + dSpaceAxis.dSA_X;
		public static final int dDA_LY = dDA__L_MIN + dSpaceAxis.dSA_Y;
		public static final int dDA_LZ = dDA__L_MIN + dSpaceAxis.dSA_Z;
		public static final int dDA__L_MAX = dDA__L_MIN + dSpaceAxis.dSA__MAX;
		public static final int dDA__A_MIN = dDA__MIN + dMotionDynamics.dMD_ANGULAR * dSpaceAxis.dSA__MAX;
		public static final int dDA_AX = dDA__A_MIN + dSpaceAxis.dSA_X;
		public static final int dDA_AY = dDA__A_MIN + dSpaceAxis.dSA_Y;
		public static final int dDA_AZ = dDA__A_MIN + dSpaceAxis.dSA_Z;
		public static final int dDA__A_MAX = dDA__A_MIN + dSpaceAxis.dSA__MAX;
		public static final int dDA__MAX = dDA__MIN + dMotionDynamics.dMD__MAX * dSpaceAxis.dSA__MAX;
	}

	// **********************************
	// The following can be found in CommonEnums.java
	// **********************************

//	static class dVec3Element {
//		public static final int dV3E__MIN = 0;
//		public static final int dV3E__AXES_MIN = dV3E__MIN;
//		public static final int dV3E_X = dV3E__AXES_MIN + dSpaceAxis.dSA_X;
//		public static final int dV3E_Y = dV3E__AXES_MIN + dSpaceAxis.dSA_Y;
//		public static final int dV3E_Z = dV3E__AXES_MIN + dSpaceAxis.dSA_Z;
//		public static final int dV3E__AXES_MAX = dV3E__AXES_MIN + dSpaceAxis.dSA__MAX;
//		public static final int dV3E_PAD = dV3E__AXES_MAX;
//		@Deprecated
//		public static final int dV3E__MAX = dV3E_PAD + 1;
//		public static final int dV3E__AXES_COUNT = dV3E__AXES_MAX - dV3E__AXES_MIN;
//	}
//
//	static class dVec4Element {
//		public static final int dV4E__MIN = 0;
//		public static final int dV4E_X = dV4E__MIN + dSpaceAxis.dSA_X;
//		public static final int dV4E_Y = dV4E__MIN + dSpaceAxis.dSA_Y;
//		public static final int dV4E_Z = dV4E__MIN + dSpaceAxis.dSA_Z;
//		public static final int dV4E_O = dV4E__MIN + dSpaceAxis.dSA__MAX;
//		@Deprecated
//		public static final int dV4E__MAX = dV4E_O + 1;
//	}
//
//	static class dMat3Element {
//		public static final int dM3E__MIN = 0;
//		public static final int dM3E__X_MIN = dM3E__MIN + dSpaceAxis.dSA_X * dVec3Element.dV3E__MAX;
//		public static final int dM3E__X_AXES_MIN = dM3E__X_MIN + dVec3Element.dV3E__AXES_MIN;
//		public static final int dM3E_XX = dM3E__X_MIN + dVec3Element.dV3E_X;
//		public static final int dM3E_XY = dM3E__X_MIN + dVec3Element.dV3E_Y;
//		public static final int dM3E_XZ = dM3E__X_MIN + dVec3Element.dV3E_Z;
//		public static final int dM3E__X_AXES_MAX = dM3E__X_MIN + dVec3Element.dV3E__AXES_MAX;
//		public static final int dM3E_XPAD = dM3E__X_MIN + dVec3Element.dV3E_PAD;
//		public static final int dM3E__X_MAX = dM3E__X_MIN + dVec3Element.dV3E__MAX;
//		public static final int dM3E__Y_MIN = dM3E__MIN + dSpaceAxis.dSA_Y * dVec3Element.dV3E__MAX;
//		public static final int dM3E__Y_AXES_MIN = dM3E__Y_MIN + dVec3Element.dV3E__AXES_MIN;
//		public static final int dM3E_YX = dM3E__Y_MIN + dVec3Element.dV3E_X;
//		public static final int dM3E_YY = dM3E__Y_MIN + dVec3Element.dV3E_Y;
//		public static final int dM3E_YZ = dM3E__Y_MIN + dVec3Element.dV3E_Z;
//		public static final int dM3E__Y_AXES_MAX = dM3E__Y_MIN + dVec3Element.dV3E__AXES_MAX;
//		public static final int dM3E_YPAD = dM3E__Y_MIN + dVec3Element.dV3E_PAD;
//		public static final int dM3E__Y_MAX = dM3E__Y_MIN + dVec3Element.dV3E__MAX;
//		public static final int dM3E__Z_MIN = dM3E__MIN + dSpaceAxis.dSA_Z * dVec3Element.dV3E__MAX;
//		public static final int dM3E__Z_AXES_MIN = dM3E__Z_MIN + dVec3Element.dV3E__AXES_MIN;
//		public static final int dM3E_ZX = dM3E__Z_MIN + dVec3Element.dV3E_X;
//		public static final int dM3E_ZY = dM3E__Z_MIN + dVec3Element.dV3E_Y;
//		public static final int dM3E_ZZ = dM3E__Z_MIN + dVec3Element.dV3E_Z;
//		public static final int dM3E__Z_AXES_MAX = dM3E__Z_MIN + dVec3Element.dV3E__AXES_MAX;
//		public static final int dM3E_ZPAD = dM3E__Z_MIN + dVec3Element.dV3E_PAD;
//		public static final int dM3E__Z_MAX = dM3E__Z_MIN + dVec3Element.dV3E__MAX;
//		public static final int dM3E__MAX = dM3E__MIN + dSpaceAxis.dSA__MAX * dVec3Element.dV3E__MAX;
//	}
//
//	static class dMat4Element {
//		public static final int dM4E__MIN = 0;
//		public static final int dM4E__X_MIN = dM4E__MIN + dVec4Element.dV4E_X * dVec4Element.dV4E__MAX;
//		public static final int dM4E_XX = dM4E__X_MIN + dVec4Element.dV4E_X;
//		public static final int dM4E_XY = dM4E__X_MIN + dVec4Element.dV4E_Y;
//		public static final int dM4E_XZ = dM4E__X_MIN + dVec4Element.dV4E_Z;
//		public static final int dM4E_XO = dM4E__X_MIN + dVec4Element.dV4E_O;
//		public static final int dM4E__X_MAX = dM4E__X_MIN + dVec4Element.dV4E__MAX;
//		public static final int dM4E__Y_MIN = dM4E__MIN + dVec4Element.dV4E_Y * dVec4Element.dV4E__MAX;
//		public static final int dM4E_YX = dM4E__Y_MIN + dVec4Element.dV4E_X;
//		public static final int dM4E_YY = dM4E__Y_MIN + dVec4Element.dV4E_Y;
//		public static final int dM4E_YZ = dM4E__Y_MIN + dVec4Element.dV4E_Z;
//		public static final int dM4E_YO = dM4E__Y_MIN + dVec4Element.dV4E_O;
//		public static final int dM4E__Y_MAX = dM4E__Y_MIN + dVec4Element.dV4E__MAX;
//		public static final int dM4E__Z_MIN = dM4E__MIN + dVec4Element.dV4E_Z * dVec4Element.dV4E__MAX;
//		public static final int dM4E_ZX = dM4E__Z_MIN + dVec4Element.dV4E_X;
//		public static final int dM4E_ZY = dM4E__Z_MIN + dVec4Element.dV4E_Y;
//		public static final int dM4E_ZZ = dM4E__Z_MIN + dVec4Element.dV4E_Z;
//		public static final int dM4E_ZO = dM4E__Z_MIN + dVec4Element.dV4E_O;
//		public static final int dM4E__Z_MAX = dM4E__Z_MIN + dVec4Element.dV4E__MAX;
//		public static final int dM4E__O_MIN = dM4E__MIN + dVec4Element.dV4E_O * dVec4Element.dV4E__MAX;
//		public static final int dM4E_OX = dM4E__O_MIN + dVec4Element.dV4E_X;
//		public static final int dM4E_OY = dM4E__O_MIN + dVec4Element.dV4E_Y;
//		public static final int dM4E_OZ = dM4E__O_MIN + dVec4Element.dV4E_Z;
//		public static final int dM4E_OO = dM4E__O_MIN + dVec4Element.dV4E_O;
//		public static final int dM4E__O_MAX = dM4E__O_MIN + dVec4Element.dV4E__MAX;
//		public static final int dM4E__MAX = dM4E__MIN + dVec4Element.dV4E__MAX * dVec4Element.dV4E__MAX;
//	}
//
//	static class dQuatElement {;
//		public static final int dQUE__MIN = 0;
//		public static final int dQUE_R = dQUE__MIN;
//		@Deprecated
//		public static final int dQUE__AXIS_MIN = dQUE_R + 1;
//		public static final int dQUE_I = dQUE__AXIS_MIN + dSpaceAxis.dSA_X;
//		public static final int dQUE_J = dQUE__AXIS_MIN + dSpaceAxis.dSA_Y;
//		public static final int dQUE_K = dQUE__AXIS_MIN + dSpaceAxis.dSA_Z;
//		public static final int dQUE__AXIS_MAX = dQUE__AXIS_MIN + dSpaceAxis.dSA__MAX;
//		public static final int dQUE__MAX = dQUE__AXIS_MAX;
//	}

	/* these types are mainly just used in headers */
//	typedef dReal dVector3[dV3E__MAX];
//	typedef dReal dVector4[dV4E__MAX];
//	typedef dReal dMatrix3[dM3E__MAX];
//	typedef dReal dMatrix4[dM4E__MAX];
//	typedef dReal dMatrix6[(dMD__MAX * dV3E__MAX) * (dMD__MAX * dSA__MAX)];
//	typedef dReal dQuaternion[dQUE__MAX];

//	/** 
//	 * These types are mainly just used in headers. 
//	 * deprecated TZ: Do we really need this class??? 
//	 */
//	public static class DMatrix4 { 
//		public DMatrix4(double d, double e, double f, double g, double h,
//				double i, double j, double k, double l, double m, double n,
//				double o, double p, double q, double r, double s) {
//			v[0] = d; v[1] = e; v[2] = f; v[3] = g; 
//			v[4] = h; v[5] = i; v[6] = j; v[7] = k; 
//			v[8] = l; v[9] = m; v[10] = n; v[11] = o; 
//			v[12] = p; v[13] = q; v[14] = r; v[15] = s; 
//		}
//
//		public double[] v = new double[4*4]; 
//	}


	/* precision dependent scalar math functions */

	//#if defined(dSINGLE)
	//
	//#define REAL(x) (x ## f)					/* form a constant */
	//#define dRecip(x) ((1.0f/(x)))				/* reciprocal */
	//#define dSqrt(x) (sqrtf(x))			/* square root */
	//#define dRecipSqrt(x) ((1.0f/sqrtf(x)))		/* reciprocal square root */
	//#define dSin(x) (sinf(x))				/* sine */
	//#define dCos(x) (cosf(x))				/* cosine */
	//#define dFabs(x) (fabsf(x))			/* absolute value */
	//#define dAtan2(y,x) (atan2f(y,x))		/* arc tangent with 2 args */
	//#define dFMod(a,b) (fmodf(a,b))		/* modulo */
	//#define dFloor(x) floorf(x)			/* floor */
	//
	//#ifdef HAVE___ISNANF
	//#define dIsNan(x) (__isnanf(x))
	//#elif defined(HAVE__ISNANF)
	//#define dIsNan(x) (_isnanf(x))
	//#elif defined(HAVE_ISNANF)
	//#define dIsNan(x) (isnanf(x))
	//#else
	//  /*
	//     fall back to _isnan which is the VC way,
	//     this may seem redundant since we already checked
	//     for _isnan before, but if isnan is detected by
	//     configure but is not found during compilation
	//     we should always make sure we check for __isnanf,
	//     _isnanf and isnanf in that order before falling
	//     back to a default
	//  */
	//#define dIsNan(x) (_isnan(x))
	//#endif
	//
	//#define dCopySign(a,b) ((dReal)copysignf(a,b))

	//#elif defined(dDOUBLE)

	//#define REAL(x) (x)
	//#define dRecip(x) (1.0/(x))
	public static double dRecip(double x) { return 1.0/x; }
	//#define dSqrt(x) sqrt(x)
	public static double dSqrt(double x) { return Math.sqrt(x); }
	//#define dRecipSqrt(x) (1.0/sqrt(x))
	public static double dRecipSqrt(double x) { return 1.0/Math.sqrt(x); }
	//#define dSin(x) sin(x)
	public static double dSin(double x) { return Math.sin(x); }
	//#define dCos(x) cos(x)
	public static double dCos(double x) { return Math.cos(x); }
	//#define dFabs(x) fabs(x)
	public static double dFabs(double x) {
		return Math.abs(x);
	}
	//#define dAtan2(y,x) atan2((y),(x))
	public static double dAtan2(double y, double x) {
		return Math.atan2(y, x);
	}
	//#define dAsin(x) (asinf(x))
	public static double dAsin(double x) {
		return Math.asin(x);
	}
	//#define dAcos(x) acos(x)
	public static double dAcos(double x) {
		return Math.acos(x);
	}

	//#define dFMod(a,b) (fmod((a),(b)))
	public static double dFMod(double x) {
		throw new UnsupportedOperationException();
		//return Math.fmod(x);
	}
	//#define dFloor(x) floor(x)
	public static double dFloor(double x) { return Math.floor(x); }

	//#define dCeil(x) ceilf(x)          /* ceil */
	public static double dCeil(double x) { return Math.ceil(x); }

	//#define dCopySign(a,b) ((dReal)copysignf(a,b)) /* copy value sign */
    public static double dCopysign(double magnitude, double sign) {
        return Math.copySign(magnitude, sign);
    }

    //#define dNextAfter(x, y) nextafterf(x, y) /* next value after */
    public static double dNextAfter(double start, double direction) {
        return Math.nextAfter(start, direction);
    }


	//#ifdef HAVE___ISNAN
	//#define dIsNan(x) (__isnan(x))
	//#elif defined(HAVE__ISNAN)
	//#define dIsNan(x) (_isnan(x))
	//#elif defined(HAVE_ISNAN)
	//#define dIsNan(x) (isnan(x))
	//#else
	//#define dIsNan(x) (_isnan(x))
	//#endif
	public final boolean dIsNan(double x) { return Double.isNaN(x); }
	public static final double dNaN = Double.NaN;

	//#define dCopySign(a,b) (copysign((a),(b)))
	public static double dCopySign(double a, double b) {
		return Math.copySign(a, b);
	}

	public static double dMin(double x, double y) { return Math.min(x, y); }
	public static double dMax(double x, double y) { return Math.max(x, y); }

	/* error numbers */

//	public enum d_ERR {
//		d_ERR_UNKNOWN,		/* unknown error */
//		d_ERR_IASSERT,		/* internal assertion failed */
//		d_ERR_UASSERT,		/* user assertion failed */
//		d_ERR_LCP;			/* user assertion failed */
//	}
	public static final int d_ERR_UNKNOWN = 0;		/* unknown error */
	public static final int d_ERR_IASSERT = 1;		/* internal assertion failed */
	public static final int d_ERR_UASSERT = 2;		/* user assertion failed */
	public static final int d_ERR_LCP = 3;			/* user assertion failed */
	public static final int d_ERR_SASSERT = 4;		/* TZ: static assertion failed */



	/* an alternative way of setting joint parameters, using joint parameter
	 * structures and member constants. we don't actually do this yet.
	 */

	/*
typedef struct dLimot {
  int mode;
  dReal lostop, histop;
  dReal vel, fmax;
  dReal fudge_factor;
  dReal bounce, soft;
  dReal suspension_erp, suspension_cfm;
} dLimot;

enum {
  dLimotLoStop		= 0x0001,
  dLimotHiStop		= 0x0002,
  dLimotVel		= 0x0004,
  dLimotFMax		= 0x0008,
  dLimotFudgeFactor	= 0x0010,
  dLimotBounce		= 0x0020,
  dLimotSoft		= 0x0040
};
	 */


	/* standard joint parameter names. why are these here? - because we don't want
	 * to include all the joint function definitions in joint.cpp. hmmmm.
	 * MSVC complains if we call D_ALL_PARAM_NAMES_X with a blank second argument,
	 * which is why we have the D_ALL_PARAM_NAMES macro as well. please copy and
	 * paste between these two.
	 */

	//#define D_ALL_PARAM_NAMES(start) \
	//  /* parameters for limits and motors */ \
	//  dParamLoStop = start, \
	//  dParamHiStop, \
	//  dParamVel, \
	//  dParamFMax, \
	//  dParamFudgeFactor, \
	//  dParamBounce, \
	//  dParamCFM, \
	//  dParamStopERP, \
	//  dParamStopCFM, \
	//  /* parameters for suspension */ \
	//  dParamSuspensionERP, \
	//  dParamSuspensionCFM, \
	//  dParamERP, \

	//////////////////////////////////////////////////////////////////////////////
	/// \enum  D_ALL_PARAM_NAMES_X
	///
	/// \var dParamGroup This is the starting value of the different group
	///                  (i.e. dParamGroup1, dParamGroup2, dParamGroup3)
	///                  It also helps in the use of parameter
	///                  (dParamGroup2 | dParamFMax) == dParamFMax2
	//////////////////////////////////////////////////////////////////////////////
	//#define D_ALL_PARAM_NAMES_X(start,x) \
	//  dParamGroup ## x = start, \
	//  /* parameters for limits and motors */ \
	//  dParamLoStop ## x = start, \
	//  dParamHiStop ## x, \
	//  dParamVel ## x, \
	//  dParamFMax ## x, \
	//  dParamFudgeFactor ## x, \
	//  dParamBounce ## x, \
	//  dParamCFM ## x, \
	//  dParamStopERP ## x, \
	//  dParamStopCFM ## x, \
	//  /* parameters for suspension */ \
	//  dParamSuspensionERP ## x, \
	//  dParamSuspensionCFM ## x, \
	//  dParamERP ## x,
	//
	//public enum D_ALL_PARAM_NAMES {
	//  D_ALL_PARAM_NAMES(0)
	//  dParamsInGroup,     ///< Number of parameter in a group
	//  D_ALL_PARAM_NAMES_X(0x000,1)
	//  D_ALL_PARAM_NAMES_X(0x100,2)
	//  D_ALL_PARAM_NAMES_X(0x200,3)
	//
	//  /* add a multiple of this constant to the basic parameter numbers to get
	//   * the parameters for the second, third etc axes.
	//   */
	//  dParamGroup=0x100
	//}



	// Private common.h

//	#ifndef SIZE_MAX
//#define SIZE_MAX  ((size_t)(-1))
//			#endif
	public static int SIZE_MAX = Integer.MAX_VALUE;


	//#ifndef offsetof
	//#define offsetof(s, m) ((size_t)&(((s *)8)->m) - (size_t)8)
	//			#endif
	//#ifndef membersize
	//#define membersize(s, m) (sizeof(((s *)8)->m))
	//			#endif
	//#ifndef endoffsetof
	//#define endoffsetof(s, m)   ((size_t)((size_t)&(((s *)8)->m) - (size_t)8) + sizeof(((s *)8)->m))
	//			#endif

	//#define dMACRO_MAX(a, b) ((a) > (b) ? (a) : (b))
	//			#define dMACRO_MIN(a, b) ((a) < (b) ? (a) : (b))
	//	public static int dMACRO_MIN(int a, int b) {
	//		return Math.min(a, b);
	//	}


//	template<typename DstType, typename SrcType>
//	inline
//	bool _cast_to_smaller(DstType &dtOutResult, const SrcType &stArgument)
//	{
//		return (SrcType)(dtOutResult = (DstType)stArgument) == stArgument;
//	}

//#if defined(__GNUC__)
//
//#define dCAST_TO_SMALLER(TargetType, SourceValue) ({ TargetType ttCastSmallerValue; dIVERIFY(_cast_to_smaller(ttCastSmallerValue, SourceValue)); ttCastSmallerValue; })
//
//
//			#else // #if !defined(__GNUC__)
//
//			#define dCAST_TO_SMALLER(TargetType, SourceValue) templateCAST_TO_SMALLER<TargetType>(SourceValue)
//
//	template <typename TTargetType, typename TSourceType>
//	inline TTargetType templateCAST_TO_SMALLER(const TSourceType &stSourceValue)
//	{
//		TTargetType ttCastSmallerValue;
//		dIVERIFY(_cast_to_smaller(ttCastSmallerValue, stSourceValue));
//		return ttCastSmallerValue;
//	}
//
//
//#endif // #if !defined(__GNUC__)


//	template<typename value_type>
//	inline
//	void dxSwap(value_type &one, value_type &another)
//	{
//		std::swap(one, another);
//	}
	@Deprecated
	public void dxSwap() {
		throw new UnsupportedOperationException();
	}
	public static void dxSwap(double[] one, int oneP, double[] another, int anotherP) {
		double tmp = one[oneP];
		one[oneP] = another[anotherP];
		another[anotherP] = tmp;
	}

	public static void dxSwap(RefDouble one, double[] another, int anotherP) {
		double tmp = one.get();
		one.set(another[anotherP]);
		another[anotherP] = tmp;
	}
	public static void dxSwap(int[] one, int oneP, int[] another, int anotherP) {
		int tmp = one[oneP];
		one[oneP] = another[anotherP];
		another[anotherP] = tmp;
	}
	public static void dxSwap(boolean[] one, int oneP, boolean[] another, int anotherP) {
		boolean tmp = one[oneP];
		one[oneP] = another[anotherP];
		another[anotherP] = tmp;
	}

//	template<typename value_type, typename lo_type, typename hi_type>
//	inline
//	value_type dxClamp(const value_type &value, const lo_type &lo, const hi_type &hi)
//	{
//		return value < lo ? (value_type)lo : value > hi ? (value_type)hi : value;
//	}
	public static double dxClamp(double value, double lo, double hi) {
		return value < lo ? lo : Math.min(value, hi);
	}
	public static int dxClamp(int value, int lo, int hi) {
		return value < lo ? lo : Math.min(value, hi);
	}


//	template <typename Type>
//	union _const_type_cast_union
//	{
//		explicit _const_type_cast_union(const void *psvCharBuffer): m_psvCharBuffer(psvCharBuffer) {}
//
//		operator const Type *() const { return m_pstTypedPointer; }
//    const Type &operator *() const { return *m_pstTypedPointer; }
//    const Type *operator ->() const { return m_pstTypedPointer; }
//    const Type &operator [](ptrdiff_t diElementIndex) const { return m_pstTypedPointer[diElementIndex]; }
//    const Type &operator [](size_t siElementIndex) const { return m_pstTypedPointer[siElementIndex]; }
//
//    const void 		*m_psvCharBuffer;
//    const Type		*m_pstTypedPointer;
//	};
//
//	template <typename Type>
//	union _type_cast_union
//	{
//		explicit _type_cast_union(void *psvCharBuffer): m_psvCharBuffer(psvCharBuffer) {}
//
//		operator Type *() const { return m_pstTypedPointer; }
//		Type &operator *() const { return *m_pstTypedPointer; }
//		Type *operator ->() const { return m_pstTypedPointer; }
//		Type &operator [](ptrdiff_t diElementIndex) const { return m_pstTypedPointer[diElementIndex]; }
//		Type &operator [](size_t siElementIndex) const { return m_pstTypedPointer[siElementIndex]; }
//
//		void			*m_psvCharBuffer;
//		Type			*m_pstTypedPointer;
//	};


//	template<size_t tsiTypeSize>
//	struct _sized_signed;
//
//	template<>
//	struct _sized_signed<sizeof(uint8)>
//	{
//		typedef int8 type;
//	};
//
//	template<>
//	struct _sized_signed<sizeof(uint16)>
//	{
//		typedef int16 type;
//	};
//
//	template<>
//	struct _sized_signed<sizeof(uint32)>
//	{
//		typedef int32 type;
//	};
//
//	template<>
//	struct _sized_signed<sizeof(uint64)>
//	{
//		typedef int64 type;
//	};
//
//	template<typename tintergraltype>
//	struct _make_signed
//	{
//		typedef typename _sized_signed<sizeof(tintergraltype)>::type type;
//	};
//
//
//	template<size_t tsiTypeSize>
//	struct _sized_unsigned;
//
//	template<>
//	struct _sized_unsigned<sizeof(int8)>
//	{
//		typedef uint8 type;
//	};
//
//	template<>
//	struct _sized_unsigned<sizeof(int16)>
//	{
//		typedef uint16 type;
//	};
//
//	template<>
//	struct _sized_unsigned<sizeof(int32)>
//	{
//		typedef uint32 type;
//	};
//
//	template<>
//	struct _sized_unsigned<sizeof(int64)>
//	{
//		typedef uint64 type;
//	};
//
//	template<typename tintergraltype>
//	struct _make_unsigned
//	{
//		typedef typename _sized_unsigned<sizeof(tintergraltype)>::type type;
//	};


// template<typename tvalueint, typename tminint, typename tmaxint>
// inline
// bool dxInRange(tvalueint viValue, tminint miMin, tmaxint miMax)
// {
//     return (typename _sized_unsigned<dMACRO_MAX(sizeof(tvalueint), sizeof(tminint))>::type)(viValue - miMin) < (typename _sized_unsigned<dMACRO_MAX(sizeof(tmaxint), sizeof(tminint))>::type)(miMax - miMin);
// }
// #define dIN_RANGE(aval, amin, amax) dxInRange(aval, amin, amax)
	public static boolean dIN_RANGE(int aval, int amin_incl, int amax_excl) {
		return amin_incl <= aval && aval < amax_excl;
	}
	public static boolean dIN_RANGE(long aval, long amin_incl, long amax_excl) {
		return amin_incl <= aval && aval < amax_excl;
	}



//#define dIN_RANGE(aval, amin, amax) ((_sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)((_sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(aval) - (_sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(amin)) < (_sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)((_sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amax) - (_sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amin)))
//			#define dTMPL_IN_RANGE(aval, amin, amax) ((typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)((typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(aval) - (typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(amin)) < (typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)((typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amax) - (typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amin)))
//			#define dCLAMP(aval, alo, ahi) dxClamp(aval, alo, ahi)
//			#define dARRAY_SIZE(aarr) (sizeof(aarr) / sizeof((aarr)[0]))
//			#define dSTATIC_ARRAY_SIZE(aclass, aarr) dARRAY_SIZE(((aclass *)sizeof(void *))->aarr)
//static bool dIN_RANGE(aval, amin, amax) {
//	((_sized_unsigned < dMACRO_MAX(sizeof(aval), sizeof(amin)) >::type)
//	((_sized_unsigned < dMACRO_MAX(sizeof(aval), sizeof(amin)) >::type)
//	(aval) - (_sized_unsigned < dMACRO_MAX(sizeof(aval), sizeof(amin)) >::type)(amin)) <
//	(_sized_unsigned < dMACRO_MAX(sizeof(amax), sizeof(amin)) >::type)
//	((_sized_unsigned < dMACRO_MAX(sizeof(amax), sizeof(amin)) >::type)
//	(amax) - (_sized_unsigned < dMACRO_MAX(sizeof(amax), sizeof(amin)) >::type)(amin)))
//}

	public static boolean dTMPL_IN_RANGE(int aval, int amin, int amax) {
		//		((typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(
		//				(typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(aval)
		//				-
		//				(typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(amin))
		//		<
		//		(typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)
		//		((typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amax)
		//				-
		//				(typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amin)))
		// (unsigned)	return (aval - amin) < (amax - amin);
		return aval >= amin && aval < amax;
	}

	public static double dCLAMP(double aval, double alo, double ahi) {
		return dxClamp(aval, alo, ahi);
	}

	public static int dCLAMP(int aval, int alo, int ahi) {
		return dxClamp(aval, alo, ahi);
	}

	//	public static int dARRAY_SIZE(int[] aarr) {
	//		return aarr.length;
	//	}
	//
	//	public static int dSTATIC_ARRAY_SIZE(aclass, aarr) {
	//		dARRAY_SIZE(((aclass *)sizeof(void *))->aarr);
	//	}

	protected Common() {}
}
