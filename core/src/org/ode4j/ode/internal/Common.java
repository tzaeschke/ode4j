/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import java.io.PrintWriter;
import java.io.StringWriter;

import org.ode4j.ode.DBody;


/**
 * configuration stuff.
 */
public class Common extends ErrorHandler { 

	/** configuration stuff */

	/** constants */

	/** pi and 1/sqrt(2) are defined here if necessary because they don't get
	 * defined in <math.h> on some platforms (like MS-Windows)
	 */

	public static final double M_PI = 3.1415926535897932384626433832795029;
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

	public static final boolean dNODEBUG = false;
	/** @deprecated TZ this can be removed? */
	public static final boolean dUSE_MALLOC_FOR_ALLOCA = false;
	/** @deprecated TZ this can be removed? */
	public static final  boolean dTLS_ENABLED = false;

	
	public static final boolean dDOUBLE = true;
	public static final boolean dSINGLE = false;
	public static final double dEpsilon;
	public static final double MAX_FLOAT;
	static {
		if (dDOUBLE == dSINGLE) {
			throw new RuntimeException("dDOUBLE == dSINGLE");
		}
		if (dDOUBLE) {
			//TODO use MIN_VALUE instead? IEEE 754 ...
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
	//TODO ???
	/** @deprecated TZ this can be removed? */
	public static final boolean  dATOMICS_ENABLED = false;
	public static final boolean  dTRIMESH_16BIT_INDICES = false;

	public static final boolean  dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER = false;

	//TODO
	//http://www.codeguru.com/forum/printthread.php?t=323835
	//TODO use MACRO
	//#define EPSILON 0.0001   // Define your own tolerance
	//#define FLOAT_EQ(x,v) (((v - EPSILON) < x) && (x <( v + EPSILON)))
	//public static final double DBL_EPSILON = 2.22045e-16;
	public static final double DBL_EPSILON = 2.2204460492503131e-016;
	
	/** Internal assertion */
	public static void dIASSERT(boolean b) {
		if (!b) {
			dDebug(d_ERR_IASSERT, "assertion failed ");
			new RuntimeException().printStackTrace();
		}
	}
	public static void dIASSERT(int i) {
		if (i == 0) {
			dDebug(d_ERR_IASSERT, "assertion failed");
			new RuntimeException().printStackTrace();
		}
	}

	public static void dUASSERT(Object  a, String msg) {
		if (a == null) {
			dDebug (d_ERR_UASSERT, msg);
		}
	}

	public static void dDEBUGMSG(String msg) {
		StringWriter sw = new StringWriter();
		new PrintWriter(sw);
		new RuntimeException(msg).printStackTrace(new PrintWriter(sw));
		String msg2 = sw.toString();
		dMessage (d_ERR_UASSERT, msg2);
	}

	public static void dAASSERT(Object ... aa) {
		for (Object a: aa)
			dUASSERT(a, "Bad argument(s)");
	}

	public static void dAASSERT(boolean b) {
		if (!b)
			dUASSERT(null, "Bad argument(s)");
	}

	// #ifndef dNODEBUG
	// #ifdef __GNUC__
	//  #define dIASSERT(a) if (!(a)) dDebug (d_ERR_IASSERT,
	//  "assertion \"" #a "\" failed in %s() [%s]",__FUNCTION__,__FILE__);
	//  #define dUASSERT(a,msg) if (!(a)) dDebug (d_ERR_UASSERT,
	//  msg " in %s()", __FUNCTION__);
	////  #define dDEBUGMSG(msg) dMessage (d_ERR_UASSERT,				\
	////  msg " in %s() File %s Line %d", __FUNCTION__, __FILE__,__LINE__);
	//// #else
	////  #define dIASSERT(a) if (!(a)) dDebug (d_ERR_IASSERT, \
	////  "assertion \"" #a "\" failed in %s:%d",__FILE__,__LINE__);
	////  #define dUASSERT(a,msg) if (!(a)) dDebug (d_ERR_UASSERT, \
	////  msg " (%s:%d)", __FILE__,__LINE__);
	////  #define dDEBUGMSG(msg) dMessage (d_ERR_UASSERT, \
	////  msg " (%s:%d)", __FILE__,__LINE__);
	//// #endif
	//#else
	// #define dIASSERT(a) ;
	// #define dUASSERT(a,msg) ;
	// #define dDEBUGMSG(msg) ;
	//#endif
	//#define dAASSERT(a) dUASSERT(a,"Bad argument(s)")

	// Macro used to suppress unused variable warning
	//TODO #define dVARIABLEUSED(a) ((void)a)
	public static void dVARIABLEUSED(Object a) {
		//Nothing
	}

	 /**
	  * @author Tilmann Zaeschke
	  * @deprecated TODO remove?
	  */
	 public interface BodyMoveCallBack {
		 void run(DBody b);
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

	public static final boolean dTRIMESH_ENABLED = false;
	public static final boolean dTRIMESH_OPCODE = false;
	public static final boolean dTRIMESH_GIMPACT = false;
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
	 * @deprecated Remove this if possible (TZ)
	 */
	public static final int dPAD(int a) {
		return (a > 1) ? ((((a)-1)|3)+1) : a;
	}
	//#define dPAD(a) (((a) > 1) ? ((((a)-1)|3)+1) : (a))

	/** 
	 * These types are mainly just used in headers. 
	 * @deprecated TZ: Do we really need this class??? 
	 */
	public static class dMatrix4 { 
		public dMatrix4(double d, double e, double f, double g, double h,
				double i, double j, double k, double l, double m, double n,
				double o, double p, double q, double r, double s) {
			v[0] = d; v[1] = e; v[2] = f; v[3] = g; 
			v[4] = h; v[5] = i; v[6] = j; v[7] = k; 
			v[8] = l; v[9] = m; v[10] = n; v[11] = o; 
			v[12] = p; v[13] = q; v[14] = r; v[15] = s; 
		}

		public double[] v = new double[4*4]; 
	}
	private static class dMatrix6 { public double[] v = new double[8*6]; }
	//typedef dReal dVector3[4];
	//typedef dReal dVector4[4];
	//typedef dReal dMatrix3[4*3];
	//typedef dReal dMatrix4[4*4];
	//typedef dReal dMatrix6[8*6];
	//typedef dReal dQuaternion[4];


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
	//#define dRecip(x) (1.0/(x)) //TODO replace
	public static final double dRecip(double x) { return 1.0/x; };
	//#define dSqrt(x) sqrt(x) //TODO replace
	public static final double dSqrt(double x) { return Math.sqrt(x); };
	//#define dRecipSqrt(x) (1.0/sqrt(x))
	public static final double dRecipSqrt(double x) { return 1.0/Math.sqrt(x); };
	//#define dSin(x) sin(x)//TODO replace
	public static final double dSin(double x) { return Math.sin(x); };
	//#define dCos(x) cos(x) //TODO replace
	public static final double dCos(double x) { return Math.cos(x); };
	//#define dFabs(x) fabs(x) //TODO replace
	public static final double dFabs(double x) {
		return Math.abs(x);
	}
	//#define dAtan2(y,x) atan2((y),(x)) //TODO replace
	public static final double dAtan2(double y, double x) {
		return Math.atan2(y, x);
		//throw new UnsupportedOperationException();
		//	return Math.atan2(x);
	}
	//#define dFMod(a,b) (fmod((a),(b))) //TODO replace
	public static final double dFMod(double x) {
		throw new UnsupportedOperationException();
		//return Math.fmod(x);
	}
	//#define dFloor(x) floor(x) //TODO replace
	public static final double dFloor(double x) { return Math.floor(x); };

	//#ifdef HAVE___ISNAN
	//#define dIsNan(x) (__isnan(x))
	//#elif defined(HAVE__ISNAN)
	//#define dIsNan(x) (_isnan(x))
	//#elif defined(HAVE_ISNAN)
	//#define dIsNan(x) (isnan(x))
	//#else
	//#define dIsNan(x) (_isnan(x))
	//#endif
	public final boolean dIsNan(double x) { return x == Double.NaN; }

	//#define dCopySign(a,b) (copysign((a),(b)))
	public final double dCopySign(double a, double b) {
		throw new UnsupportedOperationException();
	}

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

}
