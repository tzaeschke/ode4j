/**
 * ----------------------------------------------------------------------------
 * This source file is part of the ODE4J library (ported to
 * Java from the GIMPACT Library).
 * 
 * For the latest info on ODE4J, see http://www.ode4j.org/
 * For the latest info on GIMPACT, see http://gimpact.sourceforge.net/
 * 
 * Copyright of GIMPACT (c) 2006 Francisco Leon. C.C. 80087371.
 * email: projectileman@yahoo.com
 * Copyright of ODE4J (c) 2009-2014 Tilmann Zäschke.
 * email: ode4j.gmx.de
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file GIMPACT-LICENSE-LGPL.TXT and LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file GIMPACT-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-BSD.TXT, LICENSE.TXT and 
 * ODE4J-LICENSE-BSD.TXT for more details.
 * 
 * ----------------------------------------------------------------------------
 */
package org.ode4j.ode.internal.gimpact;

import java.util.Random;

/**
 * Ported to Java by Tilmann Zaeschke
 * @author Francisco Leon
 */
public class GimMath {
//
//
//	#include "config.h"
//
//	#include <math.h>
//	#include <float.h>
//	#if HAVE_SYS_TYPES_H
//	#include <sys/types.h>
//	#elif defined(_MSC_VER)
//	typedef __int32 int32_t;
//	typedef unsigned __int32 uint32_t;
//	#elif defined(__GNUC__)
//	#include <inttypes.h>
//	#else
//	#error "GIMPACT: Must define int32_t and uint32_t"
//	#endif


	/*! \defgroup BASIC_TYPES
	Basic types and constants
	Conventions:
	Types starting with G
	Constants starting with G_
	*/
	//! @{
	/*! Types */
//	#define GREAL float
//	#define GINT32 int32_t
//	#define GUINT32 uint32_t
//
//	#define GPTR void*

	/*! Constants for integers*/
	//#define GUINT32_BIT_COUNT 32
//	private static final int GUINT32_BIT_COUNT = 32;
//	//#define GUINT32_EXPONENT 5
//	private static final int GUINT32_EXPONENT = 5;
//
//	private static final boolean G_FASTMATH = true; //1;
//	private static final float G_PI = 3.14159265358979f;
//	private static final float G_HALF_PI = 1.5707963f;
//	//267948966
//	private static final float G_TWO_PI = 6.28318530f;
//	//71795864
//	private static final float G_ROOT3 = 1.73205f;
//	private static final float G_ROOT2 = 1.41421f;
	// static final int G_UINT_INFINITY = 65534;      // ----- TZ:  use Int.infinity!
	protected static final float G_REAL_INFINITY = Float.POSITIVE_INFINITY;
	protected static final float G_REAL_INFINITY_N = Float.NEGATIVE_INFINITY;
//	private static final int	G_SIGN_BITMASK	= 	0x80000000;
//	private static final boolean G_USE_EPSILON_TEST = true;
	protected static final float G_EPSILON = 0.0000001f;
	//! @}

	/*! \defgroup MATH_FUNCTIONS
	mathematical functions
	*/
	//! @{
	//#define G_DEGTORAD(X) ((X)*3.1415926f/180.0f)
//	private static final float G_DEGTORAD(float X) { return ((X)*3.1415926f/180.0f); }
//	//#define G_RADTODEG(X) ((X)*180.0f/3.1415926f)
//	private static final float  G_RADTODEG(float X) { return ((X)*180.0f/3.1415926f); };
//
//	//! Integer representation of a floating-point value.
//	//#define IR(x)					((GUINT32&)(x))
//	/** @deprecated */
//	private static int IR(float x) {return Float.floatToRawIntBits(x);}
//
//	//! Signed integer representation of a floating-point value.
//	//#define SIR(x)					((GINT32&)(x))
//	/** @deprecated */
//	private int SIR(float x) {return Float.floatToIntBits(x);}
//
//	//! Absolute integer representation of a floating-point value
//	//#define AIR(x)					(IR(x)&0x7fffffff)
//	/** @deprecated */
//	private int AIR(float x) {return Float.floatToRawIntBits(x);}
//
//	//! Floating-point representation of an integer value.
//	//#define FR(x)					((float&)(x))
//	/** @deprecated */
//	private float FR(int x) {return Float.intBitsToFloat(x);}

	//#define MAX(a,b) ((a)<(b)?(b):(a))
	protected static float MAX(float a, float b) { return Math.max(a, b); }
	//#define MIN(a,b) ((a)>(b)?(b):(a))
	protected static float MIN(float a, float b) { return Math.min(a, b); }

	//#define MAX3(a,b,c) MAX(a,MAX(b,c))
	protected static float MAX3(float a, float b, float c) { return MAX(a, MAX(b, c)); }
	//#define MIN3(a,b,c) MIN(a,MIN(b,c))
	protected static float MIN3(float a, float b, float c) { return MIN(a, MIN(b, c)); }

	//#define IS_ZERO(value) ((value) < G_EPSILON &&  (value) > -G_EPSILON)
	protected static boolean IS_ZERO(float a) { return a < G_EPSILON && a > -G_EPSILON; }

//	//#define IS_NEGATIVE(value) ((value) <= -G_EPSILON)
//	private boolean IS_NEGATIVE(float a) { return a <= -G_EPSILON; }
//
//	//#define IS_POSISITVE(value) ((value) >= G_EPSILON)
//	private boolean IS_POSITIVE(float a) { return a >= G_EPSILON; }

	///returns a clamped number
	//#define CLAMP(number,minval,maxval) ((number)<(minval)?(minval):((number)>(maxval)?(maxval):(number)))
	protected static float CLAMP(float number, float minval, float maxval) { 
		return number<minval ? minval : (Math.min(number, maxval)); }


	///Swap numbers
//	#define SWAP_NUMBERS(a,b){ \
//	    (a) = (a)+(b); \
//	    (b) = (a)-(b); \
//	    (a) = (a)-(b); \
//	}\
//	private static final void SWAP_NUMBERS(final RefFloat a, final RefFloat b){ 
//		a.d = a.d+b.d; 
//		b.d = a.d-b.d; 
//		a.d = a.d-b.d; 
//	}

	//#define GIM_INV_SQRT(va,isva)\
	protected static float GIM_INV_SQRT(final float va)
	{
		float isva;
	    if((va)<=0.0000001f)
	    {
	        (isva) = G_REAL_INFINITY;
	    }
	    else
	    {
	    	return (float) (1./Math.sqrt(va));
//	        float _x = (va) * 0.5f;
//	        GUINT32 _y = 0x5f3759df - ( IR(va) >> 1);
//	        (isva) = FR(_y);
//	        (isva) = (isva) * ( 1.5f - ( _x * (isva) * (isva) ) );
	    }
	    return isva;
	}

	//#define GIM_SQRT(va,sva)\ // sva == result !
	protected static float GIM_SQRT(final float va)
	{
//	    GIM_INV_SQRT(va,sva);
//	    (sva) = 1.0f/(sva);
		return (float) Math.sqrt(va);
	}


	//! Computes 1.0f / sqrtf(x). Comes from Quake3. See http://www.magic-software.com/3DGEDInvSqrt.html
	static float gim_inv_sqrt(final float f)
	{
	    return GIM_INV_SQRT(f);
	}

	//! Computes sqrtf(x) faster.
	/*!
	\sa gim_inv_sqrt
	*/
	static float gim_sqrt(final float f)
	{
	    return GIM_SQRT(f);
	}

	// TZ: We want everything to be predictable!
	private static final Random random = new Random(0);
	
	//!Initializes mathematical functions
	static void gim_init_math()
	{
		//srand( static_cast< unsigned int >( time( 0 ) ) );
	}

	//! Generates an unit random
	static float gim_unit_random()
	{
//	    float rn = static_cast< float >( rand() );
//	    rn/=(float)RAND_MAX;
//	    return rn;
		return random.nextFloat();
	}

	protected GimMath() {}
}
