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
package org.ode4j.cpp.internal;

import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.cpp4j.Cstdio;
import org.ode4j.ode.internal.cpp4j.FILE;


/** 
 * miscellaneous math functions. these are mostly useful for testing 
 */
public abstract class ApiCppMathMisc {

	/** 
	 * @return 1 if the random number generator is working. 
	 */
	//ODE_API 
	public static boolean dTestRand() {
		return OdeMath.dTestRand();
	}

	/** @return next 32 bit random number. this uses a not-very-random linear
	 * congruential method.
	 */
	//ODE_API unsigned 
	public static long dRand() {
		return OdeMath.dRand();
	}

	/** @return the current random number seed. */
	//ODE_API unsigned 
	public static long  dRandGetSeed() {
		return OdeMath.dRandGetSeed();
	}
	//ODE_API 
	//	void dRandSetSeed (unsigned long s);
	public static void dRandSetSeed (long s) {
		OdeMath.dRandSetSeed(s);
	}

	/** 
	 * @param n n
	 * @return a random integer between 0..n-1. the distribution will get worse
	 * as n approaches 2^32.
	 */
	//ODE_API 
	public static int dRandInt (int n) {
		return OdeMath.dRandInt(n);
	}

	/** @return a random real number between 0..1 */
	//ODE_API dReal double dRandReal(void);
	public static double dRandReal() {
		return OdeMath.dRandReal();
	}

	/** print out a matrix */
	//#ifdef __cplusplus
	//ODE_API void dPrintMatrix (const dReal *A, int n, int m, char *fmt = "%10.4f ",
	//		   FILE *f=stdout);
	//#else
	//ODE_API void dPrintMatrix (const dReal *A, int n, int m, char *fmt, FILE *f);
	//#endif
	void dPrintMatrix (final double []A, int n, int m) {
		dPrintMatrix(A, n, m, "%10.4", Cstdio.stdout);
	}
	void dPrintMatrix (final double []A, int n, int m, String fmt, FILE f) {
		throw new UnsupportedOperationException();
	}

	/** make a random vector with entries between +/- range. A has n elements. 
	 * @param A A
	 * @param n n
	 * @param range range
	 */
	//ODE_API 
	//	void dMakeRandomVector (dReal *A, int n, dReal range);
	public static void dMakeRandomVector (double []A, int n, double range) {
		OdeMath.dMakeRandomVector(A, n, range);
	}

	/** make a random matrix with entries between +/- range. A has size n*m. 
	 * @param A A
	 * @param n n
	 * @param m m
	 * @param range range 
	 */
	//ODE_API 
	//	void dMakeRandomMatrix (dReal *A, int n, int m, dReal range);
	public static void dMakeRandomMatrix (double []A, int n, int m, double range) {
		OdeMath.dMakeRandomMatrix(A, n, m, range);
	}

	/** 
	 * clear the upper triangle of a square matrix.
	 * @param A A
	 * @param n n
	 */
	//ODE_API 
	//	void dClearUpperTriangle (dReal *A, int n);
	public static void dClearUpperTriangle (double []A, int n) {
		OdeMath.dClearUpperTriangle(A, n);
	}

	/** 
	 * @param A A 
	 * @param B B
	 * @param n n
	 * @param m m
	 * @return the maximum element difference between the two n*m matrices */
	//ODE_API 
	//	dReal dMaxDifference (const dReal *A, const dReal *B, int n, int m);
	public static double dMaxDifference (final double []A, final double []B, int n, int m) {
		return OdeMath.dMaxDifference(A, B, n, m);
	}

	/** 
	 * @param A A
	 * @param B B
	 * @param n n
	 * @return the maximum element difference between the lower triangle of two
	 * n*n matrices */
	//ODE_API 
	//	dReal dMaxDifferenceLowerTriangle (const dReal *A, const dReal *B, int n);
	public static double dMaxDifferenceLowerTriangle (final double []A, final double []B, int n) {
		return OdeMath.dMaxDifferenceLowerTriangle(A, B, n);
	}

	protected ApiCppMathMisc() {}
}
