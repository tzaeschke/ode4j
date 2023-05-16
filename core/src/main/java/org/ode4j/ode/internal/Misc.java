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

import java.util.concurrent.atomic.AtomicLong;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.cpp4j.FILE;

import static org.ode4j.ode.internal.Common.dFabs;
import static org.ode4j.ode.internal.Common.dPAD;
import static org.ode4j.ode.internal.cpp4j.Cstdio.*;



/** ****************************************************************************
 * random numbers
 */
public class Misc {

	//	static unsigned long seed = 0;  //32bit unsigned
	private static final AtomicLong seed = new AtomicLong(0);

	protected Misc() {
		//private
	}
	
	
	/** 
	 * Return next 32 bit random number. this uses a not-very-random linear
	 * congruential method.
	 * @return ramdom number
	 */
	//unsigned long dRand()
	public static long
	dRand()
	{
	    long origSeed, newSeed;
        do {
            origSeed = seed.get();
            newSeed = (1664525L * origSeed + 1013904223L) & 0xffffffffL;
        } while (!seed.compareAndSet(origSeed, newSeed));
        return newSeed;
//		seed = (1664525L*seed + 1013904223L) & 0xffffffffL;
	}


	/** 
	 * get and set the current random number seed. 
	 * 
	 * @return seed
	 */
	//unsigned long  dRandGetSeed()
	public static long  dRandGetSeed()
	{
		return Math.abs(seed.get());
	}


	//void dRandSetSeed (unsigned long s)
	public static void dRandSetSeed (long s)
	{
		seed.set(s);
	}


	/** return 1 if the random number generator is working. 
	 * @return 'true' if it works 
	 */
	public static boolean dTestRand()
	{
		//	  unsigned long oldseed = seed;
		long oldseed = seed.get();
		boolean ret = true;
		seed.set(0);
		if (dRand() != 0x3c6ef35f || dRand() != 0x47502932 ||
				dRand() != 0xd1ccf6e9L || dRand() != 0xaaf95334L ||
				dRand() != 0x6252e503L ||
				dRand() != 0x9f2ec686L ||//) ret = false;
				dRand() != 0x57fe6c2dL ||
				dRand() != 0xa3d95fa8L ||
				dRand() != 0x81fdbee7L ||
				dRand() != 0x94f0af1aL ||
				dRand() != 0xcbf633b1L) ret = false;
		seed.set(oldseed);
		return ret;
	}


	/** return a random integer between 0..n-1. the distribution will get worse
	 * as n approaches 2^32.
	 * <p>
	 *  adam's all-int straightforward(?) dRandInt (0..n-1)
	 * @param n max value
	 * @return random value
	 */
	public static int dRandInt (long n)
	{
	    long result;
	    // Since there is no memory barrier macro in ODE assign via volatile variable 
	    // to prevent compiler reusing seed as value of `r'
	    long raw_r = dRand();
	    long r = raw_r;
	    
	    long un = n;
	    //dIASSERT(sizeof(n) == sizeof(un));

	    
//		// seems good; xor-fold and modulus
//		//  final unsigned long un = n;
//		//  unsigned long r = dRand();
//		final long un = n;
//		//TZ the following is unnecessary In Java because each method call has
//		//its own context.
////		// Since there is no memory barrier macro in ODE assign via volatile variable 
////		// to prevent compiler reusing seed as value of `r'
////		volatile long raw_r = dRand();
////		long r = raw_r;
//		long r = dRand();

		// note: probably more aggressive than it needs to be -- might be
		//       able to get away without one or two of the innermost branches.
//		if (un <= 0x00010000UL) {
//			r ^= (r >> 16);
//			if (un <= 0x00000100UL) {
//				r ^= (r >> 8);
//				if (un <= 0x00000010UL) {
//					r ^= (r >> 4);
//					if (un <= 0x00000004UL) {
//						r ^= (r >> 2);
//						if (un <= 0x00000002UL) {
//							r ^= (r >> 1);
//						}
//					}
//				}
//			}
//		}
		//Java version (TZ)
//		if (un <= 0x00010000L) {
//			r ^= (r >> 16);
//			if (un <= 0x00000100L) {
//				r ^= (r >> 8);
//				if (un <= 0x00000010L) {
//					r ^= (r >> 4);
//					if (un <= 0x00000004L) {
//						r ^= (r >> 2);
//						if (un <= 0x00000002L) {
//							r ^= (r >> 1);
//						}
//					}
//				}
//			}
//		}
		// Optimized version of above
		if (un <= 0x00000010L) {
		    r ^= (r >> 16);
		    r ^= (r >> 8);
		    r ^= (r >> 4);
		    if (un <= 0x00000002L) {
		        r ^= (r >> 2);
		        r ^= (r >> 1);
	            result = (r/* & (duint32)0x01*/) & (un >> 1);
		    } else {
		        if (un <= 0x00000004L) {
		            r ^= (r >> 2);
	                result = ((r & 0x03L) * un) >> 2;
	            } else {
	                result = ((r & 0x0FL) * un) >> 4;
		        }
		    }
		} else {
		    if (un <= 0x00000100L) {
		        r ^= (r >> 16);
		        r ^= (r >> 8);
	            result = ((r & 0xFFL) * un) >> 8;
		    } else {
		        if (un <= 0x00010000L) {
		            r ^= (r >> 16);
	                result = ((r & 0xFFFFL) * un) >> 16;
	            } else {
	                //result = (int)(((duint64)r * un) >> 32);
	            	result = (int)((r * un) >> 32);
		        }
		    }
		}

		return (int)result;    
	}


	/** 
	 * return a random real number between 0..1 
	 * @return random number
	 */
	public static double dRandReal()
	{
		return ((double) dRand()) / ((double) 0xffffffffL);
	}

	//****************************************************************************
	// matrix utility stuff

	/** print out a matrix */
//	void dPrintMatrix (final double []A, int n, int m, char []fmt, FILE f)
	void dPrintMatrix (final double []A, int n, int m, String fmt, FILE f)
	{
	    int skip = dPAD(m);
	    int Arowp = 0;//A;
	    for (int i=0; i<n; Arowp+=skip, ++i) {
	        for (int j=0; j<m; ++j) fprintf (f,fmt,A[Arowp+j]);
	        fprintf (f,"\n");
	    }
	}


	/** 
	 * make a random vector with entries between +/- range. A has 3 elements. 
	 * @param A A
	 * @param range range 
	 */
	public static void dMakeRandomVector (DVector3 A, double range) {
		for (int i=0; i<3; i++) A.set(i, (dRandReal()*2.0-1.0)*range );
	}
	/** 
	 * make a random vector with entries between +/- range. A has 4 elements. 
	 * @param A A 
	 * @param range range
	 */
	public static void dMakeRandomVector (DQuaternion A, double range) {
		for (int i=0; i<4; i++) A.set(i, (dRandReal()*2.0-1.0)*range );
	}
	/** 
	 * make a random vector with entries between +/- range. A has n elements. 
	 * @param A A
	 * @param n n 
	 * @param range range
	 */
	public static void dMakeRandomVector (double[]A, int n, double range)
	{
		for (int i=0; i<n; i++) A[i] = (dRandReal()*2.0-1.0)*range;
	}


	/** 
	 * make a random matrix with entries between +/- range. A has size 3*3. 
	 * @param A A
	 * @param range range
	 */
	public static void dMakeRandomMatrix (DMatrix3 A, double range) {
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) A.set(i, j, (dRandReal()*2.0-1.0)*range );
		}
	}
	/** 
	 * make a random matrix with entries between +/- range. A has size n*m. 
	 * @param A A
	 * @param n n 
	 * @param m m 
	 * @param range range
	 */
	public static void dMakeRandomMatrix (double[]A, int n, int m, double range)
	{
		int skip = dPAD(m);
		//Matrix.dSetZero (A,n*skip);
		int ArowP = 0;//A;
		for (int i=0; i<n; ArowP+=skip, ++i) {
		    for (int j=0; j<m; ++j) A[ArowP+j] = (dRandReal()*2.0-1.0)*range;
		}
	}


	/** 
	 * clear the upper triangle of a square matrix.
	 * @param A A
	 * @param n n
	 */
	public static void dClearUpperTriangle (double[]A, int n)
	{
	    int skip = dPAD(n);
	    int ArowP = 0;//A;
	    for (int i=0; i<n; ArowP+=skip, ++i) {
	      for (int j=i+1; j<n; ++j) A[ArowP+j] = 0;
		}
	}


	/** 
	 * clear the upper triangle of a square matrix. 
	 * @param A A
	 */
	public static void dClearUpperTriangle (DMatrix3 A)
	{
		A.set01(0);
		A.set02(0);
		A.set12(0);
	}


	/** 
	 * return the maximum element difference between the two n*m matrices. 
	 * @param A A
	 * @param B B
	 * @param n n
	 * @param m m
	 * @return difference 
	 */
	public static double dMaxDifference (final double[]A, final double[]B, int n, int m)
	{
	    int skip = dPAD(m);
	    double max = 0.0;
	    int Arow = 0, Brow = 0;//A, *Brow = B;
	    for (int i=0; i<n; Arow+=skip, Brow +=skip, ++i) {
	      for (int j=0; j<m; ++j) {
	        double diff = dFabs(A[Arow+j] - B[Brow+j]);
	        if (diff > max) max = diff;
	      }
	    }
		return max;
	}
	/** 
	 * return the maximum element difference between the two 3*3 matrices. 
	 * @param A A
	 * @param B B
	 * @return difference 
	 */
	public static double dMaxDifference (final DMatrix3C A, final DMatrix3C B)
	{
		double diff,max;
		max = 0;
		
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) {
				diff = Math.abs(A.get(i,j) - B.get(i,j));
				if (diff > max) {
					max = diff;
				}
			}
		}
		return max;
	}
	/** 
	 * return the maximum element difference between the two 3*1 matrices. 
	 * @param A A
	 * @param B B
	 * @return difference 
	 */
	public static double dMaxDifference (final DVector3C A, final DVector3C B) {
		double max = Math.abs(A.get0() - B.get0());
		double diff = Math.abs(A.get1() - B.get1());
		if (diff > max) max = diff;
		diff = Math.abs(A.get2() - B.get2());
		if (diff > max) max = diff;
		return max;
	}
	/** 
	 * return the maximum element difference between the two 4*1 matrices. 
	 * @param A A
	 * @param B B
	 * @param n n
	 * @param m m
	 * @return difference 
	 */
	public static double dMaxDifference (final DQuaternionC A, 
			final DQuaternionC B, int n, int m) {
		double max = Math.abs(A.get0() - B.get0());
		double diff = Math.abs(A.get1() - B.get1());
		if (diff > max) max = diff;
		diff = Math.abs(A.get2() - B.get2());
		if (diff > max) max = diff;
		diff = Math.abs(A.get3() - B.get3());
		if (diff > max) max = diff;
		return max;
	}


	/** 
	 * return the maximum element difference between the lower triangle of two
	 * n*n matrices.
	 * @param A A
	 * @param B B
	 * @param n n
	 * @return difference
	 */
	//double dMaxDifferenceLowerTriangle (final double *A, final double *B, int n)
	public static double dMaxDifferenceLowerTriangle (final double[]A, final double[]B, int n)
	{
		int skip = dPAD(n);
		double max = 0.0;
		int Arow = 0, Brow = 0;//A, *Brow = B;
		for (int i=0; i<n; Arow+=skip, Brow+=skip, ++i) {
		    for (int j=0; j<=i; ++j) {
		        double diff = dFabs(A[Arow+j] - B[Brow+j]);
		        if (diff > max) max = diff;
		    }
		}
		return max;
	}
}