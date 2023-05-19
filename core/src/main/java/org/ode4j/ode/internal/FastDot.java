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


/* generated code, do not edit. */

public class FastDot extends Misc {
	
	public static double dDot (final double[] a, final double[] b, int n)
	{
		return dDot(a, 0, b, n);
	}
	
	
	/**
	 * 
	 * @param a a
	 * @param aOfs aOfs
	 * @param b b
	 * @param n n
	 * @return xxx xxx
	 * @deprecated use other method
	 */
	@Deprecated
    public static double dDot (final double[] a, int aOfs, final double[] b, int n)
	{  
		double p0,q0,m0,p1,q1,m1,sum;
		int aPos = aOfs, bPos = 0;
		sum = 0;
		n -= 2;
		while (n >= 0) {
			p0 = a[aPos + 0]; q0 = b[bPos + 0];
			m0 = p0 * q0;
			p1 = a[aPos + 1]; q1 = b[bPos + 1];
			m1 = p1 * q1;
			sum += m0;
			sum += m1;
			aPos += 2;
			bPos += 2;
			n -= 2;
		}
		n += 2;
		while (n > 0) {
			sum += a[aPos] * b[bPos];//](*a) * (*b);
			aPos++;
			bPos++;
			n--;
		}
		return sum;
	}
	
	//TODO compact to single method
	
	public static double dDot (final double[] a, int aOfs, 
			final double[] b, int bOfs, int n)
	{  
		double p0,q0,m0,p1,q1,m1,sum;
		int aPos = aOfs, bPos = bOfs;
		sum = 0;
		n -= 2;
		while (n >= 0) {
			p0 = a[aPos + 0]; q0 = b[bPos + 0];
			m0 = p0 * q0;
			p1 = a[aPos + 1]; q1 = b[bPos + 1];
			m1 = p1 * q1;
			sum += m0;
			sum += m1;
			aPos += 2;
			bPos += 2;
			n -= 2;
		}
		n += 2;
		while (n > 0) {
			sum += a[aPos] * b[bPos];//](*a) * (*b);
			aPos++;
			bPos++;
			n--;
		}
		return sum;
	}

	//template<unsigned b_stride>
	public static double calculateLargeVectorDot (
			final double[] a, int a_pos, final double[] b, int b_pos, int n, final int b_stride)
	{
		double sum = 0;
    	int a_end = a_pos + (n & (~3));
		for (; a_pos != a_end; b_pos += 4 * b_stride, a_pos += 4) {
			double p0 = a[a_pos + 0], p1 = a[a_pos + 1], p2 = a[a_pos + 2], p3 = a[a_pos + 3];
			double q0 = b[b_pos + 0 * b_stride], q1 = b[b_pos + 1 * b_stride], q2 = b[b_pos + 2 * b_stride], q3 = b[b_pos + 3 * b_stride];
			double m0 = p0 * q0;
			double m1 = p1 * q1;
			double m2 = p2 * q2;
			double m3 = p3 * q3;
			sum += m0 + m1 + m2 + m3;
		}
		a_end += (n & 3);
		for (; a_pos != a_end; b_pos += b_stride, ++a_pos) {
			sum += (a[a_pos]) * (b[b_pos]);
		}
		return sum;
	}

	protected FastDot() {}
}