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
	 * @param a
	 * @param aOfs
	 * @param b
	 * @param n
	 * @return xxx
	 * @deprecated use other method
	 */
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
}