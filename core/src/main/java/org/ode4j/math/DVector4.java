/*************************************************************************
 *                                                                       *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.math;


public class DVector4 implements DVector4C {
	
	private final double[] v;
	private static final int LEN = 4;
	
	public DVector4(DVector4 v4) {
		this();
		set(v4.v);
	}

	public DVector4() {
		v = new double[LEN];
	}

	public void set(double a, double b, double c, double d) {
		v[0] = a;
		v[1] = b;
		v[2] = c;
		v[3] = d;
	}

	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("DVector4[ ");
		b.append(get0()).append(", ");
		b.append(get1()).append(", ");
		b.append(get2()).append(", ");
		b.append(get3()).append(" ]");
		return b.toString();
	}

	@Override
	public double get0() {
		return v[0];
	}

	@Override
	public double get1() {
		return v[1];
	}

	@Override
	public double get2() {
		return v[2];
	}

	@Override
	public double get3() {
		return v[3];
	}

	public int dim() {
		return LEN;
	}

	public DVector4 scale(double d) {
		v[0] *= d; v[1] *= d; v[2] *=d; v[3] *= d;
		return this;
	}

	@Override
	public double lengthSquared() {
		return get0()*get0() + get1()*get1() + get2()*get2() + get3()*get3();
	}

	@Override
	public double length() {
		return Math.sqrt(lengthSquared());
	}

	@Override
	public double get(int i) {
		return v[i];
	}
	
	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 */
	public final boolean safeNormalize4 ()
	{
		//TODO is this correct? Maybe the real problem was that no eps is defined?
		double d = Math.abs(get0()); //TODO use epsilon for minimal values (?)
//		for (int i = 1; i < v.length; i++) {
//			if (Math.abs(v[i]) > d) {
//				d = Math.abs(v[i]);
//			}
//		}
		if (Math.abs(get1()) > d) d = Math.abs(get1());
		if (Math.abs(get2()) > d) d = Math.abs(get2());
		if (Math.abs(get3()) > d) d = Math.abs(get3());
		
		if (d <= Double.MIN_NORMAL) {
			set(1, 0, 0, 0);
			return false;
		}
		
		scale(1/d);
//		for (int i = 0; i < v.length; i++) {
//			v[i] /= d;
//		}
		
//		double sum = 0;
//		for (double d2: v) {
//			sum += d2*d2;
//		}
		
		double l = 1./length();//Math.sqrt(sum);
//		for (int i = 0; i < v.length; i++) {
//			v[i] *= l;
//		}
		scale(l);
		return true;
	}
	
	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 */
	public void normalize()
	{
		if (!safeNormalize4()) throw new IllegalStateException(
				"Normalization failed: " + this);
	}
	
	public void set(double[] a) {
		System.arraycopy(a, 0, v, 0, v.length);
		//return (T) this;
	}
	
	
	/**
	 * Return the 'dot' product of two vectors.
	 * r = a0*b0 + a1*b1 + a2*b2 + a3*b3;
	 * @param b 
	 * @return (this) * b
	 */
	@Override
	public final double dot(DVector4C b) {
		return get0()*b.get0() + get1()*b.get1() + get2()*b.get2() + get3()*b.get3();
	}

	public void set0(double d) {
		v[0] = d;		
	}

	public void set1(double d) {
		v[1] = d;		
	}

	public void set2(double d) {
		v[2] = d;		
	}

	public void set3(double d) {
		v[3] = d;		
	}
}
