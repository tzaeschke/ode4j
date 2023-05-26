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

import java.util.Arrays;


public class DVector6 {

	private final double[] v;
	private static final int LEN = 6;
	
	public DVector6() {
		v = new double[LEN];
	}

	public DVector6(DVector6 v2) {
		this();
		set(v2);
	}

	public DVector6(double d0, double d1, double d2, double d3, double d4, 
			double d5) {
		this();
		set(d0, d1, d2, d3, d4, d5);
	}

	public DVector6 set(double d0, double d1, double d2, double d3, double d4,
			double d5) {
		v[0] = d0;
		v[1] = d1;
		v[2] = d2;
		v[3] = d3;
		v[4] = d4;
		v[5] = d5;
		return this;
	}

	public DVector6 set(double[] a) {
		v[0] = a[0];
		v[1] = a[1];
		v[2] = a[2];
		v[3] = a[3];
		v[4] = a[4];
		v[5] = a[5];
		return this;
	}

	public void set(DVector6 v2) {
		for (int i = 0; i < LEN; i++) {
			v[i] = v2.v[i]; 
		}
	}
	
	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("dVector6[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
	}

	public int dim() {
		return LEN;
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

	public double get3() {
		return v[3];
	}

	public double get4() {
		return v[4];
	}

	public double get5() {
		return v[5];
	}
	
	public final double get(int i) {
		return v[i];
	}

	public boolean isEq(DVector6 v2, double epsilon) {
		for (int i = 0; i < v.length; ++i) {
			if (Math.abs(v[i] - v2.v[i]) > epsilon) {
				return false;
			}
		}
		return true;
	}

	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public boolean isEq(DVector6 v2) {
		return Arrays.equals(v, v2.v);
	}

	/**
	 * Do not use. This can be slow, use ::isEq() instead.
	 */
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public boolean equals(Object obj) {
		if (this == obj) return true;
		if (obj == null) return false;
		if (!(obj instanceof DVector6)) return false;
		return isEq((DVector6)obj);
	}

	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public int hashCode() {
		int h = 0;
		for (double d: v) {
			h |= Double.doubleToRawLongBits(d);
			h <<= 4;
		}
		return h;
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
	
	public void set4(double d) {
		v[4] = d;
	}
	
	public void set5(double d) {
		v[5] = d;
	}

	/**
	 *  Set a vector/matrix at position i to a specific value.
	 * @param i Position of new value
	 * @param d New Value
	 */
	public final void set(int i, double d) {
		v[i] = d;
	}
	

	public void add(int i, double d) {
		v[i] += d;
	}

	/**
	 * @return The geometric length of this vector.
	 */
	public final double length() {
		return Math.sqrt( lengthSquared() );
	}

	public final double lengthSquared() {
		return get0()*get0() + get1()*get1() + get2()*get2() 
				+ get3()*get3() + get4()*get4() + get5()*get5();
	}
	
	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 * @return 'false' if vector could not be normalized.
	 */
	public final boolean safeNormalize6 ()
	{
		//TODO is this correct? Maybe the real problem was that no eps is defined?
		double d = Math.abs(v[0]); //TODO use epsilon for minimal values (?)
		for (int i = 1; i < v.length; i++) {
			if (Math.abs(v[i]) > d) {
				d = Math.abs(v[i]);
			}
		}
		
		if (d <= Double.MIN_NORMAL) {
			set(1.0, 0, 0, 0, 0, 0);
			return false;
		}
		
		for (int i = 0; i < v.length; i++) {
			v[i] /= d;
		}
		
		double sum = 0;
		for (double d2: v) {
			sum += d2*d2;
		}
		
		double l = 1./Math.sqrt(sum);
		for (int i = 0; i < v.length; i++) {
			v[i] *= l;
		}
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
	 * @return This vector.
	 */
	public DVector6 normalize()	{
		if (!safeNormalize6()) throw new IllegalStateException(
				"Normalization failed: " + this);
		return this;
	}
}
