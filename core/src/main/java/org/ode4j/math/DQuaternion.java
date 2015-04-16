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


/**
 * A quaternion consists of four numbers, [w, x, y, z].
 * They are used top represent rigid body orientations. 
 * 
 * @author Tilmann Zaeschke
 */
public class DQuaternion implements DQuaternionC {

	private final double[] v;
	public static final int LEN = 4;

	public DQuaternion() {
		v = new double[LEN];
	}
	
	public DQuaternion(double x0, double x1, double x2, double x3) {
		this();
		set(x0, x1, x2, x3);
	}
	
	public DQuaternion(DQuaternion x) {
		this();
		set(x);
	}

	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("DQuaternion[ ");
		b.append(get0()).append(", ");
		b.append(get1()).append(", ");
		b.append(get2()).append(", ");
		b.append(get3()).append(" ]");
		return b.toString();
	}
	
	public DQuaternion set(double x0, double x1, double x2, double x3) {
		v[0]=x0; v[1]=x1; v[2]=x2; v[3]=x3;
		return this;
	}

	public DQuaternion set(DQuaternionC q) {
		v[0]=q.get0(); v[1]=q.get1(); v[2]=q.get2(); v[3]=q.get3();
		return this;
	}
	
	public DQuaternion scale(double d) {
		v[0] *= d; v[1] *= d; v[2] *=d; v[3] *= d;
		return this;
	}

	public DQuaternion add(DQuaternion q) {
		v[0] += q.get0(); v[1] += q.get1(); v[2] +=q.get2(); v[3] += q.get3();
		return this;
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

	/**
	 * Sets w of [w, x, y, z].
	 * @param w
	 */
	public void set0(double w) {
		v[0] = w;
	}

	/**
	 * Sets x of [w, x, y, z].
	 * @param x
	 */
	public void set1(double x) {
		v[1] = x;
	}

	/**
	 * Sets y of [w, x, y, z].
	 * @param y
	 */
	public void set2(double y) {
		v[2] = y;
	}

	/**
	 * Sets z of [w, x, y, z].
	 * @param z
	 */
	public void set3(double z) {
		v[3] = z;
	}

	public boolean isEq(DQuaternion q) {
		return get0()==q.get0() && get1()==q.get1() && get2()==q.get2() && get3()==q.get3();
	}

	/**
	 * Do not use. This can be slow, use isEq() instead.
	 */
	@Override
	@Deprecated
	public boolean equals(Object obj) {
		if (this == obj) return true;
		if (obj == null) return false;
		if (!(obj instanceof DQuaternion)) return false;
		return isEq((DQuaternion)obj);
	}

	@Override
	public int hashCode() {
		int h = 0;
		for (double d: v) {
			h |= Double.doubleToRawLongBits(d);
			h <<= 6;
		}
		return h;
	}

	public void add(double d0, double d1, double d2, double d3) {
		v[0] += d0;
		v[1] += d1;
		v[2] += d2;
		v[3] += d3;
	}

	public final void sum(DQuaternion q1, DQuaternion q2, double d2) {
		v[0] = q1.get0() + q2.get0() * d2;
		v[1] = q1.get1() + q2.get1() * d2;
		v[2] = q1.get2() + q2.get2() * d2;
		v[3] = q1.get3() + q2.get3() * d2;
	}

	/**
	 *  Set a vector/matrix at position i to a specific value.
	 */
	public final void set(int i, double d) {
		v[i] = d;
	}

	public final void setZero() {
		set(0, 0, 0, 0);
	}

	public final void scale(int i, double l) {
		v[i] *=l;
	}

	public final double lengthSquared() {
		return get0()*get0() + get1()*get1() + get2()*get2() + get3()*get3();
	}

	@Override
	public final double get(int i) {
		return v[i];
	}

	public final double length() {
		return Math.sqrt(lengthSquared());
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
		double d = Math.abs(get0());
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

	public void setIdentity() {
		set(1, 0, 0, 0);
	}
}
