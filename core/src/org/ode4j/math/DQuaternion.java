/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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
 * The {@link DQuaternionC} implementation class.
 * 
 * @author Andrew Wagner
 */
public class DQuaternion implements DQuaternionC {
	private final double[] values;
	/**
	 * The length or size of the array containing the DQuaternion values.
	 */
	public static final int LEN = 4;

	/**
	 * Creates a new empty DQuaternion.
	 */
	public DQuaternion() {
		values = new double[LEN];
	}

	/**
	 * Creates a new DQuaternion with the specified basis values.
	 * 
	 * @param c
	 *            - the constant element.
	 * @param i
	 *            - the ith element.
	 * @param j
	 *            - the jth element.
	 * @param k
	 *            - the kth element.
	 */
	public DQuaternion(double c, double i, double j, double k) {
		this();
		set(c, i, j, k);
	}

	/**
	 * Creates a new DQuaternion using the values from the specified
	 * {@link DQuaternionC}.
	 * 
	 * @param q
	 *            - the DQuaternionC to copy.
	 */
	public DQuaternion(DQuaternionC q) {
		this();
		set(q);
	}

	@Override
	public void add(double c, double i, double j, double k) {
		values[0] += c;
		values[1] += i;
		values[2] += j;
		values[3] += k;
	}

	@Override
	public void add(DQuaternionC q) {
		values[0] += q.get0();
		values[1] += q.get1();
		values[2] += q.get2();
		values[3] += q.get3();
	}

	@Override
	public int dim() {
		return LEN;
	}

	@Override
	public boolean equals(Object q) {
		if (q instanceof DQuaternionC) {
			DQuaternionC quaternion = (DQuaternionC) q;
			return get0() == quaternion.get0() && get1() == quaternion.get1()
					&& get2() == quaternion.get2()
					&& get3() == quaternion.get3();
		}
		return false;
	}

	@Override
	public final double get(int i) {
		return values[i];
	}

	@Override
	public double get0() {
		return values[0];
	}

	@Override
	public double get1() {
		return values[1];
	}

	@Override
	public double get2() {
		return values[2];
	}

	@Override
	public double get3() {
		return values[3];
	}

	@Override
	public final double length() {
		return Math.sqrt(lengthSquared());
	}

	@Override
	public final double lengthSquared() {
		return get0() * get0() + get1() * get1() + get2() * get2() + get3()
				* get3();
	}

	@Override
	public void normalize() {
		if (!safeNormalize4())
			throw new IllegalStateException("Normalization failed: " + this);
	}

	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular
	 * vectors. we must be robust to these small vectors. to prevent numerical
	 * error, first find the component a[i] with the largest magnitude and then
	 * scale all the components by 1/a[i]. then we can compute the length of `a'
	 * and scale the components by 1/l. this has been verified to work with
	 * vectors containing the smallest representable numbers.
	 * 
	 * @return {@code true} if the normalization succeeded, {@code false}
	 *         otherwise.
	 */
	public final boolean safeNormalize4() {
		double d = Math.abs(get0());
		// for (int i = 1; i < v.length; i++) {
		// if (Math.abs(v[i]) > d) {
		// d = Math.abs(v[i]);
		// }
		// }
		if (Math.abs(get1()) > d)
			d = Math.abs(get1());
		if (Math.abs(get2()) > d)
			d = Math.abs(get2());
		if (Math.abs(get3()) > d)
			d = Math.abs(get3());

		if (d <= Double.MIN_NORMAL) {
			set(1, 0, 0, 0);
			return false;
		}

		scale(1 / d);
		// for (int i = 0; i < v.length; i++) {
		// v[i] /= d;
		// }

		// double sum = 0;
		// for (double d2: v) {
		// sum += d2*d2;
		// }

		double l = 1. / length();// Math.sqrt(sum);
		// for (int i = 0; i < v.length; i++) {
		// v[i] *= l;
		// }
		scale(l);
		return true;
	}

	@Override
	public void scale(double s) {
		values[0] *= s;
		values[1] *= s;
		values[2] *= s;
		values[3] *= s;
	}

	@Override
	public final void scale(int i, double s) {
		values[i] *= s;
	}

	@Override
	public void set(double c, double i, double j, double k) {
		values[0] = c;
		values[1] = i;
		values[2] = j;
		values[3] = k;
	}

	@Override
	public void set(DQuaternionC q) {
		values[0] = q.get0();
		values[1] = q.get1();
		values[2] = q.get2();
		values[3] = q.get3();
	}

	@Override
	public final void set(int i, double d) {
		values[i] = d;
	}

	@Override
	public void set0(double c) {
		values[0] = c;
	}

	@Override
	public void set1(double i) {
		values[1] = i;
	}

	@Override
	public void set2(double j) {
		values[2] = j;
	}

	@Override
	public void set3(double k) {
		values[3] = k;
	}

	@Override
	public void setIdentity() {
		set(1, 0, 0, 0);
	}

	@Override
	public final void setZero() {
		set(0, 0, 0, 0);
	}

	@Override
	public final void sum(DQuaternionC q1, DQuaternionC q2, double s) {
		values[0] = q1.get0() + q2.get0() * s;
		values[1] = q1.get1() + q2.get1() * s;
		values[2] = q1.get2() + q2.get2() * s;
		values[3] = q1.get3() + q2.get3() * s;
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
}
