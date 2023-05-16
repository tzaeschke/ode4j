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
 * An arbitrary length vector.
 *
 * @author Tilmann Zaeschke
 */
public class DVectorN {
	
	private final double[] v;

	public DVectorN(int len) { 
		v = new double[len];
	}
	
	public DVectorN(DVectorN v2) {
		this(v2.v.length);
		set(v2.v);
	}

	public DVectorN(double[] data) {
		this(data.length);
		System.arraycopy(data, 0, v, 0, v.length);
	}

	/**
	 * Please use @see #copy() instead. This is deprecated because we don't implement Cloneable.
	 * @return A clone() of this object.
	 */
	@Override
	@Deprecated // TODO deprecated. Should be removed. Please use copy() instead. To be removed in 0.6.0.
	public DVectorN clone() {
		return new DVectorN(this);
	}

	/**
	 * @return A copy of this object.
	 */
	public DVectorN copy() {
		return new DVectorN(this);
	}

	@Override
	public String toString() {
		StringBuilder b = new StringBuilder();
		b.append("dVector3[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
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
	
	public double get0() {
		return v[0];
	}
	
	public double get1() {
		return v[1];
	}
	
	public double get2() {
		return v[2];
	}
	
	public final double get(int i) {
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
	 * @return 'false' if vector could not be normalized
	 */
	public final boolean safeNormalizeN ()
	{
		//TODO is this correct? Maybe the real problem was that no eps is defined?
		double d = Math.abs(v[0]); //TODO use epsilon for minimal values (?)
		for (int i = 1; i < v.length; i++) {
			if (Math.abs(v[i]) > d) {
				d = Math.abs(v[i]);
			}
		}
		
		if (d <= Double.MIN_NORMAL) {
			setIdentity();
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
	 */
	public void normalize()
	{
		if (!safeNormalizeN()) throw new IllegalStateException(
				"Normalization failed: " + this);
	}

	/**
	 *  Set a vector/matrix at position i to a specific value.
	 * @param i position of new value
	 * @param d new value
	 */
	public final void set(int i, double d) {
		v[i] = d;
	}

	public void set(double[] a) {
		System.arraycopy(a, 0, v, 0, v.length);
	}
	
	public void setIdentity() {
		v[0] = 1;
		for (int i = 1; i < v.length-1; i++) {
			v[i] = 0;
		}
	}
}


