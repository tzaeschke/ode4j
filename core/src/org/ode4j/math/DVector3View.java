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
 * Represents a DVector3-like "view" of a particular object.
 * 
 * @author Andrew Wagner
 */
public abstract class DVector3View implements DVector3I {
	/**
	 * Calculates the dot product between this DVector3View and the specified
	 * DVector3View.
	 * 
	 * @param v2
	 *            - the DVector3View to dot.
	 * @return the resulting dot product.
	 */
	public double dot(DVector3View v2) {
		return get0() * v2.get0() + get1() * v2.get1() + get2() * v2.get2();
	}

	/**
	 * @{inherited
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (!(obj instanceof DVector3I))
			return false;
		DVector3I v = (DVector3I) obj;
		return get0() == v.get0() && get1() == v.get1() && get2() == v.get2();
	}

	@Override
	public abstract double get(int i);

	@Override
	public abstract double get0();

	@Override
	public abstract double get1();

	@Override
	public abstract double get2();

	@Override
	public int hashCode() {
		return (int) (Double.doubleToRawLongBits(get0())
				* Double.doubleToRawLongBits(get1()) * Double
				.doubleToRawLongBits(get2()));
	}

	/**
	 * Calculates the length or equivalently, the magnitude of the DVector3View.
	 * 
	 * @return the length/magnitude.
	 */
	public final double length() {
		return Math.sqrt(get0() * get0() + get1() * get1() + get2() * get2());
	}

	/**
	 * Scales the data which the DVector3View is representing by the specified
	 * scaling factor. This will mutate the data of the object being viewed.
	 * 
	 * @param s
	 *            - the scaling factor.
	 */
	public final void scale(double s) {
		set0(get0() * s);
		set1(get1() * s);
		set2(get2() * s);
	}

	/**
	 * Sets the three vector components of the DVector3View with the specified
	 * values.
	 * 
	 * @param x
	 *            - the x component.
	 * @param y
	 *            - the y component.
	 * @param z
	 *            - the z component.
	 */
	public final void set(double x, double y, double z) {
		set0(x);
		set1(y);
		set2(z);
	}

	/**
	 * Sets the data which the DVector3View is representing with the values of
	 * the specified {@link DVector3C}. This will mutate the data of the object
	 * being viewed.
	 * 
	 * @param v2
	 *            - the DVector3C whose values to use.
	 */
	public final void set(DVector3C v2) {
		set0(v2.get0());
		set1(v2.get1());
		set2(v2.get2());
	}

	@Override
	public abstract void set0(double d);

	@Override
	public abstract void set1(double d);

	@Override
	public abstract void set2(double d);

	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("[").append(get0()).append(", ");
		b.append(get1()).append(", ");
		b.append(get2()).append("]");
		return b.toString();
	}
}
