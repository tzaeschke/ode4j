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

public abstract class DVector3View implements DVector3I {
	@Override
	public abstract double get0();
	@Override
	public abstract double get1();
	@Override
	public abstract double get2();
	@Override
	public abstract double get(int i);
	public abstract void set0(double d);
	public abstract void set1(double d);
	public abstract void set2(double d);

	protected DVector3View() {}
	public final double length() {
		return Math.sqrt(get0()*get0() + get1()*get1() + get2()*get2());
	}

	public double dot(DVector3View v2) {
		return get0()*v2.get0() + get1()*v2.get1() + get2()*v2.get2();
	}

	public final void set(DVector3 v2) {
		set0( v2.get0() );
		set1( v2.get1() );
		set2( v2.get2() );
	}

	public final void set(double x, double y, double z) {
		set0( x );
		set1( y );
		set2( z );
	}

	public final void set(int idx, double d) {
		switch (idx) {
			case 0:
				set0(d);
				break;
			case 1:
				set1(d);
				break;
			case 2:
				set2(d);
				break;
			default:
				throw new IllegalStateException();
		}
	}

	public final void scale(double s) {
		set0( get0() * s );
		set1( get1() * s );
		set2( get2() * s );
	}

	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("[").append( get0() ).append(", ");
		b.append( get1() ).append(", ");
		b.append( get2() ).append("]");
		return b.toString();
	}
	
	
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public boolean equals(Object obj) {
		if (this == obj) return true;
		if (obj == null) return false;
		if (!(obj instanceof DVector3I)) return false;
		DVector3I v = (DVector3I) obj;
		return get0()==v.get0() && get1()==v.get1() && get2()==v.get2();
	}
	
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public int hashCode() {
		return (int) (Double.doubleToRawLongBits(get0())  * 
		Double.doubleToRawLongBits(get1()) * 
		Double.doubleToRawLongBits(get2()));
	}
}
