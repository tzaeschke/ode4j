/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector6;

/**
 * Convenience class for operations on AABB (Axis-aligned bounding boxes).
 *
 * @author Tilmann Zaeschke
 */
public class DAABB extends DVector6 implements DAABBC {
	
	
	/**
	 * Set the minimum and maximum values to -infinity and
	 * infinity.
	 */
	public void setToInfinity() {
		setMinMax(OdeConstants.dInfinity, OdeConstants.dInfinity, 
				OdeConstants.dInfinity);
	}
	
	/**
	 * Set the minimum and maximum values to -+ the given parameters.
	 */
	public void setMinMax(double mm0, double mm1, double mm2) {
		set(-mm0, mm0, -mm1, mm1, -mm2, mm2);
	}

	/**
	 * Move this AABB by <tt>pos</tt>.
	 * @param pos
	 */
	public void shiftPos(DVector3C pos) {
		//TODO optimize, use 2 dVector3 instances here!
		add(0, pos.get0());
		add(1, pos.get0());
		add(2, pos.get1());
		add(3, pos.get1());
		add(4, pos.get2());
		add(5, pos.get2());
	}

	public double len0() {
		return getMax0() - getMin0();
	}

	public double len1() {
		return getMax1() - getMin1();
	}

	public double len2() {
		return getMax2() - getMin2();
	}
	
	public double avg0() {
		return 0.5 * (getMax0() + getMin0());
	}

	public double avg1() {
		return 0.5 * (getMax1() + getMin1());
	}

	public double avg2() {
		return 0.5 * (getMax2() + getMin2());
	}
	
	/**
	 * 
	 * @return <tt>false</tt> is any of the values is NaN.
	 */
	public boolean isValid() {
		if (Double.isNaN(getMin0()) || Double.isNaN(getMax0()) ||
				Double.isNaN(getMin1()) || Double.isNaN(getMax1()) ||
				Double.isNaN(getMin2()) || Double.isNaN(getMax2()) ) {
			return false;
		}
		return true;
	}

	/**
	 * Checks whether the to AABBs are disjoint.
	 * @param aabb2
	 * @return <tt>false</tt> if the two AABBs overlap.
	 */
	public boolean isDisjoint(DAABBC aabb2) {
		if (getMin0() > aabb2.getMax0() ||
				getMax0() < aabb2.getMin0() ||
				getMin1() > aabb2.getMax1() ||
				getMax1() < aabb2.getMin1() ||
				getMin2() > aabb2.getMax2() ||
				getMax2() < aabb2.getMin2()) {
			return true;
		}
		return false;
	}

	/**
	 * Determines the AABB from a box defined by the two vectors.
	 * @param v1
	 * @param v2
	 */
	public void setMinMax(DVector3 v1, DVector3 v2) {
		if (v1.get0() < v2.get0()) {
			setMin0( v1.get0() );
			setMax0( v2.get0() );
		} else {
			setMin0( v2.get0() );
			setMax0( v1.get0() );
		}

		if (v1.get1() < v2.get1()) {
			setMin1( v1.get1() );
			setMax1( v2.get1() );
		} else {
			setMin1( v2.get1() );
			setMax1( v1.get1() );
		}

		if (v1.get2() < v2.get2()) {
			setMin2( v1.get2() );
			setMax2( v2.get2() );
		} else {
			setMin2( v2.get2() );
			setMax2( v1.get2() );
		}
	}
	
	public void setMin0(double d) {
		set0(d);
	}
	
	public void setMin1(double d) {
		set2(d);
	}
	
	public void setMin2(double d) {
		set4(d);
	}
	
	public void setMax0(double d) {
		set1(d);
	}
	
	public void setMax1(double d) {
		set3(d);
	}
	
	public void setMax2(double d) {
		set5(d);
	}
	
	public double getMin0() {
		return get0();
	}
	
	public double getMin1() {
		return get2();
	}
	
	public double getMin2() {
		return get4();
	}
	
	public double getMax0() {
		return get1();
	}
	
	public double getMax1() {
		return get3();
	}
	
	public double getMax2() {
		return get5();
	}

	@Override
	public double getMax(int i) {
		return v[2*i+1];
	}

	@Override
	public double getMin(int i) {
		return v[2*i];
	}

	public DVector3 getLengths() {
		return new DVector3(len0(), len1(), len2());
	}

	public DVector3 getCenter() {
		return new DVector3(avg0(), avg1(), avg2());
	}
}
