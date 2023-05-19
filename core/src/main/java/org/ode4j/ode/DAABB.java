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
package org.ode4j.ode;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

/**
 * Convenience class for operations on AABB (Axis-aligned bounding boxes).
 * <p>
 * Axes are numbered from 0 to 2.
 * @see DAABBC
 *
 * @author Tilmann Zaeschke
 */
public class DAABB implements DAABBC {
	
	private final DVector3 _min = new DVector3();
	private final DVector3 _max = new DVector3();
	
	public DAABB() {}

	public DAABB(double min0, double max0, double min1, double max1, double min2, double max2) {
		_min.set(min0, min1, min2);
		_max.set(max0, max1, max2);
	}


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
	 * @param mm0 min/max 0
	 * @param mm1 min/max 1
	 * @param mm2 min/max 2
	 */
	public void setMinMax(double mm0, double mm1, double mm2) {
		//set(-mm0, mm0, -mm1, mm1, -mm2, mm2);
		_min.set(-mm0, -mm1, -mm2);
		_max.set(mm0, mm1, mm2);
	}

	/**
	 * Move this AABB by <tt>pos</tt>.
	 * @param pos pos
	 */
	public void shiftPos(DVector3C pos) {
		_min.add(pos);
		_max.add(pos);
	}

	@Override
	public double len0() {
		return getMax0() - getMin0();
	}

	@Override
	public double len1() {
		return getMax1() - getMin1();
	}

	@Override
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
	@Override
	public boolean isValid() {
		return !Double.isNaN(getMin0()) && !Double.isNaN(getMax0()) &&
				!Double.isNaN(getMin1()) && !Double.isNaN(getMax1()) &&
				!Double.isNaN(getMin2()) && !Double.isNaN(getMax2());
	}

	/**
	 * Checks whether the to AABBs are disjoint.
	 * @param aabb2 aabb2
	 * @return <tt>false</tt> if the two AABBs overlap.
	 */
	@Override
	public boolean isDisjoint(DAABBC aabb2) {
		return getMin0() > aabb2.getMax0() ||
				getMax0() < aabb2.getMin0() ||
				getMin1() > aabb2.getMax1() ||
				getMax1() < aabb2.getMin1() ||
				getMin2() > aabb2.getMax2() ||
				getMax2() < aabb2.getMin2();
	}

	/**
	 * Determines the AABB from a box defined by the two vectors.
	 * @param v1 v1
	 * @param v2 v2
	 */
	public void setMinMax(DVector3C v1, DVector3C v2) {
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
		_min.set0(d);
	}
	
	public void setMin1(double d) {
		_min.set1(d);
	}
	
	public void setMin2(double d) {
		_min.set2(d);
	}
	
	public void setMax0(double d) {
		_max.set0(d);
	}
	
	public void setMax1(double d) {
		_max.set1(d);
	}
	
	public void setMax2(double d) {
		_max.set2(d);
	}
	
	/**
	 * get[0].
	 * @return min[0]
	 */
	@Override
	public double getMin0() {
		return _min.get0();
	}
	
    /**
     * get[2].
	 * @return min[1]
     */
	@Override
	public double getMin1() {
		return _min.get1();
	}
	
    /**
     * get[4].
	 * @return min[2]
     */
	@Override
	public double getMin2() {
		return _min.get2();
	}
	
    /**
     * get[1].
	 * @return max[0]
     */
	@Override
	public double getMax0() {
		return _max.get0();
	}
	
    /**
     * get[3].
	 * @return max[1]
     */
	@Override
	public double getMax1() {
		return _max.get1();
	}
	
    /**
     * get[5].
	 * @return max[2]
     */
	@Override
	public double getMax2() {
		return _max.get2();
	}

	@Override
	public double getMax(int i) {
		return _max.get(i);
	}

	@Override
	public double getMin(int i) {
		return _min.get(i);
	}

	@Override
	public DVector3 getLengths() {
		return new DVector3(len0(), len1(), len2());
	}

	@Override
	public DVector3 getCenter() {
		return new DVector3(avg0(), avg1(), avg2());
	}

	public void set(DAABBC aabb) {
		setMin0(aabb.getMin0());
		setMin1(aabb.getMin1());
		setMin2(aabb.getMin2());
		setMax0(aabb.getMax0());
		setMax1(aabb.getMax1());
		setMax2(aabb.getMax2());
	}
	
	public void set(double min0, double max0, double min1, 
			double max1, double min2, double max2) {
		_min.set(min0, min1, min2);
		_max.set(max0, max1, max2);
	}
	
	public void setZero() {
		_min.setZero();
		_max.setZero();
	}
	
	public void setMin(DVector3C min) {
		_min.set(min);
	}
	
	public void setMax(DVector3C max) {
		_max.set(max);
	}

	/**
	 * Expand this AABB to include the given point.
	 * @param point point 
	 */
	public void expand(DVector3C point) {
		if (point.get0() < _min.get0()) _min.set0(point.get0());
		if (point.get1() < _min.get1()) _min.set1(point.get1());
		if (point.get2() < _min.get2()) _min.set2(point.get2());
		if (point.get0() > _max.get0()) _max.set0(point.get0());
		if (point.get1() > _max.get1()) _max.set1(point.get1());
		if (point.get2() > _max.get2()) _max.set2(point.get2());
	}

	/**
	 * Expand this AABB to include the given AABB.
	 * @param aabb aabb
	 */
	public void expand(DAABBC aabb) {
		if (aabb.getMin0() < _min.get0()) _min.set0(aabb.getMin0());
		if (aabb.getMin1() < _min.get1()) _min.set1(aabb.getMin1());
		if (aabb.getMin2() < _min.get2()) _min.set2(aabb.getMin2());
		if (aabb.getMax0() > _max.get0()) _max.set0(aabb.getMax0());
		if (aabb.getMax1() > _max.get1()) _max.set1(aabb.getMax1());
		if (aabb.getMax2() > _max.get2()) _max.set2(aabb.getMax2());
	}
	
	@Override
	public String toString() {
		return "DAABB min=" + _min + " / max=" + _max;
	}
}
