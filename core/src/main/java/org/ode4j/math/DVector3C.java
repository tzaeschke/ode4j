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
 * Constant (unmodifiable) interface for dVector3.
 * 
 * This returns an unmodifiable view of an (most likely) modifiable object.
 * 
 * WARNING: This is only unmodifiable for the user. The class that returned
 * this object may continue to modify it, these changes will also reflect in
 * the 'unmodifiable view' that the user has.
 * If the user requires a lasting immutable object, then the object needs to 
 * be copied().
 *
 * This interface should only be implemented by DVector3.
 * This allows efficient optimisation by the JVM, which is not
 * possible with 2 (still somewhat efficient) or more (slow)
 * sub-classes.
 *
 * @author Tilmann Zaeschke
 */
public interface DVector3C {

	/**
	 * @param i The row to return [0, 1, 2].
	 * @return Value ot position i
	 */
	double get(int i);
	double get0();
	double get1();
	double get2();
	float[] toFloatArray();
	double[] toDoubleArray();

	/**
	 * Please use @see #copy() instead. This is deprecated because we don't implement Cloneable.
	 * @return A clone() of this object.
	 */
	@Deprecated // TODO deprecated. Should be removed. Please use copy() instead. To be removed in 0.6.0.
	DVector3 clone();

	/**
	 * @return A mutable copy of this object.
	 */
	DVector3 copy();

	boolean isEq(DVector3C v, double epsilon);
	boolean isEq(double x, double y, double z, double epsilon);
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	boolean isEq(DVector3C v);
	double lengthSquared();
	double length();

	/** 
	 * @param a Other vector
	 * @return Distance between this vector and b).
	 * @see DVector3#distance(DVector3C) 
	 */
	double distance(DVector3C a);

	/** 
	 * @param b Other vector
	 * @return dot product of (this) and b
	 * @see DVector3#dot(DVector3C)
	 */
	double dot(DVector3C b);

	/**
	 * @param b Other vector
	 * @return dot product of (this) and b
	 * @see DVector3#dot(DVector3C) 
	 */
	double dot(DVector3View b);

	/**
	 * @param b Other vector
	 * @return cross product of (this) and b
	 * @see DVector3#cross(DVector3C)
	 */
	DVector3 cross(DVector3C b);

	/**
	 * @param v2 other vector
	 * @return return sum of (this)+v2
	 * @see DVector3#reAdd(DVector3C)
	 */
	DVector3 reAdd(DVector3C v2);
	DVector3 reAdd(double x, double y, double z);

    /**
     * @param v2 other vector
     * @return return difference of (this)-v2
     * @see DVector3#reSub(DVector3C) 
     */
	DVector3 reSub(DVector3C v2);
	DVector3 reScale(double s);
	float[] toFloatArray4();
	double dotCol(DMatrix3C m, int col);
}
