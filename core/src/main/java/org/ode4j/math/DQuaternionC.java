/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2023 Tilmann Zaeschke     *
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
 * Constant (unmodifiable) interface for DQuaternion.
 * 
 * This returns an unmodifiable view of an (most likely) modifiable object.
 * 
 * A quaternion consists of four numbers, [w, x, y, z].
 * They are used top represent rigid body orientations. 
 *  
 * 
 * WARNING: This is only unmodifiable for the user. The class that returned
 * this object may continue to modify it, these changes will also reflect in
 * the 'unmodifiable view' that the user has.
 * If the user requires a lasting immutable object, then the object needs to 
 * be cloned. 
 *
 * @author Tilmann Zaeschke
 */
public interface DQuaternionC {

	/**
	 * @param i The row to return [0, 1, 2].
	 * @return The value at position i
	 */
	double get(int i);

	/**
	 * w of [w, x, y, z].
	 *
	 * @return w
	 */
	double get0();

	/**
	 * x of [w, x, y, z].
	 *
	 * @return x
	 */
	double get1();

	/**
	 * y of [w, x, y, z].
	 *
	 * @return y
	 */
	double get2();

	/**
	 * z of [w, x, y, z].
	 *
	 * @return z
	 */
	double get3();

	/**
	 * @return Euler angles (radians) derived from this quaternion.
	 */
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	DVector3 toEuler();

	/**
	 * @return Euler angles (degrees) derived from this quaternion.
	 */
	@Deprecated // TODO deprecated, to be removed in 0.6.0. Consider using DRotation.dRFromEulerAngles
	DVector3 toEulerDegrees();

	/**
	 * @return A mutable copy of this object.
	 */
    DQuaternion copy();

    /**
	 * @return w*w + x*x + y*y + z*z
	 */
	double lengthSquared();

	/**
	 * @return sqrt(w * w + x * x + y * y + z * z)
	 */
	double length();

	boolean isEq(DQuaternion q, double epsilon);

	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	boolean isEq(DQuaternion q);

	/**
	 * Calculates the inverse of the quaternion and returns it as a new quaternion.
	 * @return an inverted quaternion
	 */
	DQuaternion reInverse();

	/**
	 * @param b Other quaternion
	 * @return dot product of (this) and b
	 * @see DQuaternion#dot(DQuaternionC)
	 */
	double dot(DQuaternionC b);

	/**
	 * Do not use. This can be slow, use isEq() instead.
	 *
	 * @param obj object
	 * @return true if equal
	 * @deprecated
	 */
	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	boolean equals(Object obj);

	@Override
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	int hashCode();
}