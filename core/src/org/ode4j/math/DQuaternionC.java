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
 * Constant (unmodifiable) interface for dQuaternion.
 * 
 * This returns an unmodifiable view of an (most likely) modifiable object.
 * 
 * WARNING: This is only unmodifiable for the user. The class that returned this
 * object may continue to modify it, these changes will also reflect in the
 * 'unmodifiable view' that the user has. If the user requires a lasting
 * immutable object, then the object needs to be cloned.
 * 
 * @author Tilmann Zaeschke
 */
public interface DQuaternionC {

	/**
	 * Adds the specified values to the corresponding elements of this
	 * DQuaternionC.
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
	void add(double c, double i, double j, double k);

	/**
	 * Adds the values of the specified {@link DQuaternionC} to this
	 * DQuaternionC.
	 * 
	 * @param q
	 *            - the DQuaternionC to add.
	 */
	void add(DQuaternionC q);

	/**
	 * Gets the dimension or size of this DQuaternionC.
	 * 
	 * @return the dimension or size.
	 */
	int dim();

	/**
	 * Gets the element at the specified index.
	 * 
	 * @param i
	 *            - the element index.
	 * @return the element at the index.
	 */
	double get(int i);

	/**
	 * Gets the constant element of the DQuaternionC.
	 * 
	 * @return the constant element.
	 */
	double get0();

	/**
	 * Gets the ith element of the DQuaternionC.
	 * 
	 * @return the ith element.
	 */
	double get1();

	/**
	 * Gets the jth element of the DQuaternionC.
	 * 
	 * @return the jth element.
	 */
	double get2();

	/**
	 * Gets the kth element of the DQuaternionC.
	 * 
	 * @return the kth element.
	 */
	double get3();

	/**
	 * Gets the length or norm of the DQuaternionC, which is equal to the square
	 * root of the product of the quaternion and its conjugate.
	 * 
	 * @return the length.
	 */
	double length();

	/**
	 * Gets the length square of the DQuaternionC, which is equal to the product
	 * of the quaternion and its conjugate.
	 * 
	 * @return the length squared.
	 */
	double lengthSquared();

	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular
	 * vectors. we must be robust to these small vectors. to prevent numerical
	 * error, first find the component a[i] with the largest magnitude and then
	 * scale all the components by 1/a[i]. then we can compute the length of `a'
	 * and scale the components by 1/l. this has been verified to work with
	 * vectors containing the smallest representable numbers.
	 */
	void normalize();

	/**
	 * Scales the DQuaternionC by the specified scaling factor.
	 * 
	 * @param s
	 *            - the scaling factor.
	 */
	void scale(double s);

	/**
	 * Scales the specified element by the given scaling factor.
	 * 
	 * @param i
	 *            - the element index.
	 * @param s
	 *            - the scaling factor.
	 */
	void scale(int i, double s);

	/**
	 * Sets the elements of the DQuaternionC to the specified values.
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
	void set(double c, double i, double j, double k);

	/**
	 * Sets the elements of this DQuaternionC to the values in the specified
	 * DQuaternionC.
	 * 
	 * @param q
	 *            - the DQuaternionC whose values to use.
	 */
	void set(DQuaternionC q);

	/**
	 * Sets the specified element to the given value.
	 * 
	 * @param i
	 *            - the element index.
	 * @param d
	 *            - the value to set.
	 */
	void set(int i, double d);

	/**
	 * Sets the constant element of the DQuaternionC.
	 * 
	 * @param c
	 *            - the constant element.
	 */
	void set0(double c);

	/**
	 * Sets the ith element of the DQuaternionC.
	 * 
	 * @param i
	 *            - the ith element.
	 */
	void set1(double i);

	/**
	 * Sets the jth element of the DQuaternionC.
	 * 
	 * @param j
	 *            - the jth element.
	 */
	void set2(double j);

	/**
	 * Sets the kth element of the DQuaternionC.
	 * 
	 * @param k
	 *            - the kth element.
	 */
	void set3(double k);

	/**
	 * Sets the DQuaternionC to the identity quaternion.
	 */
	void setIdentity();

	/**
	 * Sets the DQuaternionC to be empty.
	 */
	void setZero();

	/**
	 * Sets the elements in this DQuaternionC to the sum of the corresponding
	 * elements in the specified DQuaternionC objects with the given scaling
	 * factor applied to the second DQuaternionC prior to summation.
	 * 
	 * @param q1
	 *            - the first DQuaternionC to sum.
	 * @param q2
	 *            - the second DQuaternionC to sum.
	 * @param s
	 *            - the scaling factor to apply to the second DQuaternionC.
	 */
	void sum(DQuaternionC q1, DQuaternionC q2, double s);
}
