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
 * This interface is should not be used externally. It is only provided to
 * enforce compatible method naming in sub-classes.
 * 
 * @author Tilmann Zaeschke
 */
interface DVector3I {
	/**
	 * Gets the value of the DVector3I at the specified index.
	 * 
	 * @param i
	 *            - the index whose value to get.
	 * @return the value at the specified index.
	 */
	double get(int i);

	/**
	 * Gets the value at index 0.
	 * 
	 * @return the value at index 0.
	 */
	double get0();

	/**
	 * Gets the value at index 1.
	 * 
	 * @return the value at index 1.
	 */
	double get1();

	/**
	 * Gets the value at index 2.
	 * 
	 * @return the value at index 2.
	 */
	double get2();

	/**
	 * Sets the value at index 0.
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set0(double d);

	/**
	 * Sets the value at index 1.
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set1(double d);

	/**
	 * Sets the value at index 2.
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set2(double d);
}
