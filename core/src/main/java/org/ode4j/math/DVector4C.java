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
 * be cloned. 
 *
 * This interface should only be implemented by DVector3.
 * This allows efficient optimisation by the JVM, which is not
 * possible with 2 (still somewhat efficient) or more (slow)
 * sub-classes.
 *
 * @author Tilmann Zaeschke
 */
public interface DVector4C {

	/**
	 * @param i The row to return [0, 1, 2].
	 * @return value at position i
	 */
	public double get(int i);
	public double get0();
	public double get1();
	public double get2();
	public double get3();
	public double lengthSquared();
	public double length();
	/** 
	 * @param b b
	 * @return Dot product of (this)*b
	 * @see DVector4#dot(DVector4C)
	 */
	public double dot(DVector4C b);
//	/** 
//	 * @see DVector3#dot(DVector4C) 
//	 */
//	public double dot(DVector3View b);
//
//	public DVector3 reSub(DVector4C pos);
//	public DVector3 reScale(double s);
//	public float[] toFloatArray4();
//	public double dotCol(DMatrix3C m, int col);
}
