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

import org.ode4j.math.DMatrix3.DVector3ColView;

/**
 * Constant (unmodifiable) interface for dMatrix3.
 * 
 * This returns an unmodifiable view of an (most likely) modifiable object.
 * 
 * <b>WARNING</b>: This is only unmodifiable for the user. The class that returned
 * this object may continue to modify it, these changes will also reflect in
 * the 'unmodifiable view' that the user has.
 * If the user requires a lasting immutable object, then the object needs to 
 * be cloned. 
 *
 * @author Tilmann Zaeschke
 */
public interface DMatrix3C {

	/**
	 * @param i row
	 * @param j column
	 * @return Value at (i,j).
	 */
	public double get(int i, int j);
	/** @return value at row 0, column 0 ([0]). */
	public double get00();
	/** @return value at row 0, column 1 ([1]). */
	public double get01();
	/** @return value at row 0, column 2 ([2]). */
	public double get02();
	/** @return value at row 2, column 0 ([4]). */
	public double get10();
	/** @return value at row 2, column 1 ([5]). */
	public double get11();
	/** @return value at row 2, column 2 ([6]). */
	public double get12();
	/** @return value at row 2, column 0 ([8]). */
	public double get20();
	/** @return value at row 2, column 1 ([9]). */
	public double get21();
	/** @return value at row 2, column 2 ([10]). */
	public double get22();
	public float[] toFloatArray();
	public float[] toFloatArray12();
	/**
	 * Please use @see #copy() instead. This is deprecated because we don't implement Cloneable.
	 * @return A clone() of this object.
	 */
	@Deprecated // TODO deprecated. Should be removed. Plese use copy() instead. To be removed in 0.6.0.
	DMatrix3 clone();
	/**
	 * @return A mutable copy of this object.
	 */
	DMatrix3 copy();
	public DVector3ColView viewCol(int _col);
	public double dotCol(int col, DVector3C b);
	public double dotRow(int row, DVector3C b);
	public double dotRow(int row, double[] c, int cOfs);
	public double dotColCol(int col, DMatrix3C m2, int col2);
	public double dotRowCol(int row, DMatrix3C m2, int col2);
	public double dotRowRow(int row, DMatrix3C m2, int row2);
	public DVector3 columnAsNewVector(int column);
	public boolean isEq(DMatrix3C m, double epsilon);
	@Deprecated // float is generally not comparable. To be removed in 0.6.0. TODO deprecated
	public boolean isEq(DMatrix3C m);
	public DMatrix3 reTranspose();
	public void getColumn0(DVector3 result);
	public void getColumn1(DVector3 result);
	public void getColumn2(DVector3 result);
}
