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

import org.ode4j.math.DMatrix3.DVector3ColView;
import org.ode4j.math.DMatrix3.DVector3RowTView;

/**
 * Constant (unmodifiable) interface for dMatrix3.
 * 
 * This returns an unmodifiable view of an (most likely) modifiable object.
 * 
 * <b>WARNING</b>: This is only unmodifiable for the user. The class that
 * returned this object may continue to modify it, these changes will also
 * reflect in the 'unmodifiable view' that the user has. If the user requires a
 * lasting immutable object, then the object needs to be cloned.
 * 
 * @author Tilmann Zaeschke
 */
public interface DMatrix3C {

	/**
	 * Creates a deep copy of this DMatrix3C object.
	 * 
	 * @return the new instance.
	 */
	DMatrix3 clone();

	/**
	 * Return a new {@link DVector3C} object containing the values of the
	 * specified column. For padding=4 this uses the elements c, 4+c, 8+c;
	 * 
	 * @param column
	 *            - the column index.
	 * @return the new DVector3.
	 */
	DVector3C columnAsNewVector(int column);

	/**
	 * Calculates the dot product of the the specified column of this DMatrix3
	 * with the given {@link DVector3C}.
	 * 
	 * @param col
	 *            - the column index.
	 * @param v
	 *            - the DVector3C to dot.
	 * @return the calculated dot product.
	 */
	double dotCol(int col, DVector3C v);

	/**
	 * Calculates the dot product of the the specified column of this DMatrix3
	 * with the specified column of another DMatrix3C.
	 * 
	 * @param col
	 *            - the column index of this DMatrix3.
	 * @param m2
	 *            - the other DMatrix3C.
	 * @param col2
	 *            - the column index of the other DMatrix3C.
	 * @return the calculated dot product.
	 */
	double dotColCol(int col, DMatrix3C m2, int col2);

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the given double array at the given offset.
	 * 
	 * @param row
	 *            - the row of this DMatrix3.
	 * @param c
	 *            - the double array.
	 * @param arrayOffset
	 *            - the offset of the double array.
	 * @return the calculated dot product.
	 */
	double dotRow(int row, double[] c, int arrayOffset);

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the given {@link DVector3C}.
	 * 
	 * @param row
	 *            - the row index.
	 * @param v
	 *            - the DVector3C.
	 * @return the calculated dot product.
	 */
	double dotRow(int row, DVector3C v);

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the specified column of another {@link DMatrix3C}.
	 * 
	 * @param row
	 *            - the row index of this DMatrix3.
	 * @param m2
	 *            - the other DMatrix3C.
	 * @param col
	 *            - the column index of the other DMatrix3C.
	 * @return the calculated dot product.
	 */
	double dotRowCol(int row, DMatrix3C m2, int col);

	/**
	 * Calculates the dot product of the the specified row of this DMatrix3 with
	 * the specified row of another {@link DMatrix3C}.
	 * 
	 * @param row
	 *            - the row index of this DMatrix3.
	 * @param m
	 *            - the other DMatrix3C.
	 * @param row2
	 *            - the row index of the other DMatrix3C.
	 * @return the calculated dot product.
	 */
	double dotRowRow(int row, DMatrix3C m, int row2);

	/**
	 * @param i
	 *            row
	 * @param j
	 *            column
	 * @return Value at (i,j).
	 */
	double get(int i, int j);

	/**
	 * Gets the value at index (0, 0).
	 * 
	 * @return the value at index (0, 0).
	 */
	double get00();

	/**
	 * Gets the value at index (0, 1).
	 * 
	 * @return the value at index (0, 1).
	 */
	double get01();

	/**
	 * Gets the value at index (0, 2).
	 * 
	 * @return the value at index (0, 2).
	 */
	double get02();

	/**
	 * Gets the value at index (1, 0).
	 * 
	 * @return the value at index (1, 0).
	 */
	double get10();

	/**
	 * Gets the value at index (1, 1).
	 * 
	 * @return the value at index (1, 1).
	 */
	double get11();

	/**
	 * Gets the value at index (1, 2).
	 * 
	 * @return the value at index (1, 2).
	 */
	double get12();

	/**
	 * Gets the value at index (2, 0).
	 * 
	 * @return the value at index (2, 0).
	 */
	double get20();

	/**
	 * Gets the value at index (2, 1).
	 * 
	 * @return the value at index (2, 1).
	 */
	double get21();

	/**
	 * Gets the value at index (2, 2).
	 * 
	 * @return the value at index (2, 2).
	 */
	double get22();

	/**
	 * Sets the value at index (0, 0).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set00(double d);

	/**
	 * Sets the value at index (0, 1).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set01(double d);

	/**
	 * Sets the value at index (0, 2).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set02(double d);

	/**
	 * Sets the value at index (1, 0).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set10(double d);

	/**
	 * Sets the value at index (1, 1).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set11(double d);

	/**
	 * Sets the value at index (1, 2).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set12(double d);

	/**
	 * Sets the value at index (2, 0).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set20(double d);

	/**
	 * Sets the value at index (2, 1).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set21(double d);

	/**
	 * Sets the value at index (2, 2).
	 * 
	 * @param d
	 *            - the value to set.
	 */
	void set22(double d);

	/**
	 * Gets the 3x3 float array representation of this DMatrix3C object.
	 * 
	 * @return the float array representation.
	 */
	float[] toFloatArray();

	/**
	 * Gets the 3x4 float array representation of this DMatrix3C object.
	 * 
	 * @return the float array representation.
	 */
	float[] toFloatArray2();

	/**
	 * Gets a {@link DVector3ColView} object for the specified column of the
	 * DMatrix3C.
	 * 
	 * @param col
	 *            - the column index.
	 * @return the DVector3ColView object.
	 */
	DVector3ColView viewCol(int col);

	/**
	 * Gets a {@link DVector3RowTView} object for the specified column of the
	 * DMatrix3C.
	 * 
	 * @param row
	 *            - the row index.
	 * @return the DVector3RowTView object.
	 */
	DVector3RowTView viewRowT(int row);
}
