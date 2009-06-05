package org.ode4j.math;

import org.ode4j.math.DMatrix3.DVector3View;

/**
 * Constant (unmodifiable) interface for dMatrix3.
 *
 * @author Tilmann Zaeschke
 */
public interface DMatrix3C {

	/**
	 * @param i The field to return.
	 */
	public double get(int i);
	/**
	 * @param i row
	 * @param j column
	 * @return Value at (i,j).
	 */
	public double get(int i, int j);
	public double get00();
	public double get01();
	public double get02();
	public double get10();
	public double get11();
	public double get12();
	public double get20();
	public double get21();
	public double get22();
	public float[] toFloatArray();
	public DMatrix3 clone();
	public DVector3View viewCol(int _col);
}
