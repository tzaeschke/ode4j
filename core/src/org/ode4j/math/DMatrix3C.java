package org.ode4j.math;

import org.ode4j.math.DMatrix3.DVector3ColView;

/**
 * Constant (unmodifiable) interface for dMatrix3.
 * 
 * This returns an unmodifiable view of an (most likely) modifiable object.
 * 
 * WARNING: This is only unmodifiable for the user. The class that returned
 * this object may continue to modify it, these changes will also reflect in
 * the 'unmodifiable view' that the user has.
 * If the user requires a lasting immutable object, then the object needs to 
 * be cloned. 
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
	public DVector3ColView viewCol(int _col);
}
