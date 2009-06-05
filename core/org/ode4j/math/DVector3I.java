package org.ode4j.math;

/**
 * This interface is  should not be used externally. It is only
 * provided to enforce compatible method naming in sub-classes.
 *
 * @author Tilmann Zaeschke
 */
interface DVector3I {

	/**
	 * @param i The row to return [0, 1, 2].
	 */
	public double get(int i);
	public double get0();
	public double get1();
	public double get2();
}
