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
	double get(int i);
	double get0();
	double get1();
	double get2();
	void set0(double d);
	void set1(double d);
	void set2(double d);
}
