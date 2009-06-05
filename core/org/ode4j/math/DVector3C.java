package org.ode4j.math;

import org.ode4j.math.DMatrix3.DVector3View;

/**
 * Constant (unmodifiable) interface for dVector3.
 *
 * This interface should only be implemented by DVector3.
 * This allows efficient optimisation by the JVM, which is not
 * possible with 2 (still somewhat efficient) or more (slow)
 * sub-classes.
 *
 * @author Tilmann Zaeschke
 */
public interface DVector3C {

	/**
	 * @param i The row to return [0, 1, 2].
	 */
	public double get(int i);
	public double get0();
	public double get1();
	public double get2();
	public float[] toFloatArray();
	public DVector3 clone();
	public double lengthSquared();
	public double length();

	public double reDot(DVector3C b);
	public double reDot(DVector3View b);
	public DVector3 reSub(DVector3C pos);
	public DVector3 reScale(double s);
}
