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
	 */
	public double get(int i);
	public double get0();
	public double get1();
	public double get2();
	public double get3();
//	public float[] toFloatArray();
//	public DVector3 clone();
	public double lengthSquared();
	public double length();
	/** 
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
