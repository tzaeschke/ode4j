package org.ode4j.math;

public class DVectorN {
	
	private final double[] v;
	public static final DVector3 ZERO = new DVector3(0, 0, 0);
	public static final int CURRENT_LENGTH = 4;

	public DVectorN(int len) { 
		v = new double[len];
	}
	
	public DVectorN(DVectorN v2) {
		this(v2.v.length);
		set(v2.v);
	}

	public DVectorN(double[] data) {
		this(data.length);
		System.arraycopy(data, 0, v, 0, v.length);
	}

	public DVectorN clone() {
		return new DVectorN(this);
	}
	
	
	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("dVector3[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
	}

//	@Override
//	public void assertLen(int n) {
//		if (n!=v.length) {
//			throw new IllegalStateException("LEN is " + v.length + ", not " + n);
//		}		
//	}
	
	public void set0(double d) {
		v[0] = d;
	}
	
	public void set1(double d) {
		v[1] = d;
	}
	
	public void set2(double d) {
		v[2] = d;
	}
	
	public double get0() {
		return v[0];
	}
	
	public double get1() {
		return v[1];
	}
	
	public double get2() {
		return v[2];
	}
	
	public final double get(int i) {
		return v[i];
	}

	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 */
	public final boolean safeNormalizeN ()
	{
		//TODO is this correct? Maybe the real problem was that no eps is defined?
		double d = Math.abs(v[0]); //TODO use epsilon for minimal values (?)
		for (int i = 1; i < v.length; i++) {
			if (Math.abs(v[i]) > d) {
				d = Math.abs(v[i]);
			}
		}
		
		if (d <= Double.MIN_NORMAL) {
			setIdentity();
			return false;
		}
		
		for (int i = 0; i < v.length; i++) {
			v[i] /= d;
		}
		
		double sum = 0;
		for (double d2: v) {
			sum += d2*d2;
		}
		
		double l = 1./Math.sqrt(sum);
		for (int i = 0; i < v.length; i++) {
			v[i] *= l;
		}
		return true;
	}
	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 */
	public void normalize()
	{
		if (!safeNormalizeN()) throw new IllegalStateException(
				"Normalization failed: " + this);
	}

	/**
	 *  Set a vector/matrix at position i to a specific value.
	 */
	public final void set(int i, double d) {
		v[i] = d;
	}

	public void set(double[] a) {
		System.arraycopy(a, 0, v, 0, v.length);
		//return (T) this;
	}
	
	public void setIdentity() {
		v[0] = 1;
		for (int i = 1; i < v.length-1; i++) {
			v[i] = 0;
		}
	}
}


