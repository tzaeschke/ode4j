package org.ode4j.math;

abstract class DVector<T extends DVector<T>>  {

//	DVector(double[] a) {
//		super(a);
//	}
//
//	DVector(int len) {
//		super(len);
//	}

//	public final double get(int i) {
//		return v[i];
//	}

	/**
	 *  Set a vector/matrix at position i to a specific value.
	 */
//	public final void set(int i, double d) {
//		v[i] = d;
//	}

//	public void scale(int i, double d) {
//		v[i] *= d;
//	}

//TODO this is useful code, maybe inplement it somehwere else? VectorN?	
//	/**
//	 * this may be called for vectors `a' with extremely small magnitude, for
//	 * example the result of a cross product on two nearly perpendicular vectors.
//	 * we must be robust to these small vectors. to prevent numerical error,
//	 * first find the component a[i] with the largest magnitude and then scale
//	 * all the components by 1/a[i]. then we can compute the length of `a' and
//	 * scale the components by 1/l. this has been verified to work with vectors
//	 * containing the smallest representable numbers.
//	 */
//	public final boolean dSafeNormalize3 ()
//	{
//		//TODO is this correct? Maybe the real problem was that no eps is defined?
//		double d = Math.abs(v[0]); //TODO use epsilon for minimal values (?)
//		for (int i = 1; i < v.length; i++) {
//			if (Math.abs(v[i]) > d) {
//				d = Math.abs(v[i]);
//			}
//		}
//		
//		if (d <= Double.MIN_NORMAL) {
//			dSetValue(0.0);
//			set(0, 1.0);
//			return false;
//		}
//		
//		for (int i = 0; i < v.length; i++) {
//			v[i] /= d;
//		}
//		
//		double sum = 0;
//		for (double d2: v) {
//			sum += d2*d2;
//		}
//		
//		double l = 1./Math.sqrt(sum);
//		for (int i = 0; i < v.length; i++) {
//			v[i] *= l;
//		}
//		return true;
//	}

//	/**
//	 * 
//	 * @return The geometric length of this vector.
//	 */
//	public final double length() {
//		double sum = 0;
//		for (double d: v) {
//			sum += d*d;
//		}
//		return Math.sqrt( sum ) ;
//	}
//
//	public final double lengthSquared() {
//		double sum = 0;
//		for (double d: v) {
//			sum += d*d;
//		}
//		return sum;
//	}

//	public void set(double[] a) {
//		System.arraycopy(a, 0, v, 0, v.length);
//		//return (T) this;
//	}
	
//	public final void set(double[] da, int da_ofs) {
//		System.arraycopy(da, da_ofs, v, 0, v.length);
//	}
	

//	public final void add(int j, double d) {
//		v[j] += d;
//	}
	
//	public T sub(T d) {
//		for (int i = 0; i < v.length; i++) {
//			v[i] -= d.v[i];
//		}
//		return (T) this;
//	}
	
//	public dVector<T> scale(T d) {
//		for (int i = 0; i < v.length; i++) {
//			v[i] *= d.v[i];
//		}
//		return this;
//	}
	
}
