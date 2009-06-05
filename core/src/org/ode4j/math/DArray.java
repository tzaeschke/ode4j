package org.ode4j.math;

abstract class DArray<T extends DArray<T>> {
	/** @deprecated use set() instead */
	public final double[] v;

	DArray(int size) {
		v = new double[size];
	}
	
	DArray(double[] a) {
		v = a;
	}
	
	/**
	 *  Set a vector/matrix of size n to all zeros. 
	 */
	public final void dSetZero() {
		for (int i = 0; i < v.length; i++) {
			v[i] = 0;
		}
	}
	
	/**
	 *  Set all elements of a vector/matrix to 'val'. 
	 */
	public final void setValues(double val) {
		for (int i = 0; i < v.length; i++) {
			v[i] = val;
		}
	}
	
	/**
	 *  Set a vector/matrix of size n to a specific value.
	 */
	public final void dSetValue(double d) {
		for (int i = 0; i < v.length; i++) {
			v[i] = d;
		}
	}

	/**
	 *  Set a vector/matrix of size n to the values of another vector/matrix.
	 */
//	public T set(T array) {
//		System.arraycopy(array.v, 0, v, 0, v.length);
//		return (T) this;
//	}
//	
//	public dArray<T> add(T d) {
//		for (int i = 0; i < v.length; i++) {
//			v[i] += d.v[i];
//		}
//		return this;
//	}

	public void assertLen(int n) {
		if (n!=v.length) {
			throw new IllegalStateException("LEN is " + v.length + ", not " + n);
		}		
	}
	
//	public T scale(double d) {
//		for (int i = 0; i < v.length; i++) {
//			v[i] *= d;
//		}
//		return (T) this;
//	}
	
	public final float[] toFloatArray() {
		float[] f = new float[v.length];
		for (int i=0; i < v.length; i++) 
			f[i] = (float)v[i];
		return f;
	}
}
