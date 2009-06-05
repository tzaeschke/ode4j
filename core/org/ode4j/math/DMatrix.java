package org.ode4j.math;

public abstract class DMatrix<R, C> extends DArray<DMatrix<R, C>> {

	protected final int MAX_I;
	protected final int MAX_J;
	
	protected DMatrix(int max_i, int max_j) {
		super(max_i * max_j);
		MAX_I = max_i;
		MAX_J = max_j;
	}
	
	DMatrix(double[] a, int i, int j) {
		super(a);
		MAX_I = i;
		MAX_J = j;
		if (i*j != a.length) {
			throw new IllegalArgumentException("i=" + i + "; j=" + j + 
					"; l=" + a.length);
		}
	}
	

	public void add(int i, int j, double d) {
		v[i*MAX_J + j] += d;
	}

	public void sub(int i, int j, double d) {
		v[i*MAX_J + j] -= d;
	}
	
	/**
	 * @param i row
	 * @param j column
	 * @return Value at (i,j).
	 */
	public double get(int i, int j) {
		return v[i*MAX_J + j];
	}

	public void set(int i, int j, double a) {
		v[i*MAX_J + j] = a;
	}
}
