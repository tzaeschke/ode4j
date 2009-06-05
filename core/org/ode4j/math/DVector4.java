package org.ode4j.math;


public class DVector4 extends DVector<DVector4> {
	
	private static final int LEN = 4;
	
	public DVector4(DVector4 v4) {
		super(4);
		set(v4.v);
	}

	public DVector4() {
		super(4);
		/** Nothing */	
	}

	public void set(double a, double b, double c, double d) {
		v[0] = a;
		v[1] = b;
		v[2] = c;
		v[3] = d;
	}

	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("dVector4[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
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

	public double get3() {
		return v[3];
	}

	public int dim() {
		return LEN;
	}
}
