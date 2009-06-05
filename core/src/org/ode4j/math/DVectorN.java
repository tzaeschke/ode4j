package org.ode4j.math;

public class DVectorN extends DVector<DVectorN> {
	public static final DVector3 ZERO = new DVector3(0, 0, 0);
	public static final int CURRENT_LENGTH = 4;

	public DVectorN(int len) { 
		super(len); 
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

	@Override
	public void assertLen(int n) {
		if (n!=v.length) {
			throw new IllegalStateException("LEN is " + v.length + ", not " + n);
		}		
	}
	
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
}


