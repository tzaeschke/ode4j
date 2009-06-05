package org.ode4j.math;

import java.util.Arrays;


public class DVector6 extends DVector<DVector6>{

	private static final int LEN = 6;
	
	public DVector6() {
		super(6);
	}

	public DVector6(DVector6 v2) {
		super(6);
		set(v2);
	}

	public DVector6(double d0, double d1, double d2, double d3, double d4, 
			double d5) {
		super(6);
		set(d0, d1, d2, d3, d4, d5);
	}

	public DVector6 set(double d0, double d1, double d2, double d3, double d4,
			double d5) {
		v[0] = d0;
		v[1] = d1;
		v[2] = d2;
		v[3] = d3;
		v[4] = d4;
		v[5] = d5;
		return this;
	}

	public void set(DVector6 v2) {
		for (int i = 0; i < LEN; i++) {
			v[i] = v2.v[i]; 
		}
	}
	
	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("dVector6[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
	}

	public int dim() {
		return LEN;
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

	public double get4() {
		return v[4];
	}

	public double get5() {
		return v[5];
	}
	
	public boolean equals(DVector6 v2) {
		return Arrays.equals(v, v2.v);
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
	
	public void set3(double d) {
		v[3] = d;
	}
	
	public void set4(double d) {
		v[4] = d;
	}
	
	public void set5(double d) {
		v[5] = d;
	}
}
