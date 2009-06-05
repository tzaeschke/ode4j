package org.ode4j.math;

import java.util.Arrays;

public class DQuaternion extends DVector<DQuaternion> implements DQuaternionC {
	public static final int LEN = 4;

	public DQuaternion() {
		super(LEN);
	}
	
	public DQuaternion(double x0, double x1, double x2, double x3) {
		super(LEN);
		set(x0, x1, x2, x3);
	}
	
	public DQuaternion(DQuaternion x) {
		super(LEN);
		set(x);
	}

	@Override
	public String toString() {
		StringBuffer b = new StringBuffer();
		b.append("dQuaternion[");
		for (int i = 0; i < v.length-1; i++) {
			b.append(v[i]).append(", ");
		}
		b.append(v[v.length-1]).append("]");
		return b.toString();
	}
	
	public DQuaternion set(double x0, double x1, double x2, double x3) {
		v[0]=x0; v[1]=x1; v[2]=x2; v[3]=x3;
		return this;
	}

	public DQuaternion set(DQuaternionC q) {
		v[0]=q.get0(); v[1]=q.get1(); v[2]=q.get2(); v[3]=q.get3();
		return this;
	}
	
	public DQuaternion scale(double d) {
		for (int i = 0; i < LEN; i++) v[i] *= d;
		return this;
	}

	public DQuaternion add(DQuaternion dq) {
		for (int i = 0; i < LEN; i++) v[i] += dq.v[i];
		return this;
	}

	@Override
	public double get0() {
		return v[0];
	}

	@Override
	public double get1() {
		return v[1];
	}

	@Override
	public double get2() {
		return v[2];
	}

	@Override
	public double get3() {
		return v[3];
	}

	public int dim() {
		return LEN;
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

	public boolean equals(DQuaternion q) {
		return Arrays.equals(v, q.v);
	}

	public boolean equals(Object q) {
		return false;
	}

	public void add(double d0, double d1, double d2, double d3) {
		v[0] += d0;
		v[1] += d1;
		v[2] += d2;
		v[3] += d3;
	}

	public void sum(DQuaternion q1, DQuaternion q2, double d2) {
		v[0] = q1.get0() + q2.get0() * d2;
		v[1] = q1.get1() + q2.get1() * d2;
		v[2] = q1.get2() + q2.get2() * d2;
		v[3] = q1.get3() + q2.get3() * d2;
	}
}
