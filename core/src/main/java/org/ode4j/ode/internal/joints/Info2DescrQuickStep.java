package org.ode4j.ode.internal.joints;

import static org.ode4j.ode.internal.joints.JointEnums.GI2_JAX;
import static org.ode4j.ode.internal.joints.JointEnums.GI2_JAY;
import static org.ode4j.ode.internal.joints.JointEnums.GI2_JAZ;
import static org.ode4j.ode.internal.joints.JointEnums.GI2_JLX;
import static org.ode4j.ode.internal.joints.JointEnums.GI2_JLY;
import static org.ode4j.ode.internal.joints.JointEnums.GI2_JLZ;

import java.util.Arrays;

import org.ode4j.math.DVector3C;

public class Info2DescrQuickStep implements Info2Descr {

	// findex vector for variables. see the LCP solver interface for a
	// description of what this does. this is set to -1 on entry.
	// note that the returned indexes are relative to the first index of
	// the constraint.
	// public int[] _findex = new int[3]; //TZ: [3] ?
	private int[] _findexA;
	private double[] _J;
	private int j1;
	private int j2;
	private int pairRhsCfm;
	private int pairLoHi;
	private int findex;

	private int rowskip;
	private int pairskip;

	@Override
	public void setFindex(int i, int val) {
		_findexA[findex + i] = val;
	}

	public void setAllP(int j1, int j2, int pairRhsCfm, int pairLoHi, int findex) {
		this.j1 = j1;
		this.j2 = j2;
		this.pairRhsCfm = pairRhsCfm;
		this.pairLoHi = pairLoHi;
		this.findex = findex;
	}

	public void setArrays(double[] j, int[] findex) {
		this._J = j;
		_findexA = findex;
	}

	@Override
	public String toString() {
		System.out.println("_J   : " + Arrays.toString(_J));
		System.out.println("_findexA: " + Arrays.toString(_findexA));
		return super.toString();
	}

	@Override
	public void setC(int i, double d) {
		_J[pairRhsCfm + i * pairskip + JointEnums.GI2_RHS] = d;
	}

	@Override
	public double getC(int i) {
		return _J[pairRhsCfm + i * pairskip + JointEnums.GI2_RHS];
	}

	@Override
	public void setCfm(int i, double d) {
		_J[pairRhsCfm + i * pairskip + JointEnums.GI2_CFM] = d;
	}

	@Override
	public void setLo(int i, double d) {
		_J[pairLoHi + i * pairskip + JointEnums.GI2_LO] = d;
	}

	@Override
	public void setHi(int i, double d) {
		_J[pairLoHi + i * pairskip + JointEnums.GI2_HI] = d;
	}

	public void setRowskip(int combinedRowskip, int pairskip) {
		this.rowskip = combinedRowskip;
		this.pairskip = pairskip;
	}

	@Override
	public void setJ1l(int row, DVector3C v) {
		setJ1l(row, v.get0(), v.get1(), v.get2());
	}

	@Override
	public void setJ1a(int row, DVector3C v) {
		setJ1a(row, v.get0(), v.get1(), v.get2());
	}

	@Override
	public void setJ2l(int row, DVector3C v) {
		setJ2l(row, v.get0(), v.get1(), v.get2());
	}

	@Override
	public void setJ2a(int row, DVector3C v) {
		setJ2a(row, v.get0(), v.get1(), v.get2());
	}

	@Override
	public void setJ2lNegated(int row, DVector3C v) {
		setJ2l(row, -v.get0(), -v.get1(), -v.get2());
	}

	@Override
	public void setJ2aNegated(int row, DVector3C v) {
		setJ2a(row, -v.get0(), -v.get1(), -v.get2());
	}

	@Override
	public void setJ1aCrossMatrix(int row, DVector3C a, double sign) {
		setJ1(row, GI2_JAY, -sign * a.get2());
		setJ1(row, GI2_JAZ, sign * a.get1());
		setJ1(row + 1, GI2_JAX, sign * a.get2());
		setJ1(row + 1, GI2_JAZ, -sign * a.get0());
		setJ1(row + 2, GI2_JAX, -sign * a.get1());
		setJ1(row + 2, GI2_JAY, sign * a.get0());
	}

	@Override
	public void setJ2aCrossMatrix(int row, DVector3C a, double sign) {
		setJ2(row, GI2_JAY, -sign * a.get2());
		setJ2(row, GI2_JAZ, sign * a.get1());
		setJ2(row + 1, GI2_JAX, sign * a.get2());
		setJ2(row + 1, GI2_JAZ, -sign * a.get0());
		setJ2(row + 2, GI2_JAX, -sign * a.get1());
		setJ2(row + 2, GI2_JAY, sign * a.get0());
	}

	@Override
	public void setJ1l(int row, double x, double y, double z) {
		setJ1(row, GI2_JLX, x);
		setJ1(row, GI2_JLY, y);
		setJ1(row, GI2_JLZ, z);
	}

	@Override
	public void setJ1a(int row, double x, double y, double z) {
		setJ1(row, GI2_JAX, x);
		setJ1(row, GI2_JAY, y);
		setJ1(row, GI2_JAZ, z);
	}

	@Override
	public void setJ2l(int row, double x, double y, double z) {
		setJ2(row, GI2_JLX, x);
		setJ2(row, GI2_JLY, y);
		setJ2(row, GI2_JLZ, z);
	}

	@Override
	public void setJ2a(int row, double x, double y, double z) {
		setJ2(row, GI2_JAX, x);
		setJ2(row, GI2_JAY, y);
		setJ2(row, GI2_JAZ, z);
	}

	@Override
	public void setJ1l(int row, int i, double d) {
		setJ1(row, i + GI2_JLX, d);
	}

	@Override
	public void setJ2l(int row, int i, double d) {
		setJ2(row, i + GI2_JLX, d);
	}

	@Override
	public void setJ1a(int row, int i, double d) {
		setJ1(row, i + GI2_JAX, d);
	}

	@Override
	public void setJ2a(int row, int i, double d) {
		setJ2(row, i + GI2_JAX, d);
	}

	private void setJ1(int row, int i, double d) {
		_J[j1 + row * rowskip + i] = d;
	}
	
	private void setJ2(int row, int i, double d) {
		_J[j2 + row * rowskip + i] = d;
	}

}