package org.ode4j.ode.internal.joints;

import java.util.Arrays;

import org.ode4j.math.DVector3C;

public class Info2DescrStep implements Info2Descr {
	// for the first and second body, pointers to two (linear and angular)
	// n*3 jacobian sub matrices, stored by rows. these matrices will have
	// been initialized to 0 on entry. if the second body is zero then the
	// J2xx pointers may be 0.
	// public double[] J1l;
	//
	// public double[] J1a;
	//
	// public double[] J2l, J2a;
	public int J1lp;
	public int J1ap;
	public int J2lp, J2ap;

	// elements to jump from one row to the next in J's
	private int _rowskip;

	public int rowskip() {
		return _rowskip;
	}

	public void setRowskip(int rs) {
		_rowskip = rs;
	}

	private double[] _J;

	public void setJ(double[] J) {
		_J = J;
	}

	// right hand sides of the equation J*v = c + cfm * lambda. cfm is the
	// "constraint force mixing" vector. c is set to zero on entry, cfm is
	// set to a constant value (typically very small or zero) value on entry.
	// public dVector3 c = new dVector3();//double[] c; //TZ: [4] ? 0,1,2,3
	// public dVector3 cfm = new dVector3(); //TZ: [3] ?
	private double[] _cA;
	int _cP;
	private double[] _cfmA;
	int _cfmP;

	@Override
	public void setC(int i, double d) {
		_cA[_cP + i] = d;
	}

	public void scaleC(int i, double d) {
		_cA[_cP + i] *= d;
	}

	public void setC(int i, DVector3C v) {
		_cA[_cP + i] = v.get0();
	}

	@Override
	public double getC(int i) {
		return _cA[_cP + i];
	}

	@Override
	public void setCfm(int i, double d) {
		_cfmA[_cfmP + i] = d;
	}

	// lo and hi limits for variables (set to -/+ infinity on entry).
	// public dVector3 lo = new dVector3();//double[] lo; //TZ: [3] ?
	// public dVector3 hi = new dVector3();//double[] hi; //TZ: [3] ?
	private double[] _loA;
	int _loP;
	private double[] _hiA;
	int _hiP;

	@Override
	public void setLo(int i, double d) {
		_loA[_loP + i] = d;
	}

	@Override
	public void setHi(int i, double d) {
		_hiA[_hiP + i] = d;
	}

	// findex vector for variables. see the LCP solver interface for a
	// description of what this does. this is set to -1 on entry.
	// note that the returned indexes are relative to the first index of
	// the constraint.
	// public int[] _findex = new int[3]; //TZ: [3] ?
	private int _findexP;
	private int[] _findexA;

	@Override
	public void setFindex(int i, int val) {
		_findexA[_findexP + i] = val;
	}

	public int getFindex(int i) {
		return _findexA[_findexP + i];
	}

	/**
	 * Set POS for c, cfm, lo, hi, findex.
	 * @param i POS
	 */
	public void setAllP(int i) {
		_cP = i;
		_cfmP = i;
		_loP = i;
		_hiP = i;
		_findexP = i;
	}

	public void setArrays(double[] J, double[] c, double[] cfm, double[] lo, double[] hi, int[] findex) {
		_J = J;
		_cA = c;
		_cfmA = cfm;
		_loA = lo;
		_hiA = hi;
		_findexA = findex;
	}

	@Override
	public String toString() {
		System.out.println("rowskip : " + _rowskip);
		System.out.println("Jxxp    : " + J1lp + "/" + J1ap + "  /  " + J2lp + "/" + J2ap);
		for (int i = 0; i < _J.length; i += 8) {
			System.out.println("_J      : " + _J[i + 0] + " " + _J[i + 1] + " " + _J[i + 2] + " " + _J[i + 3] + " / "
					+ _J[i + 4] + " " + _J[i + 5] + " " + _J[i + 6] + " " + _J[i + 7]);
		}
		System.out.println("_cA     : " + Arrays.toString(_cA));
		System.out.println("_cfmA   : " + Arrays.toString(_cfmA));
		System.out.println("_loA    : " + Arrays.toString(_loA));
		System.out.println("_hiA    : " + Arrays.toString(_hiA));
		System.out.println("_findexA: " + Arrays.toString(_findexA));
		return super.toString();
	}

	@Override
	public void setJ1l(int row, int i, double d) {
		_J[J1lp + row * _rowskip + i] = d;
	}

	@Override
	public void setJ1a(int row, int i, double d) {
		_J[J1ap + row * _rowskip + i] = d;
	}

	@Override
	public void setJ2l(int row, int i, double d) {
		_J[J2lp + row * _rowskip + i] = d;
	}

	@Override
	public void setJ2a(int row, int i, double d) {
		_J[J2ap + row * _rowskip + i] = d;
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
	public void setJ1l(int row, double x, double y, double z) {
		setJ1l(row, 0, x);
		setJ1l(row, 1, y);
		setJ1l(row, 2, z);
	}

	@Override
	public void setJ1a(int row, double x, double y, double z) {
		setJ1a(row, 0, x);
		setJ1a(row, 1, y);
		setJ1a(row, 2, z);
	}

	@Override
	public void setJ2l(int row, double x, double y, double z) {
		setJ2l(row, 0, x);
		setJ2l(row, 1, y);
		setJ2l(row, 2, z);
	}

	@Override
	public void setJ2a(int row, double x, double y, double z) {
		setJ2a(row, 0, x);
		setJ2a(row, 1, y);
		setJ2a(row, 2, z);
	}

	@Override
	public void setJ1aCrossMatrix(int row, DVector3C a, double sign) {
		setJ1a(row, 1, -sign * a.get2());
		setJ1a(row, 2, sign * a.get1());
		setJ1a(row + 1, 0, sign * a.get2());
		setJ1a(row + 1, 2, -sign * a.get0());
		setJ1a(row + 2, 0, -sign * a.get1());
		setJ1a(row + 2, 1, sign * a.get0());
	}

	@Override
	public void setJ2aCrossMatrix(int row, DVector3C a, double sign) {
		setJ2a(row, 1, -sign * a.get2());
		setJ2a(row, 2, sign * a.get1());
		setJ2a(row + 1, 0, sign * a.get2());
		setJ2a(row + 1, 2, -sign * a.get0());
		setJ2a(row + 2, 0, -sign * a.get1());
		setJ2a(row + 2, 1, sign * a.get0());
	}

}