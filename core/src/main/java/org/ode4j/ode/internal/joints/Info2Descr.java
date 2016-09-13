package org.ode4j.ode.internal.joints;

import org.ode4j.math.DVector3C;

/**
 * info returned by getInfo2 function
 */
public interface Info2Descr {
	void setCfm(int i, double d);
	void setLo(int i, double d);
	void setHi(int i, double d);
	void setFindex(int i, int val);
	void setC(int i, double d);
	double getC(int i);
	
	void setJ1l(int row, int i, double d);
	void setJ2l(int row, int i, double d);
	void setJ1a(int row, int i, double d);
	void setJ2a(int row, int i, double d);
	void setJ1l(int row, double x, double y, double z);
	void setJ1a(int row, double x, double y, double z);
	void setJ2l(int row, double x, double y, double z);
	void setJ2a(int row, double x, double y, double z);
	void setJ1l(int row, DVector3C v);
	void setJ1a(int row, DVector3C v);
	void setJ2l(int row, DVector3C v);
	void setJ2a(int row, DVector3C v);
	void setJ2lNegated(int row, DVector3C v);
	void setJ2aNegated(int row, DVector3C v);
    void setJ1aCrossMatrix(int row, DVector3C a, double sign);
    void setJ2aCrossMatrix(int row, DVector3C a, double sign);

}
