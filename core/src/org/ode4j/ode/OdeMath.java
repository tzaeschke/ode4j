/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode;

import java.util.Arrays;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector3View;
import org.ode4j.math.DVector4;
import org.ode4j.math.DMatrix3.DVector3RowTView;
import org.ode4j.ode.internal.Rotation;

public class OdeMath extends Rotation {

	private OdeMath() {
		//private
	}

//	/**
//	 * macro to access elements i,j in an NxM matrix A, independent of the
//	 * matrix storage convention.
//	 */
//	//#define dACCESS33(A,i,j) ((A)[(i)*4+(j)])
//	private double dACCESS33(double[] A, int i, int j) { 
//		return A[ i*4 + j ]; 
//	} 
//
//
//	/**
//	 * Macro to test for valid floating point values
//	 */
//	//#define dVALIDVEC3(v) (!(dIsNan(v[0]) || dIsNan(v[1]) || dIsNan(v[2])))
//	//#define dVALIDVEC4(v) (!(dIsNan(v[0]) || dIsNan(v[1]) || dIsNan(v[2]) || dIsNan(v[3])))
//	//#define dVALIDMAT3(m) (!(dIsNan(m[0]) || dIsNan(m[1]) || dIsNan(m[2]) || dIsNan(m[3]) || dIsNan(m[4]) || dIsNan(m[5]) || dIsNan(m[6]) || dIsNan(m[7]) || dIsNan(m[8]) || dIsNan(m[9]) || dIsNan(m[10]) || dIsNan(m[11])))
//	//#define dVALIDMAT4(m) (!(dIsNan(m[0]) || dIsNan(m[1]) || dIsNan(m[2]) || dIsNan(m[3]) || dIsNan(m[4]) || dIsNan(m[5]) || dIsNan(m[6]) || dIsNan(m[7]) || dIsNan(m[8]) || dIsNan(m[9]) || dIsNan(m[10]) || dIsNan(m[11]) || dIsNan(m[12]) || dIsNan(m[13]) || dIsNan(m[14]) || dIsNan(m[15]) ))
//	private boolean dVALIDVEC3(DVector3 v3) { 
//		double[] v = v3.v;
//		return (!(dIsNan(v[0]) || dIsNan(v[1]) || dIsNan(v[2])));
//	}
//	private boolean dVALIDVEC4(DVector4 v4) {
//		double[] v = v4.v;
//		return (!(dIsNan(v[0]) || dIsNan(v[1]) || dIsNan(v[2]) || dIsNan(v[3])));
//	}
//	private boolean dVALIDMAT3(DMatrix3 m3) { 
//		double[] m = m3.v;
//		return (!(dIsNan(m[0]) || dIsNan(m[1]) || dIsNan(m[2]) || dIsNan(m[3]) 
//				|| dIsNan(m[4]) || dIsNan(m[5]) || dIsNan(m[6]) || dIsNan(m[7]) 
//				|| dIsNan(m[8]) || dIsNan(m[9]) || dIsNan(m[10]) || dIsNan(m[11]))); 
//	}
//	private boolean dVALIDMAT4(dMatrix4 m4) {
//		double[] m = m4.v;
//		return (!(dIsNan(m[0]) || dIsNan(m[1]) || dIsNan(m[2]) || dIsNan(m[3]) 
//				|| dIsNan(m[4]) || dIsNan(m[5]) || dIsNan(m[6]) || dIsNan(m[7]) 
//				|| dIsNan(m[8]) || dIsNan(m[9]) || dIsNan(m[10]) || dIsNan(m[11]) 
//				|| dIsNan(m[12]) || dIsNan(m[13]) || dIsNan(m[14]) || dIsNan(m[15]) ));
//	}


	public enum OP { ADD, SUB, MUL, DIV, EQ, /** += */ ADD_EQ, /** =- */ EQ_SUB, 
		/** *= */ MUL_EQ, /** -= */ SUB_EQ; }
	
	/*
	 * General purpose vector operations with other vectors or constants.
	 */

	//#define dOP(a,op,b,c) \
	//    (a)[0] = ((b)[0]) op ((c)[0]); \
	//    (a)[1] = ((b)[1]) op ((c)[1]); \
	//    (a)[2] = ((b)[2]) op ((c)[2]);
	//#define dOPC(a,op,b,c) \
	//    (a)[0] = ((b)[0]) op (c); \
	//    (a)[1] = ((b)[1]) op (c); \
	//    (a)[2] = ((b)[2]) op (c);
	//#define dOPE(a,op,b) \
	//    (a)[0] op ((b)[0]); \
	//    (a)[1] op ((b)[1]); \
	//    (a)[2] op ((b)[2]);
	//#define dOPEC(a,op,c) \
	//    (a)[0] op (c); \
	//    (a)[1] op (c); \
	//    (a)[2] op (c);
	public static void dOP(double[] a, OP op, double[] b, double[] c) {
		switch (op) {
		case ADD: dOPadd(a, b, c); return;
		case SUB: dOPsub(a, b, c); return;
		case MUL: dOPmul(a, b, c); return;
		case DIV: dOPdiv(a, b, c); return;
		}
		throw new UnsupportedOperationException(op.name());
	}

	public static void dOPE(double[] a, int ofs, OP op, double[] b) {
		if (op == OP.EQ) {
			a[0+ofs] = b[0]; 
			a[1+ofs] = b[1]; 
			a[2+ofs] = b[2];
		} else if (op == OP.ADD_EQ) {
			a[0+ofs] += b[0]; 
			a[1+ofs] += b[1]; 
			a[2+ofs] += b[2];
		} else if (op == OP.EQ_SUB) {
			a[0+ofs] = -b[0]; 
			a[1+ofs] = -b[1]; 
			a[2+ofs] = -b[2];
		} else {
			throw new UnsupportedOperationException(op.name());
		}
	}

	public static void dOPE(double[] a, int ofs, OP op, DVector3C b) {
		if (op == OP.EQ) {
			a[0+ofs] = b.get0(); 
			a[1+ofs] = b.get1(); 
			a[2+ofs] = b.get2();
		} else if (op == OP.ADD_EQ) {
			a[0+ofs] += b.get0(); 
			a[1+ofs] += b.get1(); 
			a[2+ofs] += b.get2();
		} else if (op == OP.EQ_SUB) {
			a[0+ofs] = -b.get0(); 
			a[1+ofs] = -b.get1(); 
			a[2+ofs] = -b.get2();
		} else {
			throw new UnsupportedOperationException(op.name());
		}
	}

	private static void dOPadd(double[] a, double[] b, double[] c) {
		a[0] = ((b)[0]) + ((c)[0]); 
		a[1] = ((b)[1]) + ((c)[1]); 
		a[2] = ((b)[2]) + ((c)[2]);
	}
	private static void dOPsub(double[] a, double[] b, double[] c) {
		a[0] = ((b)[0]) - ((c)[0]); 
		a[1] = ((b)[1]) - ((c)[1]); 
		a[2] = ((b)[2]) - ((c)[2]);
	}
	private static void dOPmul(double[] a, double[] b, double[] c) {
		a[0] = ((b)[0]) * ((c)[0]); 
		a[1] = ((b)[1]) * ((c)[1]); 
		a[2] = ((b)[2]) * ((c)[2]);
	}
	private static void dOPdiv(double[] a, double[] b, double[] c) {
		a[0] = ((b)[0]) / ((c)[0]); 
		a[1] = ((b)[1]) / ((c)[1]); 
		a[2] = ((b)[2]) / ((c)[2]);
	}

	/** @deprecated Use dVector3.scale(c) instead (TZ). */
	public static void dOPEC(double[] a, OP op, double c) {
		switch (op) {
		case MUL_EQ: for (int i = 0; i < a.length; i++) a[i] *= c; return;
		}
		throw new UnsupportedOperationException(op.name());
	}

	//#define dOPC(a,op,b,c) \
	//(a)[0] = ((b)[0]) op (c); \
	//(a)[1] = ((b)[1]) op (c); \
	//(a)[2] = ((b)[2]) op (c);
	//#define dOPE(a,op,b) \
	//(a)[0] op ((b)[0]); \
	//(a)[1] op ((b)[1]); \
	//(a)[2] op ((b)[2]);
	//#define dOPEC(a,op,c) \
	//(a)[0] op (c); \
	//(a)[1] op (c); \
	//(a)[2] op (c);

	/// Define an equation with operatos
	/// For example this function can be used to replace
	/// <PRE>
	/// for (int i=0; i<3; ++i)
	///   a[i] += b[i] + c[i];
	/// </PRE>

	//TODO
	//#define dOPE2(a,op1,b,op2,c) \
	//    (a)[0] op1 ((b)[0]) op2 ((c)[0]); \
	//    (a)[1] op1 ((b)[1]) op2 ((c)[1]); \
	//    (a)[2] op1 ((b)[2]) op2 ((c)[2]);

//	public static void dOPE2(double[] a, OP op1, double[] b, OP op2, double[] c) {
//		if (op1 == OP.SUB_EQ && op2 == OP.ADD) {
//		    a[0] -= b[0] + c[0]; 
//		    a[1] -= b[1] + c[1]; 
//		    a[2] -= b[2] + c[2];
//		} else {
//			throw new UnsupportedOperationException(op1 + " / " + op2);
//		}
//	}

	/*
	 * Length, and squared length helpers. dLENGTH returns the length of a dVector3.
	 * dLENGTHSQUARED return the squared length of a dVector3.
	 */

	//#define dLENGTHSQUARED(a) (((a)[0])*((a)[0]) + ((a)[1])*((a)[1]) + ((a)[2])*((a)[2]))
	//public 
//	private double dLENGTHSQUARED(DVector3 a) {
//		return ((a.v[0])*(a.v[0]) + (a.v[1])*(a.v[1]) + (a.v[2])*(a.v[2]));
//	}

	//#ifdef __cplusplus

	//PURE_INLINE dReal dLENGTH (const dReal *a) { return dSqrt(dLENGTHSQUARED(a)); }
	//public double dReal dLENGTH (const dReal *a) { return dSqrt(dLENGTHSQUARED(a)); }

	//#else

	//#define dLENGTH(a) ( dSqrt( ((a)[0])*((a)[0]) + ((a)[1])*((a)[1]) + ((a)[2])*((a)[2]) ) )
//	public static double dLENGTH(DVector3 a) {
//		return dSqrt( (a.v[0])*(a.v[0]) + (a.v[1])*(a.v[1]) + 
//				(a.v[2])*(a.v[2]) ) ;
//	}

	//#endif /* __cplusplus */





	/*
	 * 3-way dot product. dDOTpq means that elements of `a' and `b' are spaced
	 * p and q indexes apart respectively. dDOT() means dDOT11.
	 * in C++ we could use function templates to get all the versions of these
	 * functions - but on some compilers this will result in sub-optimal code.
	 */

	//#define dDOTpq(a,b,p,q) ((a)[0]*(b)[0] + (a)[p]*(b)[q] + (a)[2*(p)]*(b)[2*(q)])
	//by TZ: multiply matrices with offset
	private static double dDOTpq(double[] a, double[] b, int p, int q) {
		return a[0]*b[0] + a[p]*b[q] + a[2*p]*b[2*q];
	}
	private static double dDOTpq(double[] a, int o1, double[] b, int o2, int p, int q) {
		return a[0+o1]*b[0+o2] + a[p+o1]*b[q+o2] + a[2*p+o1]*b[2*q+o2];
	}
	private static double dDOTpq(double[] a, int o1, int p, DVector3C b) {
		return a[0+o1]*b.get0() + a[p+o1]*b.get1() + a[2*p+o1]*b.get2();
	}
	private static double dDOTpq(double[] a, int o1, DVector3C b) {
		return a[o1]*b.get0() + a[1+o1]*b.get1() + a[2+o1]*b.get2();
	}

	//#ifdef __cplusplus
	//
	//PURE_INLINE dReal dDOT   (const dReal *a, const dReal *b) { return dDOTpq(a,b,1,1); }
	//PURE_INLINE dReal dDOT13 (const dReal *a, const dReal *b) { return dDOTpq(a,b,1,3); }
	//PURE_INLINE dReal dDOT31 (const dReal *a, const dReal *b) { return dDOTpq(a,b,3,1); }
	//PURE_INLINE dReal dDOT33 (const dReal *a, const dReal *b) { return dDOTpq(a,b,3,3); }
	//PURE_INLINE dReal dDOT14 (const dReal *a, const dReal *b) { return dDOTpq(a,b,1,4); }
	//PURE_INLINE dReal dDOT41 (const dReal *a, const dReal *b) { return dDOTpq(a,b,4,1); }
	//PURE_INLINE dReal dDOT44 (const dReal *a, const dReal *b) { return dDOTpq(a,b,4,4); }
	//
	//#else

	//#define dDOT(a,b)   dDOTpq(a,b,1,1)
	//#define dDOT13(a,b) dDOTpq(a,b,1,3)
	//#define dDOT31(a,b) dDOTpq(a,b,3,1)
	//#define dDOT33(a,b) dDOTpq(a,b,3,3)
	//#define dDOT14(a,b) dDOTpq(a,b,1,4)
	//#define dDOT41(a,b) dDOTpq(a,b,4,1)
	//#define dDOT44(a,b) dDOTpq(a,b,4,4)

	public static double dDOT(DVector3 a, DVector3 b) {
		return a.get0()*b.get0() + a.get1()*b.get1() + a.get2()*b.get2();
	}
	public static double dDOT(double[] a, int o1, DVector3C b) { return dDOTpq(a,o1,b); }
	public static double dDOT(double[] a, double[] b) { return dDOTpq(a,b,1,1); }
	public static double dDOT(double[] a, int o1, double[] b, int o2) { return dDOTpq(a,o1,b,o2,1,1); }
	public static double dDOT13(double[] a, int o1, double[] b, int o2) { return dDOTpq(a,o1,b,o2,1,3); }
	//Used? public double dDOT14(dVector3 a, dVector3 b) { return dDOTpq(a,b,1,4); }
	public static double dDOT14(DVector3 a, DMatrix3 b, int o2) { return dDOTpq(a.v,0,b.v,o2,1,4); }
	public static double dDOT14(DVector3C a, DMatrix3C b, int o2) { 
		return dDOTpq(((DVector3)a).v,0,((DMatrix3)b).v,o2,1,4); }
	public static double dDOT41(DMatrix3 a, int o1, DVector3 b) { return dDOTpq(a.v,o1,b.v,0,4,1); }
	public static double dDOT41(DMatrix3C a, int o1, DVector3C b) { 
		return dDOTpq(((DMatrix3)a).v,o1,((DVector3)b).v,0,4,1); }
	public static double dDOT44(DMatrix3 a, int o1, DMatrix3 b, int o2) { return dDOTpq(a.v,o1,b.v,o2,4,4); }
	public static double dDOT44(DMatrix3C a, int o1, DMatrix3C b, int o2) { 
		return dDOTpq(((DMatrix3)a).v,o1,((DMatrix3)b).v,o2,4,4); }
	//private double dDOT(dMatrix3 a, int o1, dVector3 b, int o2) { return dDOTpq(a.v, o1, b.v, o2, 1,1); }
	public static double dDOT14(double[] a, int o1, double[] b, int o2) { return dDOTpq(a,o1,b,o2,1,4); }
	//private static double dDOT41(double[] a, int o1, double[] b, int o2) { return dDOTpq(a,o1,b,o2,4,1); }
	private static double dDOT41(double[] a, int o1, DVector3C b, int o2) { return dDOTpq(a,o1,4, b); }
	private static double dDOT44(double[] a, int o1, double[] b, int o2) { return dDOTpq(a,o1,b,o2,4,4); }

	//#endif /* __cplusplus */


	/*
	 * cross product, set a = b x c. dCROSSpqr means that elements of `a', `b'
	 * and `c' are spaced p, q and r indexes apart respectively.
	 * dCROSS() means dCROSS111. `op' is normally `=', but you can set it to
	 * +=, -= etc to get other effects.
	 */

	//#define dCROSS(a,op,b,c) \
	//do { \
	//  (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
	//  (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
	//  (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]); \
	//} while(0)
	//#define dCROSSpqr(a,op,b,c,p,q,r) \
	//do { \
	//  (a)[  0] op ((b)[  q]*(c)[2*r] - (b)[2*q]*(c)[  r]); \
	//  (a)[  p] op ((b)[2*q]*(c)[  0] - (b)[  0]*(c)[2*r]); \
	//  (a)[2*p] op ((b)[  0]*(c)[  r] - (b)[  q]*(c)[  0]); \
	//} while(0)
	public static void dCROSS(DVector3 a, OP op, DVector3C b, DVector3C c) {
		if (op == OP.EQ) {
			a.set0( b.get1()*c.get2() - b.get2()*c.get1() ); 
			a.set1( b.get2()*c.get0() - b.get0()*c.get2() ); 
			a.set2( b.get0()*c.get1() - b.get1()*c.get0() );
		} else if (op == OP.ADD_EQ) {
			a.add0( b.get1()*c.get2() - b.get2()*c.get1() ); 
			a.add1( b.get2()*c.get0() - b.get0()*c.get2() ); 
			a.add2( b.get0()*c.get1() - b.get1()*c.get0() );
		} else if (op == OP.SUB_EQ) {
			a.add0( -b.get1()*c.get2() + b.get2()*c.get1() ); 
			a.add1( -b.get2()*c.get0() + b.get0()*c.get2() ); 
			a.add2( -b.get0()*c.get1() + b.get1()*c.get0() );
		} else {
			throw new UnsupportedOperationException(op.name());
		}
	}

	public static void dCROSS(float[] a, OP op, float[] b, float[] c) {
		if (op == OP.EQ) {
			a[0] = ((b)[1]*(c)[2] - (b)[2]*(c)[1]); 
			a[1] = ((b)[2]*(c)[0] - (b)[0]*(c)[2]); 
			a[2] = ((b)[0]*(c)[1] - (b)[1]*(c)[0]);
		} else if (op == OP.ADD_EQ) {
			a[0] += ((b)[1]*(c)[2] - (b)[2]*(c)[1]); 
			a[1] += ((b)[2]*(c)[0] - (b)[0]*(c)[2]); 
			a[2] += ((b)[0]*(c)[1] - (b)[1]*(c)[0]);
		} else if (op == OP.SUB_EQ) {
			a[0] -= ((b)[1]*(c)[2] - (b)[2]*(c)[1]); 
			a[1] -= ((b)[2]*(c)[0] - (b)[0]*(c)[2]); 
			a[2] -= ((b)[0]*(c)[1] - (b)[1]*(c)[0]);
		} else {
			throw new UnsupportedOperationException(op.name());
		}
	}

	//TODO check, this method looks very strange...
//	public void dCROSS(dMatrix3 a, int ofs, OP op, dVector3 b, dVector3 c) {
//		if (op != OP.EQ && op != OP.EQ_SUB) {
//			throw new UnsupportedOperationException(op.name());
//		}
//		a.v[0+ofs] = ((b).v[1]*(c).v[2] - (b).v[2]*(c).v[1]); 
//		a.v[1+ofs] = ((b).v[2]*(c).v[0] - (b).v[0]*(c).v[2]); 
//		a.v[2+ofs] = ((b).v[0]*(c).v[1] - (b).v[1]*(c).v[0]); 
//		if (op == OP.EQ_SUB) {
//			a.v[0+ofs] = -a.v[0+ofs];
//			a.v[1+ofs] = -a.v[1+ofs];
//			a.v[2+ofs] = -a.v[2+ofs];
//		}
//	}
	public static void dCROSS(double[] a, int ofs, OP op, DVector3 b, DVector3 c) {
		if (op == OP.EQ) {
			a[0+ofs] = (b.get1()*c.get2() - b.get2()*c.get1()); 
			a[1+ofs] = (b.get2()*c.get0() - b.get0()*c.get2()); 
			a[2+ofs] = (b.get0()*c.get1() - b.get1()*c.get0());
		} else if (op == OP.EQ_SUB) {
			a[0+ofs] = -(b.get1()*c.get2() - b.get2()*c.get1()); 
			a[1+ofs] = -(b.get2()*c.get0() - b.get0()*c.get2()); 
			a[2+ofs] = -(b.get0()*c.get1() - b.get1()*c.get0());
//			a[0+ofs] = -a[0+ofs];
//			a[1+ofs] = -a[1+ofs];
//			a[2+ofs] = -a[2+ofs];
		} else {
			throw new UnsupportedOperationException(op.name());
		}
	}

	public static void dCROSS(DVector3View a, OP op, DVector3View b, DVector3View c) {
		if (op == OP.EQ) {
			a.set0( b.get1()*c.get2() - b.get2()*c.get1() ); 
			a.set1( b.get2()*c.get0() - b.get0()*c.get2() ); 
			a.set2( b.get0()*c.get1() - b.get1()*c.get0() );
		} else {
			throw new UnsupportedOperationException(op.name());
		}
	}

	public static void dCROSS(DVector3 a, OP op, DVector3 b, DMatrix3 c) {
		if (op == OP.EQ) {
			a.set0( b.get1()*c.get02() - b.get2()*c.get01()); 
			a.set1( b.get2()*c.get00() - b.get0()*c.get02()); 
			a.set2( b.get0()*c.get01() - b.get1()*c.get00());
		} else if (op == OP.SUB_EQ) {
			a.add0( - (b.get1()*c.get02() - b.get2()*c.get01()) ); 
			a.add1( - (b.get2()*c.get00() - b.get0()*c.get02()) ); 
			a.add2( - (b.get0()*c.get01() - b.get1()*c.get00()) );
		} else {
			throw new UnsupportedOperationException(op.name());
		}
	}


//	private void dCROSSpqr(DVector3 a, OP op, DVector3 b, DVector3 c, 
//			int p, int q, int r) { 
//		(a).v[  0] = ((b).v[  q]*(c).v[2*r] - (b).v[2*q]*(c).v[  r]); 
//		(a).v[  p] = ((b).v[2*q]*(c).v[  0] - (b).v[  0]*(c).v[2*r]); 
//		(a).v[2*p] = ((b).v[  0]*(c).v[  r] - (b).v[  q]*(c).v[  0]); 
//	}
//
//	public void dCROSS114(DVector3 a, OP op, DVector3 b, DVector3 c) { 
//		dCROSSpqr(a,op,b,c,1,1,4);
//	}
//	public void dCROSS141(DVector3 a, OP op, DVector3 b, DVector3 c) { 
//		dCROSSpqr(a,op,b,c,1,4,1);
//	}
//	public void dCROSS144(DVector3 a, OP op, DVector3 b, DVector3 c) { 
//		dCROSSpqr(a,op,b,c,1,4,4);
//	}
//	public void dCROSS411(DVector3 a, OP op, DVector3 b, DVector3 c) { 
//		dCROSSpqr(a,op,b,c,4,1,1);
//	}
//	public void dCROSS414(DVector3 a, OP op, DVector3 b, DVector3 c) { 
//		dCROSSpqr(a,op,b,c,4,1,4);
//	}
//	public void dCROSS441(DVector3 a, OP op, DVector3 b, DVector3 c) { 
//		dCROSSpqr(a,op,b,c,4,4,1);
//	}
//	public void dCROSS444(DVector3 a, OP op, DVector3 b, DVector3 c) { 
//		dCROSSpqr(a,op,b,c,4,4,4);
//	}


	/*
	 * set a 3x3 submatrix of A to a matrix such that submatrix(A)*b = a x b.
	 * A is stored by rows, and has `skip' elements per row. the matrix is
	 * assumed to be already zero, so this does not write zero elements!
	 * if (plus,minus) is (+,-) then a positive version will be written.
	 * if (plus,minus) is (-,+) then a negative version will be written.
	 */

	//#define dCROSSMAT(A,a,skip,plus,minus) \
	//do { \
	//  (A)[1] = minus (a)[2]; \
	//  (A)[2] = plus (a)[1]; \
	//  (A)[(skip)+0] = plus (a)[2]; \
	//  (A)[(skip)+2] = minus (a)[0]; \
	//  (A)[2*(skip)+0] = minus (a)[1]; \
	//  (A)[2*(skip)+1] = plus (a)[0]; \
	//} while(0)
	public static void dCROSSMAT(DMatrix3 A, DVector3 a, int skip, int plus, int minus) {
		A.set01( minus * a.get2() ); 
		A.set02( plus  * a.get1() ); 
		A.set10( plus  * a.get2() ); 
		A.set12( minus * a.get0() ); 
		A.set20( minus * a.get1() ); 
		A.set21( plus  * a.get0() ); 
	}
//	public static void dCROSSMAT(DMatrix3 A, DVector4 a, int skip, int plus, int minus) {
//		(A).v[1] = minus * (a).v[2]; 
//		(A).v[2] = plus * (a).v[1]; 
//		(A).v[(skip)+0] = plus * (a).v[2]; 
//		(A).v[(skip)+2] = minus * (a).v[0]; 
//		(A).v[2*(skip)+0] = minus * (a).v[1]; 
//		(A).v[2*(skip)+1] = plus * (a).v[0]; 
//	}
	public static void dCROSSMAT(double[] A, int ofs, DVector3 a, int skip, int plus, int minus) {
		A[ofs+1] = minus * a.get2(); 
		A[ofs+2] = plus * a.get1(); 
		A[ofs+skip+0] = plus * a.get2(); 
		A[ofs+skip+2] = minus * a.get0(); 
		A[ofs+2*skip+0] = minus * a.get1(); 
		A[ofs+2*skip+1] = plus * a.get0(); 
	}


	/*
	 * compute the distance between two 3D-vectors
	 */

	//#ifdef __cplusplus
	//PURE_INLINE dReal dDISTANCE (const dVector3 a, const dVector3 b)
	//	{ return dSqrt( (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]) ); }
	public static double dDISTANCE (final DVector3C a, final DVector3C b)
	{ 
		return dSqrt( (a.get0()-b.get0())*(a.get0()-b.get0()) + 
				(a.get1()-b.get1())*(a.get1()-b.get1()) + 
				(a.get2()-b.get2())*(a.get2()-b.get2()) ); 
	}
	//#else
	//#define dDISTANCE(a,b) \
	//	(dSqrt( ((a)[0]-(b)[0])*((a)[0]-(b)[0]) + ((a)[1]-(b)[1])*((a)[1]-(b)[1]) + ((a)[2]-(b)[2])*((a)[2]-(b)[2]) ))
	//#endif


	/*
	 * special case matrix multipication, with operator selection
	 */

	//#define dMULTIPLYOP0_331(A,op,B,C) \
	//do { \
	//  (A)[0] op dDOT((B),(C)); \
	//  (A)[1] op dDOT((B+4),(C)); \
	//  (A)[2] op dDOT((B+8),(C)); \
	//} while(0)
	//#define dMULTIPLYOP1_331(A,op,B,C) \
	//do { \
	//  (A)[0] op dDOT41((B),(C)); \
	//  (A)[1] op dDOT41((B+1),(C)); \
	//  (A)[2] op dDOT41((B+2),(C)); \
	//} while(0)
	//#define dMULTIPLYOP0_133(A,op,B,C) \
	//do { \
	//  (A)[0] op dDOT14((B),(C)); \
	//  (A)[1] op dDOT14((B),(C+1)); \
	//  (A)[2] op dDOT14((B),(C+2)); \
	//} while(0)
	private static void dMULTIPLYOP0_331(DVector3 A, DMatrix3C B2, DVector3C C) {
		DMatrix3 B = (DMatrix3) B2;
		A.set0( dDOT(B.v, 0, C) ); 
		A.set1( dDOT(B.v, 4, C) );
		A.set2( dDOT(B.v, 8, C) );
	} 
	//TZ
	private static void dMULTIPLYOP0_331(DVector3 A, DMatrix3 B, DVector4 C) {
		A.set0( dDOT(B.v, 0, C.v, 0) ); 
		A.set1( dDOT(B.v, 4, C.v, 0) );
		A.set2( dDOT(B.v, 8, C.v, 0) );
	}
	//TZ
	private static void dMULTIPLYOP0_331(double[] A, int a, double[] B, int b,
			double[] C, int c) {
		A[0+a] = dDOT(B, 0+b, C, c); 
		A[1+a] = dDOT(B, 4+b, C, c);
		A[2+a] = dDOT(B, 8+b, C, c);
	}
	private static void dMULTIPLYOP1_331(DVector3 A, DMatrix3C B2, DVector3C C) {
		DMatrix3 B = (DMatrix3) B2;		
		A.set0( dDOT41(B.v, 0, C, 0) ); 
		A.set1( dDOT41(B.v, 1, C, 0) ); 
		A.set2( dDOT41(B.v, 2, C, 0) ); 
	} 
	private static void dMULTIPLYOP0_133(DVector3 A, DVector3 B, DMatrix3 C) {
		A.set0( dDOT14(B.v, 0,C.v,0) ); 
		A.set1( dDOT14(B.v, 0,C.v,1) ); 
		A.set2( dDOT14(B.v, 0,C.v,2) ); 
	} 

	//TODO private void ASSIGN()

	private static void dMULTIPLYOP0_333(DMatrix3 A, DMatrix3C B2, DMatrix3C C2) {
		DMatrix3 B = (DMatrix3) B2;
		DMatrix3 C = (DMatrix3) C2;
		(A).v[0] =  dDOT14(B.v,0,C.v,0); 
		(A).v[1] =  dDOT14(B.v,0,C.v,1); 
		(A).v[2] =  dDOT14(B.v,0,C.v,2); 
		(A).v[4] =  dDOT14(B.v,4,C.v,0); 
		(A).v[5] =  dDOT14(B.v,4,C.v,1); 
		(A).v[6] =  dDOT14(B.v,4,C.v,2); 
		(A).v[8] =  dDOT14(B.v,8,C.v,0); 
		(A).v[9] =  dDOT14(B.v,8,C.v,1); 
		(A).v[10] = dDOT14(B.v,8,C.v,2); 
	} 
	private static void dMULTIPLYOP1_333(DMatrix3 A, DMatrix3 B, DMatrix3 C) { 
		(A).v[0] = dDOT44(B.v,0,C.v,0); 
		(A).v[1] = dDOT44(B.v,0,C.v,1); 
		(A).v[2] = dDOT44(B.v,0,C.v,2); 
		(A).v[4] = dDOT44(B.v,1,C.v,0); 
		(A).v[5] = dDOT44(B.v,1,C.v,1); 
		(A).v[6] = dDOT44(B.v,1,C.v,2); 
		(A).v[8] = dDOT44(B.v,2,C.v,0); 
		(A).v[9] = dDOT44(B.v,2,C.v,1); 
		(A).v[10] = dDOT44(B.v,2,C.v,2); 
	} 
	private static void dMULTIPLYOP2_333(DMatrix3 A, DMatrix3 B2, DMatrix3C C2) {
		DMatrix3 B = (DMatrix3)B2;
		DMatrix3 C = (DMatrix3)C2;
//		A.v[0] = dDOT(B.v, 0, C.v, 0); 
//		A.v[1] = dDOT(B.v, 0, C.v, 4); 
//		A.v[2] = dDOT(B.v, 0, C.v, 8); 
//		A.v[4] = dDOT(B.v, 4, C.v, 0); 
//		A.v[5] = dDOT(B.v, 4, C.v, 4); 
//		A.v[6] = dDOT(B.v, 4, C.v, 8); 
//		A.v[8] = dDOT(B.v, 8, C.v, 0); 
//		A.v[9] = dDOT(B.v, 8, C.v, 4); 
//		A.v[10] = dDOT(B.v, 8, C.v, 8); 
		A.v[0] = B.v[0]*C.v[0] + B.v[1]*C.v[1] + B.v[2]*C.v[2];
		A.v[1] = B.v[0]*C.v[4] + B.v[1]*C.v[5] + B.v[2]*C.v[6];
		A.v[2] = B.v[0]*C.v[8] + B.v[1]*C.v[9] + B.v[2]*C.v[10];
		A.v[4] = B.v[4]*C.v[0] + B.v[5]*C.v[1] + B.v[6]*C.v[2];
		A.v[5] = B.v[4]*C.v[4] + B.v[5]*C.v[5] + B.v[6]*C.v[6];
		A.v[6] = B.v[4]*C.v[8] + B.v[5]*C.v[9] + B.v[6]*C.v[10];
		A.v[8] = B.v[8]*C.v[0] + B.v[9]*C.v[1] + B.v[10]*C.v[2];
		A.v[9] = B.v[8]*C.v[4] + B.v[9]*C.v[5] + B.v[10]*C.v[6];
		A.v[10] = B.v[8]*C.v[8] + B.v[9]*C.v[9] + B.v[10]*C.v[10];
	} 

	//#ifdef __cplusplus
	//
	//#define DECL template <class TA, class TB, class TC> PURE_INLINE void
	//
	///* 
	//Note: NEVER call any of these functions/macros with the same variable for A and C, 
	//it is not equivalent to A*=B.
	//*/
	//
	//DECL dMULTIPLY0_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_331(A,=,B,C); }
	//DECL dMULTIPLY1_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_331(A,=,B,C); }
	//DECL dMULTIPLY0_133(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_133(A,=,B,C); }
	//DECL dMULTIPLY0_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_333(A,=,B,C); }
	//DECL dMULTIPLY1_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_333(A,=,B,C); }
	//DECL dMULTIPLY2_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP2_333(A,=,B,C); }
	//
	//DECL dMULTIPLYADD0_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_331(A,+=,B,C); }
	//DECL dMULTIPLYADD1_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_331(A,+=,B,C); }
	//DECL dMULTIPLYADD0_133(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_133(A,+=,B,C); }
	//DECL dMULTIPLYADD0_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_333(A,+=,B,C); }
	//DECL dMULTIPLYADD1_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_333(A,+=,B,C); }
	//DECL dMULTIPLYADD2_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP2_333(A,+=,B,C); }
	//
	//#undef DECL
	//
	//#else

	//#define dMULTIPLY0_331(A,B,C) dMULTIPLYOP0_331(A,=,B,C)
	//#define dMULTIPLY1_331(A,B,C) dMULTIPLYOP1_331(A,=,B,C)
	//#define dMULTIPLY0_133(A,B,C) dMULTIPLYOP0_133(A,=,B,C)
	//#define dMULTIPLY0_333(A,B,C) dMULTIPLYOP0_333(A,=,B,C)
	//#define dMULTIPLY1_333(A,B,C) dMULTIPLYOP1_333(A,=,B,C)
	//#define dMULTIPLY2_333(A,B,C) dMULTIPLYOP2_333(A,=,B,C)
	public static void dMULTIPLY0_331(DVector3 A, DMatrix3C B, DVector3C C) { 
		dMULTIPLYOP0_331(A,B,C); }
	public static void dMULTIPLY0_331(DVector3 A, DMatrix3 B, DVector4 C) {
		dMULTIPLYOP0_331(A,B,C); } //TZ

	public static void dMULTIPLY0_331(double[] A, int a, double[] B, int b,
			double[] C, int c) {
		dMULTIPLYOP0_331(A, a, B, b, C, c); //TZ
	}

	public static void dMULTIPLY1_331(DVector3 A, DMatrix3C B, DVector3C C) { 
		dMULTIPLYOP1_331(A,B,C); }
	public static void dMULTIPLY0_133(DVector3 A, DVector3 B, DMatrix3 C) { 
		dMULTIPLYOP0_133(A,B,C); }
	public static void dMULTIPLY0_133(double[] A, int a, double[] B, int b, 
			double[] C, int c) {		
		A[0+a] = dDOT14(B, b, C, 0 + c); 
		A[1+a] = dDOT14(B, b, C, 1 + c); 
		A[2+a] = dDOT14(B, b, C, 2 + c); 
	}
	public static void dMULTIPLY0_333(DMatrix3 A, DMatrix3C B, DMatrix3C C) { 
		dMULTIPLYOP0_333(A,B,C); }
	public static void dMULTIPLY0_333(double[] A, int a, double[] B, int b,
			double[] C, int c) {
		A[0+a] =  dDOT14(B,0+b,C,0+c); 
		A[1+a] =  dDOT14(B,0+b,C,1+c); 
		A[2+a] =  dDOT14(B,0+b,C,2+c); 
		A[4+a] =  dDOT14(B,4+b,C,0+c); 
		A[5+a] =  dDOT14(B,4+b,C,1+c); 
		A[6+a] =  dDOT14(B,4+b,C,2+c); 
		A[8+a] =  dDOT14(B,8+b,C,0+c); 
		A[9+a] =  dDOT14(B,8+b,C,1+c); 
		A[10+a] = dDOT14(B,8+b,C,2+c); 
	}
	public static void dMULTIPLY1_333(DMatrix3 A, DMatrix3 B, DMatrix3 C) { 
		dMULTIPLYOP1_333(A,B,C); }
	public static void dMULTIPLY2_333(DMatrix3 A, DMatrix3 B, DMatrix3C C) { 
		dMULTIPLYOP2_333(A,B,C); }
	//
	//TODO ?
	//#define dMULTIPLYADD0_331(A,B,C) dMULTIPLYOP0_331(A,+=,B,C)
	public static void dMULTIPLYADD0_331(double[] A, int a, double[] B, int b, double[] C, int c) {
		A[0+a] += dDOT(B, b+0, C, c); 
		A[1+a] += dDOT(B, b+4, C, c);
		A[2+a] += dDOT(B, b+8, C, c);
	}
	//#define dMULTIPLYADD1_331(A,B,C) dMULTIPLYOP1_331(A,+=,B,C)
	//#define dMULTIPLYADD0_133(A,B,C) dMULTIPLYOP0_133(A,+=,B,C)
	//#define dMULTIPLYADD0_333(A,B,C) dMULTIPLYOP0_333(A,+=,B,C)
	//#define dMULTIPLYADD1_333(A,B,C) dMULTIPLYOP1_333(A,+=,B,C)
	//#define dMULTIPLYADD2_333(A,B,C) dMULTIPLYOP2_333(A,+=,B,C)

	//#endif


	//#ifdef __cplusplus
	//extern "C" {
	//#endif

	/*
	 * normalize 3x1 and 4x1 vectors (i.e. scale them to unit length)
	 */

	//#if defined(__ODE__)
	//
	//int  _dSafeNormalize3 (dVector3 a);
	//int  _dSafeNormalize4 (dVector4 a);

	//static __inline void _dNormalize3(dVector3 a)
	private static void _dNormalize3(DVector3 a)
	{
		boolean bNormalizationResult = _dSafeNormalize3(a);
		dIASSERT(bNormalizationResult);
		dVARIABLEUSED(bNormalizationResult);
	}

	//static __inline void _dNormalize4(dVector4 a)
	private static void _dNormalize4(double[] a)
	{
		if (!_dSafeNormalize4(a)) {
			throw new IllegalStateException(
					"Normalization failed: " + Arrays.toString(a));
		}
//		int bNormalizationResult = _dSafeNormalize4(a);
//		dIASSERT(bNormalizationResult);
//		dVARIABLEUSED(bNormalizationResult);
	}

	//#endif // defined(__ODE__)

	// For DLL export
	//ODE_API int  dSafeNormalize3 (dVector3 a);
	//ODE_API int  dSafeNormalize4 (dVector4 a);
	//ODE_API void dNormalize3 (dVector3 a); // Potentially asserts on zero vec
	//ODE_API void dNormalize4 (dVector4 a); // Potentially asserts on zero vec

	//#if defined(__ODE__)
	//
	//// For internal use
	//#define dSafeNormalize3(a) _dSafeNormalize3(a)
	//#define dSafeNormalize4(a) _dSafeNormalize4(a)
	//#define dNormalize3(a) _dNormalize3(a)
	//#define dNormalize4(a) _dNormalize4(a)
	//
	//#endif // defined(__ODE__)


	//#ifdef __cplusplus
	//}
	//#endif
	//
	//#endif




	///  FROM odemath.cpp



	/**
	 * this may be called for vectors `a' with extremely small magnitude, for
	 * example the result of a cross product on two nearly perpendicular vectors.
	 * we must be robust to these small vectors. to prevent numerical error,
	 * first find the component a[i] with the largest magnitude and then scale
	 * all the components by 1/a[i]. then we can compute the length of `a' and
	 * scale the components by 1/l. this has been verified to work with vectors
	 * containing the smallest representable numbers.
	 */
	static boolean _dSafeNormalize3 (DVector3 a)
	{
		double s;
		double aa0 = dFabs(a.get0());
		double aa1 = dFabs(a.get1());
		double aa2 = dFabs(a.get2());
		if (aa1 > aa0) {
			if (aa2 > aa1) { // aa[2] is largest
				s = aa2;
			}
			else {              // aa[1] is largest
				s = aa1;
			}
		}
		else {
			if (aa2 > aa0) {// aa[2] is largest
				s = aa2;
			}
			else {              // aa[0] might be the largest
				if (aa0 <= 0) { // aa[0] might is largest
//					a.v[0] = 1;	// if all a's are zero, this is where we'll end up.
//					a.v[1] = 0;	// return a default unit length vector.
//					a.v[2] = 0;
					a.set(1, 0, 0);
					return false;
				}
				else {
					s = aa0;
				}
			}
		}

//		a.v[0] /= aa[idx];
//		a.v[1] /= aa[idx];
//		a.v[2] /= aa[idx];
		a.scale(1./s);
		//l = dRecipSqrt (a.v[0]*a.v[0] + a.v[1]*a.v[1] + a.v[2]*a.v[2]);
//		a.v[0] *= l;
//		a.v[1] *= l;
//		a.v[2] *= l;
		a.scale(1./a.length());
		return true;
	}
	static boolean _dSafeNormalize3 (DVector3View a)
	{
		double s;
		double aa0 = dFabs(a.get0());
		double aa1 = dFabs(a.get1());
		double aa2 = dFabs(a.get2());
		if (aa1 > aa0) {
			if (aa2 > aa1) { // aa[2] is largest
				s = aa2;
			}
			else {              // aa[1] is largest
				s = aa1;
			}
		}
		else {
			if (aa2 > aa0) {// aa[2] is largest
				s = aa2;
			}
			else {              // aa[0] might be the largest
				if (aa0 <= 0) { // aa[0] might is largest
					// if all a's are zero, this is where we'll end up.
					// return a default unit length vector.
					a.set(1, 0, 0);
					return false;
				}
				else {
					s = aa0;
				}
			}
		}

		a.scale(1./s);
		a.scale(1./a.length());
		return true;
	}

	/* OLD VERSION */
	/*
void dNormalize3 (dVector3 a)
{
dIASSERT (a);
dReal l = dDOT(a,a);
if (l > 0) {
 l = dRecipSqrt(l);
 a[0] *= l;
 a[1] *= l;
 a[2] *= l;
}
else {
 a[0] = 1;
 a[1] = 0;
 a[2] = 0;
}
}
	 */

	public static boolean  dSafeNormalize3 (DVector3 a)
	{
		return _dSafeNormalize3(a);
	}

	public static boolean  dSafeNormalize3 (DVector3View a)
	{
		return _dSafeNormalize3(a);
	}

	public static void dNormalize3(DVector3 a)
	{
		_dNormalize3(a);
	}


	//private int _dSafeNormalize4 (dVector4 a)
	//{
	//}
	private static boolean _dSafeNormalize4 (double[] a)
	{
		dAASSERT (a);
		double l = dDOT(a,a)+a[3]*a[3];
		if (l > 0) {
			l = dRecipSqrt(l);
			a[0] *= l;
			a[1] *= l;
			a[2] *= l;
			a[3] *= l;
			return true;
		}
		else {
			a[0] = 1;
			a[1] = 0;
			a[2] = 0;
			a[3] = 0;
			return false;
		}
	}

	public static boolean dSafeNormalize4 (DVector4 a)
	{
		return _dSafeNormalize4(a.v);
	}

	public static void dNormalize4(DVector4 a)
	{
		_dNormalize4(a.v);
	}
	public static void dNormalize4(DQuaternion a)
	{
		_dNormalize4(a.v);
	}


	/**
	 * given a unit length "normal" vector n, generate vectors p and q vectors
	 * that are an orthonormal basis for the plane space perpendicular to n.
	 * i.e. this makes p,q such that n,p,q are all perpendicular to each other.
	 * q will equal n x p. if n is not unit length then p will be unit length but
	 * q wont be.
	 */
	//ODE_API void dPlaneSpace (const dVector3 n, dVector3 p, dVector3 q);
	public static void dPlaneSpace (final DVector3 n, DVector3 p, DVector3 q)
	{
		dAASSERT (n, p, q);
		if (dFabs(n.v[2]) > M_SQRT1_2) {
			// choose p in y-z plane
			double a = n.v[1]*n.v[1] + n.v[2]*n.v[2];
			double k = dRecipSqrt (a);
			p.v[0] = 0;
			p.v[1] = -n.v[2]*k;
			p.v[2] = n.v[1]*k;
			// set q = n x p
			q.v[0] = a*k;
			q.v[1] = -n.v[0]*p.v[2];
			q.v[2] = n.v[0]*p.v[1];
		}
		else {
			// choose p in x-y plane
			double a = n.v[0]*n.v[0] + n.v[1]*n.v[1];
			double k = dRecipSqrt (a);
			p.v[0] = -n.v[1]*k;
			p.v[1] = n.v[0]*k;
			p.v[2] = 0;
			// set q = n x p
			q.v[0] = -n.v[2]*p.v[1];
			q.v[1] = n.v[2]*p.v[0];
			q.v[2] = a*k;
		}
	}



	/**
	 * This takes what is supposed to be a rotation matrix,
	 * and make sure it is correct.
	 * Note: this operates on rows, not columns, because for rotations
	 * both ways give equivalent results.
	 */
	public static void dOrthogonalizeR(DMatrix3 m)
	{
		//double n0 = dLENGTHSQUARED(m);
		DVector3RowTView row0 = m.viewRowT(0); 
		double n0 = row0.length();
		if (n0 != 1)
			//dSafeNormalize3(m);
			dSafeNormalize3(row0);

		// project row[0] on row[1], should be zero
		//double proj = dDOT(m, m+4);
		DVector3View row1 = m.viewRowT(1);
		double proj = row0.dot( row1 );
		if (proj != 0) {
			// Gram-Schmidt step on row[1]
//			m[4] -= proj * m[0];
//			m[5] -= proj * m[1];
//			m[6] -= proj * m[2];
			m.set10( m.get10() - proj * m.get00() );
			m.set11( m.get11() - proj * m.get01() );
			m.set12( m.get12() - proj * m.get02() );
		}
		double n1 = row1.dot(row1);//dLENGTHSQUARED(m+4);
		if (n1 != 1)
			//dSafeNormalize3(m+4);
			dSafeNormalize3(row1);

		/* just overwrite row[2], this makes sure the matrix is not
		a reflection */
		//dCROSS(m+8, =, m, m+4);
		//dCROSS(m, 8, OP.EQ, m, 0, m, 4);
		DVector3View row2 = m.viewRowT(2);
		dCROSS(row2, OP.EQ, row0, row1);
		//TZ m[3] = m[4+3] = m[8+3] = 0;
	}
}