/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.FastLDLTSolve.solveEquationSystemWithLDLT;
import static org.ode4j.ode.internal.FastLSolve.solveL1Straight;
import static org.ode4j.ode.internal.FastLTSolve.solveL1Transposed;
import static org.ode4j.ode.internal.cpp4j.Cstring.memcpy;
import static org.ode4j.ode.internal.cpp4j.Cstring.memmove;

import java.util.Arrays;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemArena;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;

public class Matrix extends FastDot {

	protected Matrix() {
		super();
		// private
	}

	/**
	 * TODO use length of array.
	 *
	 * @param a a
	 * @param n n
	 */
	public static void dSetZero(double[] a, int n) {
		Arrays.fill(a, 0, n, 0);
	}

	public static void dSetZero(double[] a) {
		Arrays.fill(a, 0);
	}

	public static void dSetValue(double[] a, int n, double value) {
		Arrays.fill(a, 0, n, value);
	}

	public static void dSetValue(double[] a, int pos, int len, double value) {
		Arrays.fill(a, pos, pos + len, value);
    }

	/**
	 * Matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method is equivalent to dMultiply0(A, B, C, 3, 3, 1).
	 * @param A A
	 * @param B B
	 * @param C C
	 */
	public static void dMultiply0(DVector3 A, final DMatrix3C B,
			final DVector3C C) {
		// dMultiply0(A.v, ((DMatrix3)B).v, ((DVector3)C).v, 3, 3, 1);
		A.set0( B.get00()*C.get0() + B.get01()*C.get1() + B.get02()*C.get2() );
		A.set1( B.get10()*C.get0() + B.get11()*C.get1() + B.get12()*C.get2() );
		A.set2( B.get20()*C.get0() + B.get21()*C.get1() + B.get22()*C.get2() );
	}

	/**
	 * Matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method is equivalent to dMultiply0(A, B, C, 1, 3, 3).
	 * @param a a
	 * @param b b
	 * @param C C
	 */
	public static void dMultiply0(DVector3 a, final DVector3C b,
			final DMatrix3C C) {
		// dMultiply0(a.v, ((DVector3)b).v, ((DMatrix3)C).v, 1, 3, 3);
		// A.eqMul(B, C);
		// int i,j,k,qskip,rskip,rpad;
		// //dAASSERT (A, B, C);
		// int p = 1, q = 3, r = 3;
		// dAASSERT(p>0 && q>0 && r>0);
		// qskip = dPAD(q);
		// rskip = dPAD(r);
		// rpad = rskip - r;
		// double sum;
		// int aPos = 0;
		// //final double[] b,c,bb;
		// int bPos, bbPos =0, cPos;
		// //TZ? final double bb;
		// //TZ? bb = B;
		a.set0(b.get0() * C.get00() + b.get1() * C.get10() + b.get2()
				* C.get20());
		a.set1(b.get0() * C.get01() + b.get1() * C.get11() + b.get2()
				* C.get21());
		a.set2(b.get0() * C.get02() + b.get1() * C.get12() + b.get2()
				* C.get22());
		// for (i=1; i > 0; i--) {
		// for (j=0 ; j<3; j++) {
		// //c = C + j;
		// cPos = j;
		// //b = bb;
		// bPos = bbPos;
		// sum = 0;
		// //for (k=q; k > 0; k--, cPos+=rskip) sum += (*(b++))*(*c);
		// for (k=q; k > 0; k--, cPos+=4) sum += B[bPos++] * C[cPos];
		// //*(A++) = sum;
		// A[aPos++] = sum;
		// }
		// // A += rpad;
		// // bb += qskip;
		// aPos += 1;//rpad;
		// //bb += qskip;
		// bbPos += 4;
		// }
	}

	/**
	 * Matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method is equivalent to dMultiply0(A, B, C, 3, 3, 3).
	 * @param A A
	 * @param B B
	 * @param C C
	 */
	public static void dMultiply0(DMatrix3 A, final DMatrix3C B,
			final DMatrix3C C) {
		A.eqMul(B, C);
	}

	/**
	 * matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method should take vectors as parameters A,C, but because it is
	 * widely used for matrices, we supply this helper method.
	 * 
	 * @param A A
	 * @param B B
	 * @param C C
	 * @param p p
	 * @param q q
	 * @param r r
	 */
	public static void dMultiply0_OLD(double[] A, final double[] B,
			final double[] C, int p, int q, int r) {
		int i, j, k, qskip, rskip, rpad;
		// dAASSERT (A, B, C);
		dAASSERT(p > 0 && q > 0 && r > 0);
		qskip = dPAD(q);
		rskip = dPAD(r);
		rpad = rskip - r;
		double sum;
		int aPos = 0;
		// final double[] b,c,bb;
		int bPos, bbPos = 0, cPos;
		// TZ? final double bb;
		// TZ? bb = B;
		for (i = p; i > 0; i--) {
			for (j = 0; j < r; j++) {
				// c = C + j;
				cPos = j;
				// b = bb;
				bPos = bbPos;
				sum = 0;
				// for (k=q; k > 0; k--, cPos+=rskip) sum += (*(b++))*(*c);
				for (k = q; k > 0; k--, cPos += rskip)
					sum += B[bPos++] * C[cPos];
				// *(A++) = sum;
				A[aPos++] = sum;
			}
			// A += rpad;
			// bb += qskip;
			aPos += rpad;
			// bb += qskip;
			bbPos += qskip;
		}
	}

	public static void dMultiply0 (double[] A, final double[] B, final double[] C, int p, int q, int r)
	{
	  dAASSERT (p>0 && q>0 && r>0);
	  final int qskip = dPAD(q);
	  final int rskip = dPAD(r);
	  int aa=0; //dReal *aa = A;
	  int bb=0;//const dReal *bb = B;
	  for (int i=p; i!=0; aa+=rskip, bb+=qskip, --i) {
	    int a = aa;//dReal *a = aa;
	    int cc = 0, ccend = r;//const dReal *cc = C, *ccend = C + r;
	    for (; cc != ccend; ++a, ++cc) {
	      double sum = 0.0;//dReal sum = REAL(0.0);
	      int c = cc;//final dReal *c = cc;
	      int b = bb, bend = bb+q;//final dReal *b = bb, *bend = bb + q;
	      for (; b != bend; c+=rskip, ++b) {
	        sum += B[b]*C[c];//(*b)*(*c);
	      }
	      A[a] = sum;//(*a) = sum; 
	    }
	  }
	}


	/**
	 * matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method is equivalent to dMultiply1(A, B, C, 3, 3, 1).
	 * @param A A
	 * @param B B
	 * @param C C
	 */
	public static void dMultiply1(DVector3 A, final DMatrix3C B,
			final DVector3C C) {
		// dMultiply1(A.v, ((DMatrix3)B).v, ((DVector3)C).v, 3, 3, 1);
		A.set0( B.get00()*C.get0() + B.get10()*C.get1() + B.get20()*C.get2() );
		A.set1( B.get01()*C.get0() + B.get11()*C.get1() + B.get21()*C.get2() );
		A.set2( B.get02()*C.get0() + B.get12()*C.get1() + B.get22()*C.get2() );
	}

	/**
	 * matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method is equivalent to dMultiply1(A, B, C, 3, 3, 3).
	 * @param A A
	 * @param B B
	 * @param C C
	 */
	public static void dMultiply1(DMatrix3 A, final DMatrix3C B,
			final DMatrix3C C) {
		// dMultiply1(A.v, ((DMatrix3)B).v, ((DMatrix3)C).v, 3, 3, 3);
        dMultiply0(A, B.reTranspose(), C);
//		int i, j, k;
//		double sum;
//		// dAASSERT (A , B, C);
//		for (i = 0; i < 3; i++) {
//			for (j = 0; j < 3; j++) {
//				sum = 0;
//				// for (k=0; k<q; k++) sum += B[i+k*pskip] * C[j+k*rskip];
//				for (k = 0; k < 3; k++)
//					sum += B.get(k, i) * C.get(k, j);
//				// A[i*rskip+j] = sum;
//				A.set(i, j, sum);
//			}
//		}
	}

	/**
	 * matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * @param A A
	 * @param B B
	 * @param C C
	 * @param p p
	 * @param q q
	 * @param r r
	 */
	public static void dMultiply1(double[] A, final double[] B,
			final double[] C, int p, int q, int r) {
		// dAASSERT (A , B, C);
		dAASSERT(p > 0 && q > 0 && r > 0);
		final int pskip = dPAD(p);
		final int rskip = dPAD(r);
		int aa = 0;//dReal *aa = A;
		int bb = 0, bbend = p;//const dReal *bb = B, *bbend = B + p;
		for (; bb != bbend; aa += rskip, ++bb) {
		    int a = aa;//dReal *a = aa;
		    int cc = 0, ccend = r;//const dReal *cc = C, *ccend = C + r;
		    for (; cc != ccend; ++a, ++cc) {
		        double sum = 0.0;
		        int b = bb, c = cc;//const dReal *b = bb, *c = cc;
		        for (int k=q; k!=0; b+=pskip, c+=rskip, --k) {
		            sum += B[b]*C[c];//(*b)*(*c);
		        }
		        A[a] = sum;//(*a) = sum;
		    }
		}
	}

	/**
	 * matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method is equivalent to dMultiply2(A, B, C, 3, 3, 3).
	 * @param A A
	 * @param B B
	 * @param C C
	 */
	public static void dMultiply2(DMatrix3 A, final DMatrix3C B, final DMatrix3C C) {
		//dMultiply2(A.v, ((DMatrix3) B).v, ((DMatrix3) C).v, 3, 3, 3);
		dMultiply0(A, B, C.reTranspose());
	}

	/**
	 * matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * <p>
	 * This method is equivalent to dMultiply2(A, B, C, 3, 3, 1).
	 * @param A A
	 * @param B B
	 * @param C C
	 */
	public static void dMultiply2(DVector3 A, final DMatrix3C B, final DVector3C C) {
		//dMultiply2(A.v, ((DMatrix3) B).v, ((DVector3) C).v, 3, 3, 1);
		//TZ: this is equal to dMultiply0(...) !!!
		A.set0( B.get00()*C.get0() + B.get01()*C.get1() + B.get02()*C.get2() );
		A.set1( B.get10()*C.get0() + B.get11()*C.get1() + B.get12()*C.get2() );
		A.set2( B.get20()*C.get0() + B.get21()*C.get1() + B.get22()*C.get2() );
	}

	/**
	 * matrix multiplication. all matrices are stored in standard row format.
	 * the digit refers to the argument that is transposed: <br>
	 * 0: A = B * C (sizes: A:p*r B:p*q C:q*r) <br>
	 * 1: A = B' * C (sizes: A:p*r B:q*p C:q*r) <br>
	 * 2: A = B * C' (sizes: A:p*r B:p*q C:r*q) <br>
	 * case 1,2 are equivalent to saying that the operation is A=B*C but B or C
	 * are stored in standard column format.
	 * @param A A
	 * @param B B
	 * @param C C
	 * @param p p
	 * @param q q
	 * @param r r
	 * 
	 */
	public static void dMultiply2(double[] A, final double[] B, final double[] C, int p, int q, int r) {
	    dAASSERT(p > 0 && q > 0 && r > 0);
	    final int rskip = dPAD(r);
	    final int qskip = dPAD(q);
	    int aa = 0;//dReal *aa = A;
	    int bb = 0;//const dReal *bb = B;
	    for (int i=p; i!=0; aa+=rskip, bb+=qskip, --i) {
	        int a = aa, aend = aa+r;//dReal *a = aa, *aend = aa + r;
	        int cc = 0;//const dReal *cc = C;
	        for (; a != aend; cc+=qskip, ++a) {
	            double sum = 0.0;
	            int b = bb, c = cc, cend = cc+q;//const dReal *b = bb, *c = cc, *cend = cc + q;
	            for (; c != cend; ++b, ++c) {
	                sum += B[b]*C[c];//(*b)*(*c);
	            }
	            A[a] = sum;//(*a) = sum; 
	        }
	    }
	}

	/**
	 * do an in-place cholesky decomposition on the lower triangle of the n*n
	 * symmetric matrix A (which is stored by rows). the resulting lower
	 * triangle will be such that L*L'=A. return 1 on success and 0 on failure
	 * (on failure the matrix is not positive definite).
	 * @param A A
	 * @param n n
	 * @param tmpbuf tmpBuf 
	 * @return 'false' if it fails 
	 */
	public static boolean dxFactorCholesky(double[] A, int n, double[] tmpbuf) {
	    //dAASSERT (n > 0 && A);
	    boolean failure = false;
	    final int nskip = dPAD (n);
	    //dReal *recip = tmpbuf ? (dReal *)tmpbuf : (dReal*) ALLOCA (n * sizeof(dReal));
	    double[] recip = tmpbuf!=null ? tmpbuf : new double[n];
	    int aa = 0;//dReal *aa = A;
	    for (int i=0; i<n; aa+=nskip, ++i) {
	        int cc = aa;//dReal *cc = aa;
	        {
	            int bb = 0;//const dReal *bb = A;
	            for (int j=0; j<i; bb+=nskip, ++cc, ++j) {
	                double sum = A[cc];//dReal sum = *cc;
	                int a = aa, b = bb, bend = bb + j;//const dReal *a = aa, *b = bb, *bend = bb + j;
	                for (; b != bend; ++a, ++b) {
	                    sum -= A[a]*A[b];//(*a)*(*b);
	                }
	                A[cc] = sum*recip[j];//*cc = sum * recip[j];
	            }
	        }
	        {
	            double sum = A[cc];//dReal sum = *cc;
	            int a = aa, aend = aa+i;//dReal *a = aa, *aend = aa + i;
	            for (; a != aend; ++a) {
	                sum -= A[a]*A[a];//(*a)*(*a);
	            }
	            if (sum <= 0.0) {
	                failure = true;
	                break;
	            }
	            double sumsqrt = dSqrt(sum);
	            A[cc] = sumsqrt;
	            recip[i] = dRecip (sumsqrt);
	        }
	    }
	    return !failure;//failure ? 0 : 1;
	}

	/**
	 * do an in-place cholesky decomposition on the lower triangle of the n*n
	 * symmetric matrix A (which is stored by rows). the resulting lower
	 * triangle will be such that L*L'=A. return 1 on success and 0 on failure
	 * (on failure the matrix is not positive definite).
	 * @param A matrix
	 * @return 'false'if it failed
	 */
	public static boolean dFactorCholesky (DMatrix3 A)
	{
	    double sum, recip0, recip1;
		//********************
		//i=0; aaPos = 0; bbPos = 0; ccPos = 0
		sum = A.get00();
		if (sum <= 0.0) return false;
		A.set00( dSqrt(sum) );
		recip0 = dRecip(A.get00());
		//********************
		//i=1; aaPos = 4; bbPos = 0; ccPos = 0
		A.set10( A.get10()*recip0 );
		sum = A.get11() - A.get10()*A.get10();
		if (sum <= 0.0) return false;
		A.set11( dSqrt(sum) );
		recip1 = dRecip(A.get11());
		//********************
		//i=2; aaPos = 8; bbPos = 4; ccPos = 5
		A.set20( A.get20()*recip0 );
		sum = A.get21() - A.get20()*A.get10();  //k
		A.set21( sum*recip1 );
		sum = A.get22() - A.get20()*A.get20() - A.get21()*A.get21();
		if (sum <= 0.0) return false;
		A.set22( dSqrt(sum) );
		return true;
	}


	/**
	 * solve for x: L*L'*x = b, and put the result back into x. L is size n*n, b
	 * is size n*1. only the lower triangle of L is considered.
	 * @param L L
	 * @param b b
	 * @param n n
	 * @param tmpbuf buffer 
	 */
	public static void dxSolveCholesky(final double[] L, double[] b, int n, double[] tmpbuf) {
		int nskip = dPAD(n);
		double[] y = (tmpbuf != null?tmpbuf : new double[n]);
		dAASSERT(n > 0);
		// dAASSERT(L, b);
		int ll = 0;
		for (int i = 0; i < n; ll+=nskip, ++i) {
			double sum = 0;
			for (int k = 0; k < i; ++k)
				sum += L[ll + k] * y[k];
			y[i] = (b[i] - sum) / L[ll + i];
		}
	    ll = (n - 1) * (nskip + 1);
	    for (int i=n; i>0; ll-=nskip+1) {
	    	--i;
	        double sum = 0.0;
	        int l = ll + nskip;
	        for (int k=i+1; k<n; l+=nskip, ++k) {
	            sum += L[l]*b[k];
	        }
	        //dIASSERT(*ll != dReal(0.0));
	        b[i] = (y[i]-sum)/L[ll];//(*ll);
	    }
	}

	/**
	 * solve for x: L*L'*x = b, and put the result back into x. L is size 3*3, b
	 * is size 3*1. only the lower triangle of L is considered.
	 * @param L L
	 * @param b b
	 */
	public static void dSolveCholesky(final DMatrix3C L, DVector3 b) {
		// int i,k,nskip;
		double sum;// , y[];
		// dAASSERT (n > 0);
		// dAASSERT(L, b);
		// nskip = dPAD (n);
		// y = new double[n]; //TZ (double*) ALLOCA (n*sizeof(double));
		DVector3 y = new DVector3();
		// for (i=0; i<n; i++) {
		// sum = 0;
		// for (k=0; k < i; k++) sum += L[i*nskip+k]*y[k];
		// y[i] = (b[i]-sum)/L[i*nskip+i];
		// }
		sum = 0;
		y.set0((b.get(0) - sum) / L.get00());
		sum = L.get10() * y.get0();
		y.set1((b.get(1) - sum) / L.get11());
		sum = L.get20() * y.get0() + L.get21() * y.get1();
		y.set2((b.get(2) - sum) / L.get22());

		// for (i=n-1; i >= 0; i--) {
		// sum = 0;
		// //for (k=i+1; k < n; k++) sum += L[k*nskip+i]*b[k];
		// sum = L.get(0, i) * b.get0() + L.get(1, i) * b.get1() + L.get(2, i) *
		// b.get2();
		// b[i] = (y[i]-sum)/L[i*nskip+i];
		// }
		sum = 0;
		b.set2((y.get2() - sum) / L.get22());
		sum = L.get21() * b.get2();
		b.set1((y.get1() - sum) / L.get11());
		sum = L.get10() * b.get1() + L.get20() * b.get2();
		b.set0((y.get0() - sum) / L.get00());
	}

	/**
	 * compute the inverse of the n*n positive definite matrix A and put it in
	 * Ainv. this is not especially fast. this returns 1 on success (A was
	 * positive definite) or 0 on failure (not PD).
	 * @param A A
	 * @param Ainv Ainv result 
	 * @return 'false' on failure
	 */
	public static boolean dInvertPDMatrix(final DMatrix3C A, DMatrix3 Ainv) {
		int i;//, j, nskip;
		// double[] L,x;
		// dAASSERT (n > 0);
		// dAASSERT(A, Ainv);
		//nskip = dPAD(n);
		// L = new double[nskip*n]; //TZ (double*) ALLOCA
		// (nskip*n*sizeof(double));
		DMatrix3 L = new DMatrix3(A);
		// memcpy (L,A,nskip*n);//*sizeof(double));
		// x = new double[n]; //TZ (double*) ALLOCA (n*sizeof(double));
		DVector3 x = new DVector3();
		if (!dFactorCholesky(L))
			return false;
		// dSetZero (Ainv,n*nskip); // make sure all padding elements set to 0
		Ainv.setZero();
		for (i = 0; i < 3; i++) {
			// for (j=0; j<n; j++) x[j] = 0;
			x.setZero();
			// x[i] = 1;
			x.set(i, 1);
			dSolveCholesky(L, x);
			// for (j=0; j<n; j++) Ainv[j*nskip+i] = x[j];
			Ainv.setCol(i, x);
		}
		return true;
	}

	/**
	 * compute the inverse of the n*n positive definite matrix A and put it in
	 * Ainv. this is not especially fast. this returns 1 on success (A was
	 * positive definite) or 0 on failure (not PD).
	 * @param A A
	 * @param Ainv Ainv result 
	 * @param n n
	 * @param tmpbuf buf 
	 * @return 'false' on failure
	 */
	public static boolean dxInvertPDMatrix(final double[] A, double[] Ainv, int n, double[] tmpbuf) {
		int nskip;
		double[] L, X;
		dAASSERT(n > 0);
		// dAASSERT(A, Ainv);
		nskip = dPAD(n);
		L = new double[nskip * n]; // TZ (double*) ALLOCA
									// (nskip*n*sizeof(double));
		//memcpy(L, A, nskip * n);// *sizeof(double));
		System.arraycopy(A, 0, L, 0, nskip * n);
		X = new double[n]; // TZ (double*) ALLOCA (n*sizeof(double));
		if (!dxFactorCholesky(L, n, tmpbuf))
			return false;
		dSetZero(Ainv, n * nskip); // make sure all padding elements set to 0
		int aa = 0;
		for (int xi = 0; xi < n; ++aa, xi++) {
			for (int j = 0; j < n; j++)
				X[j] = 0;
			X[xi] = 1;
			dxSolveCholesky(L, X, n, tmpbuf);
			int a = aa;
			for (int x = 0; x < n; a += nskip, x++)
				Ainv[a] = X[x];
		}
		return true;
	}

	/**
	 * check whether an n*n matrix A is positive definite, return 1/0 (yes/no).
	 * positive definite means that x'*A*x &gt; 0 for any x. this performs a
	 * cholesky decomposition of A. if the decomposition fails then the matrix
	 * is not positive definite. A is stored by rows. A is not altered.
	 * @param A A
	 * @param n n
	 * @param tmpbuf buf 
	 * @return 'false' on failure
	 */
	public static boolean dIsPositiveDefinite(final double[] A, int n, double[] tmpbuf) {
		double[] Acopy;
		dAASSERT(n > 0);
		int nskip = dPAD(n);
		DxWorldProcessMemArena.dummyStatic();//tmpbuf
		Acopy = new double[nskip * n]; // TZ (double*) ALLOCA (nskip*n *
										// sizeof(double));
		//memcpy(Acopy, A, nskip * n);// * sizeof(double));
		System.arraycopy(A, 0, Acopy, 0, nskip * n);
		return dxFactorCholesky(Acopy, n, tmpbuf);// != false;
	}
    public static boolean dIsPositiveDefinite(final double[] A, int n) {
        return dIsPositiveDefinite(A, n, null);
    }
    
	/**
	 * check whether an n*n matrix A is positive definite, return 1/0 (yes/no).
	 * positive definite means that x'*A*x &gt; 0 for any x. this performs a
	 * cholesky decomposition of A. if the decomposition fails then the matrix
	 * is not positive definite. A is stored by rows. A is not altered.
	 * @param A A
	 * @return 'false' on failure
	 */
	public static boolean dIsPositiveDefinite(DMatrix3C A) {
		return dFactorCholesky(A.copy());
	}

	/**
	 * given `L', a n*n lower triangular matrix with ones on the diagonal, and
	 * `d', a n*1 vector of the reciprocal diagonal elements of an n*n matrix D,
	 * solve L*D*L'*x=b where x,b are n*1. x overwrites b. the leading dimension
	 * of L is `nskip'.
	 * @param L L
	 * @param d d
	 * @param b b
	 * @param n n
	 * @param nskip nskip 
	 */
	public static void dSolveLDLT(final double[] L, final double[] d,
			double[] b, int n, int nskip) {
		dIASSERT(n != 0);

		if (n != 0) {
			solveEquationSystemWithLDLT(L, d, 0, b, 0, n, nskip, 1, 1);
		}
	}

	/**
	 * Given an L*D*L' factorization of an n*n matrix A, return the updated
	 * factorization L2*D2*L2' of A plus the following "top left" matrix:
	 * 
	 * [ b a' ] &lt;-- b is a[0] [ a 0 ] &lt;-- a is a[1..n-1]
	 * 
	 * - L has size n*n, its leading dimension is nskip. L is lower triangular
	 * with ones on the diagonal. only the lower triangle of L is referenced. -
	 * d has size n. d contains the reciprocal diagonal elements of D. - a has
	 * size n. the result is written into L, except that the left column of L
	 * and d[0] are not actually modified. see ldltaddTL.m for further comments.
	 * @param L L
	 * @param d d
	 * @param a a
	 * @param n n
	 * @param nskip nskip
	 */
	public static void dLDLTAddTL(double[] L, double[] d, final double[] a,
			int n, int nskip) {
		dLDLTAddTL(L, 0, d, 0, a, n, nskip, null);
	}

	/**
	 * Given an L*D*L' factorization of an n*n matrix A, return the updated
	 * factorization L2*D2*L2' of A plus the following "top left" matrix:
	 * 
	 * [ b a' ] &lt;-- b is a[0] [ a 0 ] &lt;-- a is a[1..n-1]
	 * 
	 * - L has size n*n, its leading dimension is nskip. L is lower triangular
	 * with ones on the diagonal. only the lower triangle of L is referenced. -
	 * d has size n. d contains the reciprocal diagonal elements of D. - a has
	 * size n. the result is written into L, except that the left column of L
	 * and d[0] are not actually modified. see ldltaddTL.m for further comments.
	 * @param L L
	 * @param lOfs offset 
	 * @param d d
	 * @param dOfs offset 
	 * @param a a
	 * @param n n
	 * @param nskip nskip 
	 * @param tmpbuf buffer
	 */
	public static void dLDLTAddTL(double[] L, int lOfs, double[] d, int dOfs,
			final double[] a, int n, int nskip, double[] tmpbuf) {
		// dAASSERT (L, d, a);
		dAASSERT(n > 0 && nskip >= n);

		if (n < 2)
			return;
		DxWorldProcessMemArena.dummyStatic();
		double[] W1 = new double[n]; // (double*) ALLOCA (n*sizeof(double));
		double[] W2 = new double[n]; // (double*) ALLOCA (n*sizeof(double));

		W1[0] = 0;
		W2[0] = 0;
		for (int j = 1; j < n; j++)
			W1[j] = W2[j] = (a[j] * M_SQRT1_2);
		double W11 = ((0.5 * a[0] + 1) * M_SQRT1_2);
		double W21 = ((0.5 * a[0] - 1) * M_SQRT1_2);

		double alpha1 = 1;
		double alpha2 = 1;

		{
		    double dee = d[dOfs + 0];
		    double alphanew = alpha1 + (W11 * W11) * dee;
		    //dIASSERT(alphanew != dReal(0.0));
		    dee /= alphanew;
		    double gamma1 = W11 * dee;
		    dee *= alpha1;
		    alpha1 = alphanew;
		    alphanew = alpha2 - (W21 * W21) * dee;
		    dee /= alphanew;
		    //gamma2 = W21 * dee;
		    alpha2 = alphanew;
		    double k1 = 1.0 - W21 * gamma1;
		    double k2 = W21 * gamma1 * W11 - W21;
		    int ll = nskip + lOfs;
		    for (int p = 1; p < n; ll+=nskip, p++) {
		        double Wp = W1[p];
		        double ell = L[ll];//Ofs + p * nskip];
		        W1[p] = Wp - W11 * ell;
		        W2[p] = k1 * Wp + k2 * ell;
		    }
		}

		int ll = lOfs + nskip + 1;
		for (int j = 1; j < n; ll+=nskip+1, j++) {
            double k1 = W1[j];
            double k2 = W2[j];
			double dee = d[dOfs + j];
			double alphanew = alpha1 + (k1*k1) * dee;
			//dIASSERT(alphanew != dReal(0.0));
			dee /= alphanew;
			double gamma1 =k1 * dee;
			dee *= alpha1;
			alpha1 = alphanew;
			alphanew = alpha2 - (k2*k2) * dee;
			dee /= alphanew;
			double gamma2 = k2 * dee;
			dee *= alpha2;
			d[dOfs + j] = dee;
			alpha2 = alphanew;

			int l = ll + nskip;
			for (int p = j + 1; p < n; l+=nskip, p++) {
			    //TODO this part has changed in 0.12 .....
				double ell = L[l];//lOfs + p * nskip + j];
				double Wp = W1[p] - k1 * ell;
				ell += gamma1 * Wp;
				W1[p] = Wp;
				Wp = W2[p] - k2 * ell;
				ell -= gamma2 * Wp;
				W2[p] = Wp;
				//L[lOfs + p * nskip + j] = ell;
				L[l] = ell;
			}
		}
	}

	// macros for dLDLTRemove() for accessing A - either access the matrix
	// directly or access it via row pointers. we are only supposed to reference
	// the lower triangle of A (it is symmetric), but indexes i and j come from
	// permutation vectors so they are not predictable. so do a test on the
	// indexes - this should not slow things down too much, as we don't do this
	// in an inner loop.

	// #define _GETA(i,j) (A[i][j])
	// //#define _GETA(i,j) (A[(i)*nskip+(j)])
	// #define GETA(i,j) ((i > j) ? _GETA(i,j) : _GETA(j,i))
	// TZ
	private static double GETA(double[] A, int i, int j, int nskip) {
		return (i > j) ? A[i * nskip + j] : A[j * nskip + i];
	}

	/**
	 * Given an L*D*L' factorization of a permuted matrix A, produce a new
	 * factorization for row and column `r' removed. - A has size n1*n1, its
	 * leading dimension in nskip. A is symmetric and positive definite. only
	 * the lower triangle of A is referenced. A itself may actually be an array
	 * of row pointers. - L has size n2*n2, its leading dimension in nskip. L is
	 * lower triangular with ones on the diagonal. only the lower triangle of L
	 * is referenced. - d has size n2. d contains the reciprocal diagonal
	 * elements of D. - p is a permutation vector. it contains n2 indexes into
	 * A. each index must be in the range 0..n1-1. - r is the row/column of L to
	 * remove. the new L will be written within the old L, i.e. will have the
	 * same leading dimension. the last row and column of L, and the last
	 * element of d, are undefined on exit.
	 * 
	 * a fast O(n^2) algorithm is used. see ldltremove.m for further comments.
	 * @param A A
	 * @param p p
	 * @param L L
	 * @param d d
	 * @param n1 n1
	 * @param n2 n2
	 * @param r r
	 * @param nskip nskip 
	 */
	// void dLDLTRemove (double [][]A, final int []p, double []L, double []d,
	// int n1, int n2, int r, int nskip)
	public static void dLDLTRemove(double[] A, final int[] p, double[] L,
								   double[] d, int n1, int n2, int r, int nskip) {
		dLDLTRemove(A, p, L, d, n1, n2, r, nskip, null);
	}
	static void dLDLTRemove(double[] A, final int[] p, double[] L,
			double[] d, int n1, int n2, int r, int nskip, BlockPointer tmpbuf) {
		// dAASSERT(A, p, L, d);
		dAASSERT(n1 > 0 && n2 > 0 /*&& r >= 0*/ && r < n2 && n1 >= n2
				&& nskip >= n1);
		if (!dNODEBUG) {// #ifndef dNODEBUG
			for (int i = 0; i < n2; i++)
				dIASSERT(p[i] >= 0 && p[i] < n1);
		}// #endif

		if (r == n2 - 1) {
			return; // deleting last row/col is easy
		} else {
		DxWorldProcessMemArena.dummyStatic();
//		    int LDLTAddTL_size = dxEstimateLDLTAddTLTmpbufSize(nskip);
//		    dIASSERT(LDLTAddTL_size % 8 /*sizeof(dReal)*/ == 0);
//		    double[] tmp = tmpbuf!=null ? tmpbuf : new double[LDLTAddTL_size + n2];
		    if (r == 0) {
		        double[] a = new double[n2]; // TZ (double*) ALLOCA (n2 *
		        // sizeof(double));
		        final int p_0 = p[0];
		        for (int i = 0; i < n2; i++)
		            a[i] = -GETA(A, p[i], p_0, nskip);
		        a[0] += 1.0;
		        dLDLTAddTL(L, d, a, n2, nskip);
		    } else {
		        double[] t = new double[r]; // TZ (double*) ALLOCA (r * sizeof(double));
		        {
		            int Lcurr = r*nskip;
		            for (int i=0; i<r; ++Lcurr, ++i) {
		                dIASSERT(d[i] != 0.0);
		                t[i] = L[Lcurr] / d[i];
		            }
		        }
		        double[] a = new double[n2 - r]; //dReal *a = t + r;
		        {
		            int Lcurr = r*nskip;//dReal *Lcurr = L + r*nskip;
		            //const int *pp_r = p + r, p_r = *pp_r;
		            int pp_rP = r, p_rP = r; 
		            final int n2_minus_r = n2-r;
		            for (int i=0; i<n2_minus_r; Lcurr+=nskip,++i) {
		                a[i] = dDot(L, Lcurr,t, 0,r) - GETA(A, p[pp_rP+i],p[p_rP], nskip);
		            }
		        }
		        a[0] += (1.0);
				dLDLTAddTL(L, (nskip + 1) * r, d, r, a, n2 - r, nskip, null);

		    }
		}
//		        double[] a = new double[n2 - r]; // TZ (double*) ALLOCA ((n2-r) * sizeof(double));
//		        for (int i = 0; i < r; i++)
//		            t[i] = L[r * nskip + i] / d[i];
//		        for (int i = 0; i < (n2 - r); i++)
//		            // a[i] = O_M.dDot(L+(r+i)*nskip,t,r) - GETA(A, p[r+i],p[r]);
//		            a[i] = FastDot.dDot(L, (r + i) * nskip, t, 0, r)
//		            - GETA(A, p[r + i], p[r], nskip);
//		        a[0] += 1.0;
//		        // dLDLTAddTL (L + r*nskip+r, d + r, a, n2-r, nskip);
//		        dLDLTAddTL(L, r * nskip + r, d, r, a, n2 - r, nskip, null);
//		    }

		// snip out row/column r from L and d
		dRemoveRowCol(L, n2, nskip, r);
		if (r < (n2 - 1))
			memmove(d, r, d, r + 1, (n2 - r - 1));// *sizeof(double));
	}

	/**
	 * Given an n*n matrix A (with leading dimension nskip), remove the r'th row
	 * and column by moving elements. the new matrix will have the same leading
	 * dimension. the last row and column of A are untouched on exit.
	 * @param A A
	 * @param n n
	 * @param nskip nskip 
	 * @param r r
	 */
	public static void dRemoveRowCol(double[] A, int n, int nskip, int r) {
		dAASSERT((A != null) && n > 0 && nskip >= n && r >= 0 && r < n);
		if (r >= n - 1)
		    return;
		if (r > 0) {
		    {
		        final int move_size = (n-r-1);//*sizeof(dReal);
		        int Adst = r;//dReal *Adst = A + r;
		        for (int i=0; i<r; Adst+=nskip,++i) {
		            int Asrc = Adst + 1;//dReal *Asrc = Adst + 1;
		            memmove (A,Adst,A,Asrc,move_size);
		        }
		    }
		    {
		        final int cpy_size = r;//*sizeof(dReal);
				int Adst = nskip * r;//dReal *Adst = A + r * nskip;
				int n1 = n - 1;
				for (int i = r; i < n1; ++i) {
		            int Asrc = Adst + nskip;//dReal *Asrc = Adst + nskip;
		            memcpy (A,Adst,A,Asrc,cpy_size);
		            Adst = Asrc;
		        }
		    }
		}
		{
		    final int cpy_size = (n-r-1);//*sizeof(dReal);
			int Adst = (nskip + 1) * r;//dReal *Adst = A + r * (nskip + 1);
			int n1 = n - 1;
			for (int i = r; i < n1; ++i) {
		        int Asrc = Adst + (nskip + 1);//dReal *Asrc = Adst + (nskip + 1);
		        memcpy (A,Adst,A,Asrc,cpy_size);
		        Adst = Asrc - 1;
		    }
		}
	}

//	template <unsigned a_stride, typename element_type>
//	ODE_INLINE
	public static void dxtSetZero (double[] aA, int aP, int n, int a_stride)
	{
		int aend = aP + n * a_stride;
		for (int acurr = aP; acurr != aend; acurr += a_stride) {
        	aA[acurr] = 0;
		}
	}
	//	/*extern */
	public static void dxSetZero (double[] a, int n)
	{
		dSetZero (a, n);
	}

	public static void dxSetZero (double[] a, int aPos, int n)
	{
		Arrays.fill(a, aPos, n + aPos, 0);
	}


//	/*extern */
//	void dSetZero (double[] a, int n)
//	{
//		dxSetZero (a, n);
//	}
//
//	/*extern */
//	void dSetValue (dReal *a, int n, dReal value)
//	{
//		dxSetValue (a, n, value);
//	}
//
//// dReal dDot (const dReal *a, const dReal *b, int n);
//
//	/*extern */
//	void dMultiply0 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
//	{
//		dxMultiply0 (A, B, C, p, q, r);
//	}
//
//	/*extern */
//	void dMultiply1 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
//	{
//		dxMultiply1 (A, B, C, p, q, r);
//	}
//
//	/*extern */
//	void dMultiply2 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
//	{
//		dxMultiply2 (A, B, C, p, q, r);
//	}

	/*extern */
	public static boolean dFactorCholesky (double[] A, int n)
	{
		return dxFactorCholesky (A, n, null);
	}

	/*extern */
	public static void dSolveCholesky (final double[] L, double[] b, int n)
	{
		dxSolveCholesky (L, b, n, null);
	}

	/*extern */
	public static boolean dInvertPDMatrix (final double[] A, double[] Ainv, int n)
	{
		return dxInvertPDMatrix (A, Ainv, n, null);
	}

	/*extern */
	//	public static int dIsPositiveDefinite (final double[] A, int n)
	//	{
	//		return dxIsPositiveDefinite (A, n, null);
	//	}



	public static void dSolveL1(final double[] L, double[] B, int n, int lskip1)
	{
		dAASSERT(n != 0);

		if (n != 0)
		{
			dAASSERT(L != null);
			dAASSERT(B != null);

			solveL1Straight(L, B, 0, n, lskip1, 1);
		}
	}

	public static void dSolveL1T (final double[] L, double[] B, int rowCount, int rowSkip)
	{
		dAASSERT(rowCount != 0);

		if (rowCount != 0)
		{
			dAASSERT(L != null);
			dAASSERT(B != null);

			solveL1Transposed(L, B, 0, rowCount, rowSkip, 1);
		}
	}

	/**
	 * factorize a matrix A into L*D*L', where L is lower triangular with ones
	 * on the diagonal, and D is diagonal. A is an n*n matrix stored by rows,
	 * with a leading dimension of n rounded up to 4. L is written into the
	 * strict lower triangle of A (the ones are not written) and the reciprocal
	 * of the diagonal elements of D are written into d.
	 * @param A A
	 * @param d d
	 * @param n n
	 * @param nskip1 nskip 
	 */
	public static void dFactorLDLT(double[] A, double[] d, int n, int nskip1) {
		FastLDLTFactor.factorMatrixAsLDLT(A, d, n, nskip1, 1);
	}
	
	private static int _dEstimateFactorCholeskyTmpbufSize(int n)
	{
	  return dPAD(n) * 8;//sizeof(dReal);
	}

	private static int _dEstimateSolveCholeskyTmpbufSize(int n)
	{
	  return dPAD(n) * 8;//sizeof(dReal);
	}

	private static int _dEstimateInvertPDMatrixTmpbufSize(int n)
	{
	  int FactorCholesky_size = _dEstimateFactorCholeskyTmpbufSize(n);
	  int SolveCholesky_size = _dEstimateSolveCholeskyTmpbufSize(n);
	  int MaxCholesky_size = FactorCholesky_size > SolveCholesky_size ? FactorCholesky_size : SolveCholesky_size;
	  return dPAD(n) * (n + 1) * 8 /* sizeof(dReal) */ + MaxCholesky_size;
	}

	private static int _dEstimateIsPositiveDefiniteTmpbufSize(int n)
	{
	  return dPAD(n) * n * 8 /* sizeof(dReal) */ + _dEstimateFactorCholeskyTmpbufSize(n);
	}

	private static int _dEstimateLDLTAddTLTmpbufSize(int nskip)
	{
	  return nskip * 2 * 8 /* sizeof(dReal) */;
	}

	private static int _dEstimateLDLTRemoveTmpbufSize(int n2, int nskip)
	{
	  return n2 * 8 /* sizeof(dReal) */ + _dEstimateLDLTAddTLTmpbufSize(nskip);
	}

	public static int dEstimateFactorCholeskyTmpbufSize(int n) { return _dEstimateFactorCholeskyTmpbufSize(n); }
	public static int dEstimateSolveCholeskyTmpbufSize(int n) { return _dEstimateSolveCholeskyTmpbufSize(n); }
	public static int dEstimateInvertPDMatrixTmpbufSize(int n) { return _dEstimateInvertPDMatrixTmpbufSize(n); }
	public static int dEstimateIsPositiveDefiniteTmpbufSize(int n) { return _dEstimateIsPositiveDefiniteTmpbufSize(n); }
	public static int dEstimateLDLTAddTLTmpbufSize(int nskip) { return _dEstimateLDLTAddTLTmpbufSize(nskip); }
	public static int dEstimateLDLTRemoveTmpbufSize(int n2, int nskip) { return _dEstimateLDLTRemoveTmpbufSize(n2, nskip); }

	public static double dxDot(final double[] a, final double[] b, int n) {
		return dDot(a, b, n);
	}
	public static double dxDot(final double[] a, int aOfs, final double[] b, int bOfs, int n) {
		return dDot(a, aOfs, b, bOfs, n);
	}
}