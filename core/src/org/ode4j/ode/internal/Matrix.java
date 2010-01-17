/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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

import java.util.Arrays;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

import static org.cpp4j.Cstdio.*;

public class Matrix extends FastDot {

	protected Matrix() {
		super();
		// private
	}

	/**
	 * TODO use length of array.
	 * 
	 * @param a
	 * @param n
	 */
	public static void dSetZero(double[] a, int n) {
		// dAASSERT (a);
		dAASSERT(n >= 0);
		while (n > 0) {
			// *(a++) = 0;
			a[n - 1] = 0;
			n--;
		}
	}

	public static void dSetZero(double[] a) {
		Arrays.fill(a, 0);
	}

	public static void dSetValue(double[] a, int n, double value) {
		// dAASSERT (a);
		dAASSERT(n >= 0);
		while (n > 0) {
			// *(a++) = value;
			a[n - 1] = value;
			n--;
		}
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
	 * @param A
	 * @param B
	 * @param C
	 * @param p
	 * @param q
	 * @param r
	 */
	public static void dMultiply0(double[] A, final double[] B,
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
	 */
	public static void dMultiply1(DMatrix3 A, final DMatrix3C B,
			final DMatrix3C C) {
		// dMultiply1(A.v, ((DMatrix3)B).v, ((DMatrix3)C).v, 3, 3, 3);
		int i, j, k;
		double sum;
		// dAASSERT (A , B, C);
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				sum = 0;
				// for (k=0; k<q; k++) sum += B[i+k*pskip] * C[j+k*rskip];
				for (k = 0; k < 3; k++)
					sum += B.get(k, i) * C.get(k, j);
				// A[i*rskip+j] = sum;
				A.set(i, j, sum);
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
	 */
	public static void dMultiply1(double[] A, final double[] B,
			final double[] C, int p, int q, int r) {
		int i, j, k, pskip, rskip;
		double sum;
		// dAASSERT (A , B, C);
		dAASSERT(p > 0 && q > 0 && r > 0);
		pskip = dPAD(p);
		rskip = dPAD(r);
		for (i = 0; i < p; i++) {
			for (j = 0; j < r; j++) {
				sum = 0;
				for (k = 0; k < q; k++)
					sum += B[i + k * pskip] * C[j + k * rskip];
				A[i * rskip + j] = sum;
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
	 */
	public static void dMultiply2(DMatrix3 A, final DMatrix3C B,
			final DMatrix3C C) {
		//dMultiply2(A.v, ((DMatrix3) B).v, ((DMatrix3) C).v, 3, 3, 3);
		dMultiply0(A, B, C.clone().eqTranspose());
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
	 */
	public static void dMultiply2(DVector3 A, final DMatrix3C B,
			final DVector3C C) {
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
	 * 
	 */
	public static void dMultiply2(double[] A, final double[] B,
			final double[] C, int p, int q, int r) {
		int i, j, k, z, rpad, qskip;
		double sum;
		// final double[] bb,cc;
		// TZ:
		int aPos = 0, bPos, cPos;
		// dAASSERT (A, B , C);
		dAASSERT(p > 0 && q > 0 && r > 0);
		rpad = dPAD(r) - r;
		qskip = dPAD(q);
		// bb = B;
		bPos = 0;
		for (i = p; i > 0; i--) {
			// cc = C;
			cPos = 0;
			for (j = r; j > 0; j--) {
				z = 0;
				sum = 0;
				// for (k=q; k>0; k--,z++) sum += bb[z] * cc[z];
				for (k = q; k > 0; k--, z++)
					sum += B[bPos + z] * C[cPos + z];
				// *(A++) = sum;
				A[aPos++] = sum;
				// cc += qskip;
				cPos += qskip;
			}
			// A += rpad;
			aPos += rpad;
			// bb += qskip;
			bPos += qskip;
		}
	}

	/**
	 * do an in-place cholesky decomposition on the lower triangle of the n*n
	 * symmetric matrix A (which is stored by rows). the resulting lower
	 * triangle will be such that L*L'=A. return 1 on success and 0 on failure
	 * (on failure the matrix is not positive definite).
	 */
	public static boolean dFactorCholesky(double[] A, int n) {
		int i, j, k, nskip;
		double sum;
		// double[] a,b,aa,bb;
		int aPos, bPos, aaPos, bbPos;
		// double[] cc;
		int ccPos = 0; // TZ
		double[] recip;
		dAASSERT(n > 0);
		// dAASSERT(A);
		nskip = dPAD(n);
		recip = new double[n]; // TZ (double*) ALLOCA (n * sizeof(double));
		// TZaa = A;
		aaPos = 0;
		for (i = 0; i < n; i++) {
			bbPos = 0;// bb = A;
			ccPos = i * nskip;// cc = A + i*nskip;
			for (j = 0; j < i; j++) {
				sum = A[ccPos]; // TZsum = *cc;
				aPos = aaPos;// a = aa;
				bPos = bbPos;// b = bb;
				// for (k=j; k > 0; k--) sum -= (*(a++))*(*(b++));
				for (k = j; k > 0; k--)
					sum -= (A[aPos++]) * (A[bPos++]);
				A[ccPos] = sum * recip[j];// *cc = sum * recip[j];
				bbPos += nskip;// bb += nskip;
				ccPos++;// cc++;
			}
			sum = A[ccPos];// sum = *cc;
			aPos = aaPos;// a = aa;
			// for (k=i; k > 0; k--, a++) sum -= (*a)*(*a);
			for (k = i; k > 0; k--, aPos++)
				sum -= A[aPos] * A[aPos];
			if (sum <= 0.0)
				return false;
			A[ccPos] = dSqrt(sum);// *cc = COM.dSqrt(sum);
			recip[i] = dRecip(A[ccPos]);// recip[i] = COM.dRecip (*cc);
			aaPos += nskip;// aa += nskip;
		}
		return true;
	}

	/**
	 * do an in-place cholesky decomposition on the lower triangle of the n*n
	 * symmetric matrix A (which is stored by rows). the resulting lower
	 * triangle will be such that L*L'=A. return 1 on success and 0 on failure
	 * (on failure the matrix is not positive definite).
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


	public static boolean dFactorCholesky2(double[] A, int n) {
		System.out.println("CholeskyTZ: " + n + " " + Arrays.toString(A));
		// int i,j,k,nskip;
		// double sum;
		// //double[] a,b,aa,bb;
		// int aPos,bPos,aaPos,bbPos;
		// //double[] cc;
		// int ccPos = 0; //TZ
		// double[] recip;
		// dAASSERT (n > 0);
		// dAASSERT(A);
		// nskip = dPAD (n);
		// recip = new double[n]; //TZ (double*) ALLOCA (n * sizeof(double));
		// //TZaa = A;
		// aaPos = 0;
		// for (i=0; i<n; i++) {
		// bbPos = 0;//bb = A;
		// ccPos = i*nskip;//cc = A + i*nskip;
		// for (j=0; j<i; j++) {
		// sum = A[ccPos]; //TZsum = *cc;
		// aPos = aaPos;//a = aa;
		// bPos = bbPos;//b = bb;
		// //for (k=j; k > 0; k--) sum -= (*(a++))*(*(b++));
		// for (k=j; k > 0; k--) sum -= (A[aPos++])*(A[bPos++]);
		// A[ccPos] = sum * recip[j];//*cc = sum * recip[j];
		// bbPos += nskip;//bb += nskip;
		// ccPos++;//cc++;
		// }
		// sum = A[ccPos];//sum = *cc;
		// aPos = aaPos;//a = aa;
		// //for (k=i; k > 0; k--, a++) sum -= (*a)*(*a);
		// for (k=i; k > 0; k--, aPos++) sum -= A[aPos]*A[aPos];
		// if (sum <= 0.0) {
		// System.out.println("SUM=" + sum);
		// }
		// if (sum <= 0.0) return false;
		// A[ccPos] = dSqrt(sum);//*cc = COM.dSqrt(sum);
		// recip[i] = dRecip(A[ccPos]);//recip[i] = COM.dRecip (*cc);
		// aaPos += nskip;//aa += nskip;
		// }
		// return true;

		int nskip = dPAD(n);
		double sum = 0;
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < i - 1; j++) {
				sum = A[i * nskip + j];// a(i, j);
				for (int k = 0; k < j - 1; k++) {
					sum -= A[i * nskip + k] * A[j * nskip + k];// a(i, k) * a(j,
																// k)
				}// Next k
				A[i * nskip + j] = sum / A[j * nskip + j];// a(i, j) = Summe /
															// a(j, j)
			}// Next j
			sum = A[i * nskip + i];// a(i, i)
			for (int k = 0; k < i - 1; k++) {
				sum -= A[i * nskip + k] * A[i * nskip + k];// a(i, k) * a(i, k)
			} // Next k
			if (sum <= 0) {
				System.out.println("Sum=" + sum);
				return false;// EXIT // A ist nicht positiv definit
			} else {
				// a(i, i) = Sqrt(Summe) // Summe ist positiv
				A[i * nskip + i] = dSqrt(sum);
			}
		} // i
		return true;
	}

	/**
	 * solve for x: L*L'*x = b, and put the result back into x. L is size n*n, b
	 * is size n*1. only the lower triangle of L is considered.
	 */
	public static void dSolveCholesky(final double[] L, double[] b, int n) {
		int i, k, nskip;
		double sum, y[];
		dAASSERT(n > 0);
		// dAASSERT(L, b);
		nskip = dPAD(n);
		y = new double[n]; // TZ (double*) ALLOCA (n*sizeof(double));
		for (i = 0; i < n; i++) {
			sum = 0;
			for (k = 0; k < i; k++)
				sum += L[i * nskip + k] * y[k];
			y[i] = (b[i] - sum) / L[i * nskip + i];
		}
		for (i = n - 1; i >= 0; i--) {
			sum = 0;
			for (k = i + 1; k < n; k++)
				sum += L[k * nskip + i] * b[k];
			b[i] = (y[i] - sum) / L[i * nskip + i];
		}
	}

	/**
	 * solve for x: L*L'*x = b, and put the result back into x. L is size 3*3, b
	 * is size 3*1. only the lower triangle of L is considered.
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
	 */
	public static boolean dInvertPDMatrix(final double[] A, double[] Ainv, int n) {
		int i, j, nskip;
		double[] L, x;
		dAASSERT(n > 0);
		// dAASSERT(A, Ainv);
		nskip = dPAD(n);
		L = new double[nskip * n]; // TZ (double*) ALLOCA
									// (nskip*n*sizeof(double));
		//memcpy(L, A, nskip * n);// *sizeof(double));
		System.arraycopy(A, 0, L, 0, nskip * n);
		x = new double[n]; // TZ (double*) ALLOCA (n*sizeof(double));
		if (!dFactorCholesky(L, n))
			return false;
		dSetZero(Ainv, n * nskip); // make sure all padding elements set to 0
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++)
				x[j] = 0;
			x[i] = 1;
			dSolveCholesky(L, x, n);
			for (j = 0; j < n; j++)
				Ainv[j * nskip + i] = x[j];
		}
		return true;
	}

	/**
	 * check whether an n*n matrix A is positive definite, return 1/0 (yes/no).
	 * positive definite means that x'*A*x > 0 for any x. this performs a
	 * cholesky decomposition of A. if the decomposition fails then the matrix
	 * is not positive definite. A is stored by rows. A is not altered.
	 */
	public static boolean dIsPositiveDefinite(final double[] A, int n) {
		double[] Acopy;
		dAASSERT(n > 0);
		int nskip = dPAD(n);
		Acopy = new double[nskip * n]; // TZ (double*) ALLOCA (nskip*n *
										// sizeof(double));
		//memcpy(Acopy, A, nskip * n);// * sizeof(double));
		System.arraycopy(A, 0, Acopy, 0, nskip * n);
		return dFactorCholesky(Acopy, n);// != false;
	}

	/**
	 * check whether an n*n matrix A is positive definite, return 1/0 (yes/no).
	 * positive definite means that x'*A*x > 0 for any x. this performs a
	 * cholesky decomposition of A. if the decomposition fails then the matrix
	 * is not positive definite. A is stored by rows. A is not altered.
	 */
	public static boolean dIsPositiveDefinite(DMatrix3C A) {
		return dFactorCholesky(A.clone());
	}

	/*****
	 * this has been replaced by a faster version void dSolveL1T (const double
	 * *L, double *b, int n, int nskip) { int i,j; dAASSERT (L && b && n >= 0 &&
	 * nskip >= n); double sum; for (i=n-2; i>=0; i--) { sum = 0; for (j=i+1;
	 * j<n; j++) sum += L[j*nskip+i]*b[j]; b[i] -= sum; } }
	 */

	/** in matlab syntax: a(1:n) = a(1:n) .* d(1:n) */
	private static void dVectorScale(double[] a, final double[] d, int n) {
		// dAASSERT (a != null, d != null, n >= 0);
		for (int i = 0; i < n; i++)
			a[i] *= d[i];
	}

	/**
	 * given `L', a n*n lower triangular matrix with ones on the diagonal, and
	 * `d', a n*1 vector of the reciprocal diagonal elements of an n*n matrix D,
	 * solve L*D*L'*x=b where x,b are n*1. x overwrites b. the leading dimension
	 * of L is `nskip'.
	 */
	public static void dSolveLDLT(final double[] L, final double[] d,
			double[] b, int n, int nskip) {
		// dAASSERT (L, d, b);
		dAASSERT(n > 0 && nskip >= n);
		dSolveL1(L, b, n, nskip);
		dVectorScale(b, d, n);
		dSolveL1T(L, b, n, nskip);
	}

	/**
	 * Given an L*D*L' factorization of an n*n matrix A, return the updated
	 * factorization L2*D2*L2' of A plus the following "top left" matrix:
	 * 
	 * [ b a' ] <-- b is a[0] [ a 0 ] <-- a is a[1..n-1]
	 * 
	 * - L has size n*n, its leading dimension is nskip. L is lower triangular
	 * with ones on the diagonal. only the lower triangle of L is referenced. -
	 * d has size n. d contains the reciprocal diagonal elements of D. - a has
	 * size n. the result is written into L, except that the left column of L
	 * and d[0] are not actually modified. see ldltaddTL.m for further comments.
	 */
	public static void dLDLTAddTL(double[] L, double[] d, final double[] a,
			int n, int nskip) {
		dLDLTAddTL(L, 0, d, 0, a, n, nskip);
	}

	/**
	 * Given an L*D*L' factorization of an n*n matrix A, return the updated
	 * factorization L2*D2*L2' of A plus the following "top left" matrix:
	 * 
	 * [ b a' ] <-- b is a[0] [ a 0 ] <-- a is a[1..n-1]
	 * 
	 * - L has size n*n, its leading dimension is nskip. L is lower triangular
	 * with ones on the diagonal. only the lower triangle of L is referenced. -
	 * d has size n. d contains the reciprocal diagonal elements of D. - a has
	 * size n. the result is written into L, except that the left column of L
	 * and d[0] are not actually modified. see ldltaddTL.m for further comments.
	 */
	public static void dLDLTAddTL(double[] L, int lOfs, double[] d, int dOfs,
			final double[] a, int n, int nskip) {
		int j, p;
		double[] W1, W2;
		double W11, W21, alpha1, alpha2, alphanew, gamma1, gamma2, k1, k2, Wp, ell, dee;
		// dAASSERT (L, d, a);
		dAASSERT(n > 0 && nskip >= n);

		if (n < 2)
			return;
		W1 = new double[n]; // (double*) ALLOCA (n*sizeof(double));
		W2 = new double[n]; // (double*) ALLOCA (n*sizeof(double));

		W1[0] = 0;
		W2[0] = 0;
		for (j = 1; j < n; j++)
			W1[j] = W2[j] = (double) (a[j] * M_SQRT1_2);
		W11 = (double) ((0.5 * a[0] + 1) * M_SQRT1_2);
		W21 = (double) ((0.5 * a[0] - 1) * M_SQRT1_2);

		alpha1 = 1;
		alpha2 = 1;

		dee = d[dOfs + 0];
		alphanew = alpha1 + (W11 * W11) * dee;
		dee /= alphanew;
		gamma1 = W11 * dee;
		dee *= alpha1;
		alpha1 = alphanew;
		alphanew = alpha2 - (W21 * W21) * dee;
		dee /= alphanew;
		gamma2 = W21 * dee;
		alpha2 = alphanew;
		k1 = 1.0 - W21 * gamma1;
		k2 = W21 * gamma1 * W11 - W21;
		for (p = 1; p < n; p++) {
			Wp = W1[p];
			ell = L[lOfs + p * nskip];
			W1[p] = Wp - W11 * ell;
			W2[p] = k1 * Wp + k2 * ell;
		}

		for (j = 1; j < n; j++) {
			dee = d[dOfs + j];
			alphanew = alpha1 + (W1[j] * W1[j]) * dee;
			dee /= alphanew;
			gamma1 = W1[j] * dee;
			dee *= alpha1;
			alpha1 = alphanew;
			alphanew = alpha2 - (W2[j] * W2[j]) * dee;
			dee /= alphanew;
			gamma2 = W2[j] * dee;
			dee *= alpha2;
			d[dOfs + j] = dee;
			alpha2 = alphanew;

			k1 = W1[j];
			k2 = W2[j];
			for (p = j + 1; p < n; p++) {
				ell = L[lOfs + p * nskip + j];
				Wp = W1[p] - k1 * ell;
				ell += gamma1 * Wp;
				W1[p] = Wp;
				Wp = W2[p] - k2 * ell;
				ell -= gamma2 * Wp;
				W2[p] = Wp;
				L[lOfs + p * nskip + j] = ell;
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
	 */
	// void dLDLTRemove (double [][]A, final int []p, double []L, double []d,
	// int n1, int n2, int r, int nskip)
	public static void dLDLTRemove(double[] A, final int[] p, double[] L,
			double[] d, int n1, int n2, int r, int nskip) {
		int i;
		// dAASSERT(A, p, L, d);
		dAASSERT(n1 > 0 && n2 > 0 && r >= 0 && r < n2 && n1 >= n2
				&& nskip >= n1);
		if (!dNODEBUG) {// #ifndef dNODEBUG
			for (i = 0; i < n2; i++)
				dIASSERT(p[i] >= 0 && p[i] < n1);
		}// #endif

		if (r == n2 - 1) {
			return; // deleting last row/col is easy
		} else if (r == 0) {
			double[] a = new double[n2]; // TZ (double*) ALLOCA (n2 *
											// sizeof(double));
			for (i = 0; i < n2; i++)
				a[i] = -GETA(A, p[i], p[0], nskip);
			a[0] += 1.0;
			dLDLTAddTL(L, d, a, n2, nskip);
		} else {
			double[] t = new double[r]; // TZ (double*) ALLOCA (r *
										// sizeof(double));
			double[] a = new double[n2 - r]; // TZ (double*) ALLOCA ((n2-r) *
												// sizeof(double));
			for (i = 0; i < r; i++)
				t[i] = L[r * nskip + i] / d[i];
			for (i = 0; i < (n2 - r); i++)
				// a[i] = O_M.dDot(L+(r+i)*nskip,t,r) - GETA(A, p[r+i],p[r]);
				a[i] = FastDot.dDot(L, (r + i) * nskip, t, 0, r)
						- GETA(A, p[r + i], p[r], nskip);
			a[0] += 1.0;
			// dLDLTAddTL (L + r*nskip+r, d + r, a, n2-r, nskip);
			dLDLTAddTL(L, r * nskip + r, d, r, a, n2 - r, nskip);
		}

		// snip out row/column r from L and d
		dRemoveRowCol(L, n2, nskip, r);
		if (r < (n2 - 1))
			memmove(d, r, d, r + 1, (n2 - r - 1));// *sizeof(double));
	}

	/**
	 * Given an n*n matrix A (with leading dimension nskip), remove the r'th row
	 * and column by moving elements. the new matrix will have the same leading
	 * dimension. the last row and column of A are untouched on exit.
	 */
	public static void dRemoveRowCol(double[] A, int n, int nskip, int r) {
		int i;
		dAASSERT((A != null) && n > 0 && nskip >= n && r >= 0 && r < n);
		if (r >= n - 1)
			return;
		if (r > 0) {
			for (i = 0; i < r; i++)
				memmove(A, i * nskip + r, A, i * nskip + r + 1, (n - r - 1));// *sizeof(double));
			for (i = r; i < (n - 1); i++)
				memcpy(A, i * nskip, A, i * nskip + nskip, r);// *sizeof(double));
		}
		for (i = r; i < (n - 1); i++)
			memcpy(A, i * nskip + r, A, i * nskip + nskip + r + 1, (n - r - 1));// *sizeof(double));
	}

	/**
	 * solve L*x=b, where L is n*n lower triangular with ones on the diagonal,
	 * and x,b are n*1. b is overwritten with x. the leading dimension of L is
	 * `nskip'.
	 * <p>
	 * solve L*X=B, with B containing 1 right hand sides. L is an n*n lower
	 * triangular matrix with ones on the diagonal. L is stored by rows and its
	 * leading dimension is lskip. B is an n*1 matrix that contains the right
	 * hand sides. B is stored by columns and its leading dimension is also
	 * lskip. B is overwritten with X. this processes blocks of 4*4. if this is
	 * in the factorizer source file, n must be a multiple of 4.
	 */
	static void dSolveL1(final double[] L, double[] B, int n, int lskip1) {
		/* declare variables - Z matrix, p and q vectors, etc */
		double Z11, Z21, Z31, Z41, p1, q1, p2, p3, p4;// ,ex[];
		int exP;
		// final double[] ell;
		int ellP;
		int lskip2, lskip3, i, j;
		/* compute lskip values */
		lskip2 = 2 * lskip1;
		lskip3 = 3 * lskip1;
		/* compute all 4 x 1 blocks of X */
		for (i = 0; i <= n - 4; i += 4) {
			/* compute all 4 x 1 block of X, from rows i..i+4-1 */
			/* set the Z matrix to 0 */
			Z11 = 0;
			Z21 = 0;
			Z31 = 0;
			Z41 = 0;
			ellP = i * lskip1; // +L;
			exP = 0;// = B;
			/* the inner loop that computes outer products and adds them to Z */
			for (j = i - 12; j >= 0; j -= 12) {
				for (int k = 0; k < 12; k++) {
					/* load p and q values */
					p1 = L[ellP + k];// ell[0];
					q1 = B[exP + k];
					p2 = L[ellP + k + lskip1];// ell[lskip1];
					p3 = L[ellP + k + lskip2];// ell[lskip2];
					p4 = L[ellP + k + lskip3];// ell[lskip3];
					/* compute outer product and add it to the Z matrix */
					Z11 += p1 * q1;
					Z21 += p2 * q1;
					Z31 += p3 * q1;
					Z41 += p4 * q1;
				}
				// /* load p and q values */
				// p1=L[ellP];//ell[0];
				// q1=ex[0];
				// p2=L[ellP+lskip1];//ell[lskip1];
				// p3=L[ellP+lskip2];//ell[lskip2];
				// p4=L[ellP+lskip3];//ell[lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+1];//ell[1];
				// q1=ex[1];
				// p2=L[ellP+1+lskip1];//ell[1+lskip1];
				// p3=L[ellP+1+lskip2];//ell[1+lskip2];
				// p4=L[ellP+1+lskip3];//ell[1+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+2];//ell[2];
				// q1=ex[2];
				// p2=L[ellP+2+lskip1];//ell[2+lskip1];
				// p3=L[ellP+2+lskip2];//ell[2+lskip2];
				// p4=L[ellP+2+lskip3];//ell[2+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+3];//ell[3];
				// q1=ex[3];
				// p2=L[ellP+3+lskip1];//ell[3+lskip1];
				// p3=L[ellP+3+lskip2];//ell[3+lskip2];
				// p4=L[ellP+3+lskip3];//ell[3+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+4;//ell[4];
				// q1=ex[4];
				// p2=L[ellP+4+lskip1];//ell[4+lskip1];
				// p3=L[ellP+4+lskip2];//ell[4+lskip2];
				// p4=L[ellP+4+lskip3];//ell[4+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+5];//ell[5];
				// q1=ex[5];
				// p2=L[ellP+5+lskip1];//ell[5+lskip1];
				// p3=L[ellP+5+lskip2];//ell[5+lskip2];
				// p4=L[ellP+5+lskip3];//ell[5+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+6];//ell[6];
				// q1=ex[6];
				// p2=L[ellP+6+lskip1];//ell[6+lskip1];
				// p3=L[ellP+6+lskip2];//ell[6+lskip2];
				// p4=L[ellP+6+lskip3];//ell[6+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+7];//ell[7];
				// q1=ex[7];
				// p2=L[ellP+7+lskip1];//ell[7+lskip1];
				// p3=L[ellP+7+lskip2];//ell[7+lskip2];
				// p4=L[ellP+7+lskip3];//ell[7+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP+8];//ell[8];
				// q1=ex[8];
				// p2=L[ellP+8+lskip1];//ell[8+lskip1];
				// p3=L[ellP+8+lskip2];//ell[8+lskip2];
				// p4=L[ellP+8+lskip3];//ell[8+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[9];
				// q1=ex[9];
				// p2=L[ellP;//ell[9+lskip1];
				// p3=L[ellP;//ell[9+lskip2];
				// p4=L[ellP;//ell[9+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[10];
				// q1=ex[10];
				// p2=L[ellP;//ell[10+lskip1];
				// p3=L[ellP;//ell[10+lskip2];
				// p4=L[ellP;//ell[10+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[11];
				// q1=ex[11];
				// p2=L[ellP;//ell[11+lskip1];
				// p3=L[ellP;//ell[11+lskip2];
				// p4=L[ellP;//ell[11+lskip3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// Z21 += p2 * q1;
				// Z31 += p3 * q1;
				// Z41 += p4 * q1;
				/* advance pointers */
				ellP += 12;
				exP += 12;
				/* end of inner loop */
			}
			/* compute left-over iterations */
			j += 12;
			for (; j > 0; j--) {
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP];// ex[0];
				p2 = L[ellP + lskip1];// ell[lskip1];
				p3 = L[ellP + lskip2];// ell[lskip2];
				p4 = L[ellP + lskip3];// ell[lskip3];
				/* compute outer product and add it to the Z matrix */
				Z11 += p1 * q1;
				Z21 += p2 * q1;
				Z31 += p3 * q1;
				Z41 += p4 * q1;
				/* advance pointers */
				ellP += 1;
				exP += 1;
			}
			/* finish computing the X(i) block */
			Z11 = B[exP] - Z11;// ex[0] - Z11;
			B[exP] = Z11;// ex[0] = Z11;
			p1 = L[ellP + lskip1];// ell[lskip1];
			Z21 = B[exP + 1] - Z21 - p1 * Z11;// ex[1] - Z21 - p1*Z11;
			B[exP + 1] = Z21;// ex[1] = Z21;
			p1 = L[ellP + lskip2];// ell[lskip2];
			p2 = L[ellP + 1 + lskip2];// ell[1+lskip2];
			// Z31 = ex[2] - Z31 - p1*Z11 - p2*Z21;
			Z31 = B[exP + 2] - Z31 - p1 * Z11 - p2 * Z21;
			B[exP + 2] = Z31;// ex[2] = Z31;
			p1 = L[ellP + lskip3];// ell[lskip3];
			p2 = L[ellP + 1 + lskip3];// ell[1+lskip3];
			p3 = L[ellP + 2 + lskip3];// ell[2+lskip3];
			// Z41 = ex[3] - Z41 - p1*Z11 - p2*Z21 - p3*Z31;
			Z41 = B[exP + 3] - Z41 - p1 * Z11 - p2 * Z21 - p3 * Z31;
			B[exP + 3] = Z41;// ex[3] = Z41;
			/* end of outer loop */
		}
		/* compute rows at end that are not a multiple of block size */
		for (; i < n; i++) {
			/* compute all 1 x 1 block of X, from rows i..i+1-1 */
			/* set the Z matrix to 0 */
			Z11 = 0;
			ellP = i * lskip1;// +L;
			exP = 0;// B;
			/* the inner loop that computes outer products and adds them to Z */
			for (j = i - 12; j >= 0; j -= 12) {
				for (int k = 0; k < 12; k++) {
					/* load p and q values */
					p1 = L[ellP + k];// ell[0];
					q1 = B[exP + k];
					/* compute outer product and add it to the Z matrix */
					Z11 += p1 * q1;
				}
				// /* load p and q values */
				// p1=L[ellP;//ell[0];
				// q1=ex[0];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[1];
				// q1=ex[1];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[2];
				// q1=ex[2];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[3];
				// q1=ex[3];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[4];
				// q1=ex[4];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[5];
				// q1=ex[5];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[6];
				// q1=ex[6];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[7];
				// q1=ex[7];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[8];
				// q1=ex[8];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[9];
				// q1=ex[9];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[10];
				// q1=ex[10];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				// /* load p and q values */
				// p1=L[ellP;//ell[11];
				// q1=ex[11];
				// /* compute outer product and add it to the Z matrix */
				// Z11 += p1 * q1;
				/* advance pointers */
				ellP += 12;
				exP += 12;
				/* end of inner loop */
			}
			/* compute left-over iterations */
			j += 12;
			for (; j > 0; j--) {
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP];// ex[0];
				/* compute outer product and add it to the Z matrix */
				Z11 += p1 * q1;
				/* advance pointers */
				ellP += 1;
				exP += 1;
			}
			/* finish computing the X(i) block */
			Z11 = B[exP] - Z11;// ex[0] - Z11;
			B[exP] = Z11;// ex[0] = Z11;
		}
	}

	/**
	 * solve L'*x=b, where L is n*n lower triangular with ones on the diagonal,
	 * and x,b are n*1. b is overwritten with x. the leading dimension of L is
	 * `nskip'.
	 * <p>
	 * solve L^T * x=b, with b containing 1 right hand side. L is an n*n lower
	 * triangular matrix with ones on the diagonal. L is stored by rows and its
	 * leading dimension is lskip. b is an n*1 matrix that contains the right
	 * hand side. b is overwritten with x. this processes blocks of 4.
	 */
	static void dSolveL1T(final double[] L, double[] B, int n, int lskip1) {
		/* declare variables - Z matrix, p and q vectors, etc */
		double Z11, m11, Z21, m21, Z31, m31, Z41, m41, p1, q1, p2, p3, p4;
		// ,ex[];
		int exP;
		// final double[] ell;
		int ellP;
		int lskip2, i, j;//lskip3, i, j;
		/* special handling for L and B because we're solving L1 *transpose* */
		// L = L + (n-1)*(lskip1+1);
		// B = B + n-1;
		// TZ:
		int Lp = (n - 1) * (lskip1 + 1);
		int Bp = n - 1;

		lskip1 = -lskip1;
		/* compute lskip values */
		lskip2 = 2 * lskip1;
		//lskip3 = 3 * lskip1;
		/* compute all 4 x 1 blocks of X */
		for (i = 0; i <= n - 4; i += 4) {
			/* compute all 4 x 1 block of X, from rows i..i+4-1 */
			/* set the Z matrix to 0 */
			Z11 = 0;
			Z21 = 0;
			Z31 = 0;
			Z41 = 0;
			ellP = Lp - i;
			exP = Bp;
			/* the inner loop that computes outer products and adds them to Z */
			for (j = i - 4; j >= 0; j -= 4) {
				for (int ij = 0; ij < 4; ij++) {
					/* load p and q values */
					p1 = L[ellP];// ell[0];
					q1 = B[exP - ij];// ex[0];
					p2 = L[ellP - 1];// ell[-1];
					p3 = L[ellP - 2];// ell[-2];
					p4 = L[ellP - 3];// ell[-3];
					/* compute outer product and add it to the Z matrix */
					m11 = p1 * q1;
					m21 = p2 * q1;
					m31 = p3 * q1;
					m41 = p4 * q1;
					ellP += lskip1;
					Z11 += m11;
					Z21 += m21;
					Z31 += m31;
					Z41 += m41;
				}
				// /* load p and q values */
				// p1=L[ellP];//ell[0];
				// q1=B[exP];//ex[0];
				// p2=L[ellP-1];//ell[-1];
				// p3=L[ellP-2];//ell[-2];
				// p4=L[ellP-3];//ell[-3];
				// /* compute outer product and add it to the Z matrix */
				// m11 = p1 * q1;
				// m21 = p2 * q1;
				// m31 = p3 * q1;
				// m41 = p4 * q1;
				// ellP += lskip1;
				// Z11 += m11;
				// Z21 += m21;
				// Z31 += m31;
				// Z41 += m41;
				// /* load p and q values */
				// p1=L[ellP];//ell[0];
				// q1=B[exP-1];//ex[-1];
				// p2=L[ellP-1];//ell[-1];
				// p3=L[ellP-2];//ell[-2];
				// p4=L[ellP-3];//ell[-3];
				// /* compute outer product and add it to the Z matrix */
				// m11 = p1 * q1;
				// m21 = p2 * q1;
				// m31 = p3 * q1;
				// m41 = p4 * q1;
				// ell += lskip1;
				// Z11 += m11;
				// Z21 += m21;
				// Z31 += m31;
				// Z41 += m41;
				// /* load p and q values */
				// p1=ell[0];
				// q1=ex[-2];
				// p2=ell[-1];
				// p3=ell[-2];
				// p4=ell[-3];
				// /* compute outer product and add it to the Z matrix */
				// m11 = p1 * q1;
				// m21 = p2 * q1;
				// m31 = p3 * q1;
				// m41 = p4 * q1;
				// ell += lskip1;
				// Z11 += m11;
				// Z21 += m21;
				// Z31 += m31;
				// Z41 += m41;
				// /* load p and q values */
				// p1=ell[0];
				// q1=ex[-3];
				// p2=ell[-1];
				// p3=ell[-2];
				// p4=ell[-3];
				// /* compute outer product and add it to the Z matrix */
				// m11 = p1 * q1;
				// m21 = p2 * q1;
				// m31 = p3 * q1;
				// m41 = p4 * q1;
				// ell += lskip1;
				// Z11 += m11;
				// Z21 += m21;
				// Z31 += m31;
				// Z41 += m41;
				exP -= 4;
				/* end of inner loop */
			}
			/* compute left-over iterations */
			j += 4;
			for (; j > 0; j--) {
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP];// ex[0];
				p2 = L[ellP - 1];// ell[-1];
				p3 = L[ellP - 2];// ell[-2];
				p4 = L[ellP - 3];// ell[-3];
				/* compute outer product and add it to the Z matrix */
				m11 = p1 * q1;
				m21 = p2 * q1;
				m31 = p3 * q1;
				m41 = p4 * q1;
				ellP += lskip1;
				exP -= 1;
				Z11 += m11;
				Z21 += m21;
				Z31 += m31;
				Z41 += m41;
			}
			/* finish computing the X(i) block */
			Z11 = B[exP] - Z11;// ex[0] - Z11;
			B[exP] = Z11;// ex[0] = Z11;
			p1 = L[ellP - 1];// ell[-1];
			Z21 = B[exP - 1] - Z21 - p1 * Z11;// ex[-1] - Z21 - p1*Z11;
			B[exP - 1] = Z21;// ex[-1] = Z21;
			p1 = L[ellP - 2];// ell[-2];
			p2 = L[ellP - 2 + lskip1];// ell[-2+lskip1];
			Z31 = B[exP - 2] - Z31 - p1 * Z11 - p2 * Z21;// ex[-2] - Z31 -
															// p1*Z11 - p2*Z21;
			B[exP - 2] = Z31;// ex[-2] = Z31;
			p1 = L[ellP - 3];// ell[-3];
			p2 = L[ellP - 3 + lskip1];// ell[-3+lskip1];
			p3 = L[ellP - 3 + lskip2];// ell[-3+lskip2];
			Z41 = B[exP - 3] - Z41 - p1 * Z11 - p2 * Z21 - p3 * Z31;// ex[-3] -
																	// Z41 -
																	// p1*Z11 -
																	// p2*Z21 -
																	// p3*Z31;
			B[exP - 3] = Z41;// ex[-3] = Z41;
			/* end of outer loop */
		}
		/* compute rows at end that are not a multiple of block size */
		for (; i < n; i++) {
			/* compute all 1 x 1 block of X, from rows i..i+1-1 */
			/* set the Z matrix to 0 */
			Z11 = 0;
			ellP = Lp - i;// ell = L - i;
			exP = Bp;// ex = B;
			/* the inner loop that computes outer products and adds them to Z */
			for (j = i - 4; j >= 0; j -= 4) {
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP];// ex[0];
				/* compute outer product and add it to the Z matrix */
				m11 = p1 * q1;
				ellP += lskip1;
				Z11 += m11;
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP - 1];// ex[-1];
				/* compute outer product and add it to the Z matrix */
				m11 = p1 * q1;
				ellP += lskip1;
				Z11 += m11;
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP - 2];// ex[-2];
				/* compute outer product and add it to the Z matrix */
				m11 = p1 * q1;
				ellP += lskip1;
				Z11 += m11;
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP - 3];// ex[-3];
				/* compute outer product and add it to the Z matrix */
				m11 = p1 * q1;
				ellP += lskip1;
				exP -= 4;
				Z11 += m11;
				/* end of inner loop */
			}
			/* compute left-over iterations */
			j += 4;
			for (; j > 0; j--) {
				/* load p and q values */
				p1 = L[ellP];// ell[0];
				q1 = B[exP];// ex[0];
				/* compute outer product and add it to the Z matrix */
				m11 = p1 * q1;
				ellP += lskip1;
				exP -= 1;
				Z11 += m11;
			}
			/* finish computing the X(i) block */
			Z11 = B[exP] - Z11;// ex[0] - Z11;
			B[exP] = Z11;// ex[0] = Z11;
		}
	}

	private static final FastLDLT D_LDLT = new FastLDLT();

	/**
	 * factorize a matrix A into L*D*L', where L is lower triangular with ones
	 * on the diagonal, and D is diagonal. A is an n*n matrix stored by rows,
	 * with a leading dimension of n rounded up to 4. L is written into the
	 * strict lower triangle of A (the ones are not written) and the reciprocal
	 * of the diagonal elements of D are written into d.
	 */
	public static void dFactorLDLT(double[] A, double[] d, int n, int nskip1) {
		D_LDLT.dFactorLDLT(A, d, n, nskip1);
	}

}