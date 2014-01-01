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

import static org.ode4j.ode.OdeMath.*;

/* generated code, do not edit. */

class FastLDLT {

	/** 
	 * solve L*X=B, with B containing 1 right hand sides.
	 * L is an n*n lower triangular matrix with ones on the diagonal.
	 * L is stored by rows and its leading dimension is lskip.
	 * B is an n*1 matrix that contains the right hand sides.
	 * B is stored by columns and its leading dimension is also lskip.
	 * B is overwritten with X.
	 * this processes blocks of 2*2.
	 * if this is in the factorizer source file, n must be a multiple of 2.
	 */
	//private static void dSolveL1_1 (final double[] L, double[] B, int n, int lskip1)
	private static void dSolveL1_1 (final double[] L, int offsetBL, int n, int lskip1)
	{  
		/* declare variables - Z matrix, p and q vectors, etc */
		double Z11,m11,Z21,m21,p1,q1,p2=0;//,ex[];
		int exPos;
		double[] B = L;
		//final double[] ell;
		int ellPos;
		int i,j;
		/* compute all 2 x 1 blocks of X */
		for (i=0; i < n; i+=2) {
			/* compute all 2 x 1 block of X, from rows i..i+2-1 */
			/* set the Z matrix to 0 */
			Z11=0;
			Z21=0;
			//ell = L + i*lskip1;
			ellPos = i*lskip1;
			exPos = offsetBL;//B;
			/* the inner loop that computes outer products and adds them to Z */
			for (j=i-2; j >= 0; j -= 2) {
				/* compute outer product and add it to the Z matrix */
				p1 = L[ellPos];//p1=ell[0];
				q1 = B[exPos];//ex[0];
				m11 = p1 * q1;
				p2 = L[ellPos + lskip1];//p2=ell[lskip1];
				m21 = p2 * q1;
				Z11 += m11;
				Z21 += m21;
				/* compute outer product and add it to the Z matrix */
				p1 = L[ellPos + 1];//p1=ell[1];
				q1 = B[exPos + 1];//ex[1];
				m11 = p1 * q1;
				p2 = L[ellPos + 1 + lskip1];//ell[1+lskip1];
				m21 = p2 * q1;
				/* advance pointers */
				ellPos += 2;
				exPos += 2;
				Z11 += m11;
				Z21 += m21;
				/* end of inner loop */
			}
			/* compute left-over iterations */
			j += 2;
			for (; j > 0; j--) {
				/* compute outer product and add it to the Z matrix */
				p1=L[ellPos];//ell[0];
				q1=B[exPos];//ex[0];
				m11 = p1 * q1;
				p2=L[ellPos + lskip1];//ell[lskip1];
				m21 = p2 * q1;
				/* advance pointers */
				ellPos += 1;
				exPos += 1;
				Z11 += m11;
				Z21 += m21;
			}
			/* finish computing the X(i) block */
			Z11 = B[exPos] - Z11;//Z11 = ex[0] - Z11;
			B[exPos] = Z11;//ex[0] = Z11;
			p1 = L[ellPos + lskip1];//ell[lskip1];
			Z21 = B[exPos +1] - Z21 - p1*Z11;//Z21 = ex[1] - Z21 - p1*Z11;
			B[exPos + 1] = Z21;//ex[1] = Z21;
			/* end of outer loop */
		}
	}



	/**
	 *  solve L*X=B, with B containing 2 right hand sides.
	 * L is an n*n lower triangular matrix with ones on the diagonal.
	 * L is stored by rows and its leading dimension is lskip.
	 * B is an n*2 matrix that contains the right hand sides.
	 * B is stored by columns and its leading dimension is also lskip.
	 * B is overwritten with X.
	 * this processes blocks of 2*2.
	 * if this is in the factorizer source file, n must be a multiple of 2.
	 */
	//private static void dSolveL1_2 (final double[] L, double[] B, int n, int lskip1)
	private static void dSolveL1_2 (final double[] L, int offsetBL , 
			int n, int lskip1)
	{  
		/* declare variables - Z matrix, p and q vectors, etc */
		double Z11,m11,Z12,m12,Z21,m21,Z22,m22,p1,q1,p2,q2; //,ex[];
		int exP;
		double[]B = L;
		//final double []ell;
		int ellP;
		int i,j;
		/* compute all 2 x 2 blocks of X */
		for (i=0; i < n; i+=2) {
			/* compute all 2 x 2 block of X, from rows i..i+2-1 */
			/* set the Z matrix to 0 */
			Z11=0;
			Z12=0;
			Z21=0;
			Z22=0;
			ellP = i*lskip1;//L + i*lskip1;
			exP = offsetBL;
			/* the inner loop that computes outer products and adds them to Z */
			for (j=i-2; j >= 0; j -= 2) {
				/* compute outer product and add it to the Z matrix */
				p1 = L[ellP];//p1=ell[0];
				q1=B[exP];//ex[0];
				m11 = p1 * q1;
				q2=B[exP + lskip1];//ex[lskip1];
				m12 = p1 * q2;
				p2=L[ellP + lskip1];//ell[lskip1];
				m21 = p2 * q1;
				m22 = p2 * q2;
				Z11 += m11;
				Z12 += m12;
				Z21 += m21;
				Z22 += m22;
				/* compute outer product and add it to the Z matrix */
				p1=L[ellP + 1];//ell[1];
				q1=B[exP + 1];//ex[1];
				m11 = p1 * q1;
				q2=B[exP + 1 + lskip1];//ex[1+lskip1];
				m12 = p1 * q2;
				p2=L[ellP + 1 + lskip1];//ell[1+lskip1];
				m21 = p2 * q1;
				m22 = p2 * q2;
				/* advance pointers */
				ellP += 2;
				exP += 2;
				Z11 += m11;
				Z12 += m12;
				Z21 += m21;
				Z22 += m22;
				/* end of inner loop */
			}
			/* compute left-over iterations */
			j += 2;
			for (; j > 0; j--) {
				/* compute outer product and add it to the Z matrix */
				p1=L[ellP];//ell[0];
				q1=B[exP];//ex[0];
				m11 = p1 * q1;
				q2=B[exP + lskip1];//ex[lskip1];
				m12 = p1 * q2;
				p2=L[ellP + lskip1];//ell[lskip1];
				m21 = p2 * q1;
				m22 = p2 * q2;
				/* advance pointers */
				ellP += 1;
				exP += 1;
				Z11 += m11;
				Z12 += m12;
				Z21 += m21;
				Z22 += m22;
			}
			/* finish computing the X(i) block */
			Z11 = B[exP] - Z11;//ex[0] - Z11;
			B[exP] = Z11;//ex[0] = Z11;
			Z12 = B[exP + lskip1] - Z12;//ex[lskip1] - Z12;
			B[exP + lskip1] = Z12;//ex[lskip1] = Z12;
			p1 = L[ellP + lskip1];//ell[lskip1];
			Z21 = B[exP + 1] - Z21 - p1*Z11;//ex[1] - Z21 - p1*Z11;
			B[exP+1] = Z21;//ex[1] = Z21;
			Z22 = B[exP + 1 + lskip1] - Z22 - p1*Z12;//ex[1+lskip1] - Z22 - p1*Z12;
			B[exP + 1 + lskip1] = Z22;//ex[1+lskip1] = Z22;
			/* end of outer loop */
		}
	}


	void dFactorLDLT (double[] A, double []d, int n, int nskip1)
	{  
		int i,j;
		double sum;//,*ell,*dee;
		int ellP, deeP;
		double dd,p1,p2,q1,q2,Z11,m11,Z21,m21,Z22,m22;
		if (n < 1) return;
		
		for (i=0; i<=n-2; i += 2) {
			/* solve L*(D*l)=a, l is scaled elements in 2 x i block at A(i,0) */
			//dSolveL1_2 (A,A+i*nskip1,i,nskip1);
			dSolveL1_2 (A,i*nskip1,i,nskip1);
			/* scale the elements in a 2 x i block at A(i,0), and also */
			/* compute Z = the outer product matrix that we'll need. */
			Z11 = 0;
			Z21 = 0;
			Z22 = 0;
			ellP = i*nskip1; //+A;
			deeP = 0;//d;
			for (j=i-6; j >= 0; j -= 6) {
				for (int k = 0; k < 6; k++) {
					//TODO increment ellP and deeP inside the loop.
					p1 = A[ellP + k];//ell[0];
					p2 = A[ellP + k + nskip1];//ell[nskip1];
					dd = d[deeP + k];//dee[0];
					q1 = p1*dd;
					q2 = p2*dd;
					A[ellP + k] = q1;//ell[0] = q1;
					A[ellP + k + nskip1] = q2;//ell[nskip1] = q2;
					m11 = p1*q1;
					m21 = p2*q1;
					m22 = p2*q2;
					Z11 += m11;
					Z21 += m21;
					Z22 += m22;
				}
				//        p1 = ell[0];
				//        p2 = ell[nskip1];
				//        dd = dee[0];
				//        q1 = p1*dd;
				//        q2 = p2*dd;
				//        ell[0] = q1;
				//        ell[nskip1] = q2;
				//        m11 = p1*q1;
				//        m21 = p2*q1;
				//        m22 = p2*q2;
				//        Z11 += m11;
				//        Z21 += m21;
				//        Z22 += m22;
				//      
				//      p1 = ell[1];
				//      p2 = ell[1+nskip1];
				//      dd = dee[1];
				//      q1 = p1*dd;
				//      q2 = p2*dd;
				//      ell[1] = q1;
				//      ell[1+nskip1] = q2;
				//      m11 = p1*q1;
				//      m21 = p2*q1;
				//      m22 = p2*q2;
				//      Z11 += m11;
				//      Z21 += m21;
				//      Z22 += m22;
				
				//      p1 = ell[2];
				//      p2 = ell[2+nskip1];
				//      dd = dee[2];
				//      q1 = p1*dd;
				//      q2 = p2*dd;
				//      ell[2] = q1;
				//      ell[2+nskip1] = q2;
				//      m11 = p1*q1;
				//      m21 = p2*q1;
				//      m22 = p2*q2;
				//      Z11 += m11;
				//      Z21 += m21;
				//      Z22 += m22;
				//      p1 = ell[3];
				//      p2 = ell[3+nskip1];
				//      dd = dee[3];
				//      q1 = p1*dd;
				//      q2 = p2*dd;
				//      ell[3] = q1;
				//      ell[3+nskip1] = q2;
				//      m11 = p1*q1;
				//      m21 = p2*q1;
				//      m22 = p2*q2;
				//      Z11 += m11;
				//      Z21 += m21;
				//      Z22 += m22;
				//      p1 = ell[4];
				//      p2 = ell[4+nskip1];
				//      dd = dee[4];
				//      q1 = p1*dd;
				//      q2 = p2*dd;
				//      ell[4] = q1;
				//      ell[4+nskip1] = q2;
				//      m11 = p1*q1;
				//      m21 = p2*q1;
				//      m22 = p2*q2;
				//      Z11 += m11;
				//      Z21 += m21;
				//      Z22 += m22;
				//      p1 = ell[5];
				//      p2 = ell[5+nskip1];
				//      dd = dee[5];
				//      q1 = p1*dd;
				//      q2 = p2*dd;
				//      ell[5] = q1;
				//      ell[5+nskip1] = q2;
				//      m11 = p1*q1;
				//      m21 = p2*q1;
				//      m22 = p2*q2;
				//      Z11 += m11;
				//      Z21 += m21;
				//      Z22 += m22;
				//      
				ellP += 6;
				deeP += 6;
			}
			/* compute left-over iterations */
			j += 6;
			for (; j > 0; j--) {
				p1 = A[ellP];//ell[0];
				p2 = A[ellP + nskip1];//ell[nskip1];
				dd = d[deeP];//dee[0];
				q1 = p1*dd;
				q2 = p2*dd;
				A[ellP] = q1;//ell[0] = q1;
				A[ellP + nskip1] = q2;//ell[nskip1] = q2;
				m11 = p1*q1;
				m21 = p2*q1;
				m22 = p2*q2;
				Z11 += m11;
				Z21 += m21;
				Z22 += m22;
				ellP++;
				deeP++;
			}
			/* solve for diagonal 2 x 2 block at A(i,i) */
			Z11 = A[ellP] - Z11;//ell[0] - Z11;
			Z21 = A[ellP + nskip1] - Z21;//ell[nskip1] - Z21;
			Z22 = A[ellP + 1 + nskip1] - Z22;//ell[1+nskip1] - Z22;
			deeP = i;// dee = d + i;
			/* factorize 2 x 2 block Z,dee */
			/* factorize row 1 */
			d[deeP] = dRecip(Z11);//dee[0] = dRecip(Z11);
			/* factorize row 2 */
			sum = 0;
			q1 = Z21;
			q2 = q1 * d[deeP];//dee[0];
			Z21 = q2;
			sum += q1*q2;
			d[deeP + 1] = dRecip(Z22 - sum);//dee[1] = dRecip(Z22 - sum);
			/* done factorizing 2 x 2 block */
			A[ellP + nskip1] = Z21;//ell[nskip1] = Z21;
		}
		/* compute the (less than 2) rows at the bottom */
		switch (n-i) {
		case 0:
			break;

		case 1:
			//dSolveL1_1 (A,A+i*nskip1,i,nskip1);
			dSolveL1_1 (A,i*nskip1,i,nskip1);
			/* scale the elements in a 1 x i block at A(i,0), and also */
			/* compute Z = the outer product matrix that we'll need. */
			Z11 = 0;
			ellP = i*nskip1;//ell = A+i*nskip1;
			deeP = 0;//dee = d;
			for (j=i-6; j >= 0; j -= 6) {
				for (int k = 0; k < 6; k++) {
					p1 = A[ellP + k];//ell[0];
					dd = d[deeP + k];//dd = dee[0];
					q1 = p1*dd;
					A[ellP + k] = q1; //ell[0] = q1;
					m11 = p1*q1;
					Z11 += m11;
				}
//				p1 = ell[0];
//				dd = dee[0];
//				q1 = p1*dd;
//				ell[0] = q1;
//				m11 = p1*q1;
//				Z11 += m11;
//
//				p1 = ell[1];
//				dd = dee[1];
//				q1 = p1*dd;
//				ell[1] = q1;
//				m11 = p1*q1;
//				Z11 += m11;
//				p1 = ell[2];
//				dd = dee[2];
//				q1 = p1*dd;
//				ell[2] = q1;
//				m11 = p1*q1;
//				Z11 += m11;
//				p1 = ell[3];
//				dd = dee[3];
//				q1 = p1*dd;
//				ell[3] = q1;
//				m11 = p1*q1;
//				Z11 += m11;
//				p1 = ell[4];
//				dd = dee[4];
//				q1 = p1*dd;
//				ell[4] = q1;
//				m11 = p1*q1;
//				Z11 += m11;
//				p1 = ell[5];
//				dd = dee[5];
//				q1 = p1*dd;
//				ell[5] = q1;
//				m11 = p1*q1;
//				Z11 += m11;
				ellP += 6;
				deeP += 6;
			}
			/* compute left-over iterations */
			j += 6;
			for (; j > 0; j--) {
				p1 = A[ellP];//ell[0];
				dd = d[deeP];//dee[0];
				q1 = p1*dd;
				A[ellP] = q1;//ell[0] = q1;
				m11 = p1*q1;
				Z11 += m11;
				ellP++;
				deeP++;
			}
			/* solve for diagonal 1 x 1 block at A(i,i) */
			Z11 = A[ellP] -Z11;//ell[0] - Z11;
			deeP = i;//dee = d + i;
			/* factorize 1 x 1 block Z,dee */
			/* factorize row 1 */
			d[deeP] = dRecip(Z11);//dee[0] = dRecip(Z11);
			/* done factorizing 1 x 1 block */
			break;

		default: throw new IllegalStateException();//*((char*)0)=0;  /* this should never happen! */
		}
	}
}
