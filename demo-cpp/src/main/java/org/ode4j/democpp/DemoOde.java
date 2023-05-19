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
package org.ode4j.democpp;

import static org.ode4j.cpp.internal.ApiCppMass.dMassRotate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCapsule;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetParameters;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetZero;
import static org.ode4j.cpp.internal.ApiCppMass.dMassTranslate;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.cpp4j.Csetjmp.longjmp;
import static org.ode4j.ode.internal.cpp4j.Csetjmp.setjmp;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.sprintf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.vprintf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.vsprintf;
import static org.ode4j.ode.internal.cpp4j.Cstring.memcpy;
import static org.ode4j.ode.internal.cpp4j.Cstring.strcmp;
import static org.ode4j.ode.internal.cpp4j.Cstring.strlen;
import static org.ode4j.ode.internal.ErrorHandler.*;

import java.util.ArrayList;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DMass;
import org.ode4j.ode.internal.DLCP;
import org.ode4j.ode.internal.DxMass;
import org.ode4j.ode.internal.ErrorHandler.dMessageFunction;
import org.ode4j.ode.internal.cpp4j.Csetjmp.jmp_buf;
import org.ode4j.ode.internal.cpp4j.java.CppLongJump;
import org.ode4j.ode.internal.cpp4j.java.Ref;

class DemoOde {

	//****************************************************************************
	// matrix accessors

	//#define _A(i,j) A[(i)*4+(j)]
	//#define _I(i,j) I[(i)*4+(j)]
	//#define _R(i,j) R[(i)*4+(j)]
//	private final int _A(int i, int j) { return i*4+j; }
//	private final int _I(int i, int j) { return i*4+j; }
//	private final int _R(int i, int j) { return i*4+j; }

	//****************************************************************************
	// tolerances

	//#ifdef dDOUBLE
	private static final double tol = 1e-10;
	//#endif
	//
	//#ifdef dSINGLE
	//const double tol = 1e-5;
	//#endif

	//****************************************************************************
	// misc messages and error handling

	//#ifdef __GNUC__
	//#define HEADER printf ("%s()\n", __FUNCTION__);
	//#else
	//#define HEADER printf ("%s:%d\n",__FILE__,__LINE__);
	//#endif
	private final void HEADER() {
		//printf(args);
		System.out.println(new RuntimeException().getStackTrace()[1]);
	}

	private static jmp_buf jump_buffer = new jmp_buf();


	private static final dMessageFunction myMessageFunction = new dMessageFunction() {

		@Override
		public void call(int num, String msg, Object... args) {
			printf ("(Message %d: ",num);
			vprintf (msg,args);
			printf (")");
			dSetMessageHandler (null);
			longjmp (jump_buffer,1);
		}
	};
//	void myMessageFunction (int num, final String msg, Object ... ap)
//	{
//	}


	//#define TRAP_MESSAGE(do,ifnomsg,ifmsg) \
	//	dSetMessageHandler (&myMessageFunction); 
	//	if (setjmp (jump_buffer)) { 
	//		dSetMessageHandler (0); 
	//		ifmsg ; 
	//	} 
	//	else { 
	//		dSetMessageHandler (&myMessageFunction); 
	//		do ; 
	//		ifnomsg ; 
	//	} 
	//	dSetMessageHandler (0);
	//}
	private void TRAP_MESSAGE(Object obj, String mName, Object[] params,
			String ifnomsg, String ifmsg) {
		dSetMessageHandler (myMessageFunction); 
//		if (setjmp (jump_buffer) != 0) { 
//			dSetMessageHandler (null); 
//			printf(ifmsg);//ifmsg ; 
//		} 
//		else { 
//			dSetMessageHandler (myMessageFunction); 
//			execute(obj, mName, params); 
//			printf(ifnomsg);//ifnomsg ; 
//		}
		try {
			execute(obj, mName, params); 
			printf(ifnomsg);//ifnomsg ; 
		} catch (Exception e) {
			printf(ifmsg);//ifmsg ; 
			e.printStackTrace();
		}
		dSetMessageHandler (null);
	}

	private void execute(Object obj, String m, Object[] args) {
		if (m.equals("dMassSetParameters")) {
			dMassSetParameters((DxMass)args[0], 
					(Double)args[1], (Double)args[2], 
					(Double)args[3], (Double)args[4], 
					(Double)args[5], (Double)args[6], 
					(Double)args[7], (Double)args[8], 
					(Double)args[9], (Double)args[10]);
		} else {
			throw new UnsupportedOperationException(m);
		}
	}
	
	//****************************************************************************
	// utility stuff

	// compare two numbers, within a threshhold, return 1 if approx equal

	boolean cmp (double a, double b)
	{
		return Math.abs(a-b) < tol;
	}

	//****************************************************************************
	// matrix utility stuff

	// compare a 3x3 matrix with the identity

	boolean cmpIdentityMat3 (DMatrix3 A)
	{
		return
		(cmp(A.get00(),1.0) && cmp(A.get01(),0.0) && cmp(A.get02(),0.0) &&
				cmp(A.get10(),0.0) && cmp(A.get11(),1.0) && cmp(A.get12(),0.0) &&
				cmp(A.get20(),0.0) && cmp(A.get21(),0.0) && cmp(A.get22(),1.0));
	}


	// transpose a 3x3 matrix in-line

	void transpose3x3 (DMatrix3 A)
	{
		A.eqTranspose();
//		double tmp;
//		tmp=A.v[4]; A.v[4]=A.v[1]; A.v[1]=tmp;
//		tmp=A.v[8]; A.v[8]=A.v[2]; A.v[2]=tmp;
//		tmp=A.v[9]; A.v[9]=A.v[6]; A.v[6]=tmp;
	}

	//****************************************************************************
	// test miscellaneous math functions

	void testRandomNumberGenerator()
	{
		HEADER();
		if (dTestRand()) printf ("\tpassed\n");
		else printf ("\tFAILED\n");
	}


	@SuppressWarnings("unused")
	void testInfinity()
	{
		HEADER();
		if (1e10 < dInfinity && -1e10 > -dInfinity && -dInfinity < dInfinity)
			printf ("\tpassed\n");
		else printf ("\tFAILED\n");
	}


	void testPad()
	{
		HEADER();
		char[] s = new char[100];
		s[0]=0;
		for (int i=0; i<=16; i++) sprintf (s,strlen(s),"%d ",dPAD(i));
		printf ("\t%s\n", strcmp(s,"0 1 4 4 4 8 8 8 8 12 12 12 12 16 16 16 16 ") == 1?
				"FAILED" : "passed");
	}


	void testCrossProduct()
	{
		HEADER();

		DVector3 a1 = new DVector3(),a2 = new DVector3();
		DVector3 b = new DVector3(),c = new DVector3();
		DMatrix3 B = new DMatrix3();
		dMakeRandomVector (b,1.0);
		dMakeRandomVector (c,1.0);

		dCalcVectorCross3 (a1,b,c);

		//B.dSetZero();//dSetZero (B,12);
		dSetCrossMatrixPlus (B,b);
		dMultiply0 (a2,B,c);

		double diff = dMaxDifference(a1,a2);
		printf ("\t%s\n", diff > tol ? "FAILED" : "passed");
	}


	void testSetZero()
	{
		HEADER();
		double[] a = new double[100];
		dMakeRandomVector (a,100,1.0);
		dSetZero (a);
		for (int i=0; i<100; i++) if (a[i] != 0.0) {
			printf ("\tFAILED\n");
			return;
		}
		printf ("\tpassed\n");
	}


	void testNormalize3()
	{
		HEADER();
		int i,j,bad=0;
		DVector3 n1 = new DVector3(),n2 = new DVector3();
		for (i=0; i<1000; i++) {
			dMakeRandomVector (n1,1.0);
			for (j=0; j<3; j++) n2.set(j, n1.get(j));
			dNormalize3 (n2);
			if (dFabs(dCalcVectorDot3(n2,n2) - 1.0) > tol) bad |= 1;
			if (dFabs(n2.get0()/n1.get0() - n2.get1()/n1.get1()) > tol) bad |= 2;
			if (dFabs(n2.get0()/n1.get0() - n2.get2()/n1.get2()) > tol) bad |= 4;
			if (dFabs(n2.get1()/n1.get1() - n2.get2()/n1.get2()) > tol) bad |= 8;
			if (dFabs(dCalcVectorDot3(n2,n1) - dSqrt(dCalcVectorDot3(n1,n1))) > tol) bad |= 16;
			if (bad != 0) {
				printf ("\tFAILED (code=%x)\n",bad);
				return;
			}
		}
		printf ("\tpassed\n");
	}


	/*
void testReorthonormalize()
{
  HEADER;
  dMatrix3 R,I;
  dMakeRandomMatrix (R,3,3,1.0);
  for (int i=0; i<30; i++) dReorthonormalize (R);
  dMultiply2 (I,R,R,3,3,3);
  printf ("\t%s\n",cmpIdentityMat3 (I) ? "passed" : "FAILED");
}
	 */


	void testPlaneSpace()
	{
		HEADER();
		DVector3 n = new DVector3(),p = new DVector3(),q = new DVector3();
		int bad = 0;
		for (int i=0; i<1000; i++) {
			dMakeRandomVector (n,1.0);
			dNormalize3 (n);
			dPlaneSpace (n,p,q);
			if (Math.abs(dCalcVectorDot3(n,p)) > tol) bad = 1;
			if (Math.abs(dCalcVectorDot3(n,q)) > tol) bad = 1;
			if (Math.abs(dCalcVectorDot3(p,q)) > tol) bad = 1;
			if (Math.abs(dCalcVectorDot3(p,p)-1) > tol) bad = 1;
			if (Math.abs(dCalcVectorDot3(q,q)-1) > tol) bad = 1;
		}
		printf ("\t%s\n", bad != 0 ? "FAILED" : "passed");
	}

	//****************************************************************************
	// test matrix functions

	//#define MSIZE 21
	//#define MSIZE4 24	// MSIZE rounded up to 4
	private static final int MSIZE = 21;
	private static final int MSIZE4 = 24;	// MSIZE rounded up to 4


	void testMatrixMultiply()
	{
		// A is 2x3, B is 3x4, B2 is B except stored columnwise, C is 2x4
		double[] A = new double[8],B = new double[12];
		double[] A2 = new double[12],B2 = new double[16],C = new double[8];
		int i;

		HEADER();
		dSetZero (A);
		for (i=0; i<3; i++) A[i] = i+2;
		for (i=0; i<3; i++) A[i+4] = i+3+2;
		for (i=0; i<12; i++) B[i] = i+8;
		dSetZero (A2);
		for (i=0; i<6; i++) A2[i+2*(i/2)] = A[i+i/3];
		dSetZero (B2);
		for (i=0; i<12; i++) B2[i+i/3] = B[i];

		dMultiply0 (C,A,B,2,3,4);
		if (C[0] != 116 || C[1] != 125 || C[2] != 134 || C[3] != 143 ||
				C[4] != 224 || C[5] != 242 || C[6] != 260 || C[7] != 278)
			printf ("\tFAILED (1)\n"); else printf ("\tpassed (1)\n");

		dMultiply1 (C,A2,B,2,3,4);
		if (C[0] != 160 || C[1] != 172 || C[2] != 184 || C[3] != 196 ||
				C[4] != 196 || C[5] != 211 || C[6] != 226 || C[7] != 241)
			printf ("\tFAILED (2)\n"); else printf ("\tpassed (2)\n");

		dMultiply2 (C,A,B2,2,3,4);
		if (C[0] != 83 || C[1] != 110 || C[2] != 137 || C[3] != 164 ||
				C[4] != 164 || C[5] != 218 || C[6] != 272 || C[7] != 326)
			printf ("\tFAILED (3)\n"); else printf ("\tpassed (3)\n");
	}


	void testSmallMatrixMultiply()
	{
		DMatrix3 A = new DMatrix3(),B = new DMatrix3();
		DMatrix3 C = new DMatrix3(),A2 = new DMatrix3();
		DVector3 a = new DVector3(),a2 = new DVector3(),x = new DVector3();

		HEADER();
		dMakeRandomMatrix (A,1.0);
		dMakeRandomMatrix (B,1.0);
		dMakeRandomMatrix (C,1.0);
		dMakeRandomVector (x,1.0);

		// dMULTIPLY0_331()
		dMultiply0_331 (a,B,x);
		dMultiply0 (a2,B,x);
		printf ("\t%s (1)\n",(dMaxDifference (a,a2) > tol) ? "FAILED" :
		"passed");

		// dMULTIPLY1_331()
		dMultiply1_331 (a,B,x);
		dMultiply1 (a2,B,x);
		printf ("\t%s (2)\n",(dMaxDifference (a,a2) > tol) ? "FAILED" :
		"passed");

		// dMULTIPLY0_133
		dMultiply0_133 (a,x,B);
		dMultiply0 (a2,x,B);
		printf ("\t%s (3)\n",(dMaxDifference (a,a2) > tol) ? "FAILED" :
		"passed");

		// dMULTIPLY0_333()
		dMultiply0_333 (A,B,C);
		dMultiply0 (A2,B,C);
		printf ("\t%s (4)\n",(dMaxDifference (A,A2) > tol) ? "FAILED" :
		"passed");

		// dMULTIPLY1_333()
		dMultiply1_333 (A,B,C);
		dMultiply1 (A2,B,C);
		printf ("\t%s (5)\n",(dMaxDifference (A,A2) > tol) ? "FAILED" :
		"passed");

		// dMULTIPLY2_333()
		dMultiply2_333 (A,B,C);
		dMultiply2 (A2,B,C);
		printf ("\t%s (6)\n",(dMaxDifference (A,A2) > tol) ? "FAILED" :
		"passed");
	}


	void testCholeskyFactorization()
	{
		double[] A = new double[MSIZE4*MSIZE], B = new double[MSIZE4*MSIZE];
		double[] C = new double[MSIZE4*MSIZE];
		double diff;
		HEADER();
		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (B,A,A,MSIZE,MSIZE,MSIZE);
		memcpy (A,B,MSIZE4*MSIZE);//*sizeof(double));
		if (dFactorCholesky (B,MSIZE)) printf ("\tpassed (1)\n");
		else printf ("\tFAILED (1)\n");
		dClearUpperTriangle (B,MSIZE);
		dMultiply2 (C,B,B,MSIZE,MSIZE,MSIZE);
		diff = dMaxDifference(A,C,MSIZE,MSIZE);
		printf ("\tmaximum difference = %.6e - %s (2)\n",diff,
				diff > tol ? "FAILED" : "passed");
	}


	void testCholeskySolve()
	{
		double[] A = new double[MSIZE4*MSIZE], L = new double[MSIZE4*MSIZE];
		double[] b = new double[MSIZE],x = new double[MSIZE],btest = new double[MSIZE];
		double diff;
		HEADER();

		// get A,L = PD matrix
		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
		memcpy (A,L,MSIZE4*MSIZE);//*sizeof(double));

		// get b,x = right hand side
		dMakeRandomMatrix (b,MSIZE,1,1.0);
		memcpy (x,b,MSIZE);//*sizeof(double));

		// factor L
		if (dFactorCholesky (L,MSIZE)) printf ("\tpassed (1)\n");
		else printf ("\tFAILED (1)\n");
		dClearUpperTriangle (L,MSIZE);

		// solve A*x = b
		dSolveCholesky (L,x,MSIZE);

		// compute A*x and compare it with b
		dMultiply2 (btest,A,x,MSIZE,MSIZE,1);
		diff = dMaxDifference(b,btest,MSIZE,1);
		printf ("\tmaximum difference = %.6e - %s (2)\n",diff,
				diff > tol ? "FAILED" : "passed");
	}


	void testInvertPDMatrix()
	{
		int i,j,ok;
		double[] A = new double[MSIZE4*MSIZE], Ainv = new double[MSIZE4*MSIZE]; 
		double[] I = new double[MSIZE4*MSIZE];
		HEADER();

		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (Ainv,A,A,MSIZE,MSIZE,MSIZE);
		memcpy (A,Ainv,MSIZE4*MSIZE);//*sizeof(double));
		dSetZero (Ainv);

		if (dInvertPDMatrix (A,Ainv,MSIZE))
			printf ("\tpassed (1)\n"); else printf ("\tFAILED (1)\n");
		dMultiply0 (I,A,Ainv,MSIZE,MSIZE,MSIZE);

		// compare with identity
		ok = 1;
		for (i=0; i<MSIZE; i++) {
			for (j=0; j<MSIZE; j++) {
				if (i != j) if (cmp (I[i*MSIZE4+j],0.0)==false) ok = 0;
			}
		}
		for (i=0; i<MSIZE; i++) {
			if (cmp (I[i*MSIZE4+i],1.0)==false) ok = 0;
		}
		if (ok != 0) printf ("\tpassed (2)\n"); else printf ("\tFAILED (2)\n");
	}


	void testIsPositiveDefinite()
	{
		double[] A = new double[MSIZE4*MSIZE], B = new double[MSIZE4*MSIZE];
		HEADER();
		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (B,A,A,MSIZE,MSIZE,MSIZE);
		printf ("\t%s\n",dIsPositiveDefinite(A,MSIZE) ? "FAILED (1)":"passed (1)");
		printf ("\t%s\n",dIsPositiveDefinite(B,MSIZE) ? "passed (2)":"FAILED (2)");
	}

	void testFastLDLTFactorization()
	{
		int i,j;
		double[] A = new double[MSIZE4*MSIZE], L = new double[MSIZE4*MSIZE];
		double[] DL = new double[MSIZE4*MSIZE];
		double[] ATEST = new double[MSIZE4*MSIZE], d = new double[MSIZE];
		double diff;
		HEADER();
		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
		memcpy (A,L,MSIZE4*MSIZE);//*sizeof(double));
		
		dFactorLDLT (L,d,MSIZE,MSIZE4);
		dClearUpperTriangle (L,MSIZE);
		for (i=0; i<MSIZE; i++) L[i*MSIZE4+i] = 1.0;

		dSetZero (DL);
		for (i=0; i<MSIZE; i++) {
			for (j=0; j<MSIZE; j++) DL[i*MSIZE4+j] = L[i*MSIZE4+j] / d[j];
		}

		dMultiply2 (ATEST,L,DL,MSIZE,MSIZE,MSIZE);
		diff = dMaxDifference(A,ATEST,MSIZE,MSIZE);
		printf ("\tmaximum difference = %.6e - %s\n",diff,
				diff > tol ? "FAILED" : "passed");
	}


	void testSolveLDLT()
	{
		double[] A = new double[MSIZE4*MSIZE], L = new double[MSIZE4*MSIZE];
		double[] d = new double[MSIZE], x = new double[MSIZE];
		double[] b = new double[MSIZE], btest = new double[MSIZE];
		double diff;
		HEADER();
		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
		memcpy (A,L,MSIZE4*MSIZE); //*sizeof(double));

		dMakeRandomMatrix (b,MSIZE,1,1.0);
		memcpy (x,b,MSIZE);//*sizeof(double));

		dFactorLDLT (L,d,MSIZE,MSIZE4);
		dSolveLDLT (L,d,x,MSIZE,MSIZE4);

		dMultiply2 (btest,A,x,MSIZE,MSIZE,1);
		diff = dMaxDifference(b,btest,MSIZE,1);
		printf ("\tmaximum difference = %.6e - %s\n",diff,
				diff > tol ? "FAILED" : "passed");
	}


	void testLDLTAddTL()
	{
		int i,j;
		double[] A = new double[MSIZE4*MSIZE], L = new double[MSIZE4*MSIZE]; 
		double[] d = new double[MSIZE], a = new double[MSIZE];
		double[] DL = new double[MSIZE4*MSIZE], ATEST = new double[MSIZE4*MSIZE];
		double diff;
		HEADER();

		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
		memcpy (A,L,MSIZE4*MSIZE);//*sizeof(double));
		dFactorLDLT (L,d,MSIZE,MSIZE4);

		// delete first row and column of factorization
		for (i=0; i<MSIZE; i++) a[i] = -A[i*MSIZE4];
		a[0] += 1;
		dLDLTAddTL (L,d,a,MSIZE,MSIZE4);
		for (i=1; i<MSIZE; i++) L[i*MSIZE4] = 0;
		d[0] = 1;

		// get modified L*D*L'
		dClearUpperTriangle (L,MSIZE);
		for (i=0; i<MSIZE; i++) L[i*MSIZE4+i] = 1.0;
		dSetZero (DL);
		for (i=0; i<MSIZE; i++) {
			for (j=0; j<MSIZE; j++) DL[i*MSIZE4+j] = L[i*MSIZE4+j] / d[j];
		}
		dMultiply2 (ATEST,L,DL,MSIZE,MSIZE,MSIZE);

		// compare it to A with its first row/column removed
		for (i=1; i<MSIZE; i++) A[i*MSIZE4] = A[i] = 0;
		A[0] = 1;
		diff = dMaxDifference(A,ATEST,MSIZE,MSIZE);
		printf ("\tmaximum difference = %.6e - %s\n",diff,
				diff > tol ? "FAILED" : "passed");
	}


	void testLDLTRemove()
	{
		int i,j,r;
		int[] p = new int[MSIZE];
		double[] A = new double[MSIZE4*MSIZE];
		double[] L = new double[MSIZE4*MSIZE];
		double[] d = new double[MSIZE];
		double[] L2 = new double[MSIZE4*MSIZE];
		double[] d2 = new double[MSIZE];
		double[] DL2 = new double[MSIZE4*MSIZE];
		double[] Atest1 = new double[MSIZE4*MSIZE];
		double[] Atest2 = new double [MSIZE4*MSIZE];
		double diff, maxdiff;
		HEADER();

		// make array of A row pointers
		double[] Arows = new double[MSIZE];
		for (i=0; i<MSIZE; i++) Arows[i] = A[i*MSIZE4];//+i*MSIZE4;

		// fill permutation vector
		for (i=0; i<MSIZE; i++) p[i]=i;

		dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
		dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
		memcpy (A,L,MSIZE4*MSIZE);//*sizeof(double));
		dFactorLDLT (L,d,MSIZE,MSIZE4);

		maxdiff = 1e10;
		for (r=0; r<MSIZE; r++) {
			// get Atest1 = A with row/column r removed
			memcpy (Atest1,A,MSIZE4*MSIZE);//*sizeof(double));
			dRemoveRowCol (Atest1,MSIZE,MSIZE4,r);

			// test that the row/column removal worked
			int bad = 0;
			for (i=0; i<MSIZE; i++) {
				for (j=0; j<MSIZE; j++) {
					if (i != r && j != r) {
						int ii = i;
						int jj = j;
						if (ii >= r) ii--;
						if (jj >= r) jj--;
						if (A[i*MSIZE4+j] != Atest1[ii*MSIZE4+jj]) bad = 1;
					}
				}
			}
			if (bad != 0) printf ("\trow/col removal FAILED for row %d\n",r);

			// zero out last row/column of Atest1
			for (i=0; i<MSIZE; i++) {
				Atest1[(MSIZE-1)*MSIZE4+i] = 0;
				Atest1[i*MSIZE4+MSIZE-1] = 0;
			}    

			// get L2*D2*L2' = adjusted factorization to remove that row
			memcpy (L2,L,MSIZE4*MSIZE);//*sizeof(double));
			memcpy (d2,d,MSIZE);//*sizeof(double));
			//dLDLTRemove (/*A*/ Arows,p,L2,d2,MSIZE,MSIZE,r,MSIZE4);
			//TZ
			dLDLTRemove (A,p,L2,d2,MSIZE,MSIZE,r,MSIZE4);

			// get Atest2 = L2*D2*L2'
			dClearUpperTriangle (L2,MSIZE);
			for (i=0; i<(MSIZE-1); i++) L2[i*MSIZE4+i] = 1.0;
			for (i=0; i<MSIZE; i++) L2[(MSIZE-1)*MSIZE4+i] = 0;
			d2[MSIZE-1] = 1;
			dSetZero (DL2);
			for (i=0; i<(MSIZE-1); i++) {
				for (j=0; j<MSIZE-1; j++) DL2[i*MSIZE4+j] = L2[i*MSIZE4+j] / d2[j];
			}

			dMultiply2 (Atest2,L2,DL2,MSIZE,MSIZE,MSIZE);

			diff = dMaxDifference(Atest1,Atest2,MSIZE,MSIZE);
			if (diff < maxdiff) maxdiff = diff;

			/*
    dPrintMatrix (Atest1,MSIZE,MSIZE);
    printf ("\n");
    dPrintMatrix (Atest2,MSIZE,MSIZE);
    printf ("\n");
			 */
		}
		printf ("\tmaximum difference = %.6e - %s\n",maxdiff,
				maxdiff > tol ? "FAILED" : "passed");
	}

	//****************************************************************************
	// test mass stuff

	//#define NUMP 10		// number of particles
	private static final int NUMP = 10;		// number of particles

	void printMassParams (DMass m)
	{
		printf ("mass = %.4f\n",m.getMass());
		printf ("com  = (%.4f,%.4f,%.4f)\n",m.getC().get0(),m.getC().get1(),m.getC().get2());
		DMatrix3C I = m.getI();
		printf ("I    = [ %10.4f %10.4f %10.4f ]\n" +
				"       [ %10.4f %10.4f %10.4f ]\n" +
				"       [ %10.4f %10.4f %10.4f ]\n",
				I.get00(),I.get01(),I.get02(),
				I.get10(),I.get11(),I.get12(),
				I.get20(),I.get21(),I.get22());
	}


	//void compareMassParams (dMass *m1, dMass *m2, const char *msg)
	void compareMassParams (DMass m1, DMass m2, final String msg)
	{
		int i,j;
		boolean ok = true;
		if (!(cmp(m1.getMass(),m2.getMass()) && cmp(m1.getC().get0(),m2.getC().get0()) &&
				cmp(m1.getC().get1(),m2.getC().get1()) && cmp(m1.getC().get2(),m2.getC().get2())))
			ok = false;
		for (i=0; i<3; i++) for (j=0; j<3; j++)
			if (cmp (m1.getI().get(i,j),m2.getI().get(i,j))==false) ok = false;
		if (ok) printf ("\tpassed (%s)\n",msg); 
		else printf ("\tFAILED (%s)\n",msg);
	}


	// compute the mass parameters of a particle set

	//void computeMassParams (dMass *m, dReal q[NUMP][3], dReal pm[NUMP])
	void computeMassParams (DMass m, DVector3[] q, double[] pm) {
		dIASSERT(q.length==NUMP && pm.length==NUMP);

		int i;
		dMassSetZero (m);
		for (i=0; i<NUMP; i++) {
			m.setMass( m.getMass() + pm[i]);// += pm[i];
			//for (j=0; j<3; j++) m.getC().v[j] += pm[i]*q[i][j];
			DVector3 cTmp = new DVector3(m.getC()); 
			m.setC( cTmp.add(pm[i]*q[i].get0(), pm[i]*q[i].get1(), pm[i]*q[i].get2()) );
			DMatrix3 I = new DMatrix3(m.getI());
			I.add(0,0, pm[i]*(q[i].get1()*q[i].get1() + q[i].get2()*q[i].get2()) );
			I.add(1,1, pm[i]*(q[i].get0()*q[i].get0() + q[i].get2()*q[i].get2()) );
			I.add(2,2, pm[i]*(q[i].get0()*q[i].get0() + q[i].get1()*q[i].get1()) );
			I.add(0,1, -pm[i]*(q[i].get0()*q[i].get1()) );
			I.add(0,2, -pm[i]*(q[i].get0()*q[i].get2()) );
			I.add(1,2, -pm[i]*(q[i].get1()*q[i].get2()) );
			m.setI(I);
		}
		//for (j=0; j<3; j++) m.getC().v[j] /= m.getMass();
		DVector3 cTmp = new DVector3(m.getC());
		m.setC( cTmp.scale(1./m.getMass()) );
		DMatrix3 I = new DMatrix3(m.getI());
		I.set(1,0, I.get(0,1));
		I.set(2,0, I.get(0,2));
		I.set(2,1, I.get(1,2));
		m.setI(I);
	}


	void testMassFunctions()
	{
		DMass m = new DxMass();
		int i,j;
		//  double q[NUMP][3];		// particle positions
		DVector3[] q = new DVector3[NUMP];		// particle positions
		//  double pm[NUMP];		// particle masses
		double[] pm = new double[NUMP];		// particle masses
		DMass m1 = new DxMass(),m2 = new DxMass();
		DMatrix3 R = new DMatrix3();

		HEADER();

		printf ("\t");
		dMassSetZero (m);
//		TRAP_MESSAGE (m.dMassSetParameters (m,10, 0,0,0, 1,2,3, 4,5,6),
//		TRAP_MESSAGE (m, "dMassSetParameters", new Object[]{m,10., 0.,0.,0., 1.,2.,3., 4.,5.,6.},
//				" FAILED (1)\n", " passed (1)\n");
		//TZ: replaces TRAP_MESSAGE
		dSetMessageHandler (myMessageFunction); 
		try {
			dMassSetParameters (m,10, 0,0,0, 1,2,3, 4,5,6);
			printf(" FAILED (1)\n");//ifnomsg ; 
		} catch (CppLongJump e) {
			printf(" passed (1)\n");//ifmsg ; 
		}
		dSetMessageHandler (null);

		printf ("\t");
		dMassSetZero (m);
//		TRAP_MESSAGE (m.dMassSetParameters (m,10, 0.1,0.2,0.15, 3,5,14, 3.1,3.2,4),
//				printf (" passed (2)\n") , printf (" FAILED (2)\n"));
		TRAP_MESSAGE (m, "dMassSetParameters", new Object[]{m,10., 0.1,0.2,0.15, 3.,5.,14., 3.1,3.2,4.},
				" passed (2)\n" , " FAILED (2)\n");
		DMatrix3 I =(DMatrix3) m.getI();
		if (m.getMass()==10 && m.getC().get0()==0.1 && m.getC().get1()==0.2 &&
				m.getC().get2()==0.15 && I.get00()==3 && I.get11()==5 && I.get22()==14 &&
				I.get01()==3.1 && I.get02()==3.2 && I.get12()==4 &&
				I.get10()==3.1 && I.get20()==3.2 && I.get21()==4)
			printf ("\tpassed (3)\n"); else printf ("\tFAILED (3)\n");

		dMassSetZero (m);
		dMassSetSphere (m,1.4, 0.86);
		I = (DMatrix3)m.getI();
		if (cmp(m.getMass(),3.73002719949386) && m.getC().get0()==0 && m.getC().get1()==0 && m.getC().get2()==0 &&
				cmp(I.get00(),1.10349124669826) &&
				cmp(I.get11(),1.10349124669826) &&
				cmp(I.get22(),1.10349124669826) &&
				I.get01()==0 && I.get02()==0 && I.get12()==0 &&
				I.get10()==0 && I.get20()==0 && I.get21()==0)
			printf ("\tpassed (4)\n"); else printf ("\tFAILED (4)\n");

		dMassSetZero (m);
		dMassSetCapsule (m,1.3,1,0.76,1.53);
		I = (DMatrix3)m.getI();
		if (cmp(m.getMass(),5.99961928996029) && m.getC().get0()==0 && m.getC().get1()==0 && m.getC().get2()==0 &&
				cmp(I.get00(),1.59461986077384) &&
				cmp(I.get11(),4.21878433864904) &&
				cmp(I.get22(),4.21878433864904) &&
				I.get01()==0 && I.get02()==0 && I.get12()==0 &&
				I.get10()==0 && I.get20()==0 && I.get21()==0)
			printf ("\tpassed (5)\n"); 
		else {
			printf ("\tFAILED (5)\n");
		}

		dMassSetZero (m);
		dMassSetBox (m,0.27,3,4,5);
		I = (DMatrix3)m.getI();
		if (cmp(m.getMass(),16.2) && m.getC().get0()==0 && m.getC().get1()==0 && m.getC().get2()==0 &&
				cmp(I.get00(),55.35) && cmp(I.get11(),45.9) && cmp(I.get22(),33.75) &&
				I.get01()==0 && I.get02()==0 && I.get12()==0 &&
				I.get10()==0 && I.get20()==0 && I.get21()==0)
			printf ("\tpassed (6)\n"); else printf ("\tFAILED (6)\n");

		// test dMassAdjust?

		// make random particles and compute the mass, COM and inertia, then
		// translate and repeat.
		for (i=0; i<NUMP; i++) {
			pm[i] = dRandReal()+0.5;
			q[i] = new DVector3();
			for (j=0; j<3; j++) {
				q[i].set(j, 2.0*(dRandReal()-0.5) );
			}
		}
		computeMassParams (m1,q,pm);
		//memcpy (m2,m1,sizeof(dMass));
		//m2.set(m1);
//		m2._I = m1._I;
//		m2._c = m1._c;
		m2.setI( m1.getI() );
		m2.setC( m1.getC() );
		m2.setMass( m1.getMass() );
		dMassTranslate (m2,1,2,-3);
		for (i=0; i<NUMP; i++) {
//			q[i][0] += 1;
//			q[i][1] += 2;
//			q[i][2] -= 3;
			q[i].add(1, 2, -3);
		}
		computeMassParams (m1,q,pm);
		compareMassParams (m1,m2,"7");

		// rotate the masses
		R.set00( -0.87919618797635 );
		R.set01( 0.15278881840384 );
		R.set02( -0.45129772879842 );
		R.set10( -0.47307856232664 );
		R.set11( -0.39258064912909 );
		R.set12( 0.78871864932708 );
		R.set20( -0.05666336483842 );
		R.set21( 0.90693771059546 );
		R.set22( 0.41743652473765 );
		dMassRotate (m2,R);
		for (i=0; i<NUMP; i++) {
			DVector3 a = new DVector3();
//			dMultiply0 (a,_R(0,0),q[i][0],3,3,1);
			dMultiply0 (a,R,q[i]);
//			q[i][0] = a[0];
//			q[i][1] = a[1];
//			q[i][2] = a[2];
			q[i].set(a);
		}
		computeMassParams (m1,q,pm);
		compareMassParams (m1,m2,"8");
	}

	//****************************************************************************
	// test rotation stuff

	void makeRandomRotation (DMatrix3 R)
	{
		//double *u1 = R, *u2=R+4, *u3=R+8;
		DVector3 u1 = new DVector3(R.get00(), R.get01(), R.get02());//, R.v[3]); //TZ
		DVector3 u2 = new DVector3(R.get10(), R.get11(), R.get12());//, R.v[7]); //TZ
		DVector3 u3 = new DVector3(R.get20(), R.get21(), R.get22());//, R.v[11]); //TZ
		//dMakeRandomVector (u1P,3,1.0);
		dMakeRandomVector (u1,1.0);
		dNormalize3 (u1);
		dMakeRandomVector (u2,1.0);
		double d = dCalcVectorDot3(u1,u2);
		//		u2[0] -= d*u1[0];
		//		u2[1] -= d*u1[1];
		//		u2[2] -= d*u1[2];
//		u2.v[0] -= d*u1.v[0];
//		u2.v[1] -= d*u1.v[1];
//		u2.v[2] -= d*u1.v[2];
		u2.eqSum(u2, u1, -d);
		dNormalize3 (u2);
		dCalcVectorCross3 (u3,u1,u2);
		//TZ back to R
		R.setCol(0, u1);
		R.setCol(1, u2);
		R.setCol(2, u3);
	}


	void testRtoQandQtoR()
	{
		HEADER();
		DMatrix3 R = new DMatrix3(),I = new DMatrix3(),R2 = new DMatrix3();
		DQuaternion q = new DQuaternion();
		int i;

		// test makeRandomRotation()
		makeRandomRotation (R);
		dMultiply2 (I,R,R);
		printf ("\tmakeRandomRotation() - %s (1)\n",
				cmpIdentityMat3(I) ? "passed" : "FAILED");

		// test QtoR() on random normalized quaternions
		int ok = 1;
		for (i=0; i<100; i++) {
			dMakeRandomVector (q,1.0);
			dNormalize4 (q);
			dRfromQ(R, q); //dQtoR (q,R);
			dMultiply2 (I,R,R);
			if (cmpIdentityMat3(I)==false) ok = 0;
		}
		printf ("\tQtoR() orthonormality %s (2)\n", ok!=0 ? "passed" : "FAILED");

		// test R -> Q -> R works
		double maxdiff=0;
		for (i=0; i<100; i++) {
			makeRandomRotation (R);
			dQfromR(q, R); //dRtoQ (R,q);
			dRfromQ(R2, q); //dQtoR (q,R2);
			double diff = dMaxDifference (R,R2);
			if (diff > maxdiff) maxdiff = diff;
		}
		printf ("\tmaximum difference = %e - %s (3)\n",maxdiff,
				(maxdiff > tol) ? "FAILED" : "passed");
	}


	void testQuaternionMultiply()
	{
		HEADER();
		DMatrix3 RA = new DMatrix3(),RB = new DMatrix3();
		DMatrix3 RC = new DMatrix3(),Rtest = new DMatrix3();
		DQuaternion qa = new DQuaternion(),qb = new DQuaternion(),qc = new DQuaternion();
		double diff,maxdiff=0;

		for (int i=0; i<100; i++) {
			makeRandomRotation (RB);
			makeRandomRotation (RC);
			dQfromR(qb, RB); //dRtoQ (RB,qb);
			dQfromR(qc, RC); //dRtoQ (RC,qc);

			dMultiply0 (RA,RB,RC);
			dQMultiply0 (qa,qb,qc);
			dRfromQ(Rtest, qa); //dQtoR (qa,Rtest);
			diff = dMaxDifference (Rtest,RA);
			if (diff > maxdiff) maxdiff = diff;

			dMultiply1 (RA,RB,RC);
			dQMultiply1 (qa,qb,qc);
			dRfromQ(Rtest, qa); //dQtoR (qa,Rtest);
			diff = dMaxDifference (Rtest,RA);
			if (diff > maxdiff) maxdiff = diff;

			dMultiply2 (RA,RB,RC);
			dQMultiply2 (qa,qb,qc);
			dRfromQ(Rtest, qa); //dQtoR (qa,Rtest);
			diff = dMaxDifference (Rtest,RA);
			if (diff > maxdiff) maxdiff = diff;

			dMultiply0 (RA,RC,RB);
			transpose3x3 (RA);
			dQMultiply3 (qa,qb,qc);
			dRfromQ(Rtest, qa); //dQtoR (qa,Rtest);
			diff = dMaxDifference (Rtest,RA);
			if (diff > maxdiff) maxdiff = diff;
		}
		printf ("\tmaximum difference = %e - %s\n",maxdiff,
				(maxdiff > tol) ? "FAILED" : "passed");
	}


	void testRotationFunctions()
	{
		DMatrix3 R1 = new DMatrix3();
		HEADER();

		printf ("\tdRSetIdentity - ");
		dMakeRandomMatrix (R1,1.0);
		R1.setIdentity();
		if (cmpIdentityMat3(R1)) printf ("passed\n"); else printf ("FAILED\n");

		printf ("\tdRFromAxisAndAngle - ");

		printf ("\n");
		printf ("\tdRFromEulerAngles - ");

		printf ("\n");
		printf ("\tdRFrom2Axes - ");

		printf ("\n");
	}

	//****************************************************************************

	//#include "../src/array.h"
	//#include "../src/array.cpp"

	// matrix header on the stack

	class MatrixComparison {
		//  struct dMatInfo;
		//  dArray<dMatInfo*> mat;
		//	  int afterfirst,index;
		//		class dMatInfo {};
		//dArray<dMatInfo> mat;
		ArrayList<dMatInfo> mat = new ArrayList<dMatInfo>();
		int afterfirst,index;

		//public:
		//  ~MatrixComparison();

		private class dMatInfo {
			int n,m;		// size of matrix
			//  char name[128];	// name of the matrix
			//  dReal *data;		// matrix data
			String name = "\0";	// name of the matrix
			double[] data;		// matrix data
			int size;		// size of `data'
		}


		MatrixComparison()
		{
			afterfirst = 0;
			index = 0;
		}

		//		@Override
		//		protected void finalize() throws Throwable {
		//			reset();
		//			super.finalize();
		//		}

		/**
		 * add a new n*m matrix A to the sequence. the name of the matrix is given
		 * by the printf-style arguments (name,...). if this is the first sequence
		 * then this object will simply record the matrices and return 0.
		 * if this the second or subsequent sequence then this object will compare
		 * the matrices with the first sequence, and report any differences.
		 * the matrix error will be returned. if `lower_tri' is 1 then only the
		 * lower triangle of the matrix (including the diagonal) will be compared
		 * (the matrix must be square).
		 */
		//double nextMatrix (double *A, int n, int m, int lower_tri,
		//	     final char *name, ...)
		double nextMatrix (double[] A, int n, int m, int lower_tri,
				final String name, Object ... objects)
		{
			if (A==null || n < 1 || m < 1 || name==null) dDebug (0,"bad args to nextMatrix");
			int num = n*dPAD(m);

			if (afterfirst==0) {
				dMatInfo mi = new dMatInfo();//(dMatInfo) dAlloc (sizeof(dMatInfo));
				mi.n = n;
				mi.m = m;
				mi.size = num;// * sizeof(double);
				mi.data = new double[mi.size];//(double*) dAlloc (mi.size);
				memcpy (mi.data,A,mi.size);

				//va_list ap;
				//va_start (ap,name);
				Ref<String> r = new Ref<String>();
				vsprintf (r,name,objects);
				mi.name = r.get();
				
				//if (strlen(mi.name) >= sizeof (mi.name)) dDebug (0,"name too long");
				if (strlen(mi.name) >= mi.name.length()+1) dDebug (0,"name too long");

				//mat.push (mi);
				mat.add(mi);
				return 0;
			}
			else {
				if (lower_tri != 0 && n != m)
					dDebug (0,"MatrixComparison, lower triangular matrix must be square");
				if (index >= mat.size()) dDebug (0,"MatrixComparison, too many matrices");
				dMatInfo mp = mat.get(index);//mat[index];
				index++;

				dMatInfo mi = new dMatInfo();
				//va_list ap;
				//va_start (ap,name);
				Ref<String> r = new Ref<String>();
				vsprintf (r,name,objects);
				mi.name = r.get();

				//if (strlen(mi.name) >= sizeof (mi.name)) dDebug (0,"name too long");
				if (strlen(mi.name) >= mi.name.length()+1) dDebug (0,"name too long");

				if (strcmp(mp.name.toCharArray(),mi.name) != 0)
					dDebug (0,"MatrixComparison, name mismatch (\"%s\" and \"%s\")",
							mp.name,mi.name);
				if (mp.n != n || mp.m != m)
					dDebug (0,"MatrixComparison, size mismatch (%dx%d and %dx%d)",
							mp.n,mp.m,n,m);

				double maxdiff;
				if (lower_tri != 0) {
					maxdiff = dMaxDifferenceLowerTriangle (A,mp.data,n);
				}
				else {
					maxdiff = dMaxDifference (A,mp.data,n,m);
				}
				if (maxdiff > tol)
					dDebug (0,"MatrixComparison, matrix error " +
							"(size=%dx%d, name=\"%s\", " +
							"error=%.4e)",n,m,mi.name,maxdiff);
				return maxdiff;
			}
		}

		/**
		 * end a sequence.
		 */
		void end()
		{
			if (mat.size() <= 0) dDebug (0,"no matrices in sequence");
			afterfirst = 1;
			index = 0;
		}


		/**
		 * restarts the object, so the next sequence will be the first sequence.
		 */
		void reset()
		{
			//			for (int i=0; i<mat.size(); i++) {
			//				dFree (mat[i].data,mat[i].size);
			//				dFree (mat[i],sizeof(dMatInfo));
			//			}
			//			mat.setSize (0);
			mat.clear();
			afterfirst = 0;
			index = 0;
		}


		/** 
		 * print out info about all the matrices in the sequence
		 */
		void dump()
		{
			for (int i=0; i<mat.size(); i++)
				printf ("%d: %s (%dx%d)\n",i,mat.get(i).name,mat.get(i).n,mat.get(i).m);
		}
	}  //MatrixComparison


	//****************************************************************************
	// unit test

	//#include <setjmp.h>

	// static jmp_buf jump_buffer;

	//	static void myDebug (int num, const char *msg, va_list ap)
	//static dMessageFunction myDebug (int num, final String msg, Object ... ap)
	private static final dMessageFunction myDebug = new dMessageFunction() {

		@Override
		public void call(int num, String msg, Object... ap) {
//			printf ("(Error %d: ",num);
//			vprintf (msg,ap);
//			printf (")\n");
			longjmp (jump_buffer,1);
		}
	};


	//extern "C" 
	void dTestMatrixComparison()
	{
		//volatile
		int i;
		printf ("dTestMatrixComparison()\n");
		dMessageFunction orig_debug = dGetDebugHandler();

		MatrixComparison mc = new MatrixComparison();
		double[] A = new double[50*50];

		// make first sequence
		//unsigned 
		long seed = dRandGetSeed();
		for (i=1; i<49; i++) {
			dMakeRandomMatrix (A,i,i+1,1.0);
			mc.nextMatrix (A,i,i+1,0,"A%d",i);
		}
		mc.end();

		//mc.dump();

		// test identical sequence
		dSetDebugHandler (myDebug);
		dRandSetSeed (seed);
		if (setjmp (jump_buffer) != 0) {
			printf ("\tFAILED (1)\n");
		}
		else {
			for (i=1; i<49; i++) {
				dMakeRandomMatrix (A,i,i+1,1.0);
				mc.nextMatrix (A,i,i+1,0,"A%d",i);
			}
			mc.end();
			printf ("\tpassed (1)\n");
		}
		dSetDebugHandler (orig_debug);

		// test broken sequences (with matrix error)
		dRandSetSeed (seed);
		//volatile
		int passcount = 0;
		for (i=1; i<49; i++) {
//			if (setjmp (jump_buffer) != 0) {
//				passcount++;
//			}
//			else {
//				dSetDebugHandler (myDebug);
//				dMakeRandomMatrix (A,i,i+1,1.0);
//				A[(i-1)*dPAD(i+1)+i] += 0.01;
//				mc.nextMatrix (A,i,i+1,0,"A%d",i);
//				dSetDebugHandler (orig_debug);
//			}
			try {
				dSetDebugHandler (myDebug);
				dMakeRandomMatrix (A,i,i+1,1.0);
				A[(i-1)*dPAD(i+1)+i] += 0.01;
				mc.nextMatrix (A,i,i+1,0,"A%d",i);
				dSetDebugHandler (orig_debug);
			} catch (CppLongJump e) {
				passcount++;
			}
		}
		mc.end();
		printf ("\t%s (2)\n",(passcount == 48) ? "passed" : "FAILED");

		// test broken sequences (with name error)
		dRandSetSeed (seed);
		passcount = 0;
		for (i=1; i<49; i++) {
//			if (setjmp (jump_buffer) != 0) {
//				passcount++;
//			}
//			else {
			try {
				dSetDebugHandler (myDebug);
				dMakeRandomMatrix (A,i,i+1,1.0);
				mc.nextMatrix (A,i,i+1,0,"B%d",i);
				dSetDebugHandler (orig_debug);
			} catch (CppLongJump e) {
				passcount++;
			}
		}
		mc.end();
		printf ("\t%s (3)\n",(passcount == 48) ? "passed" : "FAILED");

		// test identical sequence again
		dSetDebugHandler (myDebug);
		dRandSetSeed (seed);
		if (setjmp (jump_buffer) != 0) {
			printf ("\tFAILED (4)\n");
		}
		else {
			for (i=1; i<49; i++) {
				dMakeRandomMatrix (A,i,i+1,1.0);
				mc.nextMatrix (A,i,i+1,0,"A%d",i);
			}
			mc.end();
			printf ("\tpassed (4)\n");
		}
		dSetDebugHandler (orig_debug);
	}

	//****************************************************************************

	// internal unit tests
	//	extern "C" void dTestDataStructures();
	//	extern "C" void dTestMatrixComparison();
	//	extern "C" void dTestSolveLCP();

	public static void main(String[] args)
	{
		new DemoOde().runAllTests();
	}

	private int runAllTests()
	{
		dInitODE2(0);
		testRandomNumberGenerator();
		testInfinity();
		testPad();
		testCrossProduct();
		testSetZero();
		testNormalize3();
		//testReorthonormalize();     ... not any more
		testPlaneSpace();
		testMatrixMultiply();
		testSmallMatrixMultiply();
		testCholeskyFactorization();
		testCholeskySolve();
		testInvertPDMatrix();
		testIsPositiveDefinite();
		testFastLDLTFactorization();
		testSolveLDLT();
		testLDLTAddTL();
		//		
		testLDLTRemove();
		testMassFunctions();
		testRtoQandQtoR();
		testQuaternionMultiply();
		testRotationFunctions();
		dTestMatrixComparison();
		DLCP.dTestSolveLCP(true);
		//OdeImpl.dTestDataStructures();
		dCloseODE();
		return 0;
	}
}
