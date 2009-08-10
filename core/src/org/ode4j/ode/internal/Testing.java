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
package org.ode4j.ode.internal;

import java.util.ArrayList;

import org.cpp4j.java.Ref;
import static org.ode4j.ode.OdeMath.*;
import org.ode4j.ode.OdeConfig;

import static org.cpp4j.C_All.*;


/**
 * compare a sequence of named matrices/vectors, i.e. to make sure that two
 * different pieces of code are giving the same results.
 */ 
class dMatrixComparison {
	//	  dArray<dMatInfo*> mat;
	ArrayList<dMatInfo> mat = new ArrayList<dMatInfo>();
	int afterfirst,index;


	//#ifdef dDOUBLE
	//static const dReal tol = 1.0e-9;
	//#else
	//static const dReal tol = 1.0e-5f;
	//#endif
	private static final double tol;
	static {
		if (OdeConfig.isDoublePrecision()) {
			tol = 1.0e-9;
		} else {
			tol = 1.0e-5f;
		}
	}


	// matrix header on the stack

	private static class dMatInfo {
		int n,m;		// size of matrix
		//TODO
		String name;// = new char[128];	// name of the matrix
		double[] data;		// matrix data
		//TODO
		//  int size;		// size of `data'
	}



	public dMatrixComparison()
	{
		afterfirst = 0;
		index = 0;
	}

	public void DESTRUCTOR() {
		reset();
	}

	@Override
	public void finalize()
	{
		DESTRUCTOR();
	}


	//dReal nextMatrix (dReal *A, int n, int m, int lower_tri, const char *name, ...);
	/** add a new n*m matrix A to the sequence. the name of the matrix is given
	 * by the printf-style arguments (name,...). if this is the first sequence
	 * then this object will simply record the matrices and return 0.
	 * if this the second or subsequent sequence then this object will compare
	 * the matrices with the first sequence, and report any differences.
	 * the matrix error will be returned. if `lower_tri' is 1 then only the
	 * lower triangle of the matrix (including the diagonal) will be compared
	 * (the matrix must be square).
	 */
	public double nextMatrix (double[] A, int n, int m, int lower_tri,
			String name, Object ... objs)
	{
		if (A==null || n < 1 || m < 1 || name==null) 
			dDebug (0,"bad args to nextMatrix");
		int num = n*dPAD(m);

		if (afterfirst==0) {
			//	    dMatInfo *mi = (dMatInfo*) dAlloc (sizeof(dMatInfo));
			dMatInfo mi = new dMatInfo();
			mi.n = n;
			mi.m = m;
			//    mi.size = num;// * sizeof(double);
			mi.data = new double[num];//(double*) dAlloc (mi.size);
			memcpy (mi.data,A,num);

			//    va_list ap;
			//    va_start (ap,name);
			//    vsprintf (mi.name,name,ap);
			Ref<String> r = new Ref<String>();
			vsprintf (r,name, objs);
			mi.name = r.get();
			//TZ if (strlen(mi.name) >= sizeof (mi.name)) COM.dDebug (0,"name too long");
			//mi.name = name;

			mat.add (mi);
			return 0;
		}
		else {
			if (lower_tri!=0 && n != m)
				dDebug (0,"dMatrixComparison, lower triangular matrix must be square");
			if (index >= mat.size()) dDebug (0,"dMatrixComparison, too many matrices");
			dMatInfo mp = mat.get(index);
			index++;

			dMatInfo mi = new dMatInfo();
			//    va_list ap;
			//    va_start (ap,name);
			//    vsprintf (mi.name,name,ap);
			Ref<String> r = new Ref<String>();
			vsprintf (r,name,objs);
			mi.name = r.get();
			//if (strlen(mi.name) >= sizeof (mi.name)) COM.dDebug (0,"name too long");
			//mi.name = name;

			//    if (strcmp(mp.name,mi.name) != 0)
			if (!mp.name.equals(mi.name))
				dDebug (0,"dMatrixComparison, name mismatch (\"%s\" and \"%s\")",
						mp.name,mi.name);
			if (mp.n != n || mp.m != m)
				dDebug (0,"dMatrixComparison, size mismatch (%dx%d and %dx%d)",
						mp.n,mp.m,n,m);

			double maxdiff;
			if (lower_tri!=0) {
				maxdiff = dMaxDifferenceLowerTriangle (A,mp.data,n);
			}
			else {
				maxdiff = dMaxDifference (A,mp.data,n,m);
			}
			if (maxdiff > tol)
				dDebug (0,"dMatrixComparison, matrix error (size=%dx%d, name=\"%s\", "+
						"error=%.4e)",n,m,mi.name,maxdiff);
			return maxdiff;
		}
	}


	/**
	 * end a sequence.
	 */
	public void end()
	{
		if (mat.size() <= 0) dDebug (0,"no matrices in sequence");
		afterfirst = 1;
		index = 0;
	}

	/**
	 *  restarts the object, so the next sequence will be the first sequence.
	 */
	public void reset()
	{
		//  for (int i=0; i<mat.size(); i++) {
		//    dFree (mat[i].data,mat[i].size);
		//    dFree (mat[i],sizeof(dMatInfo));
		//  }
		//  mat.setSize (0);
		mat.clear();
		afterfirst = 0;
		index = 0;
	}



	/**
	 *  print out info about all the matrices in the sequence
	 */
	public void dump()
	{
		for (int i=0; i<mat.size(); i++) {
			dMatInfo m = mat.get(i);
			printf ("%d: %s (%dx%d)\n", i, m.name, m.n, m.m);
		}
	}

	//****************************************************************************
	// unit test

	//#include <setjmp.h>

	private static final jmp_buf jump_buffer = null;

	//static void myDebug (int num, final char *msg, va_list ap)
//	static void myDebug (int num, final String msg, Object ... ap)
//	{
//		// printf ("(Error %d: ",num);
//		// vprintf (msg,ap);
//		// printf (")\n");
//		CPP.longjmp (jump_buffer,1);
//	}
	private static final dMessageFunction myDebug = new dMessageFunction() {

		public void call(int errnum, String msg, Object... args) {
			printf ("(Error %d: ",errnum);
			vprintf (msg,args);
			printf (")\n");
			longjmp (jump_buffer,1);
		}
	};


	//extern "C" ODE_API 
	public static void dTestMatrixComparison()
	{
		//volatile 
		int i;
		printf ("dTestMatrixComparison()\n");
		dMessageFunction orig_debug = dGetDebugHandler();

		dMatrixComparison mc = new dMatrixComparison();
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
		if (setjmp (jump_buffer)!=0) {
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
			if (setjmp (jump_buffer)!=0) {
				passcount++;
			}
			else {
				dSetDebugHandler (myDebug);
				dMakeRandomMatrix (A,i,i+1,1.0);
				A[(i-1)*dPAD(i+1)+i] += 0.01;//REAL(0.01);
				mc.nextMatrix (A,i,i+1,0,"A%d",i);
				dSetDebugHandler (orig_debug);
			}
		}
		mc.end();
		printf ("\t%s (2)\n",(passcount == 48) ? "passed" : "FAILED");

		// test broken sequences (with name error)
		dRandSetSeed (seed);
		passcount = 0;
		for (i=1; i<49; i++) {
			if (setjmp (jump_buffer)!=0) {
				passcount++;
			}
			else {
				dSetDebugHandler (myDebug);
				dMakeRandomMatrix (A,i,i+1,1.0);
				mc.nextMatrix (A,i,i+1,0,"B%d",i);
				dSetDebugHandler (orig_debug);
			}
		}
		mc.end();
		printf ("\t%s (3)\n",(passcount == 48) ? "passed" : "FAILED");

		// test identical sequence again
		dSetDebugHandler (myDebug);
		dRandSetSeed (seed);
		if (setjmp (jump_buffer)!=0) {
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
}