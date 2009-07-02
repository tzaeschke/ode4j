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

import org.ode4j.ode.DJoint;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.OdeMath.OP;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;

import static org.cpp4j.Cstdio.*;
import static org.ode4j.ode.internal.Timer.*;
import static org.ode4j.ode.OdeMath.*;


class Step extends AbstractStepper implements DxWorld.dstepper_fn_t {

	public static final Step INSTANCE = new Step();

	//****************************************************************************
	// misc defines

	//#define FAST_FACTOR
	private static final boolean FAST_FACTOR = true;
	//#define TIMING  /was commented out!
	private static final boolean TIMING = false;

	// memory allocation system
	//#ifdef dUSE_MALLOC_FOR_ALLOCA
	//unsigned int dMemoryFlag;
	//#define REPORT_OUT_OF_MEMORY fprintf(stderr, "Insufficient memory to complete rigid body simulation.  Results will not be accurate.\n")

	//#define CHECK(p)                                \
	//  if (!p) {                                     \
	//    dMemoryFlag = d_MEMORY_OUT_OF_MEMORY;       \
	//    return;                                     \
	//  }
	//
	//#define ALLOCA(t,v,s)                           \
	//  Auto<t> v(malloc(s));                         \
	//  CHECK(v)
	//
	//#else // use alloca()
	//
	//#define ALLOCA(t,v,s)                           \
	//  Auto<t> v( dALLOCA16(s) );
	private static void ALLOCA(Class cls) {
		throw new UnsupportedOperationException();
	}
	//
	//#endif

	//TODO ?
	//void dInternalStepIsland (dxWorld *world,
	//dxBody * const *body, int nb,
	//dxJoint * const *joint, int nj,
	//double stepsize);


	/* This template should work almost like std::auto_ptr
	 */

	//template<class T>
	//struct Auto {
	//  T *p;
	//  Auto(void * q) :
	//    p(reinterpret_cast<T*>(q))
	//  { }
	//
	//  ~Auto()
	//  {
	//#ifdef dUSE_MALLOC_FOR_ALLOCA
	//    free(p);
	//#endif
	//  }
	//
	//  operator T*() 
	//  {
	//    return p;
	//  }
	//  T& operator[] (int i)
	//  {
	//    return p[i];
	//  }
	//private:
	//  // intentionally undefined, don't use this
	//  template<class U>
	//  Auto& operator=(const Auto<U>&) const;
	//};
	//
	//



	//****************************************************************************
	// debugging - comparison of various vectors and matrices produced by the
	// slow and fast versions of the stepper.

	////#define COMPARE_METHODS
	//private static final boolean COMPARE_METHODS = true;
	//TODO remove, would not even compile

	//#ifdef COMPARE_METHODS
	//#include "testing.h"
	//dMatrixComparison comparator;
	//private dMatrixComparison comparator = new dMatrixComparison();//TZ new
	//#endif

	// undef to use the fast decomposition
	//#define DIRECT_CHOLESKY
	//#undef REPORT_ERROR
	public static final boolean DIRECT_CHOLESKY = true;
	public static final boolean REPORT_ERROR = false;

	//****************************************************************************
	// special matrix multipliers

	// this assumes the 4th and 8th rows of B and C are zero.

	private static void Multiply2_p8r (double[] A, int APos,
			double[] B, int BPos, double[] C, int CPos,
			int p, int r, int Askip)
	{
		int i,j;
		double sum;//,bb[],cc[];
		int bbPos, ccPos;
		dIASSERT (p>0 && r>0);
		dAASSERT(A, B, C);
		bbPos = BPos; //bb = B;
		for (i=p; i>0; i--) {
			ccPos = CPos; //cc = C;
			for (j=r; j>0; j--) {
				sum = B[bbPos] * C[ccPos]; //bb[0]*cc[0];
				sum += B[bbPos+1] * C[ccPos+1]; //sum += bb[1]*cc[1];
				sum += B[bbPos+2] * C[ccPos+2]; //sum += bb[2]*cc[2];
				sum += B[bbPos+4] * C[ccPos+4]; //sum += bb[4]*cc[4];
				sum += B[bbPos+5] * C[ccPos+5]; //sum += bb[5]*cc[5];
				sum += B[bbPos+6] * C[ccPos+6]; //sum += bb[6]*cc[6];
				A[APos++] = sum;//*(A++) = sum; 
				ccPos +=8;//cc += 8;
			}
			APos += Askip -r;//A += Askip - r;
			bbPos += 8;//bb += 8;
		}
	}


	// this assumes the 4th and 8th rows of B and C are zero.

	private static void MultiplyAdd2_p8r (double[] A, int APos,
			double[] B, int BPos, double[] C, int CPos,
			int p, int r, int Askip)
	{
		int i,j;
		double sum;//,bb[],cc[];
		int bbPos, ccPos;
		dIASSERT (p>0 && r>0);
		dAASSERT(A, B, C);
		bbPos = BPos;//bb = B;
		for (i=p; i>0; i--) {
			ccPos = CPos;//cc = C;
			for (j=r; j>0; j--) {
				sum = B[bbPos] * C[ccPos]; //sum = bb[0]*cc[0];
				sum += B[bbPos+1] * C[ccPos+1]; //sum += bb[1]*cc[1];
				sum += B[bbPos+2] * C[ccPos+2]; //sum += bb[2]*cc[2];
				sum += B[bbPos+4] * C[ccPos+4]; //sum += bb[4]*cc[4];
				sum += B[bbPos+5] * C[ccPos+5]; //sum += bb[5]*cc[5];
				sum += B[bbPos+6] * C[ccPos+6]; //sum += bb[6]*cc[6];
				A[APos++] += sum;//*(A++) += sum; 
				ccPos += 8;//cc += 8;
			}
			APos += Askip -r;//A += Askip - r;
			bbPos += 8;//bb += 8;
		}
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static void Multiply0_p81 (double[] A, int APos,
			double[] B, int BPos, double[] C, int CPos, int p)
	{
		int i;
		dIASSERT (p>0);
		dAASSERT(A, B, C);
		double sum;
		for (i=p; i>0; i--) {
			sum =  B[0+BPos]*C[0+CPos];
			sum += B[1+BPos]*C[1+CPos];
			sum += B[2+BPos]*C[2+CPos];
			sum += B[4+BPos]*C[4+CPos];
			sum += B[5+BPos]*C[5+CPos];
			sum += B[6+BPos]*C[6+CPos];
			A[APos++] = sum;//*(A++) = sum;
			BPos += 8;//B += 8;
		}
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static void MultiplyAdd0_p81 (double[] A, int APos, 
			double[] B, int BPos, double[] C, int CPos, int p)
	{
		//TODO improve by making constants for each x+XPos ?
		int i;
		dIASSERT (p>0);
		dAASSERT(A, B, C);
		double sum = 0;
		for (i=p; i>0; i--) {
			sum =  B[0+BPos]*C[0+CPos];
			sum += B[1+BPos]*C[1+CPos];
			sum += B[2+BPos]*C[2+CPos];
			sum += B[4+BPos]*C[4+CPos];
			sum += B[5+BPos]*C[5+CPos];
			sum += B[6+BPos]*C[6+CPos];
			A[APos++] += sum;//*(A++) += sum;
			BPos += 8;//B += 8;
		}
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static void MultiplyAdd1_8q1 (double[] A, int a, 
			double[] B, int b, double[] C, int c, int q)
	{
		int k;
		double sum;
		dIASSERT (q>0);
		dAASSERT(A, B, C);
		sum = 0;
		for (k=0; k<q; k++) sum += B[k*8+b] * C[k+c];
		A[0+a] += sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[1+k*8+b] * C[k+c];
		A[1+a] += sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[2+k*8+b] * C[k+c];
		A[2+a] += sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[4+k*8+b] * C[k+c];
		A[4+a] += sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[5+k*8+b] * C[k+c];
		A[5+a] += sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[6+k*8+b] * C[k+c];
		A[6+a] += sum;
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static void Multiply1_8q1 (double[] A, int a, 
			double[] B, int b, double[] C, int c, int q)
	{
		int k;
		double sum;
		dIASSERT (q>0);
		dAASSERT(A, B, C);
		sum = 0;
		for (k=0; k<q; k++) sum += B[k*8+b] * C[k+c];
		A[0+a] = sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[1+k*8+b] * C[k+c];
		A[1+a] = sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[2+k*8+b] * C[k+c];
		A[2+a] = sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[4+k*8+b] * C[k+c];
		A[4+a] = sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[5+k*8+b] * C[k+c];
		A[5+a] = sum;
		sum = 0;
		for (k=0; k<q; k++) sum += B[6+k*8+b] * C[k+c];
		A[6+a] = sum;
	}

	//****************************************************************************
	// the slow, but sure way
	// note that this does not do any joint feedback!

	// given lists of bodies and joints that form an island, perform a first
	// order timestep.
	//
	// `body' is the body array, `nb' is the size of the array.
	// `_joint' is the body array, `nj' is the size of the array.

	//void dInternalStepIsland_x1 (dxWorld *world, dxBody * const *body, int nb,
	//	     dxJoint * const *_joint, int nj, double stepsize)
	private void dInternalStepIsland_x1 (DxWorld world, final DxBody [] body, 
			int nb,
			final DxJoint [] _joint, int nj, double stepsize)
	{
		int i,j,k;
		int n6 = 6*nb;

		//#ifdef TIMING
		if (TIMING)
			dTimerStart("preprocessing");
		//#endif

		// number all bodies in the body list - set their tag values
		for (i=0; i<nb; i++) body[i].tag = i;

		// make a local copy of the joint array, because we might want to modify it.
		// (the "dxJoint *const*" declaration says we're allowed to modify the joints
		// but not the joint array, because the caller might need it unchanged).
		DxJoint[] joint = _joint.clone();//ALLOCA(dxJoint*,joint,nj*sizeof(dxJoint*));
		if (joint.length!=nj) throw new IllegalStateException("TZ: " + nj);
		//  memcpy (joint,_joint,nj * sizeof(dxJoint*));

		// for all bodies, compute the inertia tensor and its inverse in the global
		// frame, and compute the rotational force and add it to the torque
		// accumulator.
		// @@@ check computation of rotational force.
		//double[] I = new double[3*nb*4];//ALLOCA(double,I,3*nb*4*sizeof(double));
		DMatrix3[] I = new DMatrix3[nb];
		for (int ii = 0; ii < nb; ii++) I[ii] = new DMatrix3();
		//double[] invI = new double[3*nb*4];//ALLOCA(double,invI,3*nb*4*sizeof(double));
		DMatrix3[] invI = new DMatrix3[nb];
		for (int ii = 0; ii < nb; ii++) invI[ii] = new DMatrix3();

		//dSetZero (I,3*nb*4);
		//dSetZero (invI,3*nb*4);
		//TZ: moved outside the loop.
		DVector3 tmpV = new DVector3();
		DMatrix3 tmp = new DMatrix3();
		for (i=0; i<nb; i++) {
			//double[] tmp = new double[12];

			// compute inertia tensor in global frame
			dMULTIPLY2_333 (tmp,body[i].mass._I,body[i]._posr.R);
			//    dMULTIPLY0_333 (I+i*12,body[i].posr.R,tmp);
			dMULTIPLY0_333 (I[i],body[i]._posr.R,tmp);

			// compute inverse inertia tensor in global frame
			dMULTIPLY2_333 (tmp,body[i].invI,body[i]._posr.R);
			//dMULTIPLY0_333 (invI+i*12,body[i].posr.R,tmp);
			dMULTIPLY0_333 (invI[i],body[i]._posr.R,tmp);

			// compute rotational force
			//    dMULTIPLY0_331 (tmp,I+i*12,body[i].avel);
			dMULTIPLY0_331 (tmpV,I[i],body[i].avel);
			dCROSS (body[i].tacc,OP.SUB_EQ,body[i].avel,tmpV);
		}

		// add the gravity force to all bodies
		for (i=0; i<nb; i++) {
			if ((body[i].flags & DxBody.dxBodyNoGravity)==0) {
				body[i].facc.v[0] += body[i].mass._mass * world.gravity.v[0];
				body[i].facc.v[1] += body[i].mass._mass * world.gravity.v[1];
				body[i].facc.v[2] += body[i].mass._mass * world.gravity.v[2];
			}
		}

		// get m = total constraint dimension, nub = number of unbounded variables.
		// create constraint offset array and number-of-rows array for all joints.
		// the constraints are re-ordered as follows: the purely unbounded
		// constraints, the mixed unbounded + LCP constraints, and last the purely
		// LCP constraints.
		//
		// joints with m=0 are inactive and are removed from the joints array
		// entirely, so that the code that follows does not consider them.
		int m = 0;
		DxJoint.Info1[] info = new DxJoint.Info1[nj];//ALLOCA(dxJoint.Info1,info,nj*sizeof(dxJoint.Info1));
		int[] ofs = new int[nj];//ALLOCA(int,ofs,nj*sizeof(int));

		for (i=0, j=0; j<nj; j++) {	// i=dest, j=src
			joint[j].getInfo1 (info[i]);
			dIASSERT (info[i].m >= 0 && info[i].m <= 6 &&
					info[i].nub >= 0 && info[i].nub <= info[i].m);
			if (info[i].m > 0) {
				joint[i] = joint[j];
				i++;
			}
		}
		nj = i;

		// the purely unbounded constraints
		for (i=0; i<nj; i++) if (info[i].nub == info[i].m) {
			ofs[i] = m;
			m += info[i].m;
		}
		//int nub = m;
		// the mixed unbounded + LCP constraints
		for (i=0; i<nj; i++) if (info[i].nub > 0 && info[i].nub < info[i].m) {
			ofs[i] = m;
			m += info[i].m;
		}
		// the purely LCP constraints
		for (i=0; i<nj; i++) if (info[i].nub == 0) {
			ofs[i] = m;
			m += info[i].m;
		}

		// create (6*nb,6*nb) inverse mass matrix `invM', and fill it with mass
		// parameters
		//#ifdef TIMING
		if (TIMING)
			dTimerNow ("create mass matrix");
		//#endif
		int nskip = dPAD (n6);
		double[] invM = new double[n6*nskip];//ALLOCA(double, invM, n6*nskip*sizeof(double));

		//TZdSetZero (invM,n6*nskip);
		for (i=0; i<nb; i++) {
			//	  double *MM = invM+(i*6)*nskip+(i*6);
			//double[] MM = invM+(i*6)*nskip+(i*6);
			int posMM = (i*6)*nskip+(i*6);
			invM[posMM] = body[i].invMass;//MM[0] = body[i].invMass;
			invM[posMM+nskip+1] = body[i].invMass;//MM[nskip+1] = body[i].invMass;
			invM[posMM+2*nskip+2] = body[i].invMass;//MM[2*nskip+2] = body[i].invMass;
			posMM += 3*nskip+3;//MM += 3*nskip+3;
			for (j=0; j<3; j++) for (k=0; k<3; k++) {
				invM[j*nskip+k] = invI[i].v[j*4+k];//MM[j*nskip+k] = invI[i*12+j*4+k];
			}
		}

		// assemble some body vectors: fe = external forces, v = velocities
		double[] fe = new double[n6];//ALLOCA(double,fe,n6*sizeof(double));
		double[] v = new double[n6];//ALLOCA(double,v,n6*sizeof(double));

		//dSetZero (fe,n6);
		//dSetZero (v,n6);
		for (i=0; i<nb; i++) {
			for (j=0; j<3; j++) fe[i*6+j] = body[i].facc.v[j];
			for (j=0; j<3; j++) fe[i*6+3+j] = body[i].tacc.v[j];
			for (j=0; j<3; j++) v[i*6+j] = body[i].lvel.v[j];
			for (j=0; j<3; j++) v[i*6+3+j] = body[i].avel.v[j];
		}

		// this will be set to the velocity update
		double[] vnew = new double[n6];//ALLOCA(double,vnew,n6*sizeof(double));
		//TZ dSetZero (vnew,n6);

		// if there are constraints, compute cforce
		if (m > 0) {
			// create a constraint equation right hand side vector `c', a constraint
			// force mixing vector `cfm', and LCP low and high bound vectors, and an
			// 'findex' vector.
			double[] c = new double[m];//ALLOCA(double,c,m*sizeof(double));
			double[] cfm = new double[m];//ALLOCA(double,cfm,m*sizeof(double));
			double[] lo = new double[m];//ALLOCA(double,lo,m*sizeof(double));
			double[] hi = new double[m];//ALLOCA(double,hi,m*sizeof(double));
			int[] findex = new int[m];// ALLOCA(int,findex,m*sizeof(int));
			//TZ dSetZero (c,m);
			dSetValue (cfm,m,world.global_cfm);
			dSetValue (lo,m,-dInfinity);
			dSetValue (hi,m, dInfinity);
			for (i=0; i<m; i++) findex[i] = -1;

			// create (m,6*nb) jacobian mass matrix `J', and fill it with constraint
			// data. also fill the c vector.
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("create J");
			//#   endif
			double[] J = new double[m*nskip];//ALLOCA(double,J,m*nskip*sizeof(double));
			//TZ dSetZero (J,m*nskip);
			//TZ:
			DxJoint.Info1 mInfo = new DxJoint.Info1();
			DxJoint.Info2 Jinfo = new DxJoint.Info2();
			//TZ TODO remove Jinfo.setRowskip(nskip);
			//TODO rowskip(), setArrays(), ...?!?!?
			//TODO set J1l, J1a, J2l, J2a ?!?!?
			if (true) throw new UnsupportedOperationException();
			Jinfo.fps = dRecip(stepsize);
			Jinfo.erp = world.global_erp;
			for (i=0; i<nj; i++) {
				//TZ Jinfo.J1l = J + nskip*ofs[i] + 6*joint[i].node[0].body.tag;
				//TZ Jinfo.J1a = Jinfo.J1l + 3;
//				if (joint[i].node[1].body!=null) {
////					Jinfo.J2l = J + nskip*ofs[i] + 6*joint[i].node[1].body.tag;
////					Jinfo.J2a = Jinfo.J2l + 3;
//				}
//				else {
//					Jinfo.J2l = 0;
//					Jinfo.J2a = 0;
//				}
//				Jinfo.c = c + ofs[i];
//				Jinfo.cfm = cfm + ofs[i];
//				Jinfo.lo = lo + ofs[i];
//				Jinfo.hi = hi + ofs[i];
//				Jinfo.findex = findex + ofs[i];
				Jinfo.setAllP(ofs[i]);
				joint[i].getInfo2 (Jinfo);

				//TZ:
				//Now copy data into J and other arrays:
				joint[i].getInfo1 (mInfo);
//				int m2 = mInfo.getM();
				//infoToJ(J, Jinfo, nskip*ofs[i], m2 > 3 ? 3 : m2, joint[i], nskip);
//				infoTo(Jinfo.c.v, c, ofs[i], m2 > 6 ? 6 : m2);
//				infoTo(Jinfo.cfm.v, cfm, ofs[i], m2 > 3 ? 3 : m2);
//				infoTo(Jinfo.lo.v, lo, ofs[i], m2 > 3 ? 3 : m2);
//				infoTo(Jinfo.hi.v, hi, ofs[i], m2 > 3 ? 3 : m2);
//				infoTo(Jinfo.findex, findex, ofs[i], m2 > 3 ? 3 : m2);

				// adjust returned findex values for global index numbering
				for (j=0; j<info[i].m; j++) {
					if (findex[ofs[i] + j] >= 0) findex[ofs[i] + j] += ofs[i];
				}
			}

			// compute A = J*invM*J'
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("compute A");
			//#   endif
			double[] JinvM = new double[m*nskip];//ALLOCA(double,JinvM,m*nskip*sizeof(double));
			//dSetZero (JinvM,m*nskip);
			dMultiply0 (JinvM,J,invM,m,n6,n6);
			int mskip = dPAD(m);
			double[] A = new double[m*mskip];//ALLOCA(double,A,m*mskip*sizeof(double));
			//dSetZero (A,m*mskip);
			dMultiply2 (A,JinvM,J,m,n6,m);

			// add cfm to the diagonal of A
			for (i=0; i<m; i++) A[i*mskip+i] += cfm[i] * Jinfo.fps;

			//#   ifdef COMPARE_METHODS
//TODO remove			if (COMPARE_METHODS)
//				comparator.nextMatrix (A,m,m,1,"A");
			//#   endif

			// compute `rhs', the right hand side of the equation J*a=c
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("compute rhs");
			//#   endif
			double[] tmp1 = new double[n6];//ALLOCA(double,tmp1,n6*sizeof(double));
			//dSetZero (tmp1,n6);
			dMultiply0 (tmp1,invM,fe,n6,n6,1);
			for (i=0; i<n6; i++) tmp1[i] += v[i]/stepsize;
			double[] rhs = new double[m];//ALLOCA(double,rhs,m*sizeof(double));
			//dSetZero (rhs,m);
			dMultiply0 (rhs,J,tmp1,m,n6,1);
			for (i=0; i<m; i++) rhs[i] = c[i]/stepsize - rhs[i];

			//#   ifdef COMPARE_METHODS
//TODO remove			if (COMPARE_METHODS)
//				comparator.nextMatrix (c,m,1,0,"c");
//			comparator.nextMatrix (rhs,m,1,0,"rhs");
			//#   endif


			//Move here by TZ
			double[] lambda = new double[m];//ALLOCA(double,lambda,m*sizeof(double));


			//#ifndef DIRECT_CHOLESKY
			if (!DIRECT_CHOLESKY) {
				throw new UnsupportedOperationException();
				//TODO does not compile:
//				// solve the LCP problem and get lambda.
//				// this will destroy A but that's okay
//				if (TIMING)
//					dTimerNow ("solving LCP problem");
//				double[] lambda = new double[m];//ALLOCA(double,lambda,m*sizeof(double));
//				double[] residual = new double[m];//ALLOCA(double,residual,m*sizeof(double));
//				dSolveLCP (m,A,lambda,rhs,residual,nub,lo,hi,findex);
//
//				//TODO ? TZ
//				//#ifdef dUSE_MALLOC_FOR_ALLOCA
//				//    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY)
//				//      return;
//				//#endif


				//#else
			} else {

				// OLD WAY - direct factor and solve

				// factorize A (L*L'=A)
				//#   ifdef TIMING
				if (TIMING)
					dTimerNow ("factorize A");
				//#   endif
				double[] L = new double[m*mskip];//ALLOCA(double,L,m*mskip*sizeof(double));
				memcpy (L,A,m*mskip);//*sizeof(double));
				if (!dFactorCholesky (L,m)) dDebug (0,"A is not positive definite");

				// compute lambda
				//#   ifdef TIMING
				if (TIMING)
					dTimerNow ("compute lambda");
				//#   endif
//TZ move upwards				double[] lambda = new double[m];//ALLOCA(double,lambda,m*sizeof(double));
				memcpy (lambda,rhs,m);// * sizeof(double));
				dSolveCholesky (L,lambda,m);
				//#endif
			} //DIRECT_CHOLESKY

			//#   ifdef COMPARE_METHODS
//TODO remove			if (COMPARE_METHODS)
//				comparator.nextMatrix (lambda,m,1,0,"lambda");
			//#   endif

			// compute the velocity update `vnew'
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("compute velocity update");
			//#   endif
			dMultiply1 (tmp1,J,lambda,n6,m,1);
			for (i=0; i<n6; i++) tmp1[i] += fe[i];
			dMultiply0 (vnew,invM,tmp1,n6,n6,1);
			for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];

			//#ifdef REPORT_ERROR
			if (REPORT_ERROR) {
				// see if the constraint has worked: compute J*vnew and make sure it equals
				// `c' (to within a certain tolerance).
				//#   ifdef TIMING
				if (TIMING)
					dTimerNow ("verify constraint equation");
				//#   endif
				dMultiply0 (tmp1,J,vnew,m,n6,1);
				double err = 0;
				for (i=0; i<m; i++) {
					err += dFabs(tmp1[i]-c[i]);
				}
				printf ("total constraint error=%.6e\n",err);
				//#endif
			} //REPORT_ERROR

		}
		else {
			// no constraints
			dMultiply0 (vnew,invM,fe,n6,n6,1);
			for (i=0; i<n6; i++) vnew[i] = v[i] + stepsize*vnew[i];
		}

		//#ifdef COMPARE_METHODS
//TODO rmeove		if (COMPARE_METHODS)
//			comparator.nextMatrix (vnew,n6,1,0,"vnew");
		//#endif

		// apply the velocity update to the bodies
		//#ifdef TIMING
		if (TIMING)
			dTimerNow ("update velocity");
		//#endif
		for (i=0; i<nb; i++) {
			for (j=0; j<3; j++) body[i].lvel.v[j] = vnew[i*6+j];
			for (j=0; j<3; j++) body[i].avel.v[j] = vnew[i*6+3+j];
		}

		// update the position and orientation from the new linear/angular velocity
		// (over the given timestep)
		//#ifdef TIMING
		if (TIMING)
			dTimerNow ("update position");
		//#endif
		for (i=0; i<nb; i++) body[i].dxStepBody (stepsize);

		//#ifdef TIMING
		if (TIMING)
			dTimerNow ("tidy up");
		//#endif

		// zero all force accumulators
		for (i=0; i<nb; i++) {
//			body[i].facc.v[0] = 0;
//			body[i].facc.v[1] = 0;
//			body[i].facc.v[2] = 0;
//			body[i].facc.v[3] = 0;
			body[i].facc.setZero();
//			body[i].tacc.v[0] = 0;
//			body[i].tacc.v[1] = 0;
//			body[i].tacc.v[2] = 0;
//			body[i].tacc.v[3] = 0;
			body[i].tacc.setZero();
		}

		//#ifdef TIMING
		if (TIMING) {
			dTimerEnd();
			if (m > 0) dTimerReport (stdout,1);
		}
		//#endif

	}

	//****************************************************************************
	// an optimized version of dInternalStepIsland1()

	//void dInternalStepIsland_x2 (dxWorld *world, dxBody * const *body, int nb,
	//	     dxJoint * const *_joint, int nj, double stepsize)
	private void dInternalStepIsland_x2 (DxWorld world, final DxBody[] body, 
			int nb,
			final DxJoint[] _joint, int nj, double stepsize)
	{
		int i,j,k;
		if (TIMING) //#ifdef TIMING
			dTimerStart("preprocessing");
		//#endif

		double stepsize1 = dRecip(stepsize);

		// number all bodies in the body list - set their tag values
		for (i=0; i<nb; i++) body[i].tag = i;

		// make a local copy of the joint array, because we might want to modify it.
		// (the "dxJoint *const*" declaration says we're allowed to modify the joints
		// but not the joint array, because the caller might need it unchanged).
		DxJoint[] joint = new DxJoint[nj];//ALLOCA(dxJoint*,joint,nj*sizeof(dxJoint*));
		System.arraycopy(_joint, 0, joint, 0, nj);
		//memcpy (joint,_joint,nj);// * sizeof(dxJoint*));

		// for all bodies, compute the inertia tensor and its inverse in the global
		// frame, and compute the rotational force and add it to the torque
		// accumulator. invI are vertically stacked 3x4 matrices, one per body.
		// @@@ check computation of rotational force.

		double[] invI = new double[3*nb*4];//ALLOCA(double,invI,3*nb*4*sizeof(double));

		//dSetZero (I,3*nb*4);
		//dSetZero (invI,3*nb*4);
		//TZ:
		DMatrix3 tmpM = new DMatrix3();
		DVector3 tmpV = new DVector3();
		DMatrix3 tmpI = new DMatrix3();
		for (i=0; i<nb; i++) {
			//TZ double[] tmp=new double[12];

			// compute inverse inertia tensor in global frame
			dMULTIPLY2_333 (tmpM,body[i].invI,body[i]._posr.R);
			dMULTIPLY0_333 (invI,i*12,body[i]._posr.R.v,0,tmpM.v,0);

		    if (body[i].isFlagsGyroscopic()) {
		        //TZ move up for performance 
		    	//DMatrix3 I = new DMatrix3();
				// compute inertia tensor in global frame
				dMULTIPLY2_333 (tmpM,body[i].mass._I,body[i]._posr.R);
				dMULTIPLY0_333 (tmpI,body[i]._posr.R,tmpM);

				// compute rotational force
				dMULTIPLY0_331 (tmpV,tmpI,body[i].avel);
				dCROSS (body[i].tacc,OP.SUB_EQ,body[i].avel,tmpV);
			}
		}

		// add the gravity force to all bodies
		for (i=0; i<nb; i++) {
			if ((body[i].flags & DxBody.dxBodyNoGravity)==0) {
//				body[i].facc.v[0] += body[i].mass._mass * world.gravity.v[0];
//				body[i].facc.v[1] += body[i].mass._mass * world.gravity.v[1];
//				body[i].facc.v[2] += body[i].mass._mass * world.gravity.v[2];
				body[i].facc.eqSum(body[i].facc, world.gravity, body[i].mass._mass);
			}
		}

		// get m = total constraint dimension, nub = number of unbounded variables.
		// create constraint offset array and number-of-rows array for all joints.
		// the constraints are re-ordered as follows: the purely unbounded
		// constraints, the mixed unbounded + LCP constraints, and last the purely
		// LCP constraints. this assists the LCP solver to put all unbounded
		// variables at the start for a quick factorization.
		//
		// joints with m=0 are inactive and are removed from the joints array
		// entirely, so that the code that follows does not consider them.
		// also number all active joints in the joint list (set their tag values).
		// inactive joints receive a tag value of -1.

		int m = 0;
		DxJoint.Info1[] info = new DxJoint.Info1[nj];//ALLOCA(dxJoint.Info1,info,nj*sizeof(dxJoint.Info1));
		for (int ii = 0; ii < info.length; ii++) info[ii] = new DxJoint.Info1();
		int[] ofs = new int[nj];//ALLOCA(int,ofs,nj*sizeof(int));
		for (i=0, j=0; j<nj; j++) {	// i=dest, j=src
			joint[j].getInfo1 (info[i]);
			dIASSERT (info[i].m >= 0 && info[i].m <= 6 &&
					info[i].nub >= 0 && info[i].nub <= info[i].m);
			if (info[i].m > 0) {
				joint[i] = joint[j];
				joint[i].tag = i;
				i++;
			}
			else {
				joint[j].tag = -1;
			}
		}
		nj = i;

		// the purely unbounded constraints
		for (i=0; i<nj; i++) if (info[i].nub == info[i].m) {
			ofs[i] = m;
			m += info[i].m;
		}
		int nub = m;
		// the mixed unbounded + LCP constraints
		for (i=0; i<nj; i++) if (info[i].nub > 0 && info[i].nub < info[i].m) {
			ofs[i] = m;
			m += info[i].m;
		}
		// the purely LCP constraints
		for (i=0; i<nj; i++) if (info[i].nub == 0) {
			ofs[i] = m;
			m += info[i].m;
		}

		// this will be set to the force due to the constraints
		double[] cforce = new double[nb*8];//ALLOCA(double,cforce,nb*8*sizeof(double));
		//TZ dSetZero (cforce,nb*8);

		// if there are constraints, compute cforce
		if (m > 0) {
			// create a constraint equation right hand side vector `c', a constraint
			// force mixing vector `cfm', and LCP low and high bound vectors, and an
			// 'findex' vector.
			double[] c = new double[m];//ALLOCA(double,c,m*sizeof(double));
			double[] cfm = new double[m];//ALLOCA(double,cfm,m*sizeof(double));
			double[] lo = new double[m];//ALLOCA(double,lo,m*sizeof(double));
			double[] hi = new double[m];//ALLOCA(double,hi,m*sizeof(double));
			int[] findex = new int[m];//ALLOCA(int,findex,m*sizeof(int));
			//TZ dSetZero (c,m);
			dSetValue (cfm,m,world.global_cfm);
			dSetValue (lo,m,-dInfinity);
			dSetValue (hi,m, dInfinity);
			for (i=0; i<m; i++) findex[i] = -1;

			// get jacobian data from constraints. a (2*m)x8 matrix will be created
			// to store the two jacobian blocks from each constraint. it has this
			// format:
			//
			//   l l l 0 a a a 0  \    .
			//   l l l 0 a a a 0   }-- jacobian body 1 block for joint 0 (3 rows)
			//   l l l 0 a a a 0  /
			//   l l l 0 a a a 0  \    .
			//   l l l 0 a a a 0   }-- jacobian body 2 block for joint 0 (3 rows)
			//   l l l 0 a a a 0  /
			//   l l l 0 a a a 0  }--- jacobian body 1 block for joint 1 (1 row)
			//   l l l 0 a a a 0  }--- jacobian body 2 block for joint 1 (1 row)
			//   etc...
			//
			//   (lll) = linear jacobian data
			//   (aaa) = angular jacobian data
			//
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("create J");
			//#   endif
			double[] J = new double[2*m*8];//ALLOCA(double,J,2*m*8*sizeof(double));
			//TZ dSetZero (J,2*m*8);
			DxJoint.Info2 Jinfo = new DxJoint.Info2();
			Jinfo.setRowskip(8);
			Jinfo.setArrays(J, c, cfm, lo, hi, findex);
			Jinfo.fps = stepsize1;
			Jinfo.erp = world.global_erp;
			for (i=0; i<nj; i++) {
				Jinfo.J1lp = 2*8*ofs[i];//J + 2*8*ofs[i];
				Jinfo.J1ap = Jinfo.J1lp + 4;
				Jinfo.J2lp = Jinfo.J1lp + 8*info[i].m;
				Jinfo.J2ap = Jinfo.J2lp + 4;
//				Jinfo.c = c + ofs[i];
//				Jinfo.cfm = cfm + ofs[i];
//				Jinfo.lo = lo + ofs[i];
//				Jinfo.hi = hi + ofs[i];
//				Jinfo.findex = findex + ofs[i];
				Jinfo.setAllP(ofs[i]);
				joint[i].getInfo2 (Jinfo);


				// adjust returned findex values for global index numbering
				for (j=0; j<info[i].m; j++) {
					if (findex[ofs[i] + j] >= 0) findex[ofs[i] + j] += ofs[i];
				}
			}

			// compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
			// format as J so we just go through the constraints in J multiplying by
			// the appropriate scalars and matrices.
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("compute A");
			//#   endif
			double[] JinvM = new double[2*m*8];//ALLOCA(double,JinvM,2*m*8*sizeof(double));
			//TZ dSetZero (JinvM,2*m*8);
			for (i=0; i<nj; i++) {
				int b = joint[i].node[0].body.tag;
				double body_invMass = body[b].invMass;
//				double *body_invI = invI + b*12;
//				double *Jsrc = J + 2*8*ofs[i];
//				double *Jdst = JinvM + 2*8*ofs[i];
				int body_invI = b*12; //invI + b*12;
				int Jsrc = 2*8*ofs[i];//J + 2*8*ofs[i];
				int Jdst = 2*8*ofs[i]; //JinvM + 2*8*ofs[i];
				for (j=info[i].m-1; j>=0; j--) {
					//for (k=0; k<3; k++) Jdst[k] = Jsrc[k] * body_invMass;
					for (k=0; k<3; k++) JinvM[Jdst+k] = J[Jsrc+k] * body_invMass;
					dMULTIPLY0_133 (JinvM,Jdst+4,J,Jsrc+4,invI,body_invI);
					Jsrc += 8;
					Jdst += 8;
				}
				if (joint[i].node[1].body!=null) {
					b = joint[i].node[1].body.tag;
					body_invMass = body[b].invMass;
					body_invI = b*12;//invI + b*12;
					for (j=info[i].m-1; j>=0; j--) {
						//for (k=0; k<3; k++) Jdst[k] = Jsrc[k] * body_invMass;
						for (k=0; k<3; k++) JinvM[Jdst+k] = J[Jsrc+k] * body_invMass;
						dMULTIPLY0_133 (JinvM,Jdst+4,J,Jsrc+4,invI,body_invI);
						Jsrc += 8;
						Jdst += 8;
					}
				}
			}

			// now compute A = JinvM * J'. A's rows and columns are grouped by joint,
			// i.e. in the same way as the rows of J. block (i,j) of A is only nonzero
			// if joints i and j have at least one body in common. this fact suggests
			// the algorithm used to fill A:
			//
			//    for b = all bodies
			//      n = number of joints attached to body b
			//      for i = 1..n
			//        for j = i+1..n
			//          ii = actual joint number for i
			//          jj = actual joint number for j
			//          // (ii,jj) will be set to all pairs of joints around body b
			//          compute blockwise: A(ii,jj) += JinvM(ii) * J(jj)'
			//
			// this algorithm catches all pairs of joints that have at least one body
			// in common. it does not compute the diagonal blocks of A however -
			// another similar algorithm does that.

			int mskip = dPAD(m);
			double[] A = new double[m*mskip];//ALLOCA(double,A,m*mskip*sizeof(double));
			//TZ dSetZero (A,m*mskip);
			for (i=0; i<nb; i++) {
				//        for (dxJointNode *n1=body[i].firstjoint; n1!=null; n1=n1.next) {
				for (DxJointNode n1=body[i].firstjoint.get(); n1!=null; n1=n1.next) {
					//    	  for (dxJointNode *n2=n1.next; n2; n2=n2.next) {
					for (DxJointNode n2=n1.next; n2!=null; n2=n2.next) {
						// get joint numbers and ensure ofs[j1] >= ofs[j2]
						int j1 = n1.joint.tag;
						int j2 = n2.joint.tag;
						if (ofs[j1] < ofs[j2]) {
							int tmp = j1;
							j1 = j2;
							j2 = tmp;
						}

						// if either joint was tagged as -1 then it is an inactive (m=0)
						// joint that should not be considered
						if (j1==-1 || j2==-1) continue;

						// determine if body i is the 1st or 2nd body of joints j1 and j2
						int jb1 = (joint[j1].node[1].body == body[i]) ? 1 : 0;
						int jb2 = (joint[j2].node[1].body == body[i]) ? 1 : 0;
						// jb1/jb2 must be 0 for joints with only one body
						dIASSERT(joint[j1].node[1].body!=null || jb1==0);
						dIASSERT(joint[j2].node[1].body!=null || jb2==0);

						// set block of A
						MultiplyAdd2_p8r (A , ofs[j1]*mskip + ofs[j2],
								JinvM , 2*8*ofs[j1] + jb1*8*info[j1].m,
								J     , 2*8*ofs[j2] + jb2*8*info[j2].m,
								info[j1].m,info[j2].m, mskip);
					}
				}
			}
			// compute diagonal blocks of A
			for (i=0; i<nj; i++) {
				Multiply2_p8r (A , ofs[i]*(mskip+1),
						JinvM , 2*8*ofs[i],
						J , 2*8*ofs[i],
						info[i].m,info[i].m, mskip);
				if (joint[i].node[1].body!=null) {
					MultiplyAdd2_p8r (A , ofs[i]*(mskip+1),
							JinvM , 2*8*ofs[i] + 8*info[i].m,
							J , 2*8*ofs[i] + 8*info[i].m,
							info[i].m,info[i].m, mskip);
				}
			}

			// add cfm to the diagonal of A
			for (i=0; i<m; i++) A[i*mskip+i] += cfm[i] * stepsize1;

			//#   ifdef COMPARE_METHODS
//TODO remove			if (COMPARE_METHODS)
//				comparator.nextMatrix (A,m,m,1,"A");
			//#   endif

			// compute the right hand side `rhs'
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("compute rhs");
			//#   endif
			double[] tmp1 = new double[nb*8];//ALLOCA(double,tmp1,nb*8*sizeof(double));
			//dSetZero (tmp1,nb*8);
			// put v/h + invM*fe into tmp1
			for (i=0; i<nb; i++) {
				double body_invMass = body[i].invMass;
				int body_invI = i*12;//double []body_invI = invI + i*12;
				for (j=0; j<3; j++) tmp1[i*8+j] = 
					body[i].facc.v[j] * body_invMass +
					body[i].lvel.v[j] * stepsize1;
				dMULTIPLY0_331 (tmp1 , i*8 + 4,invI,body_invI,body[i].tacc.v,0);
				for (j=0; j<3; j++) tmp1[i*8+4+j] += body[i].avel.v[j] * stepsize1;
			}
			// put J*tmp1 into rhs
			double[]rhs = new double[m];//ALLOCA(double,rhs,m*sizeof(double));
			//dSetZero (rhs,m);
			for (i=0; i<nj; i++) {
				int JJ = 2*8*ofs[i];//double []JJ = J + 2*8*ofs[i];
				Multiply0_p81 (rhs,ofs[i],J,JJ,
						tmp1 , 8*joint[i].node[0].body.tag, info[i].m);
				if (joint[i].node[1].body!=null) {
					MultiplyAdd0_p81 (rhs,ofs[i],J, JJ + 8*info[i].m,
							tmp1 , 8*joint[i].node[1].body.tag, info[i].m);
				}
			}
			// complete rhs
			for (i=0; i<m; i++) rhs[i] = c[i]*stepsize1 - rhs[i];

			//#   ifdef COMPARE_METHODS
//TODO remove			if (COMPARE_METHODS) {
//				comparator.nextMatrix (c,m,1,0,"c");
//				comparator.nextMatrix (rhs,m,1,0,"rhs");
//			}//#   endif

			// solve the LCP problem and get lambda.
			// this will destroy A but that's okay
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("solving LCP problem");
			//#   endif
			double[] lambda = new double[m];//ALLOCA(double,lambda,m*sizeof(double));
			double[] residual = new double[m];//ALLOCA(double,residual,m*sizeof(double));
			DLCP.dSolveLCP (m,A,lambda,rhs,residual,nub,lo,hi,findex);

			//TODO TZ
			//#ifdef dUSE_MALLOC_FOR_ALLOCA
			//    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY)
			//      return;
			//#endif


			//  OLD WAY - direct factor and solve
			//
			//    // factorize A (L*L'=A)
			//#   ifdef TIMING
			//    dTimerNow ("factorize A");
			//#   endif
			//    dReal *L = (dReal*) ALLOCA (m*mskip*sizeof(dReal));
			//    memcpy (L,A,m*mskip*sizeof(dReal));
			//#   ifdef FAST_FACTOR
			//    dFastFactorCholesky (L,m);  // does not report non positive definiteness
			//#   else
			//    if (dFactorCholesky (L,m)==0) dDebug (0,"A is not positive definite");
			//#   endif
			//
			//    // compute lambda
			//#   ifdef TIMING
			//    dTimerNow ("Timer.dTimere lambda");
			//#   endif
			//    dReal *lambda = (dReal*) ALLOCA (m * sizeof(dReal));
			//    memcpy (lambda,rhs,m * sizeof(dReal));
			//    dSolveCholesky (L,lambda,m);

			//#   ifdef COMPARE_METHODS
//TODO remove			if (COMPARE_METHODS)
//				comparator.nextMatrix (lambda,m,1,0,"lambda");
			//#   endif

			// compute the constraint force `cforce'
			//#   ifdef TIMING
			if (TIMING)
				dTimerNow ("compute constraint force");
			//#   endif
			// compute cforce = J'*lambda
			for (i=0; i<nj; i++) {
				int JJ = 2*8*ofs[i];//double[] JJ = J + 2*8*ofs[i];
				DxBody b1 = joint[i].node[0].body;
				DxBody b2 = joint[i].node[1].body;
				DJoint.DJointFeedback fb = joint[i].feedback;

				if (fb!=null) {
					// the user has requested feedback on the amount of force that this
					// joint is applying to the bodies. we use a slightly slower
					// computation that splits out the force components and puts them
					// in the feedback structure.
					double[] data=new double[8];

					Multiply1_8q1 (data,0, J,JJ, lambda,ofs[i], info[i].m);
					int cf1 = 8*b1.tag;//double[] cf1 = cforce + 8*b1.tag;
					//TODO FIXME
					if (true) throw new UnsupportedOperationException("Fix this!");
					cforce[cf1+0] += (fb.f1.v[0] = data[0]);
					cforce[cf1+1] += (fb.f1.v[1] = data[1]);
					cforce[cf1+2] += (fb.f1.v[2] = data[2]);
					cforce[cf1+4] += (fb.t1.v[0] = data[4]);
					cforce[cf1+5] += (fb.t1.v[1] = data[5]);
					cforce[cf1+6] += (fb.t1.v[2] = data[6]);
					if (b2!=null){
						Multiply1_8q1 (data,0, J,JJ + 8*info[i].m, lambda,ofs[i], info[i].m);
						int cf2 = 8*b2.tag;//double[] cf2 = cforce + 8*b2.tag;
						//TODO FIXME
						if (true) throw new UnsupportedOperationException("Fix this!");
						cforce[cf2+0] += (fb.f2.v[0] = data[0]);
						cforce[cf2+1] += (fb.f2.v[1] = data[1]);
						cforce[cf2+2] += (fb.f2.v[2] = data[2]);
						cforce[cf2+4] += (fb.t2.v[0] = data[4]);
						cforce[cf2+5] += (fb.t2.v[1] = data[5]);
						cforce[cf2+6] += (fb.t2.v[2] = data[6]);
					}
				}
				else {
					// no feedback is required, let's compute cforce the faster way
					MultiplyAdd1_8q1 (cforce , 8*b1.tag,J,JJ, lambda,ofs[i], 
							info[i].m);
					if (b2!=null) {
						MultiplyAdd1_8q1 (cforce , 8*b2.tag,
								J,JJ + 8*info[i].m, lambda,ofs[i], info[i].m);
					}
				}
			}
		}

		// compute the velocity update
		//#ifdef TIMING
		if (TIMING)
			dTimerNow ("compute velocity update");
		//#endif

		// add fe to cforce
		for (i=0; i<nb; i++) {
			for (j=0; j<3; j++) cforce[i*8+j] += body[i].facc.get(j);
			for (j=0; j<3; j++) cforce[i*8+4+j] += body[i].tacc.get(j);
		}
		// multiply cforce by stepsize
		for (i=0; i < nb*8; i++) cforce[i] *= stepsize;
		// add invM * cforce to the body velocity
		for (i=0; i<nb; i++) {
			double body_invMass = body[i].invMass;
			int body_invI = i*12;//double []body_invI = invI + i*12;
			for (j=0; j<3; j++) body[i].lvel.add(j, body_invMass * cforce[i*8+j] );
			OdeMath.dMULTIPLYADD0_331 (body[i].avel.v,0,invI,body_invI,cforce,i*8+4);
		}

		// update the position and orientation from the new linear/angular velocity
		// (over the given timestep)
		//# ifdef TIMING
		if (TIMING)
			dTimerNow ("update position");
		//# endif
		for (i=0; i<nb; i++) body[i].dxStepBody (stepsize);

		//#ifdef COMPARE_METHODS
//TODO remove		if (COMPARE_METHODS) {
//			double[] tmp = new double[nb*6];//ALLOCA(double,tmp, nb*6*sizeof(double));
//			for (i=0; i<nb; i++) {
//				for (j=0; j<3; j++) tmp_vnew[i*6+j] = body[i].lvel.v[j];
//				for (j=0; j<3; j++) tmp_vnew[i*6+3+j] = body[i].avel.v[j];
//			}
//			comparator.nextMatrix (tmp_vnew,nb*6,1,0,"vnew");
//		}//#endif

		//#ifdef TIMING
		if (TIMING)
			dTimerNow ("tidy up");
		//#endif

		// zero all force accumulators
		for (i=0; i<nb; i++) {
			body[i].facc.dSetZero();
			body[i].tacc.dSetZero();
//			body[i].facc[0] = 0;
//			body[i].facc[1] = 0;
//			body[i].facc[2] = 0;
//			body[i].facc[3] = 0;
//			body[i].tacc[0] = 0;
//			body[i].tacc[1] = 0;
//			body[i].tacc[2] = 0;
//			body[i].tacc[3] = 0;
		}

		//#ifdef TIMING
		if (TIMING) {
			dTimerEnd();
			if (m > 0) dTimerReport (stdout,1);
		} //#endif

	}

	
	//****************************************************************************

	//void dInternalStepIsland (dxWorld *world, dxBody * const *body, int nb,
	//			  dxJoint * const *joint, int nj, double stepsize)
	void dInternalStepIsland (DxWorld world, DxBody[] body, int nb,
			DxJoint[] joint, int nj, double stepsize)
	{

		//TODO TZ
		//#ifdef dUSE_MALLOC_FOR_ALLOCA
		//  dMemoryFlag = d_MEMORY_OK;
		//#endif

		//#ifndef COMPARE_METHODS
//TODO remove		if (!COMPARE_METHODS) {
			dInternalStepIsland_x2 (world,body,nb,joint,nj,stepsize);

			//TODO TZ
			//#ifdef dUSE_MALLOC_FOR_ALLOCA
			//    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY) {
			//      REPORT_OUT_OF_MEMORY;
			//      return;
			//    }
			//#endif

//		}//#endif

		//#ifdef COMPARE_METHODS
//TODO remove		if (COMPARE_METHODS) {
//			int i;
//
//			// save body state
//			dxBody[] state = new dxBody[nb];//ALLOCA(dxBody,state,nb*sizeof(dxBody));
//
//			for (i=0; i<nb; i++) 
//				state[i] = (dxBody)body[i].clone();//memcpy (state+i,body[i],sizeof(dxBody));
//
//			// take slow step
//			comparator.reset();
//			dInternalStepIsland_x1 (world,body,nb,joint,nj,stepsize);
//			comparator.end();
//			//TODO TZ
//			//#ifdef dUSE_MALLOC_FOR_ALLOCA
//			//  if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY) {
//			//    REPORT_OUT_OF_MEMORY;
//			//    return;
//			//  }
//			//#endif
//
//			// restore state
//			for (i=0; i<nb; i++)  
//				state[i] = (dxBody)body[i].clone();//memcpy (body[i],state+i,sizeof(dxBody));
//
//			// take fast step
//			dInternalStepIsland_x2 (world,body,nb,joint,nj,stepsize);
//			comparator.end();
//			//TODO TZ
//			//#ifdef dUSE_MALLOC_FOR_ALLOCA
//			//    if (dMemoryFlag == d_MEMORY_OUT_OF_MEMORY) {
//			//      REPORT_OUT_OF_MEMORY;
//			//      return;
//			//    }
//			//#endif
//
//			//comparator.dump();
//			//_exit (1);
//		} //#endif COMPARE_METHODS
	}


	public void run(DxWorld world, DxBody[] body, int nb, DxJoint[] _joint, 
			int nj, double stepsize) {
		dInternalStepIsland(world, body, nb, _joint, nj, stepsize);
	}
}

