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

import java.util.Comparator;

import org.ode4j.ode.DJoint;
import org.ode4j.ode.internal.Objects_H.dxQuickStepParameters;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.math.DMatrix3;

import static org.cpp4j.Cstdio.*;
import static org.ode4j.ode.internal.Timer.*;
import static org.ode4j.ode.OdeMath.*;

class DxQuickStep extends AbstractStepper implements DxWorld.dstepper_fn_t {
	
	public static DxQuickStep INSTANCE = new DxQuickStep();
	
	//void dxQuickStepper (dxWorld *world, dxBody * const *body, int nb,
	//	     dxJoint * const *_joint, int nj, dReal stepsize);


	private static final boolean TIMING = false;


	//***************************************************************************
	// configuration

	/** for the SOR and CG methods:
	 * uncomment the following line to use warm starting. this definitely
	 * help for motor-driven joints. unfortunately it appears to hurt
	 * with high-friction contacts using the SOR method. use with care
	 */
	//not defined because 'ifdef 0' around 'multiply_invM_JT' causes comp error.
	//private static boolean WARM_STARTING = false;

	/** for the SOR method:
	 * uncomment the following line to determine a new constraint-solving
	 * order for each iteration. however, the qsort per iteration is expensive,
	 * and the optimal order is somewhat problem dependent.
	 * @@@ try the leaf.root ordering.
	 */
	private static final boolean REORDER_CONSTRAINTS = false;


	/** for the SOR method:
	 * uncomment the following line to randomly reorder constraint rows
	 * during the solution. depending on the situation, this can help a lot
	 * or hardly at all, but it doesn't seem to hurt.
	 */
	private static final boolean RANDOMLY_REORDER_CONSTRAINTS = true;

	//****************************************************************************
	// special matrix multipliers

	// multiply block of B matrix (q x 6) with 12 dReal per row with C vektor (q)
	private static void Multiply1_12q1 (double[] A, double[] B, int ofsB, 
			double[] C, int ofsC, int q)
	{
		int i, k;
		dIASSERT (q>0 && A!=null && B!=null && C!=null);

		double a = 0;
		double b = 0;
		double c = 0;
		double d = 0;
		double e = 0;
		double f = 0;
		double s;

		//TODO add ofsB to k  and ofsC to i
//		for(i=0, k = 0; i<q; i++, k += 12)
		for(i=ofsC, k = ofsB; i<q+ofsC; i++, k += 12)
		{
			s = C[i]; //C[i] and B[n+k] cannot overlap because its value has been read into a temporary.

			//For the rest of the loop, the only memory dependency (array) is from B[]
			a += B[  k] * s;
			b += B[1+k] * s;
			c += B[2+k] * s;
			d += B[3+k] * s;
			e += B[4+k] * s;
			f += B[5+k] * s;
		}

		A[0] = a;
		A[1] = b;
		A[2] = c;
		A[3] = d;
		A[4] = e;
		A[5] = f;
	}

	//***************************************************************************
	// testing stuff

	//#ifdef TIMING
	//#define IFTIMING(x) x
	//#else
	//#define IFTIMING(x) /* */
	//#endif


	//***************************************************************************
	// various common computations involving the matrix J

	/** 
	 * compute iMJ = inv(M)*J'
	 * (TZ) Performs a 331 multiplication on the 3-5 and 9-11 parts of
	 * each row and writes the result into iMJ.  
	 */

	//static void compute_invM_JT (int m, dRealMutablePtr J, dRealMutablePtr iMJ, int *jb,
	//	dxBody * const *body, dRealPtr invI)
	private static void compute_invM_JT (int m, double[] J, double[] iMJ, int[]jb,
			DxBody[]body, final double[] invI)
	{
		int i,j;
		//dRealMutablePtr iMJ_ptr = iMJ;
		//dRealMutablePtr J_ptr = J;
		int iMJ_ofs = 0;//TZ
		int J_ofs = 0;//TZ
		for (i=0; i<m; i++) {
			int b1 = jb[i*2];
			int b2 = jb[i*2+1];
			double k = body[b1].invMass;
//TZ			for (j=0; j<3; j++) iMJ_ptr[j] = k*J_ptr[j];
			for (j=0; j<3; j++) iMJ[j + iMJ_ofs] = k*J[j + J_ofs];
//			dMULTIPLY0_331 (iMJ_ptr + 3, invI + 12*b1, J_ptr + 3);
			dMULTIPLY0_331 (iMJ, iMJ_ofs + 3, invI, 12*b1, J,J_ofs + 3);
			if (b2 >= 0) {
				k = body[b2].invMass;
				//for (j=0; j<3; j++) iMJ_ptr[j+6] = k*J_ptr[j+6];
				for (j=0; j<3; j++) iMJ[j+6+iMJ_ofs] = k*J[j+6+J_ofs];
				//dMULTIPLY0_331 (iMJ_ptr + 9, invI + 12*b2, J_ptr + 9);
				dMULTIPLY0_331 (iMJ, iMJ_ofs + 9, invI, 12*b2, J, J_ofs + 9);
			}
			J_ofs +=12;//J_ptr += 12;
			iMJ_ofs += 12;//iMJ_ptr += 12;
		}
	}


	// compute out = inv(M)*J'*in.
	//#if 0
	//static void multiply_invM_JT (int m, int nb, dRealMutablePtr iMJ, int[] *jb,
	//		dRealMutablePtr in, dRealMutablePtr out)
//	private static void multiply_invM_JT (int m, int nb, double[] iMJ, int[] jb,
//			double[] in, double[] out)
//	{
//		if (true) {
//			return;  //By TZ for #If 0
//		}
//		int i,j;
//		dSetZero (out,6*nb);
//		int iMJ_ofs = 0;//final double[] iMJ_ptr = iMJ;
//		for (i=0; i<m; i++) {
//			int b1 = jb[i*2];
//			int b2 = jb[i*2+1];
//			int out_ofs = b1*6;//double[] out_ptr = out + b1*6;
//			//for (j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in[i];
//			for (j=0; j<6; j++) out[j + out_ofs] += iMJ[j + iMJ_ofs] * in[i];
//			iMJ_ofs +=6;//iMJ_ptr += 6;
//			if (b2 >= 0) {
//				out_ofs = b2*6;//out_ptr = out + b2*6;
//				//for (j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in[i];
//				for (j=0; j<6; j++) out[j + out_ofs] += iMJ[j + iMJ_ofs] * in[i];
//			}
//			iMJ_ofs +=6;//iMJ_ptr += 6;
//		}
//	}
	//#endif

	/** 
	 * compute out = J*in. 
	 * (TZ) Calculates the sum of each row 'm' in 'J', using only tagged bodies, and 
	 * writes the sum into 'out'.
	 * @param m total number of unbound variables, max. 6 per joint
	 * @param J double[m*12]
	 * @param jb int[m*2] of body tags. Paired per joint.
	 * @param in double[nb*6]
	 * @param out double[m]
	 */

	//static void multiply_J (int m, dRealMutablePtr J, int *jb,
	//		dRealMutablePtr in, dRealMutablePtr out)
	private static void multiply_J (int m, double[] J, int[]jb,
			double[] in, double[] out)
	{
		int i,j;
		int J_ofs = 0;//final double[] J_ptr = J;
		for (i=0; i<m; i++) {
			int b1 = jb[i*2];
			int b2 = jb[i*2+1];
			double sum = 0;
			int in_ofs = b1*6; //double[] in_ptr = in + b1*6;
			for (j=0; j<6; j++) sum += J[j + J_ofs] * in[j + in_ofs];//J_ptr[j]*in_ptr[j];
			J_ofs += 6;//J_ptr += 6;
			if (b2 >= 0) {
				in_ofs = b2*6;//in_ptr = in + b2*6;
				for (j=0; j<6; j++) sum += J[j + J_ofs] * in[j + in_ofs];//J_ptr[j] * in_ptr[j];
			}
			J_ofs += 6;//J_ptr += 6;
			out[i] = sum;
		}
	}


	// compute out = (J*inv(M)*J' + cfm)*in.
	// use z as an nb*6 temporary.
	//#if 0
	//static void multiply_J_invM_JT (int m, int nb, dRealMutablePtr J, dRealMutablePtr iMJ, int *jb,
	//		dRealPtr cfm, dRealMutablePtr z, dRealMutablePtr in, dRealMutablePtr out)
	//TODO TZ: not used?
//	private static void multiply_J_invM_JT (int m, int nb, double[] J, double[] iMJ, 
//			int []jb,
//			final double[] cfm, double[] z, double[] in, double[] out)
//	{
//		if (true) {return;} //TZ #if 0
//		multiply_invM_JT (m,nb,iMJ,jb,in,z);
//		multiply_J (m,J,jb,z,out);
//
//		// add cfm
//		for (int i=0; i<m; i++) out[i] += cfm[i] * in[i];
//	}
	//#endif

	//***************************************************************************
	// conjugate gradient method with jacobi preconditioner
	// THIS IS EXPERIMENTAL CODE that doesn't work too well, so it is ifdefed out.
	//
	// adding CFM seems to be critically important to this method.

	//TZ#if 0
	//
	////static inline dReal dot (int n, dRealPtr x, dRealPtr y)
	//static double dot (int n, dRealPtr x, dRealPtr y)
	//{
	//	double sum=0;
	//	for (int i=0; i<n; i++) sum += x[i]*y[i];
	//	return sum;
	//}
	//
	//
	//// x = y + z*alpha
	//
	//static inline void add (int n, dRealMutablePtr x, dRealPtr y, dRealPtr z, dReal alpha)
	//{
	//	for (int i=0; i<n; i++) x[i] = y[i] + z[i]*alpha;
	//}
	//
	//
	//static void CG_LCP (int m, int nb, dRealMutablePtr J, int *jb, dxBody * const *body,
	//	dRealPtr invI, dRealMutablePtr lambda, dRealMutablePtr fc, dRealMutablePtr b,
	//	dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
	//	dxQuickStepParameters *qs)
	//{
	//	int i,j;
	//	const int num_iterations = qs.num_iterations;
	//
	//	// precompute iMJ = inv(M)*J'
	//	dRealAllocaArray (iMJ,m*12);
	//	compute_invM_JT (m,J,iMJ,jb,body,invI);
	//
	//	double last_rho = 0;
	//	dRealAllocaArray (r,m);
	//	dRealAllocaArray (z,m);
	//	dRealAllocaArray (p,m);
	//	dRealAllocaArray (q,m);
	//
	//	// precompute 1 / diagonals of A
	//	dRealAllocaArray (Ad,m);
	//	dRealPtr iMJ_ptr = iMJ;
	//	dRealPtr J_ptr = J;
	//	for (i=0; i<m; i++) {
	//		double sum = 0;
	//		for (j=0; j<6; j++) sum += iMJ_ptr[j] * J_ptr[j];
	//		if (jb[i*2+1] >= 0) {
	//			for (j=6; j<12; j++) sum += iMJ_ptr[j] * J_ptr[j];
	//		}
	//		iMJ_ptr += 12;
	//		J_ptr += 12;
	//		Ad[i] = REAL(1.0) / (sum + cfm[i]);
	//	}
	//
	//#ifdef WARM_STARTING
	//	// compute residual r = b - A*lambda
	//	multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,lambda,r);
	//	for (i=0; i<m; i++) r[i] = b[i] - r[i];
	//#else
	//	dSetZero (lambda,m);
	//	memcpy (r,b,m*sizeof(dReal));		// residual r = b - A*lambda
	//#endif
	//
	//	for (int iteration=0; iteration < num_iterations; iteration++) {
	//		for (i=0; i<m; i++) z[i] = r[i]*Ad[i];	// z = inv(M)*r
	//		double rho = dot (m,r,z);		// rho = r'*z
	//
	//		// @@@
	//		// we must check for convergence, otherwise rho will go to 0 if
	//		// we get an exact solution, which will introduce NaNs into the equations.
	//		if (rho < 1e-10) {
	//			printf ("CG returned at iteration %d\n",iteration);
	//			break;
	//		}
	//
	//		if (iteration==0) {
	//			memcpy (p,z,m*sizeof(dReal));	// p = z
	//		}
	//		else {
	//			add (m,p,z,p,rho/last_rho);	// p = z + (rho/last_rho)*p
	//		}
	//
	//		// compute q = (J*inv(M)*J')*p
	//		multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,p,q);
	//
	//		double alpha = rho/dot (m,p,q);		// alpha = rho/(p'*q)
	//		add (m,lambda,lambda,p,alpha);		// lambda = lambda + alpha*p
	//		add (m,r,r,q,-alpha);			// r = r - alpha*q
	//		last_rho = rho;
	//	}
	//
	//	// compute fc = inv(M)*J'*lambda
	//	multiply_invM_JT (m,nb,iMJ,jb,lambda,fc);
	//
	////TZ#if 0
	////	// measure solution error
	////	multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,lambda,r);
	////	double error = 0;
	////	for (i=0; i<m; i++) error += dFabs(r[i] - b[i]);
	////	printf ("lambda error = %10.6e\n",error);
	////#endif
	//}
	//
	//#endif

	//***************************************************************************
	// SOR-LCP method

	// nb is the number of bodies in the body array.
	// J is an m*12 matrix of constraint rows
	// jb is an array of first and second body numbers for each constraint row
	// invI is the global frame inverse inertia for each body (stacked 3x3 matrices)
	//
	// this returns lambda and fc (the constraint force).
	// note: fc is returned as inv(M)*J'*lambda, the constraint force is actually J'*lambda
	//
	// b, lo and hi are modified on exit


	private static class IndexError {
		double error;		// error to sort on
		int findex;
		int index;		// row index
	}


	//#ifdef REORDER_CONSTRAINTS

	//static int compare_index_error (const void *a, const void *b)
	private static class IndexErrorComparator implements Comparator<IndexError> {
//	static int compare_index_error (final IndexError a, final IndexError b)
//	{
//		//	if (!REORDER_CONSTRAINTS) { return;} //TZ
//		if (!REORDER_CONSTRAINTS) { throw new IllegalStateException();} //TZ
//		final IndexError i1 = (IndexError) a;
//		final IndexError i2 = (IndexError) b;
//		if (i1.findex < 0 && i2.findex >= 0) return -1;
//		if (i1.findex >= 0 && i2.findex < 0) return 1;
//		if (i1.error < i2.error) return -1;
//		if (i1.error > i2.error) return 1;
//		return 0;
//	}

		//TODO is sort order correct (-1,0,1)??
		public int compare(IndexError a, IndexError b) {
			if (!REORDER_CONSTRAINTS) { throw new IllegalStateException();} //TZ
			final IndexError i1 = (IndexError) a;
			final IndexError i2 = (IndexError) b;
			if (i1.findex < 0 && i2.findex >= 0) return -1;
			if (i1.findex >= 0 && i2.findex < 0) return 1;
			if (i1.error < i2.error) return -1;
			if (i1.error > i2.error) return 1;
			return 0;
		}
	}
	//#endif


	//static void SOR_LCP (int m, int nb, dRealMutablePtr J, int *jb, dxBody * const *body,
	//		dRealPtr invI, dRealMutablePtr lambda, dRealMutablePtr fc, dRealMutablePtr b,
	//		dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
	//		dxQuickStepParameters *qs)
	private static void SOR_LCP (int m, int nb, double[] J, int[] jb, DxBody []body,
			final double[] invI, double[] lambda, double[] fc, double[] b,
			double[] lo, double[] hi, final double[] cfm, int []findex,
			dxQuickStepParameters qs)
	{
		final int num_iterations = qs.num_iterations;
		final double sor_w = qs.w;		// SOR over-relaxation parameter

		int j;

		//TZ not defined
//		if (WARM_STARTING) {//#ifdef WARM_STARTING
//			// for warm starting, this seems to be necessary to prevent
//			// jerkiness in motor-driven joints. i have no idea why this works.
//			for (int i=0; i<m; i++) lambda[i] *= 0.9;
//		} else { //#else
			dSetZero (lambda,m);
//		}//#endif

		double[] last_lambda = null;
		if (REORDER_CONSTRAINTS) {//TZ #ifdef REORDER_CONSTRAINTS
			// the lambda computed at the previous iteration.
			// this is used to measure error for when we are reordering the indexes.
			last_lambda = new double[m];
			//dRealAllocaArray (last_lambda,m);
		}//#endif

		// a copy of the 'hi' vector in case findex[] is being used
		double[] hicopy = new double[m];//dRealAllocaArray (hicopy,m);
		memcpy (hicopy,hi,m);//*sizeof(dReal));

		// precompute iMJ = inv(M)*J'
		//double[] iMJ = new double[m*12];//dRealAllocaArray (iMJ,m*12);
		double[] iMJ = new double[m*12];//dRealAllocaArray (iMJ,m*12);
		compute_invM_JT (m,J,iMJ,jb,body,invI);

		// compute fc=(inv(M)*J')*lambda. we will incrementally maintain fc
		// as we change lambda.
		//not defined:
//		if (WARM_STARTING) {//TZ #ifdef WARM_STARTING
//			throw new UnsupportedOperationException();
//			//multiply_invM_JT (m,nb,iMJ,jb,lambda,fc);
//		} else {//#else
			dSetZero (fc,nb*6);
//		}//#endif

		// precompute 1 / diagonals of A
		double[] Ad = new double[m];// dRealAllocaArray (Ad,m);
		int iMJ_ofs = 0;//final double[] iMJ_ptr = iMJ;
		int J_ofs = 0;//double[] J_ptr = J;
		for (int i=0; i<m; i++) {
			double sum = 0;
			for (j=0; j<6; j++) sum += iMJ[j+iMJ_ofs] * J[j+J_ofs];//iMJ_ptr[j] * J_ptr[j];
			if (jb[i*2+1] >= 0) {
				for (j=6; j<12; j++) sum += iMJ[j+iMJ_ofs] * J[j+J_ofs];//iMJ_ptr[j] * J_ptr[j];
			}
			iMJ_ofs +=12;//iMJ_ptr += 12;
			J_ofs +=12;//J_ptr += 12;
			Ad[i] = sor_w / (sum + cfm[i]);
		}

		// scale J and b by Ad
		J_ofs = 0;//J_ptr = J;
		for (int i=0; i<m; i++) {
			for (j=0; j<12; j++) {
				J[J_ofs] *= Ad[i];//J_ptr[0] *= Ad[i];
				J_ofs++;//J_ptr++;
			}
			b[i] *= Ad[i];

			// scale Ad by CFM. N.B. this should be done last since it is used above
			Ad[i] *= cfm[i];
		}

		// order to solve constraint rows in
		//IndexError *order = (IndexError*) ALLOCA (m*sizeof(IndexError));
		IndexError[] order = new IndexError[m];
		for (int iii=0; iii<m; iii++) order[iii] = new IndexError();

		if (!REORDER_CONSTRAINTS) {//TZ #ifndef REORDER_CONSTRAINTS
			// make sure constraints with findex < 0 come first.
			j=0;
			int k=1;

			// Fill the array from both ends
			for (int i=0; i<m; i++)
				if (findex[i] < 0)
					order[j++].index = i; // Place them at the front
				else
					order[m-k++].index = i; // Place them at the end

			dIASSERT ((j+k-1)==m); // -1 since k was started at 1 and not 0
		} //#endif

		for (int iteration=0; iteration < num_iterations; iteration++) {

			//TODO commented out for now because it's 'false'
			if (REORDER_CONSTRAINTS) {//#ifdef REORDER_CONSTRAINTS
				throw new UnsupportedOperationException(); //May work but is not tested
//				// constraints with findex < 0 always come first.
//				if (iteration < 2) {
//					// for the first two iterations, solve the constraints in
//					// the given order
//					for (int i=0; i<m; i++) {
//						order[i].error = i;
//						order[i].findex = findex[i];
//						order[i].index = i;
//					}
//				}
//				else {
//					// sort the constraints so that the ones converging slowest
//					// get solved last. use the absolute (not relative) error.
//					for (int i=0; i<m; i++) {
//						double v1 = dFabs (lambda[i]);
//						double v2 = dFabs (last_lambda[i]);
//						double max = (v1 > v2) ? v1 : v2;
//						if (max > 0) {
//							//@@@ relative error: order[i].error = dFabs(lambda[i]-last_lambda[i])/max;
//							order[i].error = dFabs(lambda[i]-last_lambda[i]);
//						}
//						else {
//							order[i].error = dInfinity;
//						}
//						order[i].findex = findex[i];
//						order[i].index = i;
//					}
//				}
//				//qsort (order,m,sizeof(IndexError),&compare_index_error);
//				Arrays.sort(order, new IndexErrorComparator());
//
//				//@@@ potential optimization: swap lambda and last_lambda pointers rather
//				//    than copying the data. we must make sure lambda is properly
//				//    returned to the caller
//				memcpy (last_lambda,lambda,m);//*sizeof(dReal));
			}//TZ #endif
			if (RANDOMLY_REORDER_CONSTRAINTS) {//#ifdef RANDOMLY_REORDER_CONSTRAINTS
				if ((iteration & 7) == 0) {
					for (int i=1; i<m; ++i) {
						IndexError tmp = order[i];
						int swapi = (int) dRandInt(i+1);
						order[i] = order[swapi];
						order[swapi] = tmp;
					}
				}
			}//#endif

			for (int i=0; i<m; i++) {
				// @@@ potential optimization: we could pre-sort J and iMJ, thereby
				//     linearizing access to those arrays. hmmm, this does not seem
				//     like a win, but we should think carefully about our memory
				//     access pattern.
				int index = order[i].index;
				J_ofs = index*12;//J_ptr = J + index*12;
				iMJ_ofs = index*12;//iMJ_ptr = iMJ + index*12;

				// set the limits for this constraint. note that 'hicopy' is used.
				// this is the place where the QuickStep method differs from the
				// direct LCP solving method, since that method only performs this
				// limit adjustment once per time step, whereas this method performs
				// once per iteration per constraint row.
				// the constraints are ordered so that all lambda[] values needed have
				// already been computed.
				if (findex[index] >= 0) {
					hi[index] = dFabs (hicopy[index] * lambda[findex[index]]);
					lo[index] = -hi[index];
				}

				int b1 = jb[index*2];
				int b2 = jb[index*2+1];
				double delta = b[index] - lambda[index]*Ad[index];
				int fc_ofs = 6*b1;//double[] fc_ptr = fc + 6*b1;

				// @@@ potential optimization: SIMD-ize this and the b2 >= 0 case
				delta -=fc[fc_ofs+0] * J[J_ofs+0] + fc[fc_ofs+1] * J[J_ofs+1] +
				fc[fc_ofs+2] * J[J_ofs+2] + fc[fc_ofs+3] * J[J_ofs+3] +
				fc[fc_ofs+4] * J[J_ofs+4] + fc[fc_ofs+5] * J[J_ofs+5];
				// @@@ potential optimization: handle 1-body constraints in a separate
				//     loop to avoid the cost of test & jump?
				if (b2 >= 0) {
					fc_ofs = 6*b2;//fc_ptr = fc + 6*b2;
					delta -=fc[fc_ofs+0] * J[J_ofs+6] + fc[fc_ofs+1] * J[J_ofs+7] +
					fc[fc_ofs+2] * J[J_ofs+8] + fc[fc_ofs+3] * J[J_ofs+9] +
					fc[fc_ofs+4] * J[J_ofs+10] + fc[fc_ofs+5] * J[J_ofs+11];
				}

				// compute lambda and clamp it to [lo,hi].
				// @@@ potential optimization: does SSE have clamping instructions
				//     to save test+jump penalties here?
				double new_lambda = lambda[index] + delta;
				if (new_lambda < lo[index]) {
					delta = lo[index]-lambda[index];
					lambda[index] = lo[index];
				}
				else if (new_lambda > hi[index]) {
					delta = hi[index]-lambda[index];
					lambda[index] = hi[index];
				}
				else {
					lambda[index] = new_lambda;
				}

				//@@@ a trick that may or may not help
				//dReal ramp = (1-((dReal)(iteration+1)/(dReal)num_iterations));
				//delta *= ramp;

				// update fc.
				// @@@ potential optimization: SIMD for this and the b2 >= 0 case
				fc_ofs = 6*b1;//fc_ptr = fc + 6*b1;
				fc[fc_ofs + 0] += delta * iMJ[iMJ_ofs + 0];//fc_ptr[0] += delta * iMJ_ptr[0];
				fc[fc_ofs + 1] += delta * iMJ[iMJ_ofs + 1];//fc_ptr[1] += delta * iMJ_ptr[1];
				fc[fc_ofs + 2] += delta * iMJ[iMJ_ofs + 2];//fc_ptr[2] += delta * iMJ_ptr[2];
				fc[fc_ofs + 3] += delta * iMJ[iMJ_ofs + 3];//fc_ptr[3] += delta * iMJ_ptr[3];
				fc[fc_ofs + 4] += delta * iMJ[iMJ_ofs + 4];//fc_ptr[4] += delta * iMJ_ptr[4];
				fc[fc_ofs + 5] += delta * iMJ[iMJ_ofs + 5];//fc_ptr[5] += delta * iMJ_ptr[5];
				// @@@ potential optimization: handle 1-body constraints in a separate
				//     loop to avoid the cost of test & jump?
				if (b2 >= 0) {
					fc_ofs = 6*b2;//fc_ptr = fc + 6*b2;
					fc[fc_ofs + 0] += delta * iMJ[iMJ_ofs + 6];//fc_ptr[0] += delta * iMJ_ptr[6];
					fc[fc_ofs + 1] += delta * iMJ[iMJ_ofs + 7];//fc_ptr[1] += delta * iMJ_ptr[7];
					fc[fc_ofs + 2] += delta * iMJ[iMJ_ofs + 8];//fc_ptr[2] += delta * iMJ_ptr[8];
					fc[fc_ofs + 3] += delta * iMJ[iMJ_ofs + 9];//fc_ptr[3] += delta * iMJ_ptr[9];
					fc[fc_ofs + 4] += delta * iMJ[iMJ_ofs + 10];//fc_ptr[4] += delta * iMJ_ptr[10];
					fc[fc_ofs + 5] += delta * iMJ[iMJ_ofs + 11];//fc_ptr[5] += delta * iMJ_ptr[11];
				}
			}
		}
	}


	//void dxQuickStepper (dxWorld *world, dxBody * const *body, int nb,
	//	     dxJoint * const *_joint, int nj, dReal stepsize)
	private static void dxQuickStepper (DxWorld world, DxBody[]body, int nb,
			DxJoint[]_joint, int nj, double stepsize)
	{
		int i,j;
		//IFTIMING(dTimerStart("preprocessing");)
		if (TIMING) dTimerStart("preprocessing");

		double stepsize1 = dRecip(stepsize);

		// number all bodies in the body list - set their tag values
		for (i=0; i<nb; i++) body[i].tag = i;

		// make a local copy of the joint array, because we might want to modify it.
		// (the "dxJoint *const*" declaration says we're allowed to modify the joints
		// but not the joint array, because the caller might need it unchanged).
		//@@@ do we really need to do this? we'll be sorting constraint rows individually, not joints
		//dxJoint **joint = (dxJoint**) ALLOCA (nj * sizeof(dxJoint*));
		DxJoint []joint = new DxJoint[nj];//(dxJoint**) ALLOCA (nj * sizeof(dxJoint*));
		//memcpy (joint,_joint,nj);// * sizeof(dxJoint*));
		for (i = 0; i < nj; i++) {
			//TODO this is only a simple clone!
			joint[i] = (DxJoint) _joint[i].clone();
		}

		// for all bodies, compute the inertia tensor and its inverse in the global
		// frame, and compute the rotational force and add it to the torque
		// accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
		double[] invI = new double[3*4*nb];//dRealAllocaArray (invI,3*4*nb);
		for (i=0; i<nb; i++) {
			DMatrix3 tmp = new DMatrix3();

			// compute inverse inertia tensor in global frame
			dMULTIPLY2_333 (tmp,body[i].invI,body[i]._posr.R);
			dMULTIPLY0_333 (invI,i*12,body[i]._posr.R.v,0,tmp.v,0);
			//#ifdef dGYROSCOPIC
			if (dGYROSCOPIC) {
				DMatrix3 I = new DMatrix3();
				// compute inertia tensor in global frame
				dMULTIPLY2_333 (tmp,body[i].mass._I,body[i]._posr.R);
				dMULTIPLY0_333 (I,body[i]._posr.R,tmp);
				// compute rotational force
				dMULTIPLY0_331 (tmp.v,0,I.v,0,body[i].avel.v,0);
				dCROSS (body[i].tacc,OP.SUB_EQ,body[i].avel,tmp);
			}//#endif
		}

		// add the gravity force to all bodies
		for (i=0; i<nb; i++) {
			if ((body[i].flags & DxBody.dxBodyNoGravity)==0) {
				body[i].facc.v[0] += body[i].mass._mass * world.gravity.v[0];
				body[i].facc.v[1] += body[i].mass._mass * world.gravity.v[1];
				body[i].facc.v[2] += body[i].mass._mass * world.gravity.v[2];
			}
		}

		// get joint information (m = total constraint dimension, nub = number of unbounded variables).
		// joints with m=0 are inactive and are removed from the joints array
		// entirely, so that the code that follows does not consider them.
		//@@@ do we really need to save all the info1's
		DxJoint.Info1[] info = new DxJoint.Info1[nj]; //ALLOCA (nj*sizeof(dxJoint::Info1));
		for (i=0, j=0; j<nj; j++) {	// i=dest, j=src
			info[i] = new DxJoint.Info1(); //TZ  TODO necessary?
			joint[j].getInfo1 (info[i]);
			dIASSERT (info[i].m >= 0 && info[i].m <= 6 && info[i].nub >= 0 && info[i].nub <= info[i].m);
			if (info[i].m > 0) {
				joint[i] = joint[j];
				i++;
			}
		}
		nj = i;

		// create the row offset array
		int m = 0;
		int[] ofs = new int[nj];//(int*) ALLOCA (nj*sizeof(int));
		for (i=0; i<nj; i++) {
			ofs[i] = m;
			m += info[i].getM();
		}

		// if there are constraints, compute the constraint force
		double[] J = new double[m*12]; //dRealAllocaArray (J,m*12);
		int[] jb = new int[m*2];//(int*) ALLOCA (m*2*sizeof(int));
		if (m > 0) {
			// create a constraint equation right hand side vector `c', a constraint
			// force mixing vector `cfm', and LCP low and high bound vectors, and an
			// 'findex' vector.
			double[] c = new double[m];//dRealAllocaArray (c,m);
			double[] cfm = new double[m];//dRealAllocaArray (cfm,m);
			double[] lo = new double[m];//dRealAllocaArray (lo,m);
			double[] hi = new double[m];//dRealAllocaArray (hi,m);
			int[] findex = new int[m];//(int*) ALLOCA (m*sizeof(int));
//TZ			dSetZero (c,m);
			dSetValue (cfm,m,world.global_cfm);
			dSetValue (lo,m,-dInfinity);
			dSetValue (hi,m, dInfinity);
			for (i=0; i<m; i++) findex[i] = -1;

			// get jacobian data from constraints. an m*12 matrix will be created
			// to store the two jacobian blocks from each constraint. it has this
			// format:
			//
			//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 \    .
			//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2  }-- jacobian for joint 0, body 1 and body 2 (3 rows)
			//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 /
			//   l1 l1 l1 a1 a1 a1 l2 l2 l2 a2 a2 a2 }--- jacobian for joint 1, body 1 and body 2 (3 rows)
			//   etc...
			//
			//   (lll) = linear jacobian data
			//   (aaa) = angular jacobian data
			//
			//IFTIMING (dTimerNow ("create J");)
			if (TIMING) dTimerNow ("create J");
			dSetZero (J,m*12);
			DxJoint.Info2 Jinfo = new DxJoint.Info2();
			Jinfo.setRowskip(12);
			Jinfo.setArrays(J, c, cfm, lo, hi, findex);
			Jinfo.fps = stepsize1;
			Jinfo.erp = world.global_erp;
			int mfb = 0; // number of rows of Jacobian we will have to save for joint feedback
			for (i=0; i<nj; i++) {
				Jinfo.J1lp = ofs[i]*12;//J + ofs[i]*12;
				Jinfo.J1ap = ofs[i]*12 + 3;// = Jinfo.J1l + 3;  
				Jinfo.J2lp = ofs[i]*12 + 6;//  = Jinfo.J1l + 6;
				Jinfo.J2ap = ofs[i]*12 + 9;//  = Jinfo.J1l + 9;
				Jinfo.setAllP(ofs[i]);
				
//				Jinfo.c.set(c, ofs[i]);//				Jinfo.c = c + ofs[i];
//				Jinfo.cfm.set(cfm, ofs[i]);//				Jinfo.cfm = cfm + ofs[i];
//				Jinfo.lo.set(lo, ofs[i]);//				Jinfo.lo = lo + ofs[i];
//				Jinfo.hi.set(hi, ofs[i]);//				Jinfo.hi = hi + ofs[i];
				joint[i].getInfo2 (Jinfo);
				
				// adjust returned findex values for global index numbering
				for (j=0; j<info[i].getM(); j++) {
					if (findex[ofs[i] + j] >= 0) findex[ofs[i] + j] += ofs[i];
				}
				if (joint[i].feedback!=null)
					mfb += info[i].getM();
			}

			// we need a copy of Jacobian for joint feedbacks
			// because it gets destroyed by SOR solver
			// instead of saving all Jacobian, we can save just rows
			// for joints, that requested feedback (which is normaly much less)
			double[] Jcopy = null;
			if (mfb > 0) {
				Jcopy = new double[mfb*12];//(double[]) ALLOCA (mfb*12*sizeof(dReal));
				mfb = 0;
				for (i=0; i<nj; i++)
					if (joint[i].feedback!=null) {
						memcpy(Jcopy,mfb*12, J,ofs[i]*12, info[i].getM()*12);//*sizeof(dReal));
						mfb += info[i].getM();
					}
			}


			// create an array of body numbers for each joint row
			int jb_ofs = 0;//int[] jb_ptr = jb;
			for (i=0; i<nj; i++) {
				int b1 = (joint[i].node[0].body!=null) ? (joint[i].node[0].body.tag) : -1;
				int b2 = (joint[i].node[1].body!=null) ? (joint[i].node[1].body.tag) : -1;
				for (j=0; j<info[i].getM(); j++) {
					jb[jb_ofs] = b1;//jb_ptr[0] = b1;
					jb[jb_ofs+1] = b2;//jb_ptr[1] = b2;
					jb_ofs+=2;//jb_ptr += 2;
				}
			}
			dIASSERT (jb_ofs == 2*m);//(jb_ptr == jb+2*m);

			// compute the right hand side `rhs'
			//IFTIMING (dTimerNow ("compute rhs");)
			if (TIMING) dTimerNow ("compute rhs");
			double[] tmp1 = new double[nb*6];//dRealAllocaArray (tmp1,nb*6);
			// put v/h + invM*fe into tmp1
			for (i=0; i<nb; i++) {
				double body_invMass = body[i].invMass;
				for (j=0; j<3; j++) tmp1[i*6+j] = body[i].facc.v[j] * body_invMass + 
					body[i].lvel.v[j] * stepsize1;
				dMULTIPLY0_331 (tmp1, i*6 + 3,invI, i*12,body[i].tacc.v, 0);
				for (j=0; j<3; j++) tmp1[i*6+3+j] += body[i].avel.v[j] * stepsize1;
			}

			// put J*tmp1 into rhs
			double[] rhs = new double[m];//dRealAllocaArray (rhs,m);
			multiply_J (m,J,jb,tmp1,rhs);

			// complete rhs
			for (i=0; i<m; i++) rhs[i] = c[i]*stepsize1 - rhs[i];

			// scale CFM
			for (i=0; i<m; i++) cfm[i] *= stepsize1;

			// load lambda from the value saved on the previous iteration
			double[] lambda = new double[m];//dRealAllocaArray (lambda,m);
			//TZ not defined
//			if (WARM_STARTING) {//#ifdef WARM_STARTING
//				dSetZero (lambda,m);	//@@@ shouldn't be necessary
//				for (i=0; i<nj; i++) {
//					memcpy (lambda,ofs[i],joint[i].lambda,0,info[i].getM());// * sizeof(dReal));
//				}
//			}//#endif

			// solve the LCP problem and get lambda and invM*constraint_force
			//IFTIMING (dTimerNow ("solving LCP problem");)
			if (TIMING) dTimerNow ("solving LCP problem");
			double[] cforce = new double[nb*6];//dRealAllocaArray (cforce,nb*6);
			SOR_LCP (m,nb,J,jb,body,invI,lambda,cforce,rhs,lo,hi,cfm,findex,world.qs);
//			System.err.println("SOR_LCP m=" + m + " nb=" + nb + " ");
//			System.err.println("SOR_LCP J=" + Arrays.toString(J));
//			System.err.println("SOR_LCP jb=" + Arrays.toString(jb));
//			System.err.println("SOR_LCP invI=" + Arrays.toString(invI));
//			System.err.println("SOR_LCP lambda=" + Arrays.toString(lambda));
//			System.err.println("SOR_LCP cforce=" + Arrays.toString(cforce));
//			System.err.println("SOR_LCP rhs=" + Arrays.toString(rhs));
//			System.err.println("SOR_LCP lo=" + Arrays.toString(lo));
//			System.err.println("SOR_LCP hi=" + Arrays.toString(hi));
//			System.err.println("SOR_LCP cfm=" + Arrays.toString(cfm));
//			System.err.println("SOR_LCP findex=" + Arrays.toString(findex));
//			System.err.println("SOR_LCP qs=" + world.qs.num_iterations + " w=" + world.qs.w);
			
			//TZ not defined
//			if (WARM_STARTING) {//#ifdef WARM_STARTING
//				// save lambda for the next iteration
//				//@@@ note that this doesn't work for contact joints yet, as they are
//				// recreated every iteration
//				for (i=0; i<nj; i++) {
//					memcpy (joint[i].lambda,0,lambda,ofs[i],info[i].getM());// * sizeof(dReal));
//				}
//				//#endif
//
			// note that the SOR method overwrites rhs and J at this point, so
			// they should not be used again.

			// add stepsize * cforce to the body velocity
			for (i=0; i<nb; i++) {
				for (j=0; j<3; j++) body[i].lvel.v[j] += stepsize * cforce[i*6+j];
				for (j=0; j<3; j++) body[i].avel.v[j] += stepsize * cforce[i*6+3+j];
			}


			if (mfb > 0) {
				// straightforward computation of joint constraint forces:
				// multiply related lambdas with respective J' block for joints
				// where feedback was requested
				mfb = 0;
				for (i=0; i<nj; i++) {
					if (joint[i].feedback != null) {
						DJoint.DJointFeedback fb = joint[i].feedback;
						double[] data = new double[6];
						Multiply1_12q1 (data, Jcopy, mfb*12, lambda,ofs[i], info[i].m);
						fb.f1.v[0] = data[0];
						fb.f1.v[1] = data[1];
						fb.f1.v[2] = data[2];
						fb.t1.v[0] = data[3];
						fb.t1.v[1] = data[4];
						fb.t1.v[2] = data[5];
						if (joint[i].node[1].body != null)
						{
							Multiply1_12q1 (data, Jcopy, mfb*12+6, lambda,ofs[i], info[i].m);
							fb.f2.v[0] = data[0];
							fb.f2.v[1] = data[1];
							fb.f2.v[2] = data[2];
							fb.t2.v[0] = data[3];
							fb.t2.v[1] = data[4];
							fb.t2.v[2] = data[5];
						}
						mfb += info[i].getM();
					}
				}
			}
		}

		// compute the velocity update:
		// add stepsize * invM * fe to the body velocity

		//IFTIMING (dTimerNow ("compute velocity update");)
		if (TIMING) dTimerNow ("compute velocity update");
		for (i=0; i<nb; i++) {
			double body_invMass = body[i].invMass;
			for (j=0; j<3; j++) body[i].lvel.v[j] += stepsize * body_invMass * body[i].facc.v[j];
			for (j=0; j<3; j++) body[i].tacc.v[j] *= stepsize;
			dMULTIPLYADD0_331 (body[i].avel.v,0,invI, i*12,body[i].tacc.v,0);
		}

		//#if 0
		//	// check that the updated velocity obeys the constraint (this check needs unmodified J)
		//	dRealAllocaArray (vel,nb*6);
		//	for (i=0; i<nb; i++) {
		//		for (j=0; j<3; j++) vel[i*6+j] = body[i].lvel[j];
		//		for (j=0; j<3; j++) vel[i*6+3+j] = body[i].avel[j];
		//	}
		//	dRealAllocaArray (tmp,m);
		//	multiply_J (m,J,jb,vel,tmp);
		//	double error = 0;
		//	for (i=0; i<m; i++) error += dFabs(tmp[i]);
		//	printf ("velocity error = %10.6e\n",error);
		//#endif

		// update the position and orientation from the new linear/angular velocity
		// (over the given timestep)
		//IFTIMING (dTimerNow ("update position");)
		if (TIMING) dTimerNow ("update position");
		for (i=0; i<nb; i++) body[i].dxStepBody (stepsize);

		//IFTIMING (dTimerNow ("tidy up");)
		if (TIMING) dTimerNow ("tidy up");

		// zero all force accumulators
		for (i=0; i<nb; i++) {
			body[i].facc.dSetZero();//dSetZero (body[i].facc,3);
			body[i].tacc.dSetZero();//dSetZero (body[i].tacc,3);
		}

		//IFTIMING (dTimerEnd();)
		if (TIMING) dTimerEnd();
		//IFTIMING (if (m > 0) dTimerReport (stdout,1);)
		if (TIMING) if (m > 0) dTimerReport (stdout,1);
	}

	public void run(DxWorld world, DxBody[] body, int nb, DxJoint[] joint, int nj,
			double stepsize) {
		dxQuickStepper(world, body, nb, joint, nj, stepsize);
	}
}