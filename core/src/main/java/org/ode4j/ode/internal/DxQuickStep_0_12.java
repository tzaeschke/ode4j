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

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply0_333;
import static org.ode4j.ode.OdeMath.dMultiply2_333;
import static org.ode4j.ode.OdeMath.dMultiplyAdd0_331;
import static org.ode4j.ode.OdeMath.dSubtractVectorCross3;
import static org.ode4j.ode.internal.Common.dFabs;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dRecip;
import static org.ode4j.ode.internal.Matrix.dSetValue;
import static org.ode4j.ode.internal.Matrix.dSetZero;
import static org.ode4j.ode.internal.Misc.dRandInt;
import static org.ode4j.ode.internal.Timer.dTimerEnd;
import static org.ode4j.ode.internal.Timer.dTimerNow;
import static org.ode4j.ode.internal.Timer.dTimerReport;
import static org.ode4j.ode.internal.Timer.dTimerStart;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stdout;
import static org.ode4j.ode.internal.cpp4j.Cstring.memcpy;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.internal.Objects_H.dxQuickStepParameters;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemArena;

/**
 *
 * Quickstep stepper.
 */
public class DxQuickStep_0_12 extends AbstractStepper implements DxWorld.dstepper_fn_t,
dmemestimate_fn_t {
	
    //TZ where is this defined???
    private static final boolean CHECK_VELOCITY_OBEYS_CONSTRAINT = false;
    
	/** DxQuickStep singleton instance. */
	public static final DxQuickStep_0_12 INSTANCE = new DxQuickStep_0_12();
	
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
//	private static void Multiply1_12q1 (double[] A, double[] B, final int ofsB, 
//			double[] C, final int ofsC, final int q)
//	{
//		//int i, k;
//		//dIASSERT (q>0 && A!=null && B!=null && C!=null);
//		if (q < 0) throw new IllegalArgumentException();
//
//		double a = 0;
//		double b = 0;
//		double c = 0;
//		double d = 0;
//		double e = 0;
//		double f = 0;
//		double s;
//
////		for(i=0, k = 0; i<q; i++, k += 12)
//		for(int i=ofsC, k = ofsB; i<q+ofsC; i++, k += 12)
//		{
//			s = C[i]; //C[i] and B[n+k] cannot overlap because its value has been read into a temporary.
//
//			//For the rest of the loop, the only memory dependency (array) is from B[]
//			a += B[  k] * s;
//			b += B[1+k] * s;
//			c += B[2+k] * s;
//			d += B[3+k] * s;
//			e += B[4+k] * s;
//			f += B[5+k] * s;
//		}
//
//		A[0] = a;
//		A[1] = b;
//		A[2] = c;
//		A[3] = d;
//		A[4] = e;
//		A[5] = f;
//	}

	// multiply block of B matrix (q x 6) with 12 dReal per row with C vektor (q)
	private static void Multiply1_12q1 (DVector3 A1, DVector3 A2, double[] B, final int ofsB, 
			double[] C, final int ofsC, final int q)
	{
		if (q < 0) throw new IllegalArgumentException();

		double a = 0;
		double b = 0;
		double c = 0;
		double d = 0;
		double e = 0;
		double f = 0;
		double s;

//		for(i=0, k = 0; i<q; i++, k += 12)
		for(int i=ofsC, k = ofsB; i<q+ofsC; i++, k += 12)
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

		A1.set(a, b, c);
		A2.set(d, e, f);
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
	private static void compute_invM_JT (final int m, final double[] J, final double[] iMJ, 
	        final int[]jb,
			final DxBody[]bodyP, final int bodyOfs, final double[] invI)
	{
		int i,j;
		//dRealMutablePtr iMJ_ptr = iMJ;
		//dRealMutablePtr J_ptr = J;
		int iMJ_ofs = 0;//TZ
		int J_ofs = 0;//TZ
		for (i=0; i<m; J_ofs +=12, iMJ_ofs += 12, i++) {
			int b1 = jb[i*2];
			int b2 = jb[i*2+1];
			double k1 = bodyP[b1+bodyOfs].invMass;
//TZ			for (j=0; j<3; j++) iMJ_ptr[j] = k*J_ptr[j];
			for (j=0; j<3; j++) iMJ[j + iMJ_ofs] = k1*J[j + J_ofs];
//			dMULTIPLY0_331 (iMJ_ptr + 3, invI + 12*b1, J_ptr + 3);
			dMultiply0_331 (iMJ, iMJ_ofs + 3, invI, 12*b1, J,J_ofs + 3);
			if (b2 != -1) {
				double k2 = bodyP[b2+bodyOfs].invMass;
				//for (j=0; j<3; j++) iMJ_ptr[j+6] = k*J_ptr[j+6];
				for (j=0; j<3; j++) iMJ[j+6+iMJ_ofs] = k2*J[j+6+J_ofs];
				//dMULTIPLY0_331 (iMJ_ptr + 9, invI + 12*b2, J_ptr + 9);
				dMultiply0_331 (iMJ, iMJ_ofs + 9, invI, 12*b2, J, J_ofs + 9);
			}
		}
	}


	// compute out = inv(M)*J'*in.
	//#if WARM_STARTING
	//static void multiply_invM_JT (int m, int nb, dRealMutablePtr iMJ, int[] *jb,
	//		dRealMutablePtr in, dRealMutablePtr out)
//	private static void multiply_invM_JT (int m, int nb, double[] iMJ, int[] jb,
//			double[] in, double[] out)
//	{
//		dSetZero (out,6*nb);
//		int iMJ_ofs = 0;//final double[] iMJ_ptr = iMJ;
//		for (int i=0; i<m; i++) {
//			int b1 = jb[i*2];
//			int b2 = jb[i*2+1];
//			final double in_i = in[i];
//			int out_ofs = b1*6;//double[] out_ptr = out + b1*6;
//			//for (j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in[i];
//			for (int j=0; j<6; j++) out[j + out_ofs] += iMJ[j + iMJ_ofs] * in_i;
//			iMJ_ofs +=6;//iMJ_ptr += 6;
//			if (b2 != -1) {
//				out_ofs = b2*6;//out_ptr = out + b2*6;
//				//for (j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in[i];
//				for (int j=0; j<6; j++) out[j + out_ofs] += iMJ[j + iMJ_ofs] * in_i;
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
	private static void multiply_J (int m, final double[] J, final int[]jb,
			final double[] in, final double[] out)
	{
		int J_ofs = 0;//final double[] J_ptr = J;
		for (int i=0; i<m; i++) {
			int b1 = jb[i*2];
			int b2 = jb[i*2+1];
			double sum = 0;
			int in_ofs = b1*6; //double[] in_ptr = in + b1*6;
			for (int j=0; j<6; j++) sum += J[j + J_ofs] * in[j + in_ofs];//J_ptr[j]*in_ptr[j];
			J_ofs += 6;//J_ptr += 6;
			if (b2 != -1) {
				in_ofs = b2*6;//in_ptr = in + b2*6;
				for (int j=0; j<6; j++) sum += J[j + J_ofs] * in[j + in_ofs];//J_ptr[j] * in_ptr[j];
			}
			J_ofs += 6;//J_ptr += 6;
			out[i] = sum;
		}
	}


	// compute out = (J*inv(M)*J' + cfm)*in.
	// use z as an nb*6 temporary.
	//#if WARM_STARTING
	//static void multiply_J_invM_JT (int m, int nb, dRealMutablePtr J, dRealMutablePtr iMJ, int *jb,
	//		dRealPtr cfm, dRealMutablePtr z, dRealMutablePtr in, dRealMutablePtr out)
//	private static void multiply_J_invM_JT (int m, int nb, final double[] J, final double[] iMJ, 
//			final int []jb,
//			final double[] cfm, final double[] z, final double[] in, final double[] out)
//	{
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
	//	static void CG_LCP (dxWorldProcessMemArena *memarena,
	//	        unsigned int m, unsigned int nb, dRealMutablePtr J, int *jb, dxBody * const *body,
	//	        dRealPtr invI, dRealMutablePtr lambda, dRealMutablePtr fc, dRealMutablePtr b,
	//	        dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
	//	        dxQuickStepParameters *qs)
	//	      {
	//	        const unsigned int num_iterations = qs->num_iterations;
	//
	//	        // precompute iMJ = inv(M)*J'
	//	        dReal *iMJ = memarena->AllocateArray<dReal> ((size_t)m*12);
	//	        compute_invM_JT (m,J,iMJ,jb,body,invI);
	//
	//	        dReal last_rho = 0;
	//	        dReal *r = memarena->AllocateArray<dReal> (m);
	//	        dReal *z = memarena->AllocateArray<dReal> (m);
	//	        dReal *p = memarena->AllocateArray<dReal> (m);
	//	        dReal *q = memarena->AllocateArray<dReal> (m);
	//
	//	        // precompute 1 / diagonals of A
	//	        dReal *Ad = memarena->AllocateArray<dReal> (m);
	//	        dRealPtr iMJ_ptr = iMJ;
	//	        dRealPtr J_ptr = J;
	//	        for (unsigned int i=0; i<m; i++) {
	//	          dReal sum = 0;
	//	          for (unsigned int j=0; j<6; j++) sum += iMJ_ptr[j] * J_ptr[j];
	//	          if (jb[(size_t)i*2+1] != -1) {
	//	            for (unsigned int j=6; j<12; j++) sum += iMJ_ptr[j] * J_ptr[j];
	//	          }
	//	          iMJ_ptr += 12;
	//	          J_ptr += 12;
	//	          Ad[i] = REAL(1.0) / (sum + cfm[i]);
	//	        }
	//
	//	      #ifdef WARM_STARTING
	//	        // compute residual r = b - A*lambda
	//	        multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,lambda,r);
	//	        for (unsigned int k=0; k<m; k++) r[k] = b[k] - r[k];
	//	      #else
	//	        dSetZero (lambda,m);
	//	        memcpy (r,b,(size_t)m*sizeof(dReal));     // residual r = b - A*lambda
	//	      #endif
	//
	//	        for (unsigned int iteration=0; iteration < num_iterations; iteration++) {
	//	          for (unsigned int i=0; i<m; i++) z[i] = r[i]*Ad[i]; // z = inv(M)*r
	//	          dReal rho = dot (m,r,z);        // rho = r'*z
	//
	//	          // @@@
	//	          // we must check for convergence, otherwise rho will go to 0 if
	//	          // we get an exact solution, which will introduce NaNs into the equations.
	//	          if (rho < 1e-10) {
	//	            printf ("CG returned at iteration %d\n",iteration);
	//	            break;
	//	          }
	//
	//	          if (iteration==0) {
	//	            memcpy (p,z,(size_t)m*sizeof(dReal)); // p = z
	//	          }
	//	          else {
	//	            add (m,p,z,p,rho/last_rho);   // p = z + (rho/last_rho)*p
	//	          }
	//
	//	          // compute q = (J*inv(M)*J')*p
	//	          multiply_J_invM_JT (m,nb,J,iMJ,jb,cfm,fc,p,q);
	//
	//	          dReal alpha = rho/dot (m,p,q);      // alpha = rho/(p'*q)
	//	          add (m,lambda,lambda,p,alpha);      // lambda = lambda + alpha*p
	//	          add (m,r,r,q,-alpha);           // r = r - alpha*q
	//	          last_rho = rho;
	//	        }
	//
	//	        // compute fc = inv(M)*J'*lambda
	//	        multiply_invM_JT (m,nb,iMJ,jb,lambda,fc);
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
//		double error;		// error to sort on
//		int findex;
		int index;		// row index
	}


	//#ifdef REORDER_CONSTRAINTS

	static {
		if (REORDER_CONSTRAINTS) {
			throw new UnsupportedOperationException();
		}
	}
	//static int compare_index_error (const void *a, const void *b)
//	private static class IndexErrorComparator implements Comparator<IndexError> {
////	static int compare_index_error (final IndexError a, final IndexError b)
////	{
////		//	if (!REORDER_CONSTRAINTS) { return;} //TZ
////		if (!REORDER_CONSTRAINTS) { throw new IllegalStateException();} //TZ
////		final IndexError i1 = (IndexError) a;
////		final IndexError i2 = (IndexError) b;
////		if (i1.findex == -1 && i2.findex != -1) return -1;
////		if (i1.findex != -1 && i2.findex == -1) return 1;
////		if (i1.error < i2.error) return -1;
////		if (i1.error > i2.error) return 1;
////		return 0;
////	}
//
//		//TODO is sort order correct (-1,0,1)??
//		public int compare(IndexError a, IndexError b) {
//			if (!REORDER_CONSTRAINTS) { throw new IllegalStateException();} //TZ
//			final IndexError i1 = (IndexError) a;
//			final IndexError i2 = (IndexError) b;
//			if (i1.findex == -1 && i2.findex != -1) return -1;
//			if (i1.findex != -1 && i2.findex == -1) return 1;
//			if (i1.error < i2.error) return -1;
//			if (i1.error > i2.error) return 1;
//			return 0;
//		}
//	}
	//#endif

	
	//static void SOR_LCP (int m, int nb, dRealMutablePtr J, int *jb, dxBody * const *body,
	//		dRealPtr invI, dRealMutablePtr lambda, dRealMutablePtr fc, dRealMutablePtr b,
	//		dRealMutablePtr lo, dRealMutablePtr hi, dRealPtr cfm, int *findex,
	//		dxQuickStepParameters *qs)
	private static void SOR_LCP (DxWorldProcessMemArena memarena,
	        final int m, final int nb, double[] J, int[] jb, final DxBody []bodyP,
	        final int bodyOfs,
			final double[] invI, double[] lambda, double[] fc, double[] b,
			final double[] lo, final double[] hi, final double[] cfm, final int []findex,
			dxQuickStepParameters qs)
	{
		//TZ not defined
//		if (WARM_STARTING) {//#ifdef WARM_STARTING
//			// for warm starting, this seems to be necessary to prevent
//			// jerkiness in motor-driven joints. i have no idea why this works.
//			for (int i=0; i<m; i++) lambda[i] *= 0.9;
//		} else { //#else
			dSetZero (lambda,m);
//		}//#endif

		// precompute iMJ = inv(M)*J'
		//double[] iMJ = new double[m*12];//dRealAllocaArray (iMJ,m*12);
		double[] iMJ = memarena.AllocateArrayDReal (m*12);
		compute_invM_JT (m,J,iMJ,jb,bodyP,bodyOfs,invI);

		// compute fc=(inv(M)*J')*lambda. we will incrementally maintain fc
		// as we change lambda.
		//not defined:
//		if (WARM_STARTING) {//TZ #ifdef WARM_STARTING
//			throw new UnsupportedOperationException();
//			//multiply_invM_JT (m,nb,iMJ,jb,lambda,fc);
//		} else {//#else
		//TODO (TZ) should not be necessary (is created just before given to this method)
			dSetZero (fc,nb*6);
//		}//#endif

        double[] Ad = memarena.AllocateArrayDReal (m);
        {
            final double sor_w = qs.w;      // SOR over-relaxation parameter
            // precompute 1 / diagonals of A
            int iMJ_ofs = 0;//final double[] iMJ_ptr = iMJ;
            int J_ofs = 0;//double[] J_ptr = J;
            for (int i=0; i<m; iMJ_ofs +=12, J_ofs +=12, i++ ) {
                double sum = 0;
                for (int j=0; j<6; j++) sum += iMJ[j+iMJ_ofs] * J[j+J_ofs];//iMJ_ptr[j] * J_ptr[j];
                if (jb[i*2+1] != -1) {
                    for (int j=6; j<12; j++) sum += iMJ[j+iMJ_ofs] * J[j+J_ofs];//iMJ_ptr[j] * J_ptr[j];
                }
                Ad[i] = sor_w / (sum + cfm[i]);
            }
        }

        {
            // NOTE: This may seem unnecessary but it's indeed an optimization 
            // to move multiplication by Ad[i] and cfm[i] out of iteration loop.
            
            // scale J and b by Ad
            int J_ofs = 0;//J_ptr = J;
            for (int i=0; i<m; J_ofs += 12, i++) {
                double Ad_i = Ad[i];
                for (int j=0; j<12; j++) {
                    J[J_ofs+j] *= Ad_i;//J_ptr[0] *= Ad[i];
                }
                b[i] *= Ad_i;

                // scale Ad by CFM. N.B. this should be done last since it is used above
                Ad[i] = Ad_i * cfm[i];
            }
        }
        
		// order to solve constraint rows in
		//IndexError *order = (IndexError*) ALLOCA (m*sizeof(IndexError));
		IndexError[] order = new IndexError[m];
		for (int iii=0; iii<m; iii++) order[iii] = new IndexError();

		if (!REORDER_CONSTRAINTS) {//TZ #ifndef REORDER_CONSTRAINTS
		    // make sure constraints with findex < 0 come first.
		    //IndexError *orderhead = order, *ordertail = order + (m - 1);
		    int orderhead = 0, ordertail = m-1;

		    // Fill the array from both ends
		    for (int i=0; i<m; i++) {
		        if (findex[i] == -1) {
		            order[orderhead].index = i; // Place them at the front
		            ++orderhead;
		        } else {
		            order[ordertail].index = i; // Place them at the end
		            --ordertail;
		        }
		    }
		    dIASSERT (orderhead-ordertail==1);
		} //#endif

		if (REORDER_CONSTRAINTS) { //#ifdef 
		  // the lambda computed at the previous iteration.
		  // this is used to measure error for when we are reordering the indexes.
		  double[] last_lambda = memarena.AllocateArrayDReal(m);
		} //#endif
		
		
		final int num_iterations = qs.num_iterations;
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
					for (int i=1; i<m; i++) {
					    int swapi = dRandInt(i+1);
                        IndexError tmp = order[i];
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

				int fc_ofs1;//dRealMutablePtr fc_ptr1;
				int fc_ofs2;//dRealMutablePtr fc_ptr2;
				double delta;
				final int NULL = -1;
				
				{
				    int b1 = jb[index*2];
				    int b2 = jb[index*2+1];
				    fc_ofs1 = 0 + 6*b1;//fc_ptr1 = fc + 6*b1;
				    fc_ofs2 = (b2 != -1) ? 0 + 6*b2 : NULL; //fc_ptr2 = (b2 != -1) ? fc + 6*b2 : NULL;
				}

				double old_lambda = lambda[index];

				{
				    delta = b[index] - old_lambda*Ad[index];

				    //dRealPtr J_ptr = J + index*12;
				    int J_ofs = index*12;
				    
				    // @@@ potential optimization: SIMD-ize this and the b2 >= 0 case
				    //delta -=fc_ptr1[0] * J_ptr[0] + fc_ptr1[1] * J_ptr[1] +
				    delta -=fc[fc_ofs1] * J[J_ofs] + fc[fc_ofs1+1] * J[J_ofs+1] +
				    //fc_ptr1[2] * J_ptr[2] + fc_ptr1[3] * J_ptr[3] +
				    fc[fc_ofs1+2] * J[J_ofs+2] + fc[fc_ofs1+3] * J[J_ofs+3] +
				    //fc_ptr1[4] * J_ptr[4] + fc_ptr1[5] * J_ptr[5];
				    fc[fc_ofs1+4] * J[J_ofs+4] + fc[fc_ofs1+5] * J[J_ofs+5];
				    // @@@ potential optimization: handle 1-body constraints in a separate
				    //     loop to avoid the cost of test & jump?
				    if (fc_ofs2 != NULL) {
				        //delta -=fc_ptr2[0] * J_ptr[6] + fc_ptr2[1] * J_ptr[7] +
				        delta -=fc[fc_ofs2+0] * J[J_ofs+6] + fc[fc_ofs2+1] * J[J_ofs+7] +
				        //fc_ptr2[2] * J_ptr[8] + fc_ptr2[3] * J_ptr[9] +
				        fc[fc_ofs2+2] * J[J_ofs+8] + fc[fc_ofs2+3] * J[J_ofs+9] +
				        //fc_ptr2[4] * J_ptr[10] + fc_ptr2[5] * J_ptr[11];
				        fc[fc_ofs2+4] * J[J_ofs+10] + fc[fc_ofs2+5] * J[J_ofs+11];
				    }
				}

				{
				    double hi_act, lo_act;
				
				    // set the limits for this constraint. note that 'hicopy' is used.
				    // this is the place where the QuickStep method differs from the
				    // direct LCP solving method, since that method only performs this
				    // limit adjustment once per time step, whereas this method performs
				    // once per iteration per constraint row.
				    // the constraints are ordered so that all lambda[] values needed have
				    // already been computed.
				    if (findex[index] != -1) {
				        hi_act = dFabs (hi[index] * lambda[findex[index]]);
				        lo_act = -hi_act;
				    } else {
				        hi_act = hi[index];
				        lo_act = lo[index];
				    }

				    // compute lambda and clamp it to [lo,hi].
				    // @@@ potential optimization: does SSE have clamping instructions
				    //     to save test+jump penalties here?
				    double new_lambda = old_lambda + delta;
				    if (new_lambda < lo_act) {
				        delta = lo_act-old_lambda;
				        lambda[index] = lo_act;
				    }
				    else if (new_lambda > hi_act) {
				        delta = hi_act-old_lambda;
				        lambda[index] = hi_act;
				    }
				    else {
				        lambda[index] = new_lambda;
				    }
				}

				//@@@ a trick that may or may not help
				//dReal ramp = (1-((dReal)(iteration+1)/(dReal)num_iterations));
				//delta *= ramp;

				{
				    int iMJ_ofs = 0+index*12; //dRealPtr iMJ_ptr = iMJ + (size_t)index*12;
    				// update fc.
    				// @@@ potential optimization: SIMD for this and the b2 >= 0 case
				    fc[fc_ofs1 + 0] += delta * iMJ[iMJ_ofs + 0];//fc_ptr[0] += delta * iMJ_ptr[0];
    				fc[fc_ofs1 + 1] += delta * iMJ[iMJ_ofs + 1];//fc_ptr[1] += delta * iMJ_ptr[1];
    				fc[fc_ofs1 + 2] += delta * iMJ[iMJ_ofs + 2];//fc_ptr[2] += delta * iMJ_ptr[2];
    				fc[fc_ofs1 + 3] += delta * iMJ[iMJ_ofs + 3];//fc_ptr[3] += delta * iMJ_ptr[3];
    				fc[fc_ofs1 + 4] += delta * iMJ[iMJ_ofs + 4];//fc_ptr[4] += delta * iMJ_ptr[4];
    				fc[fc_ofs1 + 5] += delta * iMJ[iMJ_ofs + 5];//fc_ptr[5] += delta * iMJ_ptr[5];
    				// @@@ potential optimization: handle 1-body constraints in a separate
    				//     loop to avoid the cost of test & jump?
    				if (fc_ofs2 != NULL) {
    					fc[fc_ofs2 + 0] += delta * iMJ[iMJ_ofs + 6];//fc_ptr[0] += delta * iMJ_ptr[6];
    					fc[fc_ofs2 + 1] += delta * iMJ[iMJ_ofs + 7];//fc_ptr[1] += delta * iMJ_ptr[7];
    					fc[fc_ofs2 + 2] += delta * iMJ[iMJ_ofs + 8];//fc_ptr[2] += delta * iMJ_ptr[8];
    					fc[fc_ofs2 + 3] += delta * iMJ[iMJ_ofs + 9];//fc_ptr[3] += delta * iMJ_ptr[9];
    					fc[fc_ofs2 + 4] += delta * iMJ[iMJ_ofs + 10];//fc_ptr[4] += delta * iMJ_ptr[10];
    					fc[fc_ofs2 + 5] += delta * iMJ[iMJ_ofs + 11];//fc_ptr[5] += delta * iMJ_ptr[11];
    				}
				}
			}
		}
	}
	
	static class DJointWithInfo1
	{
	  DxJoint joint;
	  final DxJoint.Info1 info = new DxJoint.Info1();
	}

	//void dxQuickStepper (dxWorld *world, dxBody * const *body, int nb,
	//	     dxJoint * const *_joint, int nj, dReal stepsize)
	private static void dxQuickStepper (DxWorldProcessMemArena memarena,
	        DxWorld world, DxBody[] bodyP, final int bodyOfs, final int nb,
			DxJoint[] _jointP, final int _jointOfs, final int _nj, double stepsize)
	{
		if (TIMING) dTimerStart("preprocessing");

		double stepsize1 = dRecip(stepsize);

		// number all bodies in the body list - set their tag values
		for (int i=0; i<nb; i++) bodyP[i+bodyOfs].tag = i;

		// for all bodies, compute the inertia tensor and its inverse in the global
		// frame, and compute the rotational force and add it to the torque
		// accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
		double[] invI = memarena.AllocateArrayDReal(nb*3*4);//new double[3*4*nb];//dRealAllocaArray (invI,3*4*nb);
		{
		    int invIrowP = 0;//invI;
		    
    		for (int i=0; i<nb; invIrowP += 12, i++) {
    			DMatrix3 tmp = new DMatrix3();
    			DxBody b = bodyP[i+bodyOfs];
    
    			// compute inverse inertia tensor in global frame
    			dMultiply2_333 (tmp,b.invI,b.posr().R());
    			dMultiply0_333 (invI,invIrowP,b.posr().R(),tmp);
    
    			if (b.isFlagsGyroscopic()) {
    				DMatrix3 I = new DMatrix3();
    				// compute inertia tensor in global frame
    				dMultiply2_333 (tmp,b.mass._I,b.posr().R());
    				dMultiply0_333 (I,b.posr().R(),tmp);
    				// compute rotational force
    				//dMULTIPLY0_331 (tmp.v,0,I.v,0,body[i].avel.v,0);
    				dMultiply0_331 (tmp,I,b.avel);
    				dSubtractVectorCross3(b.tacc, b.avel, tmp);
    			}//#endif
    		}
		}
		
		{
		    // add the gravity force to all bodies
		    // since gravity does normally have only one component it's more efficient
		    // to run three loops for each individual component
		    double gravity_x = world.gravity.get0();
		    if (gravity_x != 0) {
		        for (int i=0; i<nb; i++) {
		            DxBody b = bodyP[i+bodyOfs];
		            if ((b.flags & DxBody.dxBodyNoGravity)==0) {
		                b.facc.add(0, b.mass._mass * gravity_x);
		            }
		        }
		    }
		    double gravity_y = world.gravity.get1();
		    if (gravity_y != 0) {
		        for (int i=0; i<nb; i++) {
		            DxBody b = bodyP[i+bodyOfs];
		            if ((b.flags & DxBody.dxBodyNoGravity)==0) {
		                b.facc.add(1, b.mass._mass * gravity_y);
		            }
		        }
		    }
		    double gravity_z = world.gravity.get2();
		    if (gravity_z != 0) {
		        for (int i=0; i<nb; i++) {
		            DxBody b = bodyP[i+bodyOfs];
		            if ((b.flags & DxBody.dxBodyNoGravity)==0) {
		                b.facc.add(2, b.mass._mass * gravity_z);
		            }
		        }
		    }
		}

		// get joint information (m = total constraint dimension, nub = number of unbounded variables).
		// joints with m=0 are inactive and are removed from the joints array
		// entirely, so that the code that follows does not consider them.
		DxWorldProcessMemArena.dummy();  //see next line
		DJointWithInfo1[] jointiinfos = new DJointWithInfo1[_nj]; //ALLOCA (nj*sizeof(dxJoint::Info1));
		int njXXX;
		{
		    int jicurrP=0; //jicurr = 0;
		    DJointWithInfo1 jicurrO = new DJointWithInfo1();
		    for (int i=0; i<_nj; i++) {	// i=dest, j=src
			    DxJoint j = _jointP[i+_jointOfs];
                //DxJoint.Info1 jicurr = new DxJoint.Info1(); //TZ  TODO necessary?
			    j.getInfo1(jicurrO.info);
			    dIASSERT (jicurrO.info.m >= 0 && jicurrO.info.m <= 6 && jicurrO.info.nub >= 0 && jicurrO.info.nub <= jicurrO.info.m);
			    if (jicurrO.info.m > 0) {
                    jicurrO.joint = j;
			        jointiinfos[jicurrP] = jicurrO;   
			        jicurrP++;
			        jicurrO = new DJointWithInfo1();
			    }
		    }
		    njXXX = jicurrP;
        }

		memarena.ShrinkArrayDJointWithInfo1(jointiinfos, _nj, njXXX);
		
		// create the row offset array
		int m;
		int mfb = 0; // number of rows of Jacobian we will have to save for joint feedback
		{
		    int mcurr = 0, mfbcurr = 0;
		    for (int i = 0; i < njXXX; i++) {
		        int jm = jointiinfos[i].info.m;
		        mcurr += jm;
		        if (jointiinfos[i].joint.feedback != null) {
		            mfbcurr += jm;
		        }
		    }
		    
		    m = mcurr;
		    mfb = mfbcurr;
		}

		// if there are constraints, compute the constraint force
		double[] J = null;
		int[] jb = null;
		if (m > 0) {
		    double[] cfm, lo, hi, rhs, Jcopy;
		    int[] findex;
		    
		    {
		        int mlocal = m;
		        final int jelements = mlocal*12;
		        J = memarena.AllocateArrayDReal(jelements);
		        dSetZero (J);//,jelements);
		    
		        // create a constraint equation right hand side vector `c', a constraint
		        // force mixing vector `cfm', and LCP low and high bound vectors, and an
		        // 'findex' vector.
		        cfm = memarena.AllocateArrayDReal(mlocal);
		        dSetValue(cfm,mlocal,world.global_cfm);
		        
		        lo = memarena.AllocateArrayDReal (mlocal);
		        dSetValue (lo,mlocal,-dInfinity);

		        hi = memarena.AllocateArrayDReal (mlocal);
		        dSetValue (hi,mlocal, dInfinity);

		        findex = memarena.AllocateArrayInt (mlocal);
		        for (int i=0; i<mlocal; i++) findex[i] = -1;

		        final int jbelements = mlocal*2;
		        jb = memarena.AllocateArrayInt (jbelements);

		        rhs = memarena.AllocateArrayDReal (mlocal);

		        Jcopy = memarena.AllocateArrayDReal (mfb*12);
            }

		    //BEGIN_STATE_SAVE(memarena, cstate);
		    BlockPointer cstate = memarena.BEGIN_STATE_SAVE();
		    {
		        double[] c = memarena.AllocateArrayDReal (m);
		        dSetZero (c, m);

		        {
		            if (TIMING) dTimerNow ("create J");
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
		            DxJoint.Info2 Jinfo = new DxJoint.Info2();
		            Jinfo.setRowskip(12);
		            Jinfo.setArrays(J, c, cfm, lo, hi, findex);
		            Jinfo.fps = stepsize1;
		            Jinfo.erp = world.getERP();
		            
		            int JcopyrowP = 0;//double[] Jcopyrow = Jcopy;
		            int ofsi = 0;
		            // const dJointWithInfo1 *jicurr = jointiinfos;
		            for (int i = 0; i < njXXX; i++) {
		                DJointWithInfo1 jicurr = jointiinfos[i];
		                int JrowP = 0 + ofsi * 12;		                
		                Jinfo.J1lp = JrowP;
		                Jinfo.J1ap = JrowP + 3;
		                Jinfo.J2lp = JrowP + 6;
		                Jinfo.J2ap = JrowP + 9;
//		                Jinfo.setC(c, ofsi);
//		                Jinfo.setCfm( cfm, ofsi );
//		                Jinfo.setLo( lo, ofsi );
//		                Jinfo.setHi( hi, ofsi );
//		                Jinfo.setFindex( findex, ofsi );
		                Jinfo.setAllP(ofsi);
		                
		                DxJoint joint = jicurr.joint;
		                joint.getInfo2 (Jinfo);

		                final int infom = jicurr.info.m;

		                
//		                Jinfo.J1lp = ofs[i]*12;//J + ofs[i]*12;
//		                Jinfo.J1ap = ofs[i]*12 + 3;// = Jinfo.J1l + 3;  
//		                Jinfo.J2lp = ofs[i]*12 + 6;//  = Jinfo.J1l + 6;
//		                Jinfo.J2ap = ofs[i]*12 + 9;//  = Jinfo.J1l + 9;
//		                Jinfo.setAllP(ofs[i]);
				
//				Jinfo.c.set(c, ofs[i]);//				Jinfo.c = c + ofs[i];
//				Jinfo.cfm.set(cfm, ofs[i]);//				Jinfo.cfm = cfm + ofs[i];
//				Jinfo.lo.set(lo, ofs[i]);//				Jinfo.lo = lo + ofs[i];
//				Jinfo.hi.set(hi, ofs[i]);//				Jinfo.hi = hi + ofs[i];
//				joint[i].getInfo2 (Jinfo);
//				
//				// adjust returned findex values for global index numbering
//				for (j=0; j<info[i].getM(); j++) {
//					if (findex[ofs[i] + j] >= 0) findex[ofs[i] + j] += ofs[i];
//				}
//				if (joint[i].feedback!=null)
//					mfb += info[i].getM();
//			}

		                // we need a copy of Jacobian for joint feedbacks
		                // because it gets destroyed by SOR solver
		                // instead of saving all Jacobian, we can save just rows
		                // for joints, that requested feedback (which is normaly much less)
		                if (joint.feedback != null) {
		                    int rowels = infom * 12;
		                    memcpy(Jcopy, JcopyrowP, J, JrowP, rowels);
		                    JcopyrowP += rowels;
		                }

		                // adjust returned findex values for global index numbering
		                int findex_ofsi = ofsi; //findex + ofsi;
		                for (int j=0; j<infom; j++) {
		                    int fival = findex[findex_ofsi + j];
		                    if (fival != -1) {
		                        //findex_ofsi[j] = fival + ofsi;
		                        findex[findex_ofsi + j] = fival + ofsi;
		                    }
		                }

		                ofsi += infom;
		            }
		        }


		        {
		            // create an array of body numbers for each joint row
		            int jb_ofs = 0;//int[] jb_ptr = jb;
		            for (int i=0; i<njXXX; i++) {
		                DxJoint joint = jointiinfos[i].joint;
		                int infom = jointiinfos[i].info.m;
		                
		                int b1 = (joint.node[0].body!=null) ? (joint.node[0].body.tag) : -1;
		                int b2 = (joint.node[1].body!=null) ? (joint.node[1].body.tag) : -1;
		                for (int j=0; j<infom; j++) {
		                    jb[jb_ofs] = b1;//jb_ptr[0] = b1;
		                    jb[jb_ofs+1] = b2;//jb_ptr[1] = b2;
		                    jb_ofs += 2;//jb_ptr += 2;
		                }
		            }
		            dIASSERT (jb_ofs == 2*m);//(jb_ptr == jb+2*m);
		        }
    			
		        BlockPointer tmp1state = memarena.BEGIN_STATE_SAVE(); {
		            if (TIMING) dTimerNow ("compute rhs");
		        
		            // compute the right hand side `rhs'
		            double[] tmp1 = memarena.AllocateArrayDReal(nb*6);//new double[nb*6];//dRealAllocaArray (tmp1,nb*6);
		            // put v/h + invM*fe into tmp1
		            int tmp1currP = 0;//tmp1;
		            int invIrowP = 0; //invI
		            for (int i=0; i<nb; tmp1currP+=6, invIrowP+=12, i++) {
		                DxBody b = bodyP[i+bodyOfs];
		                double body_invMass = b.invMass;
		                for (int j=0; j<3; j++) {
		                    tmp1[tmp1currP+j] = 
		                        b.facc.get(j) * body_invMass + b.lvel.get(j) * stepsize1;
		                }
		                //dMULTIPLY0_331 (tmp1, i*6 + 3,invI, i*12,body[i].tacc.v, 0);
		                dMultiply0_331 (tmp1, tmp1currP + 3 ,invI, invIrowP, b.tacc);
		                for (int k=0; k<3; k++) 
		                    tmp1[tmp1currP+3+k] += b.avel.get(k) * stepsize1;
		            }

		            // put J*tmp1 into rhs
		            multiply_J (m,J,jb,tmp1,rhs);
		        }
		        memarena.END_STATE_SAVE(tmp1state);

    			// complete rhs
    			for (int i=0; i<m; i++) rhs[i] = c[i]*stepsize1 - rhs[i];
    
    			// scale CFM
    			for (int j=0; j<m; j++) cfm[j] *= stepsize1;

		    }
		    memarena.END_STATE_SAVE(cstate);
    			
			// load lambda from the value saved on the previous iteration
			double[] lambda = memarena.AllocateArrayDReal(m);//new double[m];//dRealAllocaArray (lambda,m);
			
			//TZ not defined
//			if (WARM_STARTING) {//#ifdef WARM_STARTING
//		     dReal *lambdscurr = lambda;
//		      const dJointWithInfo1 *jicurr = jointiinfos;
//		      const dJointWithInfo1 *const jiend = jicurr + nj;
//		      for (; jicurr != jiend; jicurr++) {
//		        unsigned int infom = jicurr->info.m;
//		        memcpy (lambdscurr, jicurr->joint->lambda, (size_t)infom * sizeof(dReal));
//		        lambdscurr += infom;
//		      }
//			}//#endif

			double[] cforce = memarena.AllocateArrayDReal(nb*6);
			BlockPointer lcpstate = memarena.BEGIN_STATE_SAVE(); 
			{
	            if (TIMING) dTimerNow ("solving LCP problem");
	            // solve the LCP problem and get lambda and invM*constraint_force
	            SOR_LCP (memarena,m,nb,J,jb,bodyP,bodyOfs,invI,lambda,cforce,rhs,lo,hi,cfm,findex,world.qs);
			}
			memarena.END_STATE_SAVE(lcpstate);
			    
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
//		    {
//		        // save lambda for the next iteration
//		        //@@@ note that this doesn't work for contact joints yet, as they are
//		        // recreated every iteration
//		        const dReal *lambdacurr = lambda;
//		        const dJointWithInfo1 *jicurr = jointiinfos;
//		        const dJointWithInfo1 *const jiend = jicurr + nj;
//		        for (; jicurr != jiend; jicurr++) {
//		          unsigned int infom = jicurr->info.m;
//		          memcpy (jicurr->joint->lambda, lambdacurr, (size_t)infom * sizeof(dReal));
//		          lambdacurr += infom;
//		        }
//		      }
//				//#endif

			// note that the SOR method overwrites rhs and J at this point, so
			// they should not be used again.

			{
			    // add stepsize * cforce to the body velocity
			    int cforcecurrP = 0; //cforce
			    for (int i=0; i<nb; cforcecurrP+=6, i++) {
			        DxBody b = bodyP[i+bodyOfs];
			        for (int j=0; j<3; j++) {
			            b.lvel.add(j, stepsize * cforce[cforcecurrP+j] );
			            b.avel.add(j, stepsize * cforce[cforcecurrP+3+j] );
			        }
			    }
			}

			if (mfb > 0) {
			    // straightforward computation of joint constraint forces:
			    // multiply related lambdas with respective J' block for joints
			    // where feedback was requested
			    //double[] data = new double[6];
			    int lambdacurrP = 0;//lambda;
			    int JcopyrowP = 0;//Jcopy;
			    //int jicurrP = 0;//jointiinfos;
				//mfb = 0;
				for (int i=0; i<njXXX; i++) {
				    DxJoint joint = jointiinfos[i].joint;
				    int infom = jointiinfos[i].info.m;
					if (joint.feedback != null) {
						DJoint.DJointFeedback fb = joint.feedback;
//						double[] data = new double[6];
			          Multiply1_12q1 (fb.f1, fb.t1, Jcopy, JcopyrowP, lambda, lambdacurrP, infom);
////						fb.f1.v[0] = data[0];
////						fb.f1.v[1] = data[1];
////						fb.f1.v[2] = data[2];
////						fb.t1.v[0] = data[3];
////						fb.t1.v[1] = data[4];
////						fb.t1.v[2] = data[5];
						if (joint.node[1].body != null)
						{
						    //Multiply1_12q1 (data, Jcopyrow+6, lambdacurr, infom);
				            Multiply1_12q1 (fb.f2, fb.t2, Jcopy, JcopyrowP+6, lambda, lambdacurrP, infom);
////							fb.f2.v[0] = data[0];
////							fb.f2.v[1] = data[1];
////							fb.f2.v[2] = data[2];
////							fb.t2.v[0] = data[3];
////							fb.t2.v[1] = data[4];
////							fb.t2.v[2] = data[5];
						}
				          JcopyrowP += infom * 12;
			        }
			      
			        lambdacurrP += infom;
				}
			}
		}

		{
		    if (TIMING) dTimerNow ("compute velocity update");
		    // compute the velocity update:
		    // add stepsize * invM * fe to the body velocity
		    int invIrowP = 0;//invI
		    for (int i=0; i<nb; invIrowP += 12, i++) {
		        DxBody b = bodyP[i+bodyOfs]; 
		        double body_invMass_mul_stepsize = stepsize * b.invMass;
		        for (int j=0; j<3; j++) {
		            b.lvel.add(j, body_invMass_mul_stepsize * b.facc.get(j) );
		            b.tacc.scale( stepsize );
		        }
		        dMultiplyAdd0_331 (b.avel, invI,invIrowP, b.tacc);
		    }
		}

		if (CHECK_VELOCITY_OBEYS_CONSTRAINT) {//#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
		    throw new UnsupportedOperationException();
//		  if (m > 0) {
//		    BEGIN_STATE_SAVE(memarena, velstate) {
//		      dReal *vel = memarena->AllocateArray<dReal>((size_t)nb*6);
//
//		      // check that the updated velocity obeys the constraint (this check needs unmodified J)
//		      dReal *velcurr = vel;
//		      dxBody *bodycurr = body, *const bodyend = body + nb;
//		      for (; bodycurr != bodyend; velcurr += 6, bodycurr++) {
//		        for (unsigned int j=0; j<3; j++) {
//		          velcurr[j] = bodycurr->lvel[j];
//		          velcurr[3+j] = bodycurr->avel[j];
//		        }
//		      }
//		      dReal *tmp = memarena->AllocateArray<dReal> (m);
//		      multiply_J (m,J,jb,vel,tmp);
//		      dReal error = 0;
//		      for (unsigned int i=0; i<m; i++) error += dFabs(tmp[i]);
//		      printf ("velocity error = %10.6e\n",error);
//		    
//		    } END_STATE_SAVE(memarena, velstate)
//		  }
		} //#endif

		{
    		// update the position and orientation from the new linear/angular velocity
    		// (over the given timestep)
		    if (TIMING) dTimerNow ("update position");
		    for (int i=0; i<nb; i++) {
		        bodyP[i+bodyOfs].dxStepBody (stepsize);
		    }
		}
		
		{
		    if (TIMING) dTimerNow ("tidy up");
    
    		// zero all force accumulators
    		for (int i=0; i<nb; i++) {
    		    DxBody b = bodyP[i+bodyOfs];
    			b.facc.setZero();//dSetZero (body[i].facc,3);
    			b.tacc.setZero();//dSetZero (body[i].tacc,3);
    		}
		}

		if (TIMING) dTimerEnd();
		if (TIMING) if (m > 0) dTimerReport (stdout,1);
	}

//	static size_t EstimateGR_LCPMemoryRequirements(unsigned int m)
//	{
//	    //TZ not defined
//	    #ifdef USE_CG_LCP
//	  size_t res = dEFFICIENT_SIZE(sizeof(dReal) * 12 * (size_t)m); // for iMJ
//	  res += 5 * dEFFICIENT_SIZE(sizeof(dReal) * (size_t)m); // for r, z, p, q, Ad
//	  return res;
//	    #endif
//	}

//	private static final int sizeof(Class<?> c) {
//	    if (c == double.class) {
//	        return 8;
//	    } else if (c == int.class) {
//	        return 4;
//	    } else {
//	        throw new IllegalArgumentException(c.getName());
//	    }
//	}
	
//	static int EstimateSOR_LCPMemoryRequirements(int m)
//	{
//	    int res = dEFFICIENT_SIZE(sizeof(double.class) * 12 * m); // for iMJ
//	    res += dEFFICIENT_SIZE(sizeof(double.class) * m); // for Ad
//	    res += dEFFICIENT_SIZE(sizeof(IndexError.class) * m); // for order
//	    if (REORDER_CONSTRAINTS) {//	#ifdef REORDER_CONSTRAINTS
//	        res += dEFFICIENT_SIZE(sizeof(double.class) * m); // for last_lambda
//	    }//#endif
//	    return res;
//	}
//
//	//	size_t dxEstimateQuickStepMemoryRequirements (
//	//	  dxBody * const *body, unsigned int nb, dxJoint * const *_joint, unsigned int _nj)
	@Override
	public int dxEstimateMemoryRequirements (
	        DxBody[] body, int bodyOfs, int nb, 
	        DxJoint[] _joint, int jointOfs, int _nj)
	{
//	    int nj, m, mfb;
//
//	    {
//	        int njcurr = 0, mcurr = 0, mfbcurr = 0;
//	        DxJoint.SureMaxInfo info;
//	        //	    DxJoint *const *const _jend = _joint + _nj;
//	        //	    for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; _jcurr++) {
//	        for (int i = 0; i < _nj; i++) {
//	            DxJoint j = _joint[i];//_jcurr;
//	            j.getSureMaxInfo (&info);
//
//	            int jm = info.max_m;
//	            if (jm > 0) {
//	                njcurr++;
//
//	                mcurr += jm;
//	                if (j.feedback != null) {
//	                    mfbcurr += jm;
//	                }
//	            }
//	        }
//	        nj = njcurr; m = mcurr; mfb = mfbcurr;
//	    }
//
//	    int res = 0;
//
//	    res += dEFFICIENT_SIZE(sizeof(double.class) * 3 * 4 * nb); // for invI
//
//	    {
//	        int sub1_res1 = dEFFICIENT_SIZE(sizeof(DJointWithInfo1.class) * _nj); // for initial jointiinfos
//
//	        int sub1_res2 = dEFFICIENT_SIZE(sizeof(DJointWithInfo1.class) * nj); // for shrunk jointiinfos
//	        if (m > 0) {
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(double.class) * 12 * m); // for J
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(int.class) * 12 * m); // for jb
//	            sub1_res2 += 4 * dEFFICIENT_SIZE(sizeof(double.class) * m); // for cfm, lo, hi, rhs
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(int.class) * m); // for findex
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(double.class) * 12 * mfb); // for Jcopy
//	            {
//	        int sub2_res1 = dEFFICIENT_SIZE(sizeof(double.class) * m); // for c
//	        {
//	            int sub3_res1 = dEFFICIENT_SIZE(sizeof(double.class) * 6 * nb); // for tmp1
//	    
//	            int sub3_res2 = 0;
//
//	          sub2_res1 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
//	        }
//
//	        int sub2_res2 = dEFFICIENT_SIZE(sizeof(double.class) * m); // for lambda
//	        sub2_res2 += dEFFICIENT_SIZE(sizeof(double.class) * 6 * nb); // for cforce
//	        {
//	            int sub3_res1 = EstimateSOR_LCPMemoryRequirements(m); // for SOR_LCP
//
//	            int sub3_res2 = 0;
//	if (CHECK_VELOCITY_OBEYS_CONSTRAINT) {//#ifdef CHECK_VELOCITY_OBEYS_CONSTRAINT
//	          {
//	              int sub4_res1 = dEFFICIENT_SIZE(sizeof(double.class) * 6 * nb); // for vel
//	            sub4_res1 += dEFFICIENT_SIZE(sizeof(double.class) * m); // for tmp
//
//	            int sub4_res2 = 0;
//
//	            sub3_res2 += (sub4_res1 >= sub4_res2) ? sub4_res1 : sub4_res2;
//	          }
//	}//#endif
//	          sub2_res2 += (sub3_res1 >= sub3_res2) ? sub3_res1 : sub3_res2;
//	        }
//
//	        sub1_res2 += (sub2_res1 >= sub2_res2) ? sub2_res1 : sub2_res2;
//	      }
//	    }
//	    
//	    res += (sub1_res1 >= sub1_res2) ? sub1_res1 : sub1_res2;
//	  }
//
//	  return res;
	    return -1;
	}


	@Override
	public void run(DxWorldProcessMemArena memarena, 
	        DxWorld world, DxBody[] body, int bodyOfs, int nb, 
	        DxJoint[] joint, int jointOfs, int nj,
			double stepsize) {
		dxQuickStepper(memarena, world, body, bodyOfs, nb, joint, jointOfs, nj, stepsize);
	}
}