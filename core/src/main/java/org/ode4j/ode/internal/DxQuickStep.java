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

import static org.ode4j.ode.DMatrix.dSetValue;
import static org.ode4j.ode.DMatrix.dSetZero;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.dInvertMatrix3;
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply0_333;
import static org.ode4j.ode.OdeMath.dMultiply2_333;
import static org.ode4j.ode.OdeMath.dMultiplyAdd0_331;
import static org.ode4j.ode.OdeMath.dSetCrossMatrixMinus;
import static org.ode4j.ode.internal.Common.dFabs;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dIVERIFY;
import static org.ode4j.ode.internal.Common.dRecip;
import static org.ode4j.ode.internal.Matrix.dSetValue;
import static org.ode4j.ode.internal.Matrix.dSetZero;
import static org.ode4j.ode.internal.Misc.dRandInt;
import static org.ode4j.ode.internal.Timer.dTimerEnd;
import static org.ode4j.ode.internal.Timer.dTimerNow;
import static org.ode4j.ode.internal.Timer.dTimerReport;
import static org.ode4j.ode.internal.Timer.dTimerStart;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stdout;

import java.util.concurrent.atomic.AtomicInteger;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.internal.Objects_H.dxQuickStepParameters;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dmaxcallcountestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemArena;
import org.ode4j.ode.threading.ThreadingUtils;
import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;
import org.ode4j.ode.threading.Threading_H.dThreadedCallFunction;

/**
 *
 * Quickstep stepper.
 */
public class DxQuickStep extends AbstractStepper implements dstepper_fn_t,
dmemestimate_fn_t, dmaxcallcountestimate_fn_t {
	
    //TZ where is this defined???
    private static final boolean CHECK_VELOCITY_OBEYS_CONSTRAINT = false;
    
	/** DxQuickStep singleton instance. */
	public static final DxQuickStep INSTANCE = new DxQuickStep();
	
	//void dxQuickStepper (dxWorld *world, dxBody * const *body, int nb,
	//	     dxJoint * const *_joint, int nj, dReal stepsize);


	private static final boolean TIMING = false;
	
	//#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
	//#define dMAX(A,B)  ((B)>(A) ? (B) : (A))
	//TZ not used...
	//private static final int dMIN(int A, int B) { return ((A)>(B) ? (B) : (A)); }
	//private static final int dMAX(int A, int B) { return ((B)>(A) ? (B) : (A)); }

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



	private static class DJointWithInfo1
	{
		DxJoint joint;
		final DxJoint.Info1 info = new DxJoint.Info1();
	}

	private static class dxQuickStepperStage0Outputs
	{
		int                    nj;
		int                    m;
		int                    mfb;
	}

	private static class dxQuickStepperStage1CallContext implements CallContext
	{
		void Initialize(DxStepperProcessingCallContext stepperCallContext, 
				BlockPointer stageMemArenaState, double[] invI, DJointWithInfo1[] jointinfos)
		{
			m_stepperCallContext = stepperCallContext;
			m_stageMemArenaState = stageMemArenaState; 
			m_invI = invI;
			m_jointinfos = jointinfos;
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		BlockPointer                       m_stageMemArenaState;
		double[]                           m_invI;
		DJointWithInfo1[]                 m_jointinfos;
		final dxQuickStepperStage0Outputs     m_stage0Outputs = new dxQuickStepperStage0Outputs();
	}

	private static class dxQuickStepperStage0BodiesCallContext implements CallContext
	{
		void Initialize(final DxStepperProcessingCallContext stepperCallContext, 
				double[] invI)
		{
			m_stepperCallContext = stepperCallContext;
			m_invI = invI;
			//m_tagsTaken = 0;
			//m_gravityTaken = 0;
			//m_inertiaBodyIndex = 0;
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		double[]                           m_invI;
		final AtomicInteger                     m_tagsTaken = new AtomicInteger();
		final AtomicInteger                     m_gravityTaken = new AtomicInteger();
		//unsigned int                    volatile m_inertiaBodyIndex;
		final AtomicInteger                    m_inertiaBodyIndex = new AtomicInteger();
	}

	private static class dxQuickStepperStage0JointsCallContext implements CallContext
	{
		void Initialize(DxStepperProcessingCallContext stepperCallContext, 
				DJointWithInfo1[] jointinfos, dxQuickStepperStage0Outputs stage0Outputs)
		{
			m_stepperCallContext = stepperCallContext;
			m_jointinfos = jointinfos;
			m_stage0Outputs = stage0Outputs;
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		DJointWithInfo1[]                 m_jointinfos;
		dxQuickStepperStage0Outputs     m_stage0Outputs;
	}

	//static int dxQuickStepIsland_Stage0_Bodies_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static int dxQuickStepIsland_Stage0_Joints_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static int dxQuickStepIsland_Stage1_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

	//static void dxQuickStepIsland_Stage0_Bodies(dxQuickStepperStage0BodiesCallContext *callContext);
	//static void dxQuickStepIsland_Stage0_Joints(dxQuickStepperStage0JointsCallContext *callContext);
	//static void dxQuickStepIsland_Stage1(dxQuickStepperStage1CallContext *callContext);


	private static class dxQuickStepperLocalContext
	{
//		void Initialize(dReal *invI, dJointWithInfo1 *jointinfos, unsigned int nj, 
//				unsigned int m, unsigned int mfb, const unsigned int *mindex, int *findex, 
//				dReal *J, dReal *cfm, dReal *lo, dReal *hi, int *jb, dReal *rhs, dReal *Jcopy)
		void Initialize(double[] invI, DJointWithInfo1[] jointinfos, int nj, 
				int m, int mfb, final int[] mindex, int[] findex, 
				double[] J, double[] cfm, double[] lo, double[] hi, int[] jb, double[] rhs, double[] Jcopy)
		{
			m_invI = invI;
			m_jointinfos = jointinfos;
			m_nj = nj;
			m_m = m;
			m_mfb = mfb;
			m_mindex = mindex;
			m_findex = findex; 
			m_J = J;
			m_cfm = cfm;
			m_lo = lo;
			m_hi = hi;
			m_jb = jb;
			m_rhs = rhs;
			m_Jcopy = Jcopy;
		}

		double[]                        m_invI;
		DJointWithInfo1[]                 m_jointinfos;
		int                    m_nj;
		int                    m_m;
		int                    m_mfb;
		int[]              m_mindex;
		int[]                            m_findex;
		double[]                         m_J;
		double[]                         m_cfm;
		double[]                         m_lo;
		double[]                         m_hi;
		int[]                            m_jb;
		double[]                         m_rhs;
		double[]                         m_Jcopy;
	};

	private static class dxQuickStepperStage3CallContext implements CallContext
	{
		void Initialize(DxStepperProcessingCallContext callContext, 
				dxQuickStepperLocalContext localContext, 
				BlockPointer stage1MemArenaState)
		{
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			m_stage1MemArenaState = stage1MemArenaState;
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		dxQuickStepperLocalContext   m_localContext;
		BlockPointer                            m_stage1MemArenaState;
	};

	private static class dxQuickStepperStage2CallContext implements CallContext
	{
		void Initialize(DxStepperProcessingCallContext callContext, 
				dxQuickStepperLocalContext localContext, 
				double[] rhs_tmp)
		{
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			m_rhs_tmp = rhs_tmp;
			//m_ji_J = 0;
			//m_ji_jb = 0;
			//m_bi = 0;
			//m_Jrhsi = 0;
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		dxQuickStepperLocalContext   m_localContext;
		double[]                           m_rhs_tmp;
//		volatile unsigned int           m_ji_J;
//		volatile unsigned int           m_ji_jb;
//		volatile unsigned int           m_bi;
//		volatile unsigned int           m_Jrhsi;
		final AtomicInteger           m_ji_J = new AtomicInteger();
		final AtomicInteger           m_ji_jb = new AtomicInteger();
		final AtomicInteger           m_bi = new AtomicInteger();
		final AtomicInteger           m_Jrhsi = new AtomicInteger();
	};

	//static int dxQuickStepIsland_Stage2a_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static int dxQuickStepIsland_Stage2aSync_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static int dxQuickStepIsland_Stage2b_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static int dxQuickStepIsland_Stage2bSync_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static int dxQuickStepIsland_Stage2c_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static int dxQuickStepIsland_Stage3_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

	//static void dxQuickStepIsland_Stage2a(dxQuickStepperStage2CallContext *callContext);
	//static void dxQuickStepIsland_Stage2b(dxQuickStepperStage2CallContext *callContext);
	//static void dxQuickStepIsland_Stage2c(dxQuickStepperStage2CallContext *callContext);
	//static void dxQuickStepIsland_Stage3(dxQuickStepperStage3CallContext *callContext);


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
		int iMJ_ofs = 0;//TZ
		int J_ofs = 0;//TZ
		for (int i=0; i<m; J_ofs +=12, iMJ_ofs += 12, i++) {
			int b1 = jb[i*2];
			int b2 = jb[i*2+1];
			double k1 = bodyP[b1+bodyOfs].invMass;
//TZ			for (j=0; j<3; j++) iMJ_ptr[j] = k*J_ptr[j];
			for (int j=0; j<3; j++) iMJ[j + iMJ_ofs] = k1*J[j + J_ofs];
//			dMULTIPLY0_331 (iMJ_ptr + 3, invI + 12*b1, J_ptr + 3);
			dMultiply0_331 (iMJ, iMJ_ofs + 3, invI, 12*b1, J,J_ofs + 3);
			if (b2 != -1) {
				double k2 = bodyP[b2+bodyOfs].invMass;
				//for (j=0; j<3; j++) iMJ_ptr[j+6] = k*J_ptr[j+6];
				for (int j=0; j<3; j++) iMJ[j+6+iMJ_ofs] = k2*J[j+6+J_ofs];
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
	private static void multiplyAdd_J (AtomicInteger mi_storage, 
			int m, final double[] J, final int[] jb, final double[] in, double[] out)
	{
		int mi;
		while ((mi = ThreadingUtils.ThrsafeIncrementIntUpToLimit(mi_storage, m)) != m) {
			int b1 = jb[mi*2];
			int b2 = jb[mi*2+1];
			int J_ofs = mi*12;//const dReal *J_ptr = J + mi * 12;
			double sum = 0.0;
			int in_ofs = b1*6;//const dReal *in_ptr = in + (size_t)(unsigned)b1*6;
			for (int j = 0; j < 6; ++j) sum += J[J_ofs+j]*in[in_ofs+j];//J_ptr[j] * in_ptr[j];
			if (b2 != -1) {
				//in_ptr = in + (size_t)(unsigned)b2*6;
				in_ofs = b2*6;
				//for (int j=0; j<6; j++) sum += J_ptr[6 + j] * in_ptr[j];
				for (int j=0; j<6; j++) sum += J[J_ofs + 6 + j] * in[in_ofs + j];
			}
			out[mi] += sum;
		}
	}

	// Single threaded versionto be removed later
	//#if defined(WARM_STARTING) || defined(CHECK_VELOCITY_OBEYS_CONSTRAINT)
	//TZ: not used...
//	private static void _multiply_J (int m, final double[] J, final int[]jb,
//			final double[] in, final double[] out)
//	{
//		int J_ofs = 0;//final double[] J_ptr = J;
//		for (int i=0; i<m; i++) {
//			int b1 = jb[i*2];
//			int b2 = jb[i*2+1];
//			double sum = 0;
//			int in_ofs = b1*6; //double[] in_ptr = in + b1*6;
//			for (int j=0; j<6; j++) sum += J[j + J_ofs] * in[j + in_ofs];//J_ptr[j]*in_ptr[j];
//			J_ofs += 6;//J_ptr += 6;
//			if (b2 != -1) {
//				in_ofs = b2*6;//in_ptr = in + b2*6;
//				for (int j=0; j<6; j++) sum += J[j + J_ofs] * in[j + in_ofs];//J_ptr[j] * in_ptr[j];
//			}
//			J_ofs += 6;//J_ptr += 6;
//			out[i] = sum;
//		}
//	}
	//#endif

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
//		_multiply_J (m,J,jb,z,out);
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
	//			const dReal *iMJ_ptr = iMJ;
	//			const dReal *J_ptr = J;
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
		int head_size = 0;

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
	        head_size = orderhead;// - order;
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
//				    head_size = 0;
//					for (int i=0; i<m; i++) {
//						int findex_i = findex[i];				
//						order[i].error = i;
//						order[i].findex = findex_i;
//						order[i].index = i;
//		                if (findex_i == -1) { ++head_size; }
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
//		                int findex_i = findex[i];
//						order[i].findex = findex_i;
//						order[i].index = i;
//		                if (findex_i == -1) { ++head_size; }
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
					for (int i=1; i<head_size; i++) {
					    int swapi = dRandInt(i+1);
                        IndexError tmp = order[i];
						order[i] = order[swapi];
						order[swapi] = tmp;
					}
		            int tail_size = m - head_size;
		            for (int j=1; j<tail_size; j++) {
		                int swapj = dRandInt(j+1);
		                IndexError tmp = order[head_size + j];
		                order[head_size + j] = order[head_size + swapj];
		                order[head_size + swapj] = tmp;
		            }
				}
			}//#endif

			for (int i=0; i<m; i++) {
				// @@@ potential optimization: we could pre-sort J and iMJ, thereby
				//     linearizing access to those arrays. hmmm, this does not seem
				//     like a win, but we should think carefully about our memory
				//     access pattern.
				int index = order[i].index;

				int fc_ofs1;//dReal *fc_ptr1;
				int fc_ofs2;//dReal *fc_ptr2;
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
				    final int J_ofs = index*12;
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
				    final int iMJ_ofs = 0+index*12; //dRealPtr iMJ_ptr = iMJ + (size_t)index*12;
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
	

	/*extern */
	private void dxQuickStepIsland(DxStepperProcessingCallContext callContext)
	{
		if (TIMING) dTimerStart("preprocessing");

	    DxWorldProcessMemArena memarena = callContext.m_stepperArena();
	    DxWorld world = callContext.m_world();
	    int nb = callContext.m_islandBodiesCount();
	    int _nj = callContext.m_islandJointsCount();

	    double[] invI = memarena.AllocateArrayDReal(nb*3*4);//new double[3*4*nb];//dRealAllocaArray (invI,3*4*nb);
	    //dJointWithInfo1[] const jointinfos = memarena.AllocateArray<dJointWithInfo1>(_nj);
	    memarena.dummy();
	    DJointWithInfo1[] jointinfos = new DJointWithInfo1[_nj];
	    //TZ: init TODO is this necessary?
//	    for (int i = 0; i < jointinfos.length; i++) {
//	    	jointinfos[i] = new DJointWithInfo1();
//	    }
	    
	    final int allowedThreads = callContext.m_stepperAllowedThreads();
	    dIASSERT(allowedThreads != 0);

	    BlockPointer stagesMemArenaState = memarena.SaveState();

	    memarena.dummy();
	    dxQuickStepperStage1CallContext stage1CallContext = new dxQuickStepperStage1CallContext(); 
	    		//(dxQuickStepperStage1CallContext )memarena.AllocateBlock(sizeof(dxQuickStepperStage1CallContext));
	    stage1CallContext.Initialize(callContext, stagesMemArenaState, invI, jointinfos);

	    memarena.dummy();
	    dxQuickStepperStage0BodiesCallContext stage0BodiesCallContext = new dxQuickStepperStage0BodiesCallContext(); 
	    		//(dxQuickStepperStage0BodiesCallContext)memarena.AllocateBlock(sizeof(dxQuickStepperStage0BodiesCallContext));
	    stage0BodiesCallContext.Initialize(callContext, invI);

	    memarena.dummy();
	    dxQuickStepperStage0JointsCallContext stage0JointsCallContext = new dxQuickStepperStage0JointsCallContext(); 
	    		//(dxQuickStepperStage0JointsCallContext)memarena.AllocateBlock(sizeof(dxQuickStepperStage0JointsCallContext));
	    stage0JointsCallContext.Initialize(callContext, jointinfos, stage1CallContext.m_stage0Outputs);

	    if (allowedThreads == 1)
	    {
	        dxQuickStepIsland_Stage0_Bodies(stage0BodiesCallContext);
	        dxQuickStepIsland_Stage0_Joints(stage0JointsCallContext);
	        dxQuickStepIsland_Stage1(stage1CallContext);
	    }
	    else
	    {
	        int bodyThreads = allowedThreads;
	        int jointThreads = 1;

	        Ref<DCallReleasee> stage1CallReleasee = new Ref<>();
	        world.threading().PostThreadedCallForUnawareReleasee(null, stage1CallReleasee, 
	        		bodyThreads + jointThreads, callContext.m_finalReleasee(), 
	        		null, dxQuickStepIsland_Stage1_Callback, stage1CallContext, 0, 
	        		"QuickStepIsland Stage1");

	        world.threading().PostThreadedCallsGroup(null, bodyThreads, stage1CallReleasee.get(), 
	        		dxQuickStepIsland_Stage0_Bodies_Callback, stage0BodiesCallContext, 
	        		"QuickStepIsland Stage0-Bodies");

	        world.threading().PostThreadedCall(null, null, 0, stage1CallReleasee.get(), null, 
	        		dxQuickStepIsland_Stage0_Joints_Callback, stage0JointsCallContext, 0, 
	        		"QuickStepIsland Stage0-Joints");
	        dIASSERT(jointThreads == 1);
	    }
	}    

	private static dThreadedCallFunction dxQuickStepIsland_Stage0_Bodies_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _callContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxQuickStepperStage0BodiesCallContext callContext = (dxQuickStepperStage0BodiesCallContext)_callContext;
			dxQuickStepIsland_Stage0_Bodies(callContext);
			return true;
		}
	};

	private static 
	void dxQuickStepIsland_Stage0_Bodies(dxQuickStepperStage0BodiesCallContext callContext)
	{
	    DxBody[] bodyP = callContext.m_stepperCallContext.m_islandBodiesStartA();
	    int bodyOfs = callContext.m_stepperCallContext.m_islandBodiesStartOfs();
	    int nb = callContext.m_stepperCallContext.m_islandBodiesCount();

	    if (ThreadingUtils.ThrsafeExchange(callContext.m_tagsTaken, 1) == 0)
	    {
	        // number all bodies in the body list - set their tag values
	        for (int i=0; i<nb; i++) bodyP[bodyOfs+i].tag = i;
	    }

	    if (ThreadingUtils.ThrsafeExchange(callContext.m_gravityTaken, 1) == 0)
	    {
	        DxWorld world = callContext.m_stepperCallContext.m_world();

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

	    // for all bodies, compute the inertia tensor and its inverse in the global
	    // frame, and compute the rotational force and add it to the torque
	    // accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
	    {
	        double[] invIrowA = callContext.m_invI;
	        int invIrowP = 0;
	        int bodyIndex = ThreadingUtils.ThrsafeIncrementIntUpToLimit(callContext.m_inertiaBodyIndex, nb);

	        for (int i = 0; i != nb; invIrowP += 12, ++i) {
	            if (i == bodyIndex) {
	                DMatrix3 tmp = new DMatrix3();
	                DxBody b = bodyP[bodyOfs+i];

	                // compute inverse inertia tensor in global frame
	                dMultiply2_333 (tmp,b.invI,b.posr().R());
	                dMultiply0_333 (invIrowA, invIrowP,b.posr().R(),tmp);

	                // Don't apply gyroscopic torques to bodies
	                // if not flagged or the body is kinematic
	                if (b.isFlagsGyroscopic() && (b.invMass>0)) {
	                    DMatrix3 I = new DMatrix3();
	                    // compute inertia tensor in global frame
	                    dMultiply2_333 (tmp,b.mass._I,b.posr().R());
	                    dMultiply0_333 (I,b.posr().R(),tmp);
	                    // compute rotational force
//	#if 0
//	                    // Explicit computation
//	                    dMultiply0_331 (tmp,I,b.avel);
//	                    dSubtractVectorCross3(b.tacc,b.avel,tmp);
//	#else
	                    // Do the implicit computation based on 
	                    //"Stabilizing Gyroscopic Forces in Rigid Multibody Simulations"
	                    // (LacoursiÃ¨re 2006)
	                    double h = callContext.m_stepperCallContext.m_stepSize(); // Step size
	                    DVector3 L = new DVector3(); // Compute angular momentum
	                    dMultiply0_331(L,I,b.avel);
	                    
	                    // Compute a new effective 'inertia tensor'
	                    // for the implicit step: the cross-product 
	                    // matrix of the angular momentum plus the
	                    // old tensor scaled by the timestep.  
	                    // Itild may not be symmetric pos-definite, 
	                    // but we can still use it to compute implicit
	                    // gyroscopic torques.
	                    DMatrix3 Itild= new DMatrix3();//{0};  
	                    dSetCrossMatrixMinus(Itild,L);//,4);
//	                    for (int ii=0;ii<12;++ii) {
//	                      Itild[ii]=Itild[ii]*h+I[ii];
//	                    }
	                    Itild.scale(h);
	                    Itild.add(I);

	                    // Scale momentum by inverse time to get 
	                    // a sort of "torque"
	                    L.scale(dRecip(h));//dScaleVector3(L,dRecip(h)); 
	                    // Invert the pseudo-tensor
	                    DMatrix3 itInv = new DMatrix3();
	                    // This is a closed-form inversion.
	                    // It's probably not numerically stable
	                    // when dealing with small masses with
	                    // a large asymmetry.
	                    // An LU decomposition might be better.
	                    if (dInvertMatrix3(itInv,Itild)!=0) {
	                        // "Divide" the original tensor
	                        // by the pseudo-tensor (on the right)
	                        dMultiply0_333(Itild,I,itInv);
	                        // Subtract an identity matrix
	                        //Itild[0]-=1; Itild[5]-=1; Itild[10]-=1;
	                        Itild.sub(0, 0, 1);
	                        Itild.sub(1, 1, 1);
	                        Itild.sub(2, 2, 1);

	                        // This new inertia matrix rotates the 
	                        // momentum to get a new set of torques
	                        // that will work correctly when applied
	                        // to the old inertia matrix as explicit
	                        // torques with a semi-implicit update
	                        // step.
	                        DVector3 tau0 = new DVector3();
	                        dMultiply0_331(tau0,Itild,L);
	                        
	                        // Add the gyro torques to the torque 
	                        // accumulator
	                        //for (int ii=0;ii<3;++ii) {
	                        //  b.tacc[ii]+=tau0[ii];
	                        //}
	                        b.tacc.add(tau0);
	                    }
//	#endif
	                }

	                bodyIndex = ThreadingUtils.ThrsafeIncrementIntUpToLimit(callContext.m_inertiaBodyIndex, nb);
	            }
	        }
	    }
	}

	private static dThreadedCallFunction dxQuickStepIsland_Stage0_Joints_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _callContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxQuickStepperStage0JointsCallContext callContext = (dxQuickStepperStage0JointsCallContext)_callContext;
			dxQuickStepIsland_Stage0_Joints(callContext);
			return true;
		}
	};

	private static 
	void dxQuickStepIsland_Stage0_Joints(dxQuickStepperStage0JointsCallContext callContext)
	{
	    DxJoint[] _jointP = callContext.m_stepperCallContext.m_islandJointsStartA();
	    int _jointOfs = callContext.m_stepperCallContext.m_islandJointsStartOfs();
	    int _nj = callContext.m_stepperCallContext.m_islandJointsCount();

		// get joint information (m = total constraint dimension, nub = number of unbounded variables).
	    // joints with m=0 are inactive and are removed from the joints array
	    // entirely, so that the code that follows does not consider them.
	    {
	    	int mcurr = 0, mfbcurr = 0;
	    	DJointWithInfo1[] jicurrA = callContext.m_jointinfos;
	    	int jicurrP=0; //jicurr = 0;
	    	DJointWithInfo1 jicurrO = new DJointWithInfo1();
	    	for (int i=0; i<_nj; i++) {	// i=dest, j=src
	    		DxJoint j = _jointP[i+_jointOfs];
	    		j.getInfo1(jicurrO.info);
	    		dIASSERT (/*jicurr->info.m >= 0 && */jicurrO.info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurrO.info.nub <= jicurrO.info.m);

	    		int jm = jicurrO.info.m;
	    		if (jm != 0) {
	    			mcurr += jm;
	    			if (j.feedback != null) {
	    				mfbcurr += jm;
	    			}
	    			jicurrO.joint = j;
	    			jicurrA[jicurrP] = jicurrO;   
	    			jicurrP++;
	    			jicurrO = new DJointWithInfo1();
	    		}
	    	}
	    	callContext.m_stage0Outputs.nj = jicurrP;// - callContext.m_jointinfos;
	    	callContext.m_stage0Outputs.m = mcurr;
	    	callContext.m_stage0Outputs.mfb = mfbcurr;
	    }
	}

	private static dThreadedCallFunction dxQuickStepIsland_Stage1_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage1CallContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxQuickStepperStage1CallContext stage1CallContext = (dxQuickStepperStage1CallContext)_stage1CallContext;
			dxQuickStepIsland_Stage1(stage1CallContext);
			return true;
		}
	};

	private static 
	void dxQuickStepIsland_Stage1(dxQuickStepperStage1CallContext stage1CallContext)
	{
		final DxStepperProcessingCallContext callContext = stage1CallContext.m_stepperCallContext;
		double[] invI = stage1CallContext.m_invI;
		DJointWithInfo1[] jointinfos = stage1CallContext.m_jointinfos;
		int nj = stage1CallContext.m_stage0Outputs.nj;
		int m = stage1CallContext.m_stage0Outputs.m;
		int mfb = stage1CallContext.m_stage0Outputs.mfb;

		DxWorldProcessMemArena memarena = callContext.m_stepperArena();
		memarena.RestoreState(stage1CallContext.m_stageMemArenaState);
		stage1CallContext = null; // WARNING! _stage1CallContext is not valid after this point!
		dIVERIFY(stage1CallContext == null); // To suppress unused variable assignment warnings

		{
			int _nj = callContext.m_islandJointsCount();
			memarena.ShrinkArrayDJointWithInfo1(jointinfos, _nj, nj);
		}

		DxWorld world = callContext.m_world();
		//dxBody * const *body = callContext.m_islandBodiesStart;
		int nb = callContext.m_islandBodiesCount();

		int[] mindex = null;
		double[] J = null, cfm = null, lo = null, hi = null, rhs = null, Jcopy = null;
		int[] jb = null, findex = null;

		// if there are constraints, compute the constraint force
		if (m > 0) {
			//mindex = memarena.AllocateArray<unsigned int>(2 * (size_t)(nj + 1));
			memarena.dummy();
			mindex = new int[2 * (nj + 1)];
			{
				int mcurrO = 0;//mindex;
				int moffs = 0, mfboffs = 0;
				mindex[mcurrO+0] = moffs;
				mindex[mcurrO+1] = mfboffs;
				mcurrO += 2;

				for (DJointWithInfo1 jicurr: jointinfos) {
					//const dJointWithInfo1 *const jiend = jointinfos + nj;
					//for (const dJointWithInfo1 *jicurr = jointinfos; jicurr != jiend; ++jicurr) {
					DxJoint joint = jicurr.joint;
					moffs += jicurr.info.m;
					if (joint.feedback != null) { mfboffs += jicurr.info.m; }
					mindex[mcurrO+0] = moffs;
					mindex[mcurrO+1] = mfboffs;
					mcurrO += 2;
				}
			}

			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			findex = new int[m];//memarena.AllocateArray<int>(m);
			J = new double[m*12];//memarena.AllocateArray<dReal>((size_t)m*12);
			cfm = new double[m];//memarena.AllocateArray<dReal>(m);
			lo = new double[m];//memarena.AllocateArray<dReal>(m);
			hi = new double[m];//memarena.AllocateArray<dReal>(m);
			jb = new int[m*2];//memarena.AllocateArray<int>((size_t)m*2);
			rhs = new double[m];//memarena.AllocateArray<dReal>(m);
			Jcopy = new double[mfb*12];//memarena.AllocateArray<dReal>((size_t)mfb*12);
		}

		memarena.dummy();
		dxQuickStepperLocalContext localContext = new dxQuickStepperLocalContext(); 
		//(dxQuickStepperLocalContext *)memarena.AllocateBlock(sizeof(dxQuickStepperLocalContext));
		localContext.Initialize(invI, jointinfos, nj, m, mfb, mindex, findex, J, cfm, lo, hi, jb, rhs, Jcopy);

		BlockPointer stage1MemarenaState = memarena.SaveState();
		memarena.dummy();
		dxQuickStepperStage3CallContext stage3CallContext = new dxQuickStepperStage3CallContext();
		//(dxQuickStepperStage3CallContext*)memarena.AllocateBlock(sizeof(dxQuickStepperStage3CallContext));
		stage3CallContext.Initialize(callContext, localContext, stage1MemarenaState);

		if (m > 0) {
			// create a constraint equation right hand side vector `rhs', a constraint
			// force mixing vector `cfm', and LCP low and high bound vectors, and an
			// 'findex' vector.
			//dReal *rhs_tmp = memarena.AllocateArray<dReal>((size_t)nb*6);
			memarena.dummy();
			double[] rhs_tmp = new double[nb*6];

			memarena.dummy();
			dxQuickStepperStage2CallContext stage2CallContext = new dxQuickStepperStage2CallContext(); 
			//(dxQuickStepperStage2CallContext*)memarena.AllocateBlock(sizeof(dxQuickStepperStage2CallContext));
			stage2CallContext.Initialize(callContext, localContext, rhs_tmp);

			final int allowedThreads = callContext.m_stepperAllowedThreads();
			dIASSERT(allowedThreads != 0);

			if (allowedThreads == 1)
			{
				dxQuickStepIsland_Stage2a(stage2CallContext);
				dxQuickStepIsland_Stage2b(stage2CallContext);
				dxQuickStepIsland_Stage2c(stage2CallContext);
				dxQuickStepIsland_Stage3(stage3CallContext);
			}
			else
			{
				final Ref<DCallReleasee> stage3CallReleasee = new Ref<>();
				world.threading().PostThreadedCallForUnawareReleasee(null, stage3CallReleasee, 1, callContext.m_finalReleasee(), 
						null, dxQuickStepIsland_Stage3_Callback, stage3CallContext, 0, "QuickStepIsland Stage3");

				final Ref<DCallReleasee> stage2bSyncReleasee = new Ref<>();
				world.threading().PostThreadedCall(
						null, stage2bSyncReleasee, 1, stage3CallReleasee.get(), 
						null, dxQuickStepIsland_Stage2bSync_Callback, stage2CallContext, 0, "QuickStepIsland Stage2b Sync");

				final Ref<DCallReleasee> stage2aSyncReleasee = new Ref<>();
				world.threading().PostThreadedCall(
						null, stage2aSyncReleasee, allowedThreads, stage2bSyncReleasee.get(), 
						null, dxQuickStepIsland_Stage2aSync_Callback, stage2CallContext, 0, "QuickStepIsland Stage2a Sync");

				world.threading().PostThreadedCallsGroup(null, allowedThreads, stage2aSyncReleasee.get(), 
						dxQuickStepIsland_Stage2a_Callback, stage2CallContext, "QuickStepIsland Stage2a");
			}
		}
		else {
			dxQuickStepIsland_Stage3(stage3CallContext);
		}
	}


	private static dThreadedCallFunction dxQuickStepIsland_Stage2a_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxQuickStepperStage2CallContext stage2CallContext = (dxQuickStepperStage2CallContext)_stage2CallContext;
			dxQuickStepIsland_Stage2a(stage2CallContext);
			return true;
		}
	};

	private static 
	void dxQuickStepIsland_Stage2a(dxQuickStepperStage2CallContext stage2CallContext)
	{
	    final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
	    final dxQuickStepperLocalContext localContext = stage2CallContext.m_localContext;
	    DJointWithInfo1[] jointinfos = localContext.m_jointinfos;
	    int nj = localContext.m_nj;
	    final int[] mindex = localContext.m_mindex;

	    DxWorld world = callContext.m_world();
	    final double stepsizeRecip = dRecip(callContext.m_stepSize());
	    {
	        int[] findex = localContext.m_findex;
	        double[] J = localContext.m_J;
	        double[] cfm = localContext.m_cfm;
	        double[] lo = localContext.m_lo;
	        double[] hi = localContext.m_hi;
	        double[] Jcopy = localContext.m_Jcopy;
	        double[] rhs = localContext.m_rhs;

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
	        final double worldERP = world.getERP();

	        DxJoint.Info2Descr Jinfo = new DxJoint.Info2Descr();
	        Jinfo.setRowskip(12);
	        Jinfo.setArrays(J, rhs, cfm, lo, hi, findex);
		            
	        int ji;
	        while ((ji = ThreadingUtils.ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_J, nj)) != nj) {
	            final int ofsi = mindex[ji * 2 + 0];
	            final int infom = mindex[ji * 2 + 2] - ofsi;

//		            int JcopyrowP = 0;//double[] Jcopyrow = Jcopy;
//		            int ofsi = 0;
//		            // const dJointWithInfo1 *jicurr = jointiinfos;
//		            for (int i = 0; i < njXXX; i++) {
//		                DJointWithInfo1 jicurr = jointiinfos[i];
	            int JrowP = 0 + ofsi * 12;		                
	            Jinfo.J1lp = JrowP;
	            Jinfo.J1ap = JrowP + 3;
	            Jinfo.J2lp = JrowP + 6;
	            Jinfo.J2ap = JrowP + 9;
	            Jinfo.setAllP(ofsi);
	            dSetZero (J, JrowP, infom*12);
	            //Jinfo.c = rhs + ofsi;
	            //dSetZero (Jinfo.c, infom);
	            dSetZero(rhs, ofsi, infom);
	            //Jinfo.cfm = cfm + ofsi;
	            dSetValue (cfm, ofsi, infom, world.global_cfm);
	            //Jinfo.lo = lo + ofsi;
	            dSetValue (lo, ofsi, infom, -dInfinity);
	            //Jinfo.hi = hi + ofsi;
	            dSetValue (hi, ofsi, infom, dInfinity);
	            //Jinfo.findex = findex + ofsi;
	            dSetValue(findex, ofsi, infom, -1);

	            DxJoint joint = jointinfos[ji].joint;
	            joint.getInfo2(stepsizeRecip, worldERP, Jinfo);

	            //double *rhs_row = Jinfo.c;
	            //double *cfm_row = Jinfo.cfm;
	            for (int i = 0; i != infom; ++i) {
//	            	rhs_row[i] *= stepsizeRecip;
//	            	cfm_row[i] *= stepsizeRecip;
	            	rhs[i+ofsi] *= stepsizeRecip;
	            	cfm[i+ofsi] *= stepsizeRecip;
	            }

	            // adjust returned findex values for global index numbering
	            //int *findex_row = Jinfo.findex;
	            for (int j = infom; j != 0; ) {
	            	--j;
	            	int fival = findex[j+ofsi];
	            	if (fival != -1) 
	            		findex[j+ofsi] = fival + ofsi;
	            }

	            // we need a copy of Jacobian for joint feedbacks
	            // because it gets destroyed by SOR solver
	            // instead of saving all Jacobian, we can save just rows
	            // for joints, that requested feedback (which is normally much less)
	            int mfbcurr = mindex[ji * 2 + 1], mfbnext = mindex[ji * 2 + 3];
	            if (mfbcurr != mfbnext) {
	            	//dReal *Jcopyrow = Jcopy + mfbcurr * 12;
	            	//memcpy(Jcopyrow, Jrow, (mfbnext - mfbcurr) * 12 * sizeof(dReal));
	            	System.arraycopy(Jcopy, mfbcurr * 12, J, JrowP, (mfbnext - mfbcurr) * 12);
	            }
	        }
	    }

	    {
	        int[] jb = localContext.m_jb;

	        // create an array of body numbers for each joint row
	        int ji;
	        while ((ji = ThreadingUtils.ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_jb, nj)) != nj) {
	        	DxJoint joint = jointinfos[ji].joint;
		                
	        	int b1 = (joint.node[0].body!=null) ? (joint.node[0].body.tag) : -1;
	        	int b2 = (joint.node[1].body!=null) ? (joint.node[1].body.tag) : -1;

	        	int jb_end = 2 * mindex[ji * 2 + 2];
	        	int jb_ptr = 2 * mindex[ji * 2 + 0];
	        	for (; jb_ptr != jb_end; jb_ptr += 2) {
	        		jb[jb_ptr] = b1;//jb_ptr[0] = b1;
	        		jb[jb_ptr+1] = b2;//jb_ptr[1] = b2;
	        	}
	        }
	    }
	}

	private static 
	dThreadedCallFunction dxQuickStepIsland_Stage2aSync_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, int callInstanceIndex,
				DCallReleasee callThisReleasee) {
			//callInstanceIndex; // unused
			dxQuickStepperStage2CallContext stage2CallContext = (dxQuickStepperStage2CallContext)_stage2CallContext;
			final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
			DxWorld world = callContext.m_world();
			final int allowedThreads = callContext.m_stepperAllowedThreads();

			world.threading().AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads);
			world.threading().PostThreadedCallsGroup(null, allowedThreads, callThisReleasee, 
					dxQuickStepIsland_Stage2b_Callback, stage2CallContext, "QuickStepIsland Stage2b");

			return true;
		}
	};

	private static 
	dThreadedCallFunction dxQuickStepIsland_Stage2b_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, int callInstanceIndex,
				DCallReleasee callThisReleasee) {
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxQuickStepperStage2CallContext stage2CallContext = (dxQuickStepperStage2CallContext)_stage2CallContext;
			dxQuickStepIsland_Stage2b(stage2CallContext);
			return true;
		}
	};

	private static 
	void dxQuickStepIsland_Stage2b(dxQuickStepperStage2CallContext stage2CallContext)
	{
	    final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
	    final dxQuickStepperLocalContext localContext = stage2CallContext.m_localContext;

	    final double stepsizeRecip = dRecip(callContext.m_stepSize());
	    {
	        // Warning!!!
	        // This code reads facc/tacc fields of body objects which (the fields)
	        // may be modified by dxJoint::getInfo2(). Therefore the code must be
	        // in different sub-stage from Jacobian construction in Stage2a 
	        // to ensure proper synchronization and avoid accessing numbers being modified.
	        // Warning!!!
	        DxBody[] bodyA = callContext.m_islandBodiesStartA();
	        int bodyOfs = callContext.m_islandBodiesStartOfs(); 
	        final int nb = callContext.m_islandBodiesCount();
	        double[] invI = localContext.m_invI;
	        double[] rhs_tmp = stage2CallContext.m_rhs_tmp;

	        // compute the right hand side `rhs'
	        if (TIMING) dTimerNow ("compute rhs_tmp");

	        // put -(v/h + invM*fe) into rhs_tmp
	        int bi;
	        while ((bi = ThreadingUtils.ThrsafeIncrementIntUpToLimit(stage2CallContext.m_bi, nb)) != nb) {
	            int tmp1currOfs = bi * 6; //+rhs_tmp
	            int invIrowOfs = bi * (6 * 2); //+invI
	            DxBody b = bodyA[bodyOfs+bi];
	            double body_invMass = b.invMass;
	            for (int j=0; j<3; ++j) 
	            	rhs_tmp[tmp1currOfs+j] = -(b.facc.get(j) * body_invMass + b.lvel.get(j) * stepsizeRecip);
	            dMultiply0_331 (rhs_tmp, tmp1currOfs + 3, invI, invIrowOfs, b.tacc);
	            for (int k=0; k<3; ++k) 
	            	rhs_tmp[tmp1currOfs+3+k] = -(b.avel.get(k) * stepsizeRecip) - rhs_tmp[tmp1currOfs+3+k];
	        }
	    }
	}

	private static 
	dThreadedCallFunction dxQuickStepIsland_Stage2bSync_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, int callInstanceIndex,
				DCallReleasee callThisReleasee) {
			//(void)callInstanceIndex; // unused
			dxQuickStepperStage2CallContext stage2CallContext = (dxQuickStepperStage2CallContext)_stage2CallContext;
			final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
			DxWorld world = callContext.m_world();
			final int allowedThreads = callContext.m_stepperAllowedThreads();

			world.threading().AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads);
			world.threading().PostThreadedCallsGroup(null, allowedThreads, callThisReleasee, 
					dxQuickStepIsland_Stage2c_Callback, stage2CallContext, "QuickStepIsland Stage2c");

			return true;
		}
	};

	private static 
	dThreadedCallFunction dxQuickStepIsland_Stage2c_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, int callInstanceIndex,
				DCallReleasee callThisReleasee) {
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxQuickStepperStage2CallContext stage2CallContext = (dxQuickStepperStage2CallContext)_stage2CallContext;
			dxQuickStepIsland_Stage2c(stage2CallContext);
			return true;
		}
	};

	private static 
	void dxQuickStepIsland_Stage2c(dxQuickStepperStage2CallContext stage2CallContext)
	{
	    //const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
		final dxQuickStepperLocalContext localContext = stage2CallContext.m_localContext;

	    //const dReal stepsizeRecip = dRecip(callContext->m_stepSize);
	    {
	        // Warning!!!
	        // This code depends on rhs_tmp and therefore must be in different sub-stage 
	        // from rhs_tmp calculation in Stage2b to ensure proper synchronization 
	        // and avoid accessing numbers being modified.
	        // Warning!!!
	        double[] rhs = localContext.m_rhs;
	        double[] J = localContext.m_J;
	        int[] jb = localContext.m_jb;
	        double[] rhs_tmp = stage2CallContext.m_rhs_tmp;
	        final int m = localContext.m_m;

	        // add J*rhs_tmp to rhs
	        multiplyAdd_J(stage2CallContext.m_Jrhsi, m, J, jb, rhs_tmp, rhs);
	    }
	}


	private static 
	dThreadedCallFunction dxQuickStepIsland_Stage3_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage3CallContext, int callInstanceIndex,
				DCallReleasee callThisReleasee) {
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxQuickStepperStage3CallContext stage3CallContext = (dxQuickStepperStage3CallContext)_stage3CallContext;
			dxQuickStepIsland_Stage3(stage3CallContext);
			return true;
		}
	};

	private static 
	void dxQuickStepIsland_Stage3(dxQuickStepperStage3CallContext stage3CallContext)
	{
	    final DxStepperProcessingCallContext callContext = stage3CallContext.m_stepperCallContext;
	    final dxQuickStepperLocalContext localContext = stage3CallContext.m_localContext;

	    DxWorldProcessMemArena memarena = callContext.m_stepperArena();
	    memarena.RestoreState(stage3CallContext.m_stage1MemArenaState);
	    stage3CallContext = null; // WARNING! stage3CallContext is not valid after this point!
	    dIVERIFY(stage3CallContext == null); // To suppress unused variable assignment warnings

	    double[] invI = localContext.m_invI;
	    DJointWithInfo1[] jointinfos = localContext.m_jointinfos;
	    int nj = localContext.m_nj;
	    int m = localContext.m_m;
	    int mfb = localContext.m_mfb;
	    //const unsigned int *mindex = localContext->m_mindex;
	    int[] findex = localContext.m_findex;
	    double[] J = localContext.m_J;
	    double[] cfm = localContext.m_cfm;
	    double[] lo = localContext.m_lo;
	    double[] hi = localContext.m_hi;
	    int[] jb = localContext.m_jb;
	    double[] rhs = localContext.m_rhs;
	    double[] Jcopy = localContext.m_Jcopy;

	    DxWorld world = callContext.m_world();
	    DxBody[] bodyA = callContext.m_islandBodiesStartA();
	    int bodyOfs = callContext.m_islandBodiesStartOfs();
	    int nb = callContext.m_islandBodiesCount();

	    if (m > 0) {
    			
			// load lambda from the value saved on the previous iteration
			double[] lambda = memarena.AllocateArrayDReal(m);//new double[m];//dRealAllocaArray (lambda,m);
			
			//TZ not defined
//			if (WARM_STARTING) {//#ifdef WARM_STARTING
//	           dReal *lambdscurr = lambda;
//	            const dJointWithInfo1 *jicurr = jointinfos;
//	            const dJointWithInfo1 *const jiend = jicurr + nj;
//	            for (; jicurr != jiend; jicurr++) {
//	                unsigned int infom = jicurr->info.m;
//	                memcpy (lambdscurr, jicurr->joint->lambda, infom * sizeof(dReal));
//	                lambdscurr += infom;
//	            }
//			}//#endif

			double[] cforce = memarena.AllocateArrayDReal(nb*6);
			BlockPointer lcpstate = memarena.BEGIN_STATE_SAVE(); 
			{
	            if (TIMING) dTimerNow ("solving LCP problem");
	            // solve the LCP problem and get lambda and invM*constraint_force
	            SOR_LCP (memarena,m,nb,J,jb,bodyA,bodyOfs,invI,lambda,cforce,rhs,lo,hi,cfm,findex,world.qs);
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
//            // save lambda for the next iteration
//            //@@@ note that this doesn't work for contact joints yet, as they are
//            // recreated every iteration
//            const dReal *lambdacurr = lambda;
//            const dJointWithInfo1 *jicurr = jointinfos;
//            const dJointWithInfo1 *const jiend = jicurr + nj;
//            for (; jicurr != jiend; jicurr++) {
//                unsigned int infom = jicurr->info.m;
//                memcpy (jicurr->joint->lambda, lambdacurr, infom * sizeof(dReal));
//                lambdacurr += infom;
//            }
//		      }
//				//#endif

			// note that the SOR method overwrites rhs and J at this point, so
			// they should not be used again.

			{
	            double stepsize = callContext.m_stepSize();
			    // add stepsize * cforce to the body velocity
			    int cforcecurrP = 0; //cforce
			    for (int i=0; i<nb; cforcecurrP+=6, i++) {
			        DxBody b = bodyA[i+bodyOfs];
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
				for (int i=0; i<nj; i++) {
				    DxJoint joint = jointinfos[i].joint;
				    int infom = jointinfos[i].info.m;
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
	        double stepsize = callContext.m_stepSize();
		    // compute the velocity update:
		    // add stepsize * invM * fe to the body velocity
		    int invIrowP = 0;//invI
		    for (int i=0; i<nb; invIrowP += 12, i++) {
		        DxBody b = bodyA[i+bodyOfs]; 
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
//		      _multiply_J (m,J,jb,vel,tmp);
//		      dReal error = 0;
//		      for (unsigned int i=0; i<m; i++) error += dFabs(tmp[i]);
//		      printf ("velocity error = %10.6e\n",error);
//		    
//		    } END_STATE_SAVE(memarena, velstate)
//		  }
		} //#endif

		{
	        double stepsize = callContext.m_stepSize();
    		// update the position and orientation from the new linear/angular velocity
    		// (over the given timestep)
		    if (TIMING) dTimerNow ("update position");
		    for (int i=0; i<nb; i++) {
		        bodyA[i+bodyOfs].dxStepBody (stepsize);
		    }
		}
		
		{
		    if (TIMING) dTimerNow ("tidy up");
    
    		// zero all force accumulators
    		for (int i=0; i<nb; i++) {
    		    DxBody b = bodyA[i+bodyOfs];
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
//        sub1_res2 += dEFFICIENT_SIZE(sizeof(dxQuickStepperLocalContext)); // for dxQuickStepLocalContext
//	        if (m > 0) {
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(double.class) * 12 * m); // for J
//        sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 12 * m); // for J
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(int.class) * 12 * m); // for jb
//	            sub1_res2 += 4 * dEFFICIENT_SIZE(sizeof(double.class) * m); // for cfm, lo, hi, rhs
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(int.class) * m); // for findex
//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(double.class) * 12 * mfb); // for Jcopy
//	            {
//        size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dxQuickStepperStage3CallContext)); // for dxQuickStepperStage3CallContext
//        sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for rhs_tmp
//        sub2_res1 += dEFFICIENT_SIZE(sizeof(dxQuickStepperStage2CallContext)); // for dxQuickStepperStage2CallContext
//
//        size_t sub2_res2 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda
//        sub2_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 6 * nb); // for cforce
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
//	            sub3_res2 += dMax(sub4_res1, sub4_res2);
//	          }
//	}//#endif
//	          sub2_res2 += dMax(sub3_res1, sub3_res2);
//	        }
//
//	        sub1_res2 += dMax(sub2_res1, sub2_res2);
//	      }
//	}
//        else {
//            sub1_res2 += dEFFICIENT_SIZE(sizeof(dxQuickStepperStage3CallContext)); // for dxQuickStepperStage3CallContext
//	    }
//	    
//        size_t sub1_res12_max = dMAX(sub1_res1, sub1_res2);
//        size_t stage01_contexts = dEFFICIENT_SIZE(sizeof(dxQuickStepperStage0BodiesCallContext))
//            + dEFFICIENT_SIZE(sizeof(dxQuickStepperStage0JointsCallContext))
//            + dEFFICIENT_SIZE(sizeof(dxQuickStepperStage1CallContext));
//        res += dMAX(sub1_res12_max, stage01_contexts);
//	  }
//
//	  return res;
	    return -1;
	}

	/*extern */
	private int dxEstimateQuickStepMaxCallCount(int activeThreadCount, int allowedThreadCount)
	{
		//(void)activeThreadCount; // unused
	    int result = 1 // dxQuickStepIsland itself
	        + (2 * allowedThreadCount + 2) // (dxQuickStepIsland_Stage2a + dxQuickStepIsland_Stage2b) * allowedThreadCount + 2 * dxStepIsland_Stage2?_Sync
	        + 1; // dxStepIsland_Stage3
	    return result;
	}


	@Override
	public int run(int activeThreadCount, int allowedThreadCount) {
		return dxEstimateQuickStepMaxCallCount(activeThreadCount, allowedThreadCount);
	}

	@Override
	public void run(DxStepperProcessingCallContext callContext) {
		dxQuickStepIsland(callContext);
	}
}