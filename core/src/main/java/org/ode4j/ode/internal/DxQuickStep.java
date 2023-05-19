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
import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.Matrix.dSetZero;
import static org.ode4j.ode.internal.Misc.dRandInt;
import static org.ode4j.ode.internal.CommonEnums.*;
import static org.ode4j.ode.internal.QuickStepEnums.*;
import static org.ode4j.ode.internal.Timer.*;
import static org.ode4j.ode.internal.Timer.dTimerReport;
import static org.ode4j.ode.internal.cpp4j.Cstring.memcpy;

import java.io.PrintStream;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.internal.Objects_H.dxQuickStepParameters;
import org.ode4j.ode.internal.cpp4j.FILE;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dmaxcallcountestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemArena;
import org.ode4j.ode.threading.Atomics;
import org.ode4j.ode.threading.task.Task;
import org.ode4j.ode.threading.task.TaskGroup;

/**
 *
 * Quickstep stepper.
 */
public class DxQuickStep extends AbstractStepper implements dstepper_fn_t,
dmemestimate_fn_t, dmaxcallcountestimate_fn_t {

	//TODO evaluate this
	/**
	 * Experimental improvement to reduce GC, see issue #36
	 */
	public static boolean REUSE_OBJECTS = false;
    /**
     * Due to too many small tasks created for LCP iterations,
     * the overhead involved in submitting a single task to the scheduler 
     * and context switching it turns out to be much more efficient to run the iterations
     * as a simple loop rather than related tasks executed concurrently.
     */
    public static boolean ENABLE_LCP_ITERATIONS_MULTITHREADING = false;
    
    public static int RANDOM_CONSTRAINTS_REORDERING_FREQUENCY = 8;
    public static int RRS_REORDERING = 0;
    // public static int RRS_REVERSAL = 1;
    // public static int RRS_MIN = 0;
    public static int RRS_MAX = 1;

	private static final boolean CHECK_VELOCITY_OBEYS_CONSTRAINT = false;

	/** DxQuickStep singleton instance. */
	public static final DxQuickStep INSTANCE = new DxQuickStep();

	private static final boolean TIMING = false;
	private static void IFTIMING_dTimerStart(String name) {
		if (TIMING) {
			dTimerStart(name);
		}
	}
	private static void IFTIMING_dTimerNow(String name) {
		if (TIMING) {
			dTimerNow(name);
		}
	}
	private static void IFTIMING_dTimerEnd() {
		if (TIMING) {
			dTimerEnd();
		}
	}
	private static void IFTIMING_dTimerReport (PrintStream fout, int average) {
		if (TIMING) {
			dTimerReport(new FILE(fout), average);
		}
	}


	//***************************************************************************
	// configuration

	// for the SOR and CG methods:
	// uncomment the following line to use warm starting. this definitely
	// help for motor-driven joints. unfortunately it appears to hurt
	// with high-friction contacts using the SOR method. use with care

	private static final boolean WARM_STARTING = false; // This is disabled in ODE

	private enum ReorderingMethod {
		REORDERING_METHOD__DONT_REORDER,
		REORDERING_METHOD__BY_ERROR,
		REORDERING_METHOD__RANDOMLY
	}

	// for the SOR method:
	// uncomment the following line to determine a new constraint-solving
	// order for each iteration. however, the qsort per iteration is expensive,
	// and the optimal order is somewhat problem dependent.
	// @@@ try the leaf->root ordering.

	// private static final ReorderingMethod CONSTRAINTS_REORDERING_METHOD = ReorderingMethod.REORDERING_METHOD__BY_ERROR;

	// for the SOR method:
	// uncomment the following line to randomly reorder constraint rows
	// during the solution. depending on the situation, this can help a lot
	// or hardly at all, but it doesn't seem to hurt.
	private static final ReorderingMethod CONSTRAINTS_REORDERING_METHOD = ReorderingMethod.REORDERING_METHOD__RANDOMLY;

	//		#if CONSTRAINTS_REORDERING_METHOD == REORDERING_METHOD__RANDOMLY
	//	#if !defined(RANDOM_CONSTRAINTS_REORDERING_FREQUENCY)
	//	#define RANDOM_CONSTRAINTS_REORDERING_FREQUENCY 8U
	//	#endif
	//		dSASSERT(RANDOM_CONSTRAINTS_REORDERING_FREQUENCY != 0);
	//	#endif
	// TZ: see further up.
	//	enum dxRandomReorderStage {
	//		//RRS__MIN,
	//		RRS_REORDERING,// = RRS__MIN,
	//		RRS__MAX,
	//	};

	//***************************************************************************
	// macros, typedefs, forwards and inlines

	private static final int dxQUICKSTEPISLAND_STAGE2B_STEP = 16;
	private static final int dxQUICKSTEPISLAND_STAGE2C_STEP = 32;
	private static final int dxQUICKSTEPISLAND_STAGE4A_STEP = WARM_STARTING ? 256 : 512;
	private static final int dxQUICKSTEPISLAND_STAGE4LCP_IMJ_STEP = 8;
	private static final int dxQUICKSTEPISLAND_STAGE4LCP_AD_STEP = 8;
	private static final int dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP = WARM_STARTING ? 128 : dxQUICKSTEPISLAND_STAGE4A_STEP / 2;
	// IF WARM_STARTING
	//private static final int dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP = 128;
	private static final int dxQUICKSTEPISLAND_STAGE4LCP_FC_COMPLETE_TO_PREPARE_COMPLEXITY_DIVISOR = 4;
	private static final int dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP_PREPARE = (dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP * dxQUICKSTEPISLAND_STAGE4LCP_FC_COMPLETE_TO_PREPARE_COMPLEXITY_DIVISOR);
	private static final int dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP_COMPLETE = (dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP);
	// ENDIF WARM_STARTING
	private static final int dxQUICKSTEPISLAND_STAGE4B_STEP = 256;
	private static final int dxQUICKSTEPISLAND_STAGE6A_STEP = 16;
	private static final int dxQUICKSTEPISLAND_STAGE6B_STEP = 1;

	private static int CalculateOptimalThreadsCount(int complexity, int max_threads, int step_size) {
		int raw_threads = Math.max(complexity, step_size) / step_size; // Round down on division
		int optimum = Math.min(raw_threads, max_threads);
		return optimum;
	}

	private static int dxENCODE_INDEX(int index) {
		return index + 1;
	}

	private static int dxDECODE_INDEX(int code) {
		return code - 1;
	}

	private static final int dxHEAD_INDEX = 0;

	//****************************************************************************
	// special matrix multipliers


	// multiply block of B matrix (q x 6) with 12 dReal per row with C vector (q)
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

	private static class DJointWithInfo1
	{
		DxJoint joint;
		final DxJoint.Info1 info = new DxJoint.Info1();
	}

	// TZ: We do not do this, this is very inefficient in Java
	//	private static class dxMIndexItem {
	//		int mIndex;
	//		int fbIndex;
	//	}
	private static void setMIndex(int[] mindex, int pos, int mIndex, int fbIndex) {
		mindex[2 * pos] = mIndex;
		mindex[2 * pos + 1] = fbIndex;
	}
	private static int getMIndex(int[] mindex, int pos) {
		return mindex[2 * pos];
	}
	private static int getFbIndex(int[] mindex, int pos) {
		return mindex[pos * 2 + 1];
	}

	//
	//	private static class dxJBodiesItem {
	//		int first;
	//		int second; // The index is optional and can equal to -1
	//	}
	private static int first(int[] jb, int m) {
		return jb[2 * m];
	}
	private static int second(int[] jb, int m) {
		return jb[2 * m + 1];
	}

	private static class dxQuickStepperStage0Outputs
	{
		int                    nj;
		int                    m;
		int                    mfb;
	}

	private static class dxQuickStepperStage1CallContext
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

	private static class dxQuickStepperStage0BodiesCallContext
	{
		void Initialize(final DxStepperProcessingCallContext stepperCallContext, 
				double[] invI)
		{
			m_stepperCallContext = stepperCallContext;
			m_invI = invI;
			m_tagsTaken.set(0);
			m_gravityTaken.set(0);
			m_inertiaBodyIndex.set(0);
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		double[]                           m_invI;
		final AtomicInteger                     m_tagsTaken = new AtomicInteger();
		final AtomicInteger                     m_gravityTaken = new AtomicInteger();
		final AtomicInteger                    m_inertiaBodyIndex = new AtomicInteger();
	}

	private static class dxQuickStepperStage0JointsCallContext
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

	private static class dxQuickStepperLocalContext
	{
		void Initialize(double[] invI, DJointWithInfo1[] jointinfos, int nj, 
				int m, int mfb, final int[] mindex, int[] jb, int[] findex,
				double[] J, double[] Jcopy)
		{
			m_invI = invI;
			m_jointinfos = jointinfos;
			m_nj = nj;
			m_m = m;
			m_mfb = mfb;
			m_valid_findices.set(0);
			m_mindex = mindex;
			m_jb = jb;
			m_findex = findex;
			m_J = J;
			m_Jcopy = Jcopy;
		}

		double[]                m_invI;
		DJointWithInfo1[]       m_jointinfos;
		int                    	m_nj;
		int                    	m_m;
		int                    	m_mfb;
		final AtomicInteger  	m_valid_findices = new AtomicInteger();
		int[]              		m_mindex;
		int[]			        m_jb;
		int[]                   m_findex;
		double[]                m_J;
		double[]                m_Jcopy;
	}

	private static class dxQuickStepperStage3CallContext
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
	}

	private static class dxQuickStepperStage2CallContext
	{
		void Initialize(DxStepperProcessingCallContext callContext, 
				dxQuickStepperLocalContext localContext, 
				double[] rhs_tmp)
		{
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			m_rhs_tmp = rhs_tmp;
			m_ji_J.set(0);
			m_ji_jb.set(0);
			m_bi.set(0);
			m_Jrhsi.set(0);
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		dxQuickStepperLocalContext   m_localContext;
		double[]                           m_rhs_tmp;
		final AtomicInteger           m_ji_J = new AtomicInteger();
		final AtomicInteger           m_ji_jb = new AtomicInteger();
		final AtomicInteger           m_bi = new AtomicInteger();
		final AtomicInteger           m_Jrhsi = new AtomicInteger();
	}

	private static class dxQuickStepperStage5CallContext {
		void Initialize(DxStepperProcessingCallContext callContext, dxQuickStepperLocalContext localContext,
						BlockPointer stage3MemArenaState) {
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			m_stage3MemArenaState = stage3MemArenaState;
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		dxQuickStepperLocalContext m_localContext;
		BlockPointer m_stage3MemArenaState;
	}

    private static class dxQuickStepperStage4CallContext {
        void Initialize(DxStepperProcessingCallContext callContext, dxQuickStepperLocalContext localContext,
                double[] lambda, double[] cforce, double[] iMJ, IndexError[] order, double[] last_lambda,
                AtomicInteger[] bi_links_or_mi_levels, AtomicInteger[] mi_links) {
            m_stepperCallContext = callContext;
            m_localContext = localContext;
            m_lambda = lambda;
            m_cforce = cforce;
            m_iMJ = iMJ;
            m_order = order;
            m_last_lambda = last_lambda;
            m_bi_links_or_mi_levels = bi_links_or_mi_levels;
            m_mi_links = mi_links;
            m_LCP_IterationSyncReleasee = null;
            m_LCP_IterationAllowedThreads = 0;
            m_LCP_fcStartReleasee = null;
            m_ji_4a.set(0);
            m_mi_iMJ.set(0);
            m_mi_fc.set(0);
            m_mi_Ad.set(0);
            m_LCP_iteration = 0;
            m_cf_4b.set(0);
            m_ji_4b.set(0);
        }

        void AssignLCP_IterationData(TaskGroup releaseeInstance, int iterationAllowedThreads) {
            m_LCP_IterationSyncReleasee = releaseeInstance;
            m_LCP_IterationAllowedThreads = iterationAllowedThreads;
        }

        void AssignLCP_fcStartReleasee(TaskGroup releaseeInstance) {
            m_LCP_fcStartReleasee = releaseeInstance;
        }

        void AssignLCP_fcAllowedThreads(int prepareThreads, int completeThreads) {
            m_LCP_fcPrepareThreadsRemaining.set(prepareThreads);
            m_LCP_fcCompleteThreadsTotal = completeThreads;
        }

        void ResetLCP_fcComputationIndex() {
            m_mi_fc.set(0);
        }

        void ResetSOR_ConstraintsReorderVariables(int reorderThreads) {
            m_SOR_reorderHeadTaken.set(0);
            m_SOR_reorderTailTaken.set(0);
            m_SOR_bi_zeroHeadTaken.set(0);
            m_SOR_bi_zeroTailTaken.set(0);
            m_SOR_mi_zeroHeadTaken.set(0);
            m_SOR_mi_zeroTailTaken.set(0);
            m_SOR_reorderThreadsRemaining.set(reorderThreads);
        }

        void RecordLCP_IterationStart(int totalThreads, TaskGroup nextReleasee) {
            m_LCP_iterationThreadsTotal = totalThreads;
            m_LCP_iterationThreadsRemaining.set(totalThreads);
            m_LCP_iterationNextReleasee = nextReleasee;
        }

        DxStepperProcessingCallContext m_stepperCallContext;
        dxQuickStepperLocalContext m_localContext;
        double[] m_lambda;
        double[] m_cforce;
        double[] m_iMJ;
        IndexError[] m_order;
        double[] m_last_lambda;
        AtomicInteger[] m_bi_links_or_mi_levels;
        AtomicInteger[] m_mi_links;
        TaskGroup m_LCP_IterationSyncReleasee;
        int m_LCP_IterationAllowedThreads;
        TaskGroup m_LCP_fcStartReleasee;
        final AtomicInteger m_ji_4a = new AtomicInteger();
        final AtomicInteger m_mi_iMJ = new AtomicInteger();
        final AtomicInteger m_mi_fc = new AtomicInteger();
        final AtomicInteger m_LCP_fcPrepareThreadsRemaining = new AtomicInteger();
        int m_LCP_fcCompleteThreadsTotal;
        final AtomicInteger m_mi_Ad = new AtomicInteger();
        int m_LCP_iteration;
        int m_LCP_iterationThreadsTotal;
        final AtomicInteger m_LCP_iterationThreadsRemaining = new AtomicInteger();
        TaskGroup m_LCP_iterationNextReleasee;
        final AtomicInteger m_SOR_reorderHeadTaken = new AtomicInteger();
        final AtomicInteger m_SOR_reorderTailTaken = new AtomicInteger();
        final AtomicInteger m_SOR_bi_zeroHeadTaken = new AtomicInteger();
        final AtomicInteger m_SOR_bi_zeroTailTaken = new AtomicInteger();
        final AtomicInteger m_SOR_mi_zeroHeadTaken = new AtomicInteger();
        final AtomicInteger m_SOR_mi_zeroTailTaken = new AtomicInteger();
        final AtomicInteger m_SOR_reorderThreadsRemaining = new AtomicInteger();
        final AtomicInteger m_cf_4b = new AtomicInteger();
        final AtomicInteger m_ji_4b = new AtomicInteger();
    }

    private static class dxQuickStepperStage6CallContext {
    	void Initialize(DxStepperProcessingCallContext callContext, dxQuickStepperLocalContext localContext) {
            m_stepperCallContext = callContext;
            m_localContext = localContext;
            m_bi_6a.set(0);
            m_bi_6b.set(0);
        }

        DxStepperProcessingCallContext m_stepperCallContext;
        dxQuickStepperLocalContext m_localContext;
        final AtomicInteger            m_bi_6a = new AtomicInteger();
        final AtomicInteger            m_bi_6b = new AtomicInteger();
    }

    //***************************************************************************
	// various common computations involving the matrix J

	/** 
	 * compute iMJ = inv(M)*J'
	 * (TZ) Performs a 331 multiplication on the 3-5 and 9-11 parts of
	 * each row and writes the result into iMJ.  
	 */
	private static void compute_invM_JT (AtomicInteger mi_storage, final int m, final double[] J, final double[] iMJ, 
	        final int[] jb,
			final DxBody[]bodyP, final int bodyOfs, final double[] invI, int step_size)
	{
		int m_steps = (m + (step_size - 1)) / step_size;
	    int mi_step;
	    while ((mi_step = Atomics.ThrsafeIncrementIntUpToLimit(mi_storage, m_steps)) != m_steps) {
	        int mi = mi_step * step_size;
	        int miend = mi + Math.min(step_size, m - mi);
	        int iMJ_ptr = mi * IMJ__MAX;
	        int J_ptr = mi * JME__MAX;
	        while (true) {
		        int b1 = first(jb, mi);
				int b2 = second(jb, mi);
				double k1 = bodyP[b1+bodyOfs].invMass;
	            for (int j = 0; j != JVE__L_COUNT; j++) iMJ[iMJ_ptr + IMJ__1L_MIN + j] = k1 * J[J_ptr + JME__J1L_MIN + j];
	            int invIrow1 = b1 * IIE__MAX + IIE__MATRIX_MIN;
	            dMultiply0_331 (iMJ, iMJ_ptr + IMJ__1A_MIN, invI, invIrow1, J, J_ptr + JME__J1A_MIN);

				if (b2 != -1) {
					double k2 = bodyP[b2+bodyOfs].invMass;
		            for (int j = 0; j != JVE__L_COUNT; ++j) iMJ[iMJ_ptr + IMJ__2L_MIN + j] = k2 * J[J_ptr + JME__J2L_MIN + j];
		            int invIrow2 = b2 * IIE__MAX + IIE__MATRIX_MIN;
		            dMultiply0_331 (iMJ, iMJ_ptr + IMJ__2A_MIN, invI, invIrow2, J, J_ptr + JME__J2A_MIN);
				}

	            if (++mi == miend) {
	                break;
	            }			
	            iMJ_ptr += IMJ__MAX;
	            J_ptr += JME__MAX;
	        }
		}
	}

	/** 
	 * compute out = J*in. 
	 * (TZ) Calculates the sum of each row 'm' in 'J', using only tagged bodies, and 
	 * writes the sum into 'out'.
	 * @param m total number of unbound variables, max. 6 per joint
	 * @param J double[m*12]
	 * @param jb int[m*2] of body tags. Paired per joint.
	 * @param in double[nb*6]
	 */
	private static void multiplyAdd_J (AtomicInteger mi_storage, 
			int m, int in_offset, int in_stride, final double[] J, final int[] jb, final double[] in, int step_size)
	{
		int m_steps = (m + (step_size - 1)) / step_size;
	    int mi_step;
	    while ((mi_step = Atomics.ThrsafeIncrementIntUpToLimit(mi_storage, m_steps)) != m_steps) {
	        int mi = mi_step * step_size;
	        int miend = mi + Math.min(step_size, m - mi);
	        int J_ptr = mi * JME__MAX;
	        while (true) {
				int b1 = first(jb, mi);
				int b2 = second(jb, mi);
				double sum = 0.0;
				int in_ofs = b1 * in_stride + in_offset;
				for (int j = 0; j != JME__J1_COUNT; ++j) sum += J[J_ptr + j + JME__J1_MIN] * in[in_ofs+j];
				if (b2 != -1) {
					in_ofs = b2 * in_stride + in_offset;
					for (int j = 0; j != JME__J2_COUNT; ++j) sum += J[J_ptr + j + JME__J2_MIN] * in[in_ofs+j];
				}
				J[J_ptr + JME_RHS] += sum;
	            if (++mi == miend) {
	                break;
	            }
	            J_ptr += JME__MAX;
	        }
		}
	}

	private static class IndexError {
//		double error;		// error to sort on
//		int findex;
		int index;		// row index
	}


	private static boolean IsSORConstraintsReorderRequiredForIteration(int iteration) {
		boolean result = false;
		if (CONSTRAINTS_REORDERING_METHOD == ReorderingMethod.REORDERING_METHOD__BY_ERROR) {
			result = true;
		} else if (CONSTRAINTS_REORDERING_METHOD == ReorderingMethod.REORDERING_METHOD__RANDOMLY) {
			// This logic is intended to skip randomization on the very first iteration
			if (iteration >= RANDOM_CONSTRAINTS_REORDERING_FREQUENCY
					? (iteration % RANDOM_CONSTRAINTS_REORDERING_FREQUENCY < RRS_MAX)
					: iteration == 0) {
				result = true;
			}
		} else {
			result = iteration == 0;
		}
		return result;
	}

	private final double[] buf_invI = new double[100];

	private double[] ensureSize_invI(int size) {
		double[] a = buf_invI;
		if (a.length < size || !REUSE_OBJECTS) {
			a = new double[size];
		} else {
			Arrays.fill(a, 0);
		}
		return a;
	}
	private final DJointWithInfo1[] buf_jointinfos = new DJointWithInfo1[0];

	private DJointWithInfo1[] ensureSize_jointinfos(int size) {
		DJointWithInfo1[] a = buf_jointinfos;
		if (a.length < size || !REUSE_OBJECTS) {
			//TODO we could partially copy the old array...
			a = new DJointWithInfo1[size];
			for (int i = 0; i < size; i++) {
				a[i] = new DJointWithInfo1();
			}
		} else {
			//Obviously this doesn't reset all objects, only the
			//ones that are likely to be needed.
			for (int i = 0; i < size; i++) {
				DJointWithInfo1 j = a[i];
				if (j != null) {
					j.joint = null;
					j.info.m = 0;
					j.info.nub = 0;
				}
			}
		}
		return a;
	}
	
	/*extern */
	private void dxQuickStepIsland(DxStepperProcessingCallContext callContext)
	{
	    DxWorldProcessMemArena memarena = callContext.m_stepperArena();
	    int nb = callContext.m_islandBodiesCount();
	    int _nj = callContext.m_islandJointsCount();

	    //double[] invI = memarena.AllocateOveralignedArray(nb * IIE__MAX, INVI_ALIGNMENT);//AllocateOveralignedArray (invI,nb * IIE__MAX, INVI_ALIGNMENT);
	    double[] invI = ensureSize_invI(nb * IIE__MAX);
	    
	    //dJointWithInfo1[] const jointinfos = memarena.AllocateArray<dJointWithInfo1>(_nj);
	    memarena.dummy();
	    //DJointWithInfo1[] jointinfos = new DJointWithInfo1[_nj];
	    DJointWithInfo1[] jointinfos = ensureSize_jointinfos(_nj);	    
	    
	    final int allowedThreads = callContext.m_stepperAllowedThreads();
	    dIASSERT(allowedThreads != 0);

	    BlockPointer stagesMemArenaState = memarena.SaveState();

	    memarena.dummy();
	    final dxQuickStepperStage1CallContext stage1CallContext = new dxQuickStepperStage1CallContext(); 
	    		//(dxQuickStepperStage1CallContext )memarena.AllocateBlock(sizeof(dxQuickStepperStage1CallContext));
	    stage1CallContext.Initialize(callContext, stagesMemArenaState, invI, jointinfos);

	    memarena.dummy();
	    final dxQuickStepperStage0BodiesCallContext stage0BodiesCallContext = new dxQuickStepperStage0BodiesCallContext(); 
	    		//(dxQuickStepperStage0BodiesCallContext)memarena.AllocateBlock(sizeof(dxQuickStepperStage0BodiesCallContext));
	    stage0BodiesCallContext.Initialize(callContext, invI);

	    memarena.dummy();
	    final dxQuickStepperStage0JointsCallContext stage0JointsCallContext = new dxQuickStepperStage0JointsCallContext(); 
	    		//(dxQuickStepperStage0JointsCallContext)memarena.AllocateBlock(sizeof(dxQuickStepperStage0JointsCallContext));
	    stage0JointsCallContext.Initialize(callContext, jointinfos, stage1CallContext.m_stage0Outputs);

	    if (allowedThreads == 1)
	    {
			IFTIMING_dTimerStart("preprocessing");
	        dxQuickStepIsland_Stage0_Bodies(stage0BodiesCallContext);
	        dxQuickStepIsland_Stage0_Joints(stage0JointsCallContext);
	        dxQuickStepIsland_Stage1(stage1CallContext);
	    }
	    else
	    {
	        TaskGroup stage1 = callContext.m_taskGroup().subgroup("QuickStepIsland Stage1", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage1(stage1CallContext);
                }
            });
            Task joints = stage1.subtask("QuickStepIsland Stage0-Joints", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage0_Joints(stage0JointsCallContext);
                }
            });
            joints.submit();
            for (int i = 1; i < allowedThreads; i ++) {
    	        Task bodies = stage1.subtask("QuickStepIsland Stage0-Bodies", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage0_Bodies(stage0BodiesCallContext);
                    }
                });
    	        bodies.submit();
            }
            dxQuickStepIsland_Stage0_Bodies(stage0BodiesCallContext);
            stage1.submit();
	    }
	}    

	private static
	void dxQuickStepIsland_Stage0_Bodies(dxQuickStepperStage0BodiesCallContext callContext)
	{
	    DxBody[] bodyP = callContext.m_stepperCallContext.m_islandBodiesStartA();
	    int bodyOfs = callContext.m_stepperCallContext.m_islandBodiesStartOfs();
	    int nb = callContext.m_stepperCallContext.m_islandBodiesCount();

	    if (Atomics.ThrsafeExchange(callContext.m_tagsTaken, 1) == 0)
	    {
	        // number all bodies in the body list - set their tag values
	        for (int i=0; i<nb; i++) bodyP[bodyOfs+i].tag = i;
	    }

	    if (Atomics.ThrsafeExchange(callContext.m_gravityTaken, 1) == 0)
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
        double[] invIA = callContext.m_invI;
        int bodyIndex ;
        while ((bodyIndex = Atomics.ThrsafeIncrementIntUpToLimit(callContext.m_inertiaBodyIndex, nb)) != nb) {
            int invIrowP = bodyIndex * IIE__MAX;
            DMatrix3 tmp = new DMatrix3();
            DxBody b = bodyP[bodyIndex + bodyOfs];

            // compute inverse inertia tensor in global frame
            dMultiply2_333 (tmp,b.invI,b.posr().R());
			dMultiply0_333(invIA, invIrowP + IIE__MATRIX_MIN, b.posr().R(), tmp);

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
	    }
	}

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
	    	//DJointWithInfo1 jicurrO = new DJointWithInfo1();
	    	for (int i=0; i<_nj; i++) {	// i=dest, j=src
	    		DJointWithInfo1 jicurrO = jicurrA[jicurrP];
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
	    			//jicurrA[jicurrP] = jicurrO;   
	    			jicurrP++;
	    			//jicurrO = new DJointWithInfo1();
	    		}
	    	}
	    	callContext.m_stage0Outputs.m = mcurr;
	    	callContext.m_stage0Outputs.mfb = mfbcurr;
	    	// No need to substract the m_jointinfo pointer here -> jicurrP is already a relative address
			callContext.m_stage0Outputs.nj = jicurrP;// - callContext.m_jointinfos;
	    }
	}

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

		int[] mindex = null;
		int[] jb = null, findex = null;
		double[] J = null, Jcopy = null;

		// if there are constraints, compute the constraint force
		if (m > 0) {
			mindex = memarena.AllocateArrayInt(2 * (nj + 1));
			memarena.dummy();
			//mindex = new int[2 * (nj + 1)];
			{
				int mcurrO = 0;//mindex;
				int moffs = 0, mfboffs = 0;
				setMIndex(mindex, mcurrO, moffs, mfboffs);
				++mcurrO;

				//for (DJointWithInfo1 jicurr: jointinfos) {
				for (int i = 0; i < nj; i++) {
					DJointWithInfo1 jicurr = jointinfos[i];
					//const dJointWithInfo1 *const jiend = jointinfos + nj;
					//for (const dJointWithInfo1 *jicurr = jointinfos; jicurr != jiend; ++jicurr) {
					DxJoint joint = jicurr.joint;
					moffs += jicurr.info.m;
					if (joint.feedback != null) { mfboffs += jicurr.info.m; }
					setMIndex(mindex, mcurrO, moffs, mfboffs);
					++mcurrO;
				}
			}

			jb = memarena.AllocateArrayInt(m*2);
			findex = memarena.AllocateArrayInt(m);
			J = memarena.AllocateOveralignedArrayDReal(m * JME__MAX, JACOBIAN_ALIGNMENT);
			Jcopy = memarena.AllocateOveralignedArrayDReal(m * JME__MAX, JACOBIAN_ALIGNMENT);
		}

		memarena.dummy();
		dxQuickStepperLocalContext localContext = new dxQuickStepperLocalContext(); 
		//(dxQuickStepperLocalContext *)memarena.AllocateBlock(sizeof(dxQuickStepperLocalContext));
		localContext.Initialize(invI, jointinfos, nj, m, mfb, mindex, jb, findex, J, Jcopy);

		BlockPointer stage1MemarenaState = memarena.SaveState();
		memarena.dummy();
		final dxQuickStepperStage3CallContext stage3CallContext = new dxQuickStepperStage3CallContext();
		//(dxQuickStepperStage3CallContext*)memarena.AllocateBlock(sizeof(dxQuickStepperStage3CallContext));
		stage3CallContext.Initialize(callContext, localContext, stage1MemarenaState);

		if (m > 0) {
			int nb = callContext.m_islandBodiesCount();
			// create a constraint equation right hand side vector `rhs', a constraint
			// force mixing vector `cfm', and LCP low and high bound vectors, and an
			// 'findex' vector.
			//dReal *rhs_tmp = memarena.AllocateArray<dReal>((size_t)nb*6);
			memarena.dummy();
			double[] rhs_tmp = new double[nb * RHS__MAX];

			memarena.dummy();
			final dxQuickStepperStage2CallContext stage2CallContext = new dxQuickStepperStage2CallContext(); 
			//(dxQuickStepperStage2CallContext*)memarena.AllocateBlock(sizeof(dxQuickStepperStage2CallContext));
			stage2CallContext.Initialize(callContext, localContext, rhs_tmp);

			final int allowedThreads = callContext.m_stepperAllowedThreads();
			dIASSERT(allowedThreads != 0);

			if (allowedThreads == 1)
			{
				IFTIMING_dTimerNow ("create J");
				dxQuickStepIsland_Stage2a(stage2CallContext);
				IFTIMING_dTimerNow ("compute rhs_tmp");
				dxQuickStepIsland_Stage2b(stage2CallContext);
				dxQuickStepIsland_Stage2c(stage2CallContext);
				dxQuickStepIsland_Stage3(stage3CallContext);
			}
			else
			{
	            final TaskGroup stage3 = callContext.m_taskGroup().subgroup("QuickStepIsland Stage3", new Runnable() {
	                @Override
	                public void run() {
	                    dxQuickStepIsland_Stage3(stage3CallContext);
	                }
	            });
	            final TaskGroup stage2bSync = stage3.subgroup("QuickStepIsland Stage2b Sync", new Runnable() {
	                @Override
	                public void run() {
	                    dxQuickStepIsland_Stage2bSync(stage2CallContext, stage3);
	                }
	            });
                TaskGroup stage2aSync = stage2bSync.subgroup("QuickStepIsland Stage2a Sync", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage2aSync(stage2CallContext, stage2bSync);
                    }
                });
                int stage2a_allowedThreads = CalculateOptimalThreadsCount(nj, allowedThreads, 1);
                for (int i = 1; i < stage2a_allowedThreads; i ++) {
                    Task bodies = stage2aSync.subtask("QuickStepIsland Stage2a", new Runnable() {
                        @Override
                        public void run() {
                            dxQuickStepIsland_Stage2a(stage2CallContext);
                        }
                    });
                    bodies.submit();
                }
                dxQuickStepIsland_Stage2a(stage2CallContext);
                stage2aSync.submit();
                stage2bSync.submit();
                stage3.submit();
			}
		}
		else {
			dxQuickStepIsland_Stage3(stage3CallContext);
		}
	}

	private static 
	void dxQuickStepIsland_Stage2aSync(final dxQuickStepperStage2CallContext stage2CallContext, TaskGroup group)
	{
	    DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
	    int nb = callContext.m_islandBodiesCount();

	    int allowedThreads = callContext.m_stepperAllowedThreads();
	    int stage2b_allowedThreads = CalculateOptimalThreadsCount(nb, allowedThreads, dxQUICKSTEPISLAND_STAGE2B_STEP);

        for (int i = 1; i < stage2b_allowedThreads; i ++) {
	        Task task = group.subtask("QuickStepIsland Stage2b", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage2b(stage2CallContext);
                }
            });
	        task.submit();	        
	    }
	    dxQuickStepIsland_Stage2b(stage2CallContext);
	}

	private static 
	void dxQuickStepIsland_Stage2bSync(final dxQuickStepperStage2CallContext stage2CallContext, TaskGroup group)
	{
        DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
        int allowedThreads = callContext.m_stepperAllowedThreads();

	    dxQuickStepperLocalContext localContext = stage2CallContext.m_localContext;
	    int m = localContext.m_m;

	    int stage2c_allowedThreads = CalculateOptimalThreadsCount(m, allowedThreads, dxQUICKSTEPISLAND_STAGE2C_STEP);

        for (int i = 1; i < stage2c_allowedThreads; i ++) {
            Task task = group.subtask("QuickStepIsland Stage2c", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage2c(stage2CallContext);
                }
            });
            task.submit();
        }
	    dxQuickStepIsland_Stage2c(stage2CallContext);
	}
	
	private static 
	void dxQuickStepIsland_Stage2a(dxQuickStepperStage2CallContext stage2CallContext)
	{
	    final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
	    final dxQuickStepperLocalContext localContext = stage2CallContext.m_localContext;
	    DJointWithInfo1[] jointinfos = localContext.m_jointinfos;
	    int nj = localContext.m_nj;
	    final int[] mindex = localContext.m_mindex;

	    final double stepsizeRecip = dRecip(callContext.m_stepSize());
	    {
	        int[] findex = localContext.m_findex;
	        double[] J = localContext.m_J;
	        double[] Jcopy = localContext.m_Jcopy;
            int jCopy_ofs = 0;

	        // get jacobian data from constraints. an m*16 matrix will be created
	        // to store the two jacobian blocks from each constraint. it has this
	        // format:
	        //
	        //   l1 l1 l1 a1 a1 a1 rhs cfm l2 l2 l2 a2 a2 a2 lo hi \    .
	        //   l1 l1 l1 a1 a1 a1 rhs cfm l2 l2 l2 a2 a2 a2 lo hi  }-- jacobian for joint 0, body 1 and body 2 (3 rows)
	        //   l1 l1 l1 a1 a1 a1 rhs cfm l2 l2 l2 a2 a2 a2 lo hi /
	        //   l1 l1 l1 a1 a1 a1 rhs cfm l2 l2 l2 a2 a2 a2 lo hi }--- jacobian for joint 1, body 1 and body 2 (3 rows)
	        //   etc...
	        //
	        //   (lll) = linear jacobian data
	        //   (aaa) = angular jacobian data
	        //
			DxWorld world = callContext.m_world();
	        final double worldERP = world.getERP();
	        final double worldCFM = world.getCFM();

	        int validFIndices = 0;

	        int ji;
	        while ((ji = Atomics.ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_J, nj)) != nj) {
				final int ofsi = getMIndex(mindex, ji);
				final int infom = getMIndex(mindex, ji + 1) - ofsi;
	            int jRow = ofsi * JME__MAX;
				{
					int jEnd = jRow + infom * JME__MAX;
					for (int jCurr = jRow; jCurr != jEnd; jCurr += JME__MAX) {
						dSetZero(J, jCurr + JME__J1_MIN, JME__J1_COUNT);
						J[jCurr + JME_RHS] = 0.0;
						J[jCurr + JME_CFM] = worldCFM;
						dSetZero(J, jCurr + JME__J2_MIN, JME__J2_COUNT);
						J[jCurr + JME_LO] = -dInfinity;
						J[jCurr + JME_HI] = dInfinity;
					}
					dSASSERT(JME__J1_COUNT + 2 + JME__J2_COUNT + 2 == JME__MAX);
				}
	            int findexRow = ofsi;
	            dSetValue(findex, findexRow, infom, -1);

	            DxJoint joint = jointinfos[ji].joint;
				joint.getInfo2(stepsizeRecip, worldERP, JME__MAX, J, jRow + JME__J1_MIN, J, jRow + JME__J2_MIN,
						JME__MAX, J, jRow + JME__RHS_CFM_MIN, J, jRow + JME__LO_HI_MIN, findex, findexRow);

	            // findex iteration is compact and is not going to pollute caches - do it first
				{
					// adjust returned findex values for global index numbering
					int findicesEndOfs = findexRow + infom;
					for (int findexCurrOfs = findexRow; findexCurrOfs != findicesEndOfs; ++findexCurrOfs) {
						int fival = findex[findexCurrOfs]; // *findexCurr;
						if (fival != -1) {
							findex[findexCurrOfs] = fival + ofsi; // *findexCurr = fival + ofsi;
							++validFIndices;
						}
					}
					// TZ: This looks alright, but is different from the original where we use findexRow
//					for (int j = infom; j != 0; ) {
//						--j;
//						int fival = findex[j + ofsi];
//						if (fival != -1) {
//							findex[j + ofsi] = fival + ofsi;
//							++validFIndices;
//						}
//					}
				}
				{
					// dReal *const JEnd = JRow + infom * JME__MAX;
					int jEnd = jRow + infom * JME__MAX;
					for (int jCurr = jRow; jCurr != jEnd; jCurr += JME__MAX) {
						J[jCurr + JME_RHS] *= stepsizeRecip;
						J[jCurr + JME_CFM] *= stepsizeRecip;
					}
				}
	            // we need a copy of Jacobian for joint feedbacks
	            // because it gets destroyed by SOR solver
	            // instead of saving all Jacobian, we can save just rows
	            // for joints, that requested feedback (which is normally much less)
				int mfbIndex = getFbIndex(mindex, ji); //mindex[ji].fbIndex;
				if (mfbIndex != getMIndex(mindex, ji + 1)) { //mindex[ji + 1].fbIndex) {
				// dReal *const JEnd = JRow + infom * JME__MAX;
				// dReal *JCopyRow = JCopy + mfbIndex * JCE__MAX; // Random access by mfbIndex here! Do not optimize!
					final int jEnd = jRow + infom * JME__MAX;
					int JCopyRow = jCopy_ofs + mfbIndex * JCE__MAX; // Random access by mfbIndex here! Do not optimize!
					//for (const dReal *JCurr = JRow; ; ) {
					for (int jCurr = jRow; jCurr < jEnd ;) {
						//for (unsigned i = 0; i != JME__J1_COUNT; ++i) { JCopyRow[i + JCE__J1_MIN] = JCurr[i + JME__J1_MIN]; }
						//for (unsigned j = 0; j != JME__J2_COUNT; ++j) { JCopyRow[j + JCE__J2_MIN] = JCurr[j + JME__J2_MIN]; }
						System.arraycopy(J, jCurr + JME__J1_MIN, Jcopy, JCopyRow + JCE__J1_MIN, JME__J1_COUNT);
						System.arraycopy(J, jCurr + JME__J2_MIN, Jcopy, JCopyRow + JCE__J2_MIN, JME__J2_COUNT);
						JCopyRow += JCE__MAX;

						// TZ move this outside of loop
						// dSASSERT((unsigned)JCE__J1_COUNT == JME__J1_COUNT);
						// dSASSERT((unsigned)JCE__J2_COUNT == JME__J2_COUNT);
						// dSASSERT(JCE__J1_COUNT + JCE__J2_COUNT == JCE__MAX);

						if ((jCurr += JME__MAX) == jEnd) {
							break;
						}
					}
                }
	        }
    	    Atomics.ThrsafeAdd(localContext.m_valid_findices, validFIndices);
	    }

		// TZ moved this here from inside loop above.
		dSASSERT(JCE__J1_COUNT == JME__J1_COUNT);
		dSASSERT(JCE__J2_COUNT == JME__J2_COUNT);
		dSASSERT(JCE__J1_COUNT + JCE__J2_COUNT == JCE__MAX);

		{
	        int[] jb = localContext.m_jb;

	        // create an array of body numbers for each joint row
	        int ji;
	        while ((ji = Atomics.ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_jb, nj)) != nj) {
	        	DxJoint joint = jointinfos[ji].joint;
		                
	        	int b1 = (joint.node[0].body!=null) ? (joint.node[0].body.tag) : -1;
	        	int b2 = (joint.node[1].body!=null) ? (joint.node[1].body.tag) : -1;

	        	int jb_end = 2 * getMIndex(mindex, ji + 1);
	        	int jb_ptr = 2 * getMIndex(mindex, ji);
	        	for (; jb_ptr != jb_end; jb_ptr += 2) {
	        		jb[jb_ptr] = b1;
	        		jb[jb_ptr+1] = b2;
	        	}
	        }
	    }
	}

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

	        int step_size = dxQUICKSTEPISLAND_STAGE2B_STEP;
	        int nb_steps = (nb + (step_size - 1)) / step_size;

	        // put -(v/h + invM*fe) into rhs_tmp
	        int bi_step;
	        while ((bi_step = Atomics.ThrsafeIncrementIntUpToLimit(stage2CallContext.m_bi, nb_steps)) != nb_steps) {
	            int bi = bi_step * step_size;
	            int biend = bi + Math.min(step_size, nb - bi);
	            int rhscurr = bi * RHS__MAX;
	            int invIrow = bi * IIE__MAX;
	            while (true) {
	            	DxBody b = bodyA[bodyOfs+bi];
		            double body_invMass = b.invMass;
		            for (int j=dSA__MIN; j<dSA__MAX; ++j) 
		            	rhs_tmp[rhscurr + RHS__L_MIN + j] = -(b.facc.get(dV3E__AXES_MIN + j) * body_invMass + b.lvel.get(dV3E__AXES_MIN + j) * stepsizeRecip);
		            dMultiply0_331 (rhs_tmp, rhscurr + RHS__A_MIN, invI, invIrow + IIE__MATRIX_MIN, b.tacc);
		            for (int k=dSA__MIN; k<dSA__MAX; ++k) 
		            	rhs_tmp[rhscurr + RHS__A_MIN + k] = -(b.avel.get(dV3E__AXES_MIN + k) * stepsizeRecip) - rhs_tmp[rhscurr + RHS__A_MIN + k];
	                if (++bi == biend) {
	                    break;
	                }
	                rhscurr += RHS__MAX;
	                invIrow += IIE__MAX;
	            }
	        }
	    }
	}

	private static 
	void dxQuickStepIsland_Stage2c(dxQuickStepperStage2CallContext stage2CallContext)
	{
		final dxQuickStepperLocalContext localContext = stage2CallContext.m_localContext;

	    {
	        // Warning!!!
	        // This code depends on rhs_tmp and therefore must be in different sub-stage 
	        // from rhs_tmp calculation in Stage2b to ensure proper synchronization 
	        // and avoid accessing numbers being modified.
	        // Warning!!!
	        double[] J = localContext.m_J;
	        int[] jb = localContext.m_jb;
	        double[] rhs_tmp = stage2CallContext.m_rhs_tmp;
	        final int m = localContext.m_m;

	        // add J*rhs_tmp to rhs
	        multiplyAdd_J(stage2CallContext.m_Jrhsi, m, RHS__DYNAMICS_MIN, RHS__MAX, J, jb, rhs_tmp, dxQUICKSTEPISLAND_STAGE2C_STEP);
			// THis was originally done inside multiplyAdd_J()
	        dSASSERT(RHS__DYNAMICS_MIN + JME__J1_COUNT <= RHS__MAX);
	    }
	}

    private static 
    void dxQuickStepIsland_Stage3(dxQuickStepperStage3CallContext stage3CallContext)
    {
        final DxStepperProcessingCallContext callContext = stage3CallContext.m_stepperCallContext;
        final dxQuickStepperLocalContext localContext = stage3CallContext.m_localContext;

        DxWorldProcessMemArena memarena = callContext.m_stepperArena();
        memarena.RestoreState(stage3CallContext.m_stage1MemArenaState);
        stage3CallContext = null; // WARNING! stage3CallContext is not valid after this point!
        dIVERIFY(stage3CallContext == null); // To suppress unused variable assignment warnings

		BlockPointer stage3MemarenaState = memarena.SaveState();
	    final dxQuickStepperStage5CallContext stage5CallContext = new dxQuickStepperStage5CallContext();
	    stage5CallContext.Initialize(callContext, localContext, stage3MemarenaState);
	    
        int m = localContext.m_m;

        if (m > 0) {
            // load lambda from the value saved on the previous iteration
	        double[] lambda = memarena.AllocateArrayDReal(m);
            int nb = callContext.m_islandBodiesCount();

            double[] cforce = memarena.AllocateArrayDReal(nb * CFE__MAX);
            double[] iMJ = memarena.AllocateArrayDReal(m * IMJ__MAX);//, INVMJ_ALIGNMENT);
	        // order to solve constraint rows in
	        IndexError[] order = new IndexError[m];
	        for (int i = 0; i < m; i++) {
	        	order[i] = new IndexError();
	        }
	        double[] last_lambda = null;
	        if (CONSTRAINTS_REORDERING_METHOD == ReorderingMethod.REORDERING_METHOD__BY_ERROR) {
		        // the lambda computed at the previous iteration.
		        // this is used to measure error for when we are reordering the indexes.
	        	last_lambda = memarena.AllocateArrayDReal(m);
	        }

	        int allowedThreads = callContext.m_stepperAllowedThreads();
	        boolean singleThreadedExecution = allowedThreads == 1;

	        AtomicInteger[] bi_links_or_mi_levels = null;
	        AtomicInteger[] mi_links = null;

	        if (!singleThreadedExecution && ENABLE_LCP_ITERATIONS_MULTITHREADING) {
	        	// TODO TZ consider pooling?
				bi_links_or_mi_levels = new AtomicInteger[Math.max(nb, m)];// memarena->AllocateArray<atomicord32>(dMAX(nb, m));
	        	mi_links = new AtomicInteger[2 * (m + 1)];// memarena->AllocateArray<atomicord32>(2 * (m + 1));
	        }

	        final dxQuickStepperStage4CallContext stage4CallContext = new dxQuickStepperStage4CallContext();
	        stage4CallContext.Initialize(callContext, localContext, lambda, cforce, iMJ, order, last_lambda, bi_links_or_mi_levels, mi_links);
        
	        if (singleThreadedExecution) {
	            dxQuickStepIsland_Stage4a(stage4CallContext);

				IFTIMING_dTimerNow ("solving LCP problem");
				dxQuickStepIsland_Stage4LCP_iMJComputation(stage4CallContext);
	            dxQuickStepIsland_Stage4LCP_STfcComputation(stage4CallContext);
	            dxQuickStepIsland_Stage4LCP_AdComputation(stage4CallContext);
	            dxQuickStepIsland_Stage4LCP_ReorderPrep(stage4CallContext);
	            
	            DxWorld world = callContext.m_world();
	            int num_iterations = world.qs.num_iterations;
				for (int iteration = 0; iteration < num_iterations; iteration++) {
					if (IsSORConstraintsReorderRequiredForIteration(iteration)) {
						stage4CallContext.ResetSOR_ConstraintsReorderVariables(0);
						dxQuickStepIsland_Stage4LCP_ConstraintsShuffling(stage4CallContext, iteration);
					}
					dxQuickStepIsland_Stage4LCP_STIteration(stage4CallContext);
	            }
	            dxQuickStepIsland_Stage4b(stage4CallContext);
	            dxQuickStepIsland_Stage5(stage5CallContext);
	        } else {
	            final TaskGroup stage5 = callContext.m_taskGroup().subgroup("QuickStepIsland Stage5", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage5(stage5CallContext);
                    }
                });
	            TaskGroup stage4LCP_IterationSync = stage5.subgroup("QuickStepIsland Stage4LCP_Iteration Sync", new Runnable() {
	                @Override
	                public void run() {
	                    dxQuickStepIsland_Stage4LCP_IterationSync(stage4CallContext, stage5);
	                }
	            });
	            int stage4LCP_Iteration_allowedThreads = CalculateOptimalThreadsCount(m, allowedThreads, 1);
	            stage4CallContext.AssignLCP_IterationData(stage4LCP_IterationSync, stage4LCP_Iteration_allowedThreads);

	            final TaskGroup stage4LCP_IterationStart = stage4LCP_IterationSync.subgroup("QuickStepIsland Stage4LCP_Iteration Start", new Runnable() {
                    @Override
                    public void run() {
                    	if (ENABLE_LCP_ITERATIONS_MULTITHREADING) {
                    		dxQuickStepIsland_Stage4LCP_IterationStart(stage4CallContext);
                    	} else {
                    		dxQuickStepIsland_Stage4LCP_IterationStartSingleThread(stage4CallContext);
                    	}
                    }
                });
	            TaskGroup stage4LCP_fcStart = stage4LCP_IterationStart.subgroup("QuickStepIsland Stage4LCP_fc Start", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage4LCP_fcStart(stage4CallContext, stage4LCP_IterationStart);                        
                    }
                });
	            TaskGroup stage4LCP_iMJSync = stage4LCP_IterationStart.subgroup("QuickStepIsland Stage4LCP_iMJ Sync", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage4LCP_iMJSync(stage4CallContext, stage4LCP_IterationStart);
                    }
                });
	            Task stage4LCP_ReorderPrep = stage4LCP_IterationStart.subtask("QuickStepIsland Stage4LCP_ReorderPrep", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage4LCP_ReorderPrep(stage4CallContext);
                    }
                });
	            stage4LCP_ReorderPrep.submit();
                int nj = localContext.m_nj;
                int stage4a_allowedThreads = CalculateOptimalThreadsCount(nj, allowedThreads, dxQUICKSTEPISLAND_STAGE4A_STEP);
                for (int i = 0; i < stage4a_allowedThreads; i++) {
                    Task stage4a = stage4LCP_fcStart.subtask("QuickStepIsland Stage4a", new Runnable() {
                        @Override
                        public void run() {
                            dxQuickStepIsland_Stage4a(stage4CallContext);
                        }
                    });
                    stage4a.submit();
                }
                stage4LCP_fcStart.submit();
                int stage4LCP_iMJ_allowedThreads = CalculateOptimalThreadsCount(m, allowedThreads, dxQUICKSTEPISLAND_STAGE4LCP_IMJ_STEP);
                for (int i = 1; i < stage4LCP_iMJ_allowedThreads; i++) {
                    Task stage4LCP_iMJComputation = stage4LCP_iMJSync.subtask("QuickStepIsland Stage4LCP_iMJ", new Runnable() {
                        @Override
                        public void run() {
                            dxQuickStepIsland_Stage4LCP_iMJComputation(stage4CallContext);
                        }
                    });
                    stage4LCP_iMJComputation.submit();
                }
                dxQuickStepIsland_Stage4LCP_iMJComputation(stage4CallContext);
                stage4LCP_iMJSync.submit();
                stage4LCP_IterationStart.submit();
                stage4LCP_IterationSync.submit();
                stage5.submit();
	        }
        } else {
        	dxQuickStepIsland_Stage5(stage5CallContext);
        }
    }

    private static 
    void dxQuickStepIsland_Stage4a(dxQuickStepperStage4CallContext stage4CallContext)
    {
        dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
        double[] lambda = stage4CallContext.m_lambda;
        int[] mindex = localContext.m_mindex;

        int nj = localContext.m_nj;
        int step_size = dxQUICKSTEPISLAND_STAGE4A_STEP;
        int nj_steps = (nj + (step_size - 1)) / step_size;
        
        int ji_step;
        while ((ji_step = Atomics.ThrsafeIncrementIntUpToLimit(stage4CallContext.m_ji_4a, nj_steps)) != nj_steps) {
            int ji = ji_step * step_size;
            int lambdacurr = getMIndex(mindex, ji);
            int lambdsnext = getMIndex(mindex, ji + Math.min(step_size, nj - ji));
            dSetZero(lambda, lambdacurr, lambdsnext - lambdacurr);
        }
    }

    private static 
    int dxQuickStepIsland_Stage4LCP_IterationSync(final dxQuickStepperStage4CallContext stage4CallContext, TaskGroup group)
    {
    	DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
        dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
        
        int stage4b_allowedThreads = 1;
        if (IsStage4bJointInfosIterationRequired(localContext)) {
            int allowedThreads = callContext.m_stepperAllowedThreads();
            stage4b_allowedThreads += CalculateOptimalThreadsCount(localContext.m_nj, allowedThreads - stage4b_allowedThreads, dxQUICKSTEPISLAND_STAGE4B_STEP);
        }

        for (int i = 1; i < stage4b_allowedThreads; i ++) {
            Task task = group.subtask("QuickStepIsland Stage4b", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage4b(stage4CallContext);
                }
            });
            task.submit();
        }
        dxQuickStepIsland_Stage4b(stage4CallContext);
        return 1;
    }
    
    private static 
	void dxQuickStepIsland_Stage4LCP_iMJComputation(dxQuickStepperStage4CallContext stage4CallContext)
	{
	    DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
	    dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
	
	    double[]iMJ = stage4CallContext.m_iMJ;
	    int m = localContext.m_m;
	    double[]J = localContext.m_J;
	    int[]jb = localContext.m_jb;
	    DxBody[] body = callContext.m_islandBodiesStartA();
	    int bodyOfs = callContext.m_islandBodiesStartOfs();
	    double[]invI = localContext.m_invI;
	
	    // precompute iMJ = inv(M)*J'
	    compute_invM_JT(stage4CallContext.m_mi_iMJ, m, J, iMJ, jb, body, bodyOfs, invI, dxQUICKSTEPISLAND_STAGE4LCP_IMJ_STEP);
	}

    private static
    void dxQuickStepIsland_Stage4LCP_iMJSync(final dxQuickStepperStage4CallContext stage4CallContext, TaskGroup group)
    {
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
        dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
        
        int m = localContext.m_m;
        int allowedThreads = callContext.m_stepperAllowedThreads();

        int stage4LCP_Ad_allowedThreads = CalculateOptimalThreadsCount(m, allowedThreads, dxQUICKSTEPISLAND_STAGE4LCP_AD_STEP);

        for (int i = 1; i < stage4LCP_Ad_allowedThreads; i ++) {
            Task task = group.subtask("QuickStepIsland Stage4LCP_Ad", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage4LCP_AdComputation(stage4CallContext);
                }
            });
            task.submit();
        }
        dxQuickStepIsland_Stage4LCP_AdComputation(stage4CallContext);
    }

    private static 
    void dxQuickStepIsland_Stage4LCP_fcStart(final dxQuickStepperStage4CallContext stage4CallContext, TaskGroup group)
    {
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
        dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;

        int fcPrepareComplexity = localContext.m_m;
        int fcCompleteComplexity = 0;

        int allowedThreads = callContext.m_stepperAllowedThreads();
        int stage4LCP_fcPrepare_allowedThreads = CalculateOptimalThreadsCount(fcPrepareComplexity, allowedThreads, dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP);
        int stage4LCP_fcComplete_allowedThreads = CalculateOptimalThreadsCount(fcCompleteComplexity, allowedThreads, dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP);
        stage4CallContext.AssignLCP_fcAllowedThreads(stage4LCP_fcPrepare_allowedThreads, stage4LCP_fcComplete_allowedThreads);

        for (int i = 1; i < stage4LCP_fcPrepare_allowedThreads; i ++) {
            Task task = group.subtask("QuickStepIsland Stage4LCP_fc", new Runnable() {
                @Override
                public void run() {
                	dxQuickStepIsland_Stage4LCP_MTfcComputation(stage4CallContext);
                }
            });
            task.submit();
        }
        dxQuickStepIsland_Stage4LCP_MTfcComputation(stage4CallContext);
    }

    private static 
    void dxQuickStepIsland_Stage4LCP_MTfcComputation(dxQuickStepperStage4CallContext stage4CallContext)
    {
        dxQuickStepIsland_Stage4LCP_MTfcComputation_cold(stage4CallContext);
    }

    private static 
    void dxQuickStepIsland_Stage4LCP_MTfcComputation_cold(dxQuickStepperStage4CallContext stage4CallContext)
    {
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
    
        double[] fc = stage4CallContext.m_cforce;
        int nb = callContext.m_islandBodiesCount();
        int step_size = dxQUICKSTEPISLAND_STAGE4LCP_FC_STEP;
        int nb_steps = (nb + (step_size - 1)) / step_size;
    
        int bi_step;
        while ((bi_step = Atomics.ThrsafeIncrementIntUpToLimit(stage4CallContext.m_mi_fc, nb_steps)) != nb_steps) {
            int bi = bi_step * step_size;
            int bicnt = Math.min(step_size, nb - bi);
            dSetZero(fc, bi * CFE__MAX, bicnt * CFE__MAX);
        }
    }

    // TODO: not needed if we stick to allocating m_cforce on each call
    private static
    void dxQuickStepIsland_Stage4LCP_STfcComputation(dxQuickStepperStage4CallContext stage4CallContext)
    {
    	double[] fc = stage4CallContext.m_cforce;
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
        int nb = callContext.m_islandBodiesCount();
        dSetZero(fc, nb * CFE__MAX);
    }
    
    private static
    void dxQuickStepIsland_Stage4LCP_AdComputation(dxQuickStepperStage4CallContext stage4CallContext)
    {
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
	    dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;

	    int[]jb = localContext.m_jb;
	    double[]J = localContext.m_J;
	    int m = localContext.m_m;

        DxWorld world = callContext.m_world();
        dxQuickStepParameters qs = world.qs;
        double sor_w = qs.w;		// SOR over-relaxation parameter

        double[]iMJ = stage4CallContext.m_iMJ;

        int step_size = dxQUICKSTEPISLAND_STAGE4LCP_AD_STEP;
        int m_steps = (m + (step_size - 1)) / step_size;

        int mi_step;
        while ((mi_step = Atomics.ThrsafeIncrementIntUpToLimit(stage4CallContext.m_mi_Ad, m_steps)) != m_steps) {
            int mi = mi_step * step_size;
            int miend = mi + Math.min(step_size, m - mi);

            int iMJ_ptr = mi * IMJ__MAX;
            int j_ptr = mi * JME__MAX;
            while (true) {
                double sum = 0;
                for (int j=JVE__MIN; j != JVE__MAX; j++) sum += iMJ[iMJ_ptr + j + IMJ__1_MIN] * J[j_ptr + j + JME__J1_MIN];
                int b2 = jb[mi*2+1];
                if (b2 != -1) {
                    for (int j=JVE__MIN; j != JVE__MAX; j++) sum += iMJ[iMJ_ptr + j + IMJ__2_MIN] * J[j_ptr + j + JME__J2_MIN];
                }
                double cfm_i = J[j_ptr + JME_CFM];
                double Ad_i = sor_w / (sum + cfm_i);

                // NOTE: This may seem unnecessary but it's indeed an optimization 
                // to move multiplication by Ad[i] and cfm[i] out of iteration loop.
                
                // scale J and b by Ad
                J[j_ptr + JME_CFM] = cfm_i * Ad_i;
                J[j_ptr + JME_RHS] *= Ad_i;

                for (int j = JVE__MIN; j != JVE__MAX; ++j) J[j_ptr + JME__J1_MIN + j] *= Ad_i;

                if (b2 != -1) {
                    for (int k = JVE__MIN; k != JVE__MAX; ++k) J[j_ptr + JME__J2_MIN + k] *= Ad_i;
                }
                if (++mi == miend) {
                    break;
                }
                iMJ_ptr += IMJ__MAX;
                j_ptr += JME__MAX;
            }
            
        }
    }
    
    private static
    void dxQuickStepIsland_Stage4LCP_ReorderPrep(dxQuickStepperStage4CallContext stage4CallContext)
    {
	    dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
	    int m = localContext.m_m;
		int valid_findices = localContext.m_valid_findices.get();

        IndexError []order = stage4CallContext.m_order;
        // make sure constraints with findex < 0 come first.
        int orderhead = 0;
        int ordertail = m - valid_findices;
        int[] findex = localContext.m_findex;

        // Fill the array from both ends
        for (int i = 0; i != m; ++i) {
            if (findex[i] == -1) {
                order[orderhead].index = i; // Place them at the front
                ++orderhead;
            } else {
				order[ordertail].index = i; // Place them at the end
				++ordertail;
            }
        }
    }

    private static
    void dxQuickStepIsland_Stage4LCP_IterationStartSingleThread(final dxQuickStepperStage4CallContext stage4CallContext)
    {
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
        DxWorld world = callContext.m_world();
        int num_iterations = world.qs.num_iterations;
		for (int iteration = 0; iteration < num_iterations; iteration++) {
			if (IsSORConstraintsReorderRequiredForIteration(iteration)) {
				stage4CallContext.ResetSOR_ConstraintsReorderVariables(0);
				dxQuickStepIsland_Stage4LCP_ConstraintsShuffling(stage4CallContext, iteration);
			}
			dxQuickStepIsland_Stage4LCP_STIteration(stage4CallContext);
        }

    }

    private static
    int dxQuickStepIsland_Stage4LCP_IterationStart(final dxQuickStepperStage4CallContext stage4CallContext)
    {
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;

        DxWorld world = callContext.m_world();
        dxQuickStepParameters qs = world.qs;

        int num_iterations = qs.num_iterations;
        int iteration = stage4CallContext.m_LCP_iteration;
        
        if (iteration < num_iterations)
        {
            int stage4LCP_Iteration_allowedThreads = stage4CallContext.m_LCP_IterationAllowedThreads;

            boolean reorderRequired = false;

            if (IsSORConstraintsReorderRequiredForIteration(iteration))
            {
                reorderRequired = true;
            }

			// TODO CHECK-TZ why don't we need this?
			// unsigned syncCallDependencies = reorderRequired ? 1 : stage4LCP_Iteration_allowedThreads;

			// Increment iterations counter in advance as anyway it needs to be incremented
            // before independent tasks (the reordering or the iteration) are posted
            // (otherwise next iteration may complete before the increment 
            // and the same iteration index may be used again).
            stage4CallContext.m_LCP_iteration = iteration + 1;

            TaskGroup iterationSync = stage4CallContext.m_LCP_IterationSyncReleasee;
            TaskGroup stage4LCP_IterationStart = null;
            if (iteration + 1 != num_iterations) {
            	stage4LCP_IterationStart = iterationSync.subgroup("QuickStepIsland Stage4LCP_Iteration Start", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage4LCP_IterationStart(stage4CallContext);
                    }
                });
            }
            final TaskGroup finalGroup = stage4LCP_IterationStart != null ? stage4LCP_IterationStart : iterationSync;
            
            if (reorderRequired) {
                int reorderThreads = 2;

                stage4CallContext.ResetSOR_ConstraintsReorderVariables(reorderThreads);

                TaskGroup stage4LCP_ConstraintsReorderingSync = finalGroup.subgroup("QuickStepIsland Stage4LCP_ConstraintsReordering Sync", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage4LCP_ConstraintsReorderingSync(stage4CallContext, finalGroup);                        
                    }
                });
                for (int i = 1; i < reorderThreads; i++) {
                    Task stage4LCP_ConstraintsReordering = stage4LCP_ConstraintsReorderingSync.subtask("QuickStepIsland Stage4LCP_ConstraintsReordering", new Runnable() {
                        @Override
                        public void run() {
                            dxQuickStepIsland_Stage4LCP_ConstraintsReordering(stage4CallContext);
                        }
                    });
                    stage4LCP_ConstraintsReordering.submit();
                }
                dxQuickStepIsland_Stage4LCP_ConstraintsReordering(stage4CallContext);
                stage4LCP_ConstraintsReorderingSync.submit();
            } else {
                dxQuickStepIsland_Stage4LCP_DependencyMapFromSavedLevelsReconstruction(stage4CallContext);
                stage4CallContext.RecordLCP_IterationStart(stage4LCP_Iteration_allowedThreads, finalGroup);
                final int knownToBeCompletedLevel = dxHEAD_INDEX;
                for (int i = 1; i < stage4LCP_Iteration_allowedThreads; i ++) {
                    Task task = finalGroup.subtask("QuickStepIsland Stage4LCP_Iteration", new Runnable() {
                        @Override
                        public void run() {
                        	dxQuickStepIsland_Stage4LCP_MTIteration(stage4CallContext, knownToBeCompletedLevel);
                        }
                    });
                    task.submit();
                }
                dxQuickStepIsland_Stage4LCP_MTIteration(stage4CallContext, knownToBeCompletedLevel);
            }
            if (stage4LCP_IterationStart != null) {
            	stage4LCP_IterationStart.submit();
            }
        } else {
        	// NOTE: So far, this branch is only called in CONSTRAINTS_REORDERING_METHOD == REORDERING_METHOD__BY_ERROR case
			if (Atomics.ThrsafeExchangeAdd(stage4CallContext.m_SOR_reorderThreadsRemaining, -1) == 1) { // If last thread has exited the
				dxQuickStepIsland_Stage4LCP_DependencyMapFromSavedLevelsReconstruction(stage4CallContext);
			}
        }

        return 1;
    }
    
    private static 
    void dxQuickStepIsland_Stage4LCP_ConstraintsReordering(dxQuickStepperStage4CallContext stage4CallContext)
    {
        int iteration = stage4CallContext.m_LCP_iteration - 1; // Iteration is pre-incremented before scheduled tasks are released for execution
        if (dxQuickStepIsland_Stage4LCP_ConstraintsShuffling(stage4CallContext, iteration)) {
            dxQuickStepIsland_Stage4LCP_LinksArraysZeroing(stage4CallContext);
            if (Atomics.ThrsafeExchangeAdd(stage4CallContext.m_SOR_reorderThreadsRemaining, -1) == 1) { // If last thread has exited the reordering routine...
                // Rebuild the object dependency map
                dxQuickStepIsland_Stage4LCP_DependencyMapForNewOrderRebuilding(stage4CallContext);
            }
        }
		else {
			// NOTE: So far, this branch is only called in CONSTRAINTS_REORDERING_METHOD == REORDERING_METHOD__BY_ERROR case
			if (Atomics.ThrsafeExchangeAdd(stage4CallContext.m_SOR_reorderThreadsRemaining, -1) == 1) { // If last thread has exited the reordering routine...
				dIASSERT(iteration != 0);
				dxQuickStepIsland_Stage4LCP_DependencyMapFromSavedLevelsReconstruction(stage4CallContext);
			}
		}
    }

    private static 
    boolean dxQuickStepIsland_Stage4LCP_ConstraintsShuffling(dxQuickStepperStage4CallContext stage4CallContext, int iteration)
    {
        boolean result = false;
		// #if CONSTRAINTS_REORDERING_METHOD == REORDERING_METHOD__BY_ERROR
        if (CONSTRAINTS_REORDERING_METHOD == ReorderingMethod.REORDERING_METHOD__BY_ERROR) {
            /*
                struct ConstraintsReorderingHelper
                {
                    void operator ()(dxQuickStepperStage4CallContext *stage4CallContext, int int startIndex, int int endIndex)
                    {
                        const dReal *lambda = stage4CallContext->m_lambda;
                        dReal *last_lambda = stage4CallContext->m_last_lambda;
                        IndexError *order = stage4CallContext->m_order;

                        for (int int index = startIndex; index != endIndex; ++index) {
                            int int i = order[index].index;
                            dReal lambda_i = lambda[i];
                            if (lambda_i != REAL(0.0)) {
                                //@@@ relative error: order[i].error = dFabs(lambda[i]-last_lambda[i])/max;
                                order[index].error = dFabs(lambda_i - last_lambda[i]);
                            }
                            else if (last_lambda[i] != REAL(0.0)) {
                                //@@@ relative error: order[i].error = dFabs(lambda[i]-last_lambda[i])/max;
                                order[index].error = dFabs(last_lambda[i]); // lambda_i == 0
                            }
                            else {
                                order[index].error = dInfinity;
                            }
                            // Finally copy the lambda for the next iteration
                            last_lambda[i] = lambda_i;
                        }
                        qsort (order + startIndex, endIndex - startIndex, sizeof(IndexError), &compare_index_error);
                    }
                };

                if (iteration > 1) { // Only reorder starting from iteration #2
                    // sort the constraints so that the ones converging slowest
                    // get solved last. use the absolute (not relative) error.
					//
					//  Full reorder needs to be done.
					//  Even though this contradicts the initial idea of moving dependent constraints
					//  to the order end the algorithm does not work the other way well.
					//  It looks like the iterative method needs a shake after it already found
					//  some initial approximations and those incurred errors help it to converge even better.
					//
                    if (ThrsafeExchange(&stage4CallContext->m_SOR_reorderHeadTaken, 1) == 0) {
                        // Process the head
                        const dxQuickStepperLocalContext *localContext = stage4CallContext->m_localContext;
                        ConstraintsReorderingHelper()(stage4CallContext, 0, localContext->m_m - localContext->m_valid_findices);
                    }
                    
                    if (ThrsafeExchange(&stage4CallContext->m_SOR_reorderTailTaken, 1) == 0) {
                        // Process the tail
                        const dxQuickStepperLocalContext *localContext = stage4CallContext->m_localContext;
                        ConstraintsReorderingHelper()(stage4CallContext, localContext->m_m - localContext->m_valid_findices, localContext->m_m);
                    }

                    result = true;
                }
                else if (iteration == 1) {
                    if (ThrsafeExchange(&stage4CallContext->m_SOR_reorderHeadTaken, 1) == 0) {
                        // Process the first half
                        const dxQuickStepperLocalContext *localContext = stage4CallContext->m_localContext;
                        int int startIndex = 0;
                        int int indicesCount = localContext->m_m / 2;
                        // Just copy the lambdas for the next iteration
                        memcpy(stage4CallContext->m_last_lambda + startIndex, stage4CallContext->m_lambda + startIndex, indicesCount * sizeof(dReal));
                    }

                    if (ThrsafeExchange(&stage4CallContext->m_SOR_reorderTailTaken, 1) == 0) {
                        // Process the second half
                        const dxQuickStepperLocalContext *localContext = stage4CallContext->m_localContext;
                        int int startIndex = localContext->m_m / 2;
                        int int indicesCount = localContext->m_m - startIndex;
                        // Just copy the lambdas for the next iteration
                        memcpy(stage4CallContext->m_last_lambda + startIndex, stage4CallContext->m_lambda + startIndex, indicesCount * sizeof(dReal));
                    }

                    // result = false; -- already 'false'
                } 
                else {
                    result = true; // return true on 0th iteration to build dependency map for the initial order 
                }
        */
			throw new UnsupportedOperationException();
        // #elif CONSTRAINTS_REORDERING_METHOD == REORDERING_METHOD__RANDOMLY
        } else if (CONSTRAINTS_REORDERING_METHOD == ReorderingMethod.REORDERING_METHOD__RANDOMLY) {
        	if (iteration != 0) {
				dIASSERT(!dIN_RANGE(iteration, 0, RANDOM_CONSTRAINTS_REORDERING_FREQUENCY));

				dIASSERT(iteration % RANDOM_CONSTRAINTS_REORDERING_FREQUENCY == RRS_REORDERING);
				{
					dIASSERT(!dIN_RANGE(iteration, 0, RANDOM_CONSTRAINTS_REORDERING_FREQUENCY));

					dIASSERT(iteration % RANDOM_CONSTRAINTS_REORDERING_FREQUENCY == RRS_REORDERING);
					//					{
					//					class ConstraintsReorderingHelper {
					//						void operator ()(dxQuickStepperStage4CallContext *stage4CallContext, unsigned int startIndex, unsigned int indicesCount)
					//						{
					//							IndexError *order = stage4CallContext->m_order + startIndex;
					//
					//							for (unsigned int index = 1; index < indicesCount; ++index) {
					//							int swapIndex = dRandInt(index + 1);
					//							IndexError tmp = order[index];
					//							order[index] = order[swapIndex];
					//							order[swapIndex] = tmp;
					//						}
					//						}
					//					};

					/*
					 *  Full reorder needs to be done.
					 *  Even though this contradicts the initial idea of moving dependent constraints
					 *  to the order end the algorithm does not work the other way well.
					 *  It looks like the iterative method needs a shake after it already found
					 *  some initial approximations and those incurred errors help it to converge even better.
					 */
					if (Atomics.ThrsafeExchange(stage4CallContext.m_SOR_reorderHeadTaken, 1) == 0) {
						// Process the head
						final dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
						ConstraintsReorderingHelper(stage4CallContext, 0, localContext.m_m);
					}
				}
				// dIASSERT((RRS__MAX, true)); // A reference to RRS__MAX to be located by Find in Files
        	} else {
        	// Just return true and skip the randomization for the very first iteration
        	}
			result = true;
		// #else // #if CONSTRAINTS_REORDERING_METHOD != REORDERING_METHOD__BY_ERROR && CONSTRAINTS_REORDERING_METHOD != REORDERING_METHOD__RANDOMLY
        } else {
			dIASSERT(iteration == 0);  // The reordering request is only returned for the first iteration
			result = true;
        }
        return result;
    }

	private static void ConstraintsReorderingHelper(dxQuickStepperStage4CallContext stage4CallContext, int startIndex,
			int indicesCount) {
		IndexError[] order = stage4CallContext.m_order;
		for (int index = 1; index < indicesCount; ++index) {
			int swapIndex = dRandInt(index + 1);
			IndexError tmp = order[startIndex + index];
			order[startIndex + index] = order[startIndex + swapIndex];
			order[startIndex + swapIndex] = tmp;
		}
	}

	// NOTE: Respective loop called a single thread only
	private static 
	void dxQuickStepIsland_Stage4LCP_LinksArraysZeroing(dxQuickStepperStage4CallContext stage4CallContext)
	{
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
        dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
	    if (Atomics.ThrsafeExchange(stage4CallContext.m_SOR_bi_zeroHeadTaken, 1) == 0) {
	        AtomicInteger[] bi_links = stage4CallContext.m_bi_links_or_mi_levels;/*=[nb]*/
	        int nb = callContext.m_islandBodiesCount();
	        int m = localContext.m_m;
	        int l = Math.max(m, nb) / 2;
            for (int i = 0; i < l; i++) {
	            bi_links[i] = new AtomicInteger();
	        }
	    }
	    if (Atomics.ThrsafeExchange(stage4CallContext.m_SOR_bi_zeroTailTaken, 1) == 0) {
            AtomicInteger[] bi_links = stage4CallContext.m_bi_links_or_mi_levels;/*=[nb]*/
            int nb = callContext.m_islandBodiesCount();
	        int m = localContext.m_m;
	        int max = Math.max(m, nb);
	        int l = max / 2;
            for (int i = l; i < max; i++) {
                bi_links[i] = new AtomicInteger();
            }
	    }
	    if (Atomics.ThrsafeExchange(stage4CallContext.m_SOR_mi_zeroHeadTaken, 1) == 0) {
	        AtomicInteger[] mi_links = stage4CallContext.m_mi_links;/*=[2*(m + 1)]*/
	        int m = localContext.m_m;
            for (int i = 0; i < m + 1; i++) {
                mi_links[i] = new AtomicInteger();
            }
	    }
	    if (Atomics.ThrsafeExchange(stage4CallContext.m_SOR_mi_zeroTailTaken, 1) == 0) {
            AtomicInteger[] mi_links = stage4CallContext.m_mi_links;/*=[2*(m + 1)]*/
	        int m = localContext.m_m;
            for (int i = 0; i < m + 1; i++) {
                mi_links[i + m + 1] = new AtomicInteger();
            }
	    }
	}

	// NOTE: Run in a single thread only
    private static 
    void dxQuickStepIsland_Stage4LCP_DependencyMapForNewOrderRebuilding(dxQuickStepperStage4CallContext stage4CallContext)
    {
        dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
        
        AtomicInteger[] bi_links = stage4CallContext.m_bi_links_or_mi_levels;
        AtomicInteger[] mi_links = stage4CallContext.m_mi_links;

        IndexError[] order = stage4CallContext.m_order;
        int[] jb = localContext.m_jb;

        int m = localContext.m_m;
        for (int i = 0; i != m; ++i) {
            int index = order[i].index;

            int b1 = jb[index * 2];
            int b2 = jb[index * 2 + 1];

            int encioded_i = dxENCODE_INDEX(i);

            int encoded_depi = bi_links[b1].get();
            bi_links[b1].set(encioded_i);

            if (b2 != -1 && b2 != b1) {
                if (encoded_depi < bi_links[b2].get()) {
                    encoded_depi = bi_links[b2].get();
                }
                bi_links[b2].set(encioded_i);
            }

            // OD: There is also a dependency on findex[index],
            // however the findex can only refer to the rows of the same joint 
            // and hence that index is going to have the same bodies. Since the 
            // indices are sorted in a way that the meaningful findex values 
            // always come last, the dependency of findex[index] is going to
            // be implicitly satisfied via matching bodies at smaller "i"s.

            // Check that the dependency targets an earlier "i"
            dIASSERT(encoded_depi < encioded_i);

            int encoded_downi = mi_links[encoded_depi * 2 + 1].get();
            mi_links[encoded_depi * 2 + 1].set(encioded_i); // Link i as down-dependency for depi
            mi_links[encioded_i * 2].set(encoded_downi); // Link previous down-chain as the level-dependency with i
        }
    }

    private final static int INVALID_LINK= dxENCODE_INDEX(-1);

    private static 
    void dxQuickStepIsland_Stage4LCP_DependencyMapFromSavedLevelsReconstruction(dxQuickStepperStage4CallContext stage4CallContext)
    {
        dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;

        AtomicInteger[] mi_levels = stage4CallContext.m_bi_links_or_mi_levels;
        AtomicInteger[] mi_links = stage4CallContext.m_mi_links;

		// NOTE!
		// OD: The mi_links array is not zero-filled before the reconstruction.
		// Iteration ends with all the down links zeroed. And since down links
		// are moved to the next level links when parent-child relations are established,
		// the horizontal levels are properly terminated.
		// The leaf nodes had their links zero-initialized initially
		// and those zeros remain intact during the solving. This way the down links
		// are properly terminated as well.
		// This is very obscure and error prone and would need an assertion check at least
		// but the simplest assertion approach I can imagine would be
		// zero filling and building another tree with the memory buffer comparison afterwards.
		// That would be stupid, obviously.
		//
		// NOTE!
		// OD: This routine can be threaded. However having two threads messing
		// in one integer array with random access and kicking each other memory lines
		// out of cache would probably work worse than letting a single thread do the whole job.
        int m = localContext.m_m;
        for (int i = 0; i != m; ++i) {
            int currentLevelRoot = mi_levels[i].get();
            int currentLevelFirstLink = mi_links[2 * currentLevelRoot + 1].get();
            int encoded_i = dxENCODE_INDEX(i);
            mi_links[2 * currentLevelRoot + 1].set(encoded_i);
            mi_links[2 * encoded_i + 0].set(currentLevelFirstLink);
        }

        // Additionally reset available level root's list head
        mi_links[2 * dxHEAD_INDEX + 0].set(dxHEAD_INDEX);
    }

    private static 
    void dxQuickStepIsland_Stage4LCP_ConstraintsReorderingSync(final dxQuickStepperStage4CallContext stage4CallContext, TaskGroup group)
    {
        int stage4LCP_Iteration_allowedThreads = stage4CallContext.m_LCP_IterationAllowedThreads;

        stage4CallContext.RecordLCP_IterationStart(stage4LCP_Iteration_allowedThreads, group);

        final int knownToBeCompletedLevel = dxHEAD_INDEX;
        for (int i = 1; i < stage4LCP_Iteration_allowedThreads; i ++) {
            Task task = group.subtask("QuickStepIsland Stage4LCP_Iteration", new Runnable() {
                @Override
                public void run() {
                	dxQuickStepIsland_Stage4LCP_MTIteration(stage4CallContext, knownToBeCompletedLevel);
                }
            });
            task.submit();
        }
        dxQuickStepIsland_Stage4LCP_MTIteration(stage4CallContext, knownToBeCompletedLevel);
    }

	public static AtomicInteger mtIterations = new AtomicInteger();

	/*
	 *	       +0                +0
	 * Root───┬─────────────────┬──...
	 *      +1│               +1│
	 *       ┌┴┐+0   ┌─┐+0      .
	 *       │A├─────┤B├─...
	 *       └┬┘     └┬┘
	 *      +1│     +1│
	 *       ┌┴┐+0    .
	 *       │C├─...
	 *       └┬┘
	 *      +1│
	 *        .
	 *
	 *  Lower tree levels depend on their parents. Same level nodes are independent with respect to each other.
	 *
	 *  1. B is linked in place of A
	 *  2. A is processed
	 *  3. C is inserted at the Root level
	 *
	 *  The tree starts with a single child subtree at the root level ("down" link of slot #0 is used for that).
	 *  Then, additional "C" nodes are added to the root level by building horizontal link via slots of
	 *  their former parent "A"s that had become free.
	 *  The "level" link of slot #0 is used to find the root level head.
	 *
	 *  Since the tree is altered during iteration, mi_levels record each node parents so that the tree could be reconstructed.
	 */
    private static
    void dxQuickStepIsland_Stage4LCP_MTIteration(final dxQuickStepperStage4CallContext stage4CallContext, final int initiallyKnownToBeCompletedLevel)
    {
    	mtIterations.incrementAndGet();
        AtomicInteger[] mi_levels = stage4CallContext.m_bi_links_or_mi_levels;
        AtomicInteger[] mi_links = stage4CallContext.m_mi_links;

        int knownToBeCompletedLevel = initiallyKnownToBeCompletedLevel;

        while (true) {
            int initialLevelRoot = mi_links[2 * dxHEAD_INDEX + 0].get();
            if (initialLevelRoot != dxHEAD_INDEX && initialLevelRoot == knownToBeCompletedLevel) {
                // No work is (currently) available
                break;
            }
            
            for (int currentLevelRoot = initialLevelRoot; ; currentLevelRoot = mi_links[2 * currentLevelRoot + 0].get()) {
                while (true) {

                    int currentLevelFirstLink = mi_links[2 * currentLevelRoot + 1].get();
                    if (currentLevelFirstLink == INVALID_LINK) {
                        break;
                    }
                    
                    // Try to extract first record from linked list
                    int currentLevelNextLink = mi_links[2 * currentLevelFirstLink + 0].get();
                    if (Atomics.ThrsafeCompareExchange(mi_links[2 * currentLevelRoot + 1], currentLevelFirstLink, currentLevelNextLink)) {
                        // if succeeded, execute selected iteration step...
                        dxQuickStepIsland_Stage4LCP_IterationStep(stage4CallContext, dxDECODE_INDEX(currentLevelFirstLink));

                        // Check if there are any dependencies
                        int level0DownLink = mi_links[2 * currentLevelFirstLink + 1].get();
                        if (level0DownLink != INVALID_LINK) {
                    		// ...and if yes, insert the record into the list of available level roots
                    		int levelRootsFirst;
                    		do {
                    		    levelRootsFirst = mi_links[2 * dxHEAD_INDEX + 0].get();
                    		    mi_links[2 * currentLevelFirstLink + 0].set(levelRootsFirst);
                    		}
                    		while (!Atomics.ThrsafeCompareExchange(mi_links[2 * dxHEAD_INDEX + 0], levelRootsFirst, currentLevelFirstLink));

                    		// If another level was added and some threads have already exited...
                    		int threadsTotal = stage4CallContext.m_LCP_iterationThreadsTotal;
                    		int threadsRemaining = Atomics.ThrsafeIncrementIntUpToLimit(stage4CallContext.m_LCP_iterationThreadsRemaining, threadsTotal);
                    		if (threadsRemaining != threadsTotal) {
                    			/* PP: This can easily result in creation of too many tasks 
                    		    // ...go on an schedule one more...
                    		    DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
                    		    // ...passing knownToBeCompletedLevel as the initial one for the spawned call
                    		    final int index = knownToBeCompletedLevel;
                    		    Task stage4LCP_Iteration = stage4CallContext.m_LCP_iterationNextReleasee.subtask("QuickStepIsland Stage4LCP_Iteration", new Runnable() {
                    				@Override
                    				public void run() {
                    					dxQuickStepIsland_Stage4LCP_MTIteration(stage4CallContext, index);
                    			}
                    			});
                    		    stage4LCP_Iteration.submit();
                    		    */
                    		    // NOTE: it's hard to predict whether it is reasonable to re-post a call
                    		    // each time a new level is added (provided some calls have already exited, of course).
                    		    // The efficiency very much depends on dependencies patterns between levels 
                    		    // (i.e. it depends on the amount of available work added with each level).
                    		    // The strategy of re-posting exited calls as frequently as possible
                    		    // leads to potential wasting execution cycles in some cores for the aid
                    		    // of keeping other cores busy as much as possible and not letting all the
                    		    // work be executed by just a partial cores subset. With emergency of large
                    		    // available work amounts (the work that is not dependent on anything and 
                    		    // ready to be executed immediately) this strategy is going to transit into 
                    		    // full cores set being busy executing useful work. If amounts of work 
                    		    // emerging from added levels are small, the strategy should lead to 
                    		    // approximately the same efficiency as if the work was done by only a cores subset 
                    		    // with the remaining cores wasting (some) cycles for re-scheduling calls 
                    		    // to those busy cores rather than being idle or handling other islands. 
                    		}
                        }

                        // Finally record the root index of current record's level
                        mi_levels[dxDECODE_INDEX(currentLevelFirstLink)].set(currentLevelRoot);
                    }
                }

                if (currentLevelRoot == knownToBeCompletedLevel) {
                    break;
                }
                dIASSERT(currentLevelRoot != dxHEAD_INDEX); // Zero level is expected to be the deepest one in the list and execution must not loop past it.
            }
            // Save the level root we started from as known to be completed
            knownToBeCompletedLevel = initialLevelRoot;
        }

        // Decrement running threads count on exit
        Atomics.ThrsafeAdd(stage4CallContext.m_LCP_iterationThreadsRemaining, -1);
    }

	private static
    void dxQuickStepIsland_Stage4LCP_STIteration(dxQuickStepperStage4CallContext stage4CallContext)
    {
	    dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
	    int m = localContext.m_m;
        for (int i = 0; i != m; ++i) {
            dxQuickStepIsland_Stage4LCP_IterationStep(stage4CallContext, i);
        }
    }

	//***************************************************************************
	// SOR-LCP method

		// nb is the number of bodies in the body array.
	// J is an m*16 matrix of constraint rows with rhs, cfm, lo and hi in padding
	// jb is an array of first and second body numbers for each constraint row
	// invI is the global frame inverse inertia for each body (stacked 3x3 matrices)
	//
	// this returns lambda and fc (the constraint force).
	// note: fc is returned as inv(M)*J'*lambda, the constraint force is actually J'*lambda
	//
	// b, lo and hi are modified on exit
    private static
    void dxQuickStepIsland_Stage4LCP_IterationStep(dxQuickStepperStage4CallContext stage4CallContext, int i)
    {
    	
    	dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;
        IndexError[ ]order = stage4CallContext.m_order;
        int index = order[i].index;

        double delta = 0;
        
        double[] lambda = stage4CallContext.m_lambda;
		double old_lambda = lambda[index];

		double[] J = localContext.m_J;
        int J_ptr = index * JME__MAX;

        delta = J[J_ptr + JME_RHS] - old_lambda * J[J_ptr + JME_CFM];

        double[] fc = stage4CallContext.m_cforce;

        int[] jb = localContext.m_jb;
        int b1 = jb[index * 2];
        int b2 = jb[index * 2 + 1];
        
	    // @@@ potential optimization: SIMD-ize this and the b2 >= 0 case
        int fc_ptr1 = b1 * CFE__MAX;
        delta -= fc[fc_ptr1 + CFE_LX] * J[J_ptr + JME_J1LX] + fc[fc_ptr1 + CFE_LY] * J[J_ptr + JME_J1LY] +
        		fc[fc_ptr1 + CFE_LZ] * J[J_ptr + JME_J1LZ] + fc[fc_ptr1 + CFE_AX] * J[J_ptr + JME_J1AX] +
        		fc[fc_ptr1 + CFE_AY] * J[J_ptr + JME_J1AY] + fc[fc_ptr1 + CFE_AZ] * J[J_ptr + JME_J1AZ];
        // @@@ potential optimization: handle 1-body constraints in a separate
	    // @@@ potential optimization: handle 1-body constraints in a separate
	    //     loop to avoid the cost of test & jump?
	    if (b2 != -1) {
	        int fc_ptr2 = b2 * CFE__MAX;
	        delta -= fc[fc_ptr2 + CFE_LX] * J[J_ptr + JME_J2LX] + fc[fc_ptr2 + CFE_LY] * J[J_ptr + JME_J2LY] +
	        		fc[fc_ptr2 + CFE_LZ] * J[J_ptr + JME_J2LZ] + fc[fc_ptr2 + CFE_AX] * J[J_ptr + JME_J2AX] +
	        		fc[fc_ptr2 + CFE_AY] * J[J_ptr + JME_J2AY] + fc[fc_ptr2 + CFE_AZ] * J[J_ptr + JME_J2AZ];
	    }

	    double hi_act, lo_act;
		
	    // set the limits for this constraint. note that 'hicopy' is used.
	    // this is the place where the QuickStep method differs from the
	    // direct LCP solving method, since that method only performs this
	    // limit adjustment once per time step, whereas this method performs
	    // once per iteration per constraint row.
	    // the constraints are ordered so that all lambda[] values needed have
	    // already been computed.
        int[] findex = localContext.m_findex;
	    if (findex[index] != -1) {
	        hi_act = dFabs (J[J_ptr + JME_HI] * lambda[findex[index]]);
	        lo_act = -hi_act;
	    } else {
	        hi_act = J[J_ptr + JME_HI];
	        lo_act = J[J_ptr + JME_LO];
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

		//@@@ a trick that may or may not help
		//dReal ramp = (1-((dReal)(iteration+1)/(dReal)num_iterations));
		//delta *= ramp;
	    double[] iMJ = stage4CallContext.m_iMJ;
	    final int iMJ_ptr = index * IMJ__MAX;
		// update fc.
		// @@@ potential optimization: SIMD for this and the b2 >= 0 case
	    fc[fc_ptr1 + CFE_LX] += delta * iMJ[iMJ_ptr + 0];
		fc[fc_ptr1 + CFE_LY] += delta * iMJ[iMJ_ptr + 1];
		fc[fc_ptr1 + CFE_LZ] += delta * iMJ[iMJ_ptr + 2];
		fc[fc_ptr1 + CFE_AX] += delta * iMJ[iMJ_ptr + 3];
		fc[fc_ptr1 + CFE_AY] += delta * iMJ[iMJ_ptr + 4];
		fc[fc_ptr1 + CFE_AZ] += delta * iMJ[iMJ_ptr + 5];
		// @@@ potential optimization: handle 1-body constraints in a separate
		//     loop to avoid the cost of test & jump?
	    if (b2 != -1) {
	        int fc_ptr2 = b2 * CFE__MAX;
			fc[fc_ptr2 + CFE_LX] += delta * iMJ[iMJ_ptr + 6];
			fc[fc_ptr2 + CFE_LY] += delta * iMJ[iMJ_ptr + 7];
			fc[fc_ptr2 + CFE_LZ] += delta * iMJ[iMJ_ptr + 8];
			fc[fc_ptr2 + CFE_AX] += delta * iMJ[iMJ_ptr + 9];
			fc[fc_ptr2 + CFE_AY] += delta * iMJ[iMJ_ptr + 10];
			fc[fc_ptr2 + CFE_AZ] += delta * iMJ[iMJ_ptr + 11];
		}
    }  

	private static boolean IsStage4bJointInfosIterationRequired(dxQuickStepperLocalContext localContext) {
		return localContext.m_mfb > 0;
	}
    
	private static
    void dxQuickStepIsland_Stage4b(dxQuickStepperStage4CallContext stage4CallContext)
    {
        DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
	    dxQuickStepperLocalContext localContext = stage4CallContext.m_localContext;

        if (Atomics.ThrsafeExchange(stage4CallContext.m_cf_4b, 1) == 0) {
            DxBody[] bodyA = callContext.m_islandBodiesStartA();
            int bodyOfs = callContext.m_islandBodiesStartOfs();
            int nb = callContext.m_islandBodiesCount();
            double[] cforce = stage4CallContext.m_cforce;
            double stepsize = callContext.m_stepSize();
            // add stepsize * cforce to the body velocity
            int cforcecurr = 0;
            int bodyend = nb;
            for (int bodycurr = 0; bodycurr != bodyend; cforcecurr += CFE__MAX, bodycurr++) {
                DxBody b = bodyA[bodyOfs + bodycurr];
                for (int j = dSA__MIN; j != dSA__MAX; j++) {
                    b.lvel.add(dV3E__AXES_MIN + j, stepsize * cforce[cforcecurr + CFE__L_MIN + j]);
                    b.avel.add(dV3E__AXES_MIN + j, stepsize * cforce[cforcecurr + CFE__A_MIN + j]);
                }
            }
        }

        // note that the SOR method overwrites rhs and J at this point, so
        // they should not be used again.

        if (IsStage4bJointInfosIterationRequired(localContext)) {
			DVector3 dataL = new DVector3(); //JVE__MAX;
			DVector3 dataA = new DVector3(); //JVE__MAX;
            double[] Jcopy = localContext.m_Jcopy;
            double[] lambda = stage4CallContext.m_lambda;
            int[] mindex = localContext.m_mindex;
            DJointWithInfo1[] jointinfos = localContext.m_jointinfos;

            int nj = localContext.m_nj;
            int step_size = dxQUICKSTEPISLAND_STAGE4B_STEP;
            int nj_steps = (nj + (step_size - 1)) / step_size;

            int ji_step;
            while ((ji_step = Atomics.ThrsafeIncrementIntUpToLimit(stage4CallContext.m_ji_4b, nj_steps)) != nj_steps) {
                int ji = ji_step * step_size;
				final int jiend = ji + Math.min(step_size, nj - ji);

				//const dReal *Jcopycurr = Jcopy + (sizeint)mindex[ji].fbIndex * JCE__MAX;
				int Jcopycurr = getFbIndex(mindex, ji) * JCE__MAX;

                while (true) {
                    // straightforward computation of joint constraint forces:
                    // multiply related lambdas with respective J' block for joints
                    // where feedback was requested
                	final int fb_infom = getFbIndex(mindex, ji + 1) - getFbIndex(mindex, ji);
					if (fb_infom != 0) {
						dIASSERT(fb_infom == getMIndex(mindex, ji + 1) - getMIndex(mindex, ji));

                    	//const dReal *lambdacurr = lambda + mindex[ji].mIndex;
						final int lambdacurrOfs = getMIndex(mindex, ji);
						DxJoint joint = jointinfos[ji].joint;

				// #ifdef WARM_STARTING
						if (WARM_STARTING) {
							memcpy(joint.lambda, 0, lambda, lambdacurrOfs, fb_infom);
						}
				// #endif

						DJoint.DJointFeedback fb = joint.feedback;

						dAssertVec3Element(); // ode4j specific assertion
						if (joint.node[1].body != null) {
							Multiply1_12q1 (dataL, dataA, Jcopy, Jcopycurr + JCE__J2_MIN, lambda, lambdacurrOfs, fb_infom);
							dSASSERT(JCE__MAX == 12);

							// fb.f2[dSA_X] = data[JVE_LX];
							// fb->f2[dSA_Y] = data[JVE_LY];
							// fb->f2[dSA_Z] = data[JVE_LZ];
							// fb->t2[dSA_X] = data[JVE_AX];
							// fb->t2[dSA_Y] = data[JVE_AY];
							// fb->t2[dSA_Z] = data[JVE_AZ];
							fb.f2.set(dataL);
							fb.t2.set(dataA);
						}

						Multiply1_12q1 (dataL, dataA, Jcopy, Jcopycurr + JCE__J1_MIN, lambda, lambdacurrOfs, fb_infom);
						dSASSERT(JCE__MAX == 12);

						// fb->f1[dSA_X] = data[JVE_LX];
						// fb->f1[dSA_Y] = data[JVE_LY];
						// fb->f1[dSA_Z] = data[JVE_LZ];
						// fb->t1[dSA_X] = data[JVE_AX];
						// fb->t1[dSA_Y] = data[JVE_AY];
						// fb->t1[dSA_Z] = data[JVE_AZ];
						fb.f2.set(dataL);
						fb.t2.set(dataA);

						Jcopycurr += fb_infom * JCE__MAX;
					}
					else {
				// #ifdef WARM_STARTING
						if (WARM_STARTING) {
                    		int lambdacurrOfs = getMIndex(mindex, ji);
                    		int infom = getMIndex(mindex,  ji + 1) - getMIndex(mindex, ji);
							DxJoint joint = jointinfos[ji].joint;
							memcpy(joint.lambda, 0, lambda, lambdacurrOfs, infom);
				// #endif
						}
					}

					if (++ji == jiend) {
						break;
					}
				}
            }
        }
    }
    
	private static
	void dxQuickStepIsland_Stage5(dxQuickStepperStage5CallContext stage5CallContext)
	{
		final DxStepperProcessingCallContext callContext = stage5CallContext.m_stepperCallContext;
	    dxQuickStepperLocalContext localContext = stage5CallContext.m_localContext;

	    DxWorldProcessMemArena memarena = callContext.m_stepperArena();
	    memarena.RestoreState(stage5CallContext.m_stage3MemArenaState);

	    final dxQuickStepperStage6CallContext stage6CallContext = new dxQuickStepperStage6CallContext();
	    stage6CallContext.Initialize(callContext, localContext);

	    int allowedThreads = callContext.m_stepperAllowedThreads();

	    if (allowedThreads == 1) {
			IFTIMING_dTimerNow ("compute velocity update");
	        dxQuickStepIsland_Stage6a(stage6CallContext);
	        dxQuickStepIsland_Stage6_VelocityCheck(stage6CallContext);
			IFTIMING_dTimerNow ("update position and tidy up");
			dxQuickStepIsland_Stage6b(stage6CallContext);
			IFTIMING_dTimerEnd();
			IFTIMING_dTimerReport(System.out, 1);
		} else {
	        int nb = callContext.m_islandBodiesCount();
	        int stage6a_allowedThreads = CalculateOptimalThreadsCount(nb, allowedThreads, dxQUICKSTEPISLAND_STAGE6A_STEP);
	        TaskGroup stage6aSync = callContext.m_taskGroup().subgroup("QuickStepIsland Stage6a Sync", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage6aSync(stage6CallContext, callContext.m_taskGroup());                    
                }
            });
            for (int i = 1; i < stage6a_allowedThreads; i++) {
                Task stage6a = stage6aSync.subtask("QuickStepIsland Stage6a", new Runnable() {
                    @Override
                    public void run() {
                        dxQuickStepIsland_Stage6a(stage6CallContext);
                    }
                });
                stage6a.submit();
            }
            dxQuickStepIsland_Stage6a(stage6CallContext);
            stage6aSync.submit();
	    }
	}
	
	private static
	void dxQuickStepIsland_Stage6a(dxQuickStepperStage6CallContext stage6CallContext)
	{
        DxStepperProcessingCallContext callContext = stage6CallContext.m_stepperCallContext;
	    dxQuickStepperLocalContext localContext = stage6CallContext.m_localContext;

	    double stepsize = callContext.m_stepSize();
	    double[] invI = localContext.m_invI;
	    DxBody[] bodyA = callContext.m_islandBodiesStartA();
        int bodyOfs = callContext.m_islandBodiesStartOfs();

	    int nb = callContext.m_islandBodiesCount();
	    int step_size = dxQUICKSTEPISLAND_STAGE6A_STEP;
	    int nb_steps = (nb + (step_size - 1)) / step_size;

	    int bi_step;
	    while ((bi_step = Atomics.ThrsafeIncrementIntUpToLimit(stage6CallContext.m_bi_6a, nb_steps)) != nb_steps) {
	        int bi = bi_step * step_size;
	        int bicnt = Math.min(step_size, nb - bi);

	        int invIrow = bi * IIE__MAX;
	        int bodycurr = bi;
	        int bodyend = bodycurr + bicnt;
	        while (true) {
	            // compute the velocity update:
	            // add stepsize * invM * fe to the body velocity
	            DxBody b = bodyA[bodyOfs + bodycurr];
	            double body_invMass_mul_stepsize = stepsize * b.invMass;
	            b.lvel.addScaled(b.facc, body_invMass_mul_stepsize);
                b.tacc.scale(stepsize);
                dMultiplyAdd0_331 (b.avel, invI, invIrow + IIE__MATRIX_MIN, b.tacc);

                if (++bodycurr == bodyend) {
	                break;
	            }
	            invIrow += IIE__MAX;
	        }
	    }
	}

	private static 
    void dxQuickStepIsland_Stage6aSync(final dxQuickStepperStage6CallContext stage6CallContext, TaskGroup group)
    {
	    dxQuickStepIsland_Stage6_VelocityCheck(stage6CallContext);

        DxStepperProcessingCallContext callContext = stage6CallContext.m_stepperCallContext;

        int allowedThreads = callContext.m_stepperAllowedThreads();
        int nb = callContext.m_islandBodiesCount();
        int stage6b_allowedThreads = CalculateOptimalThreadsCount(nb, allowedThreads, dxQUICKSTEPISLAND_STAGE6B_STEP);

        for (int i = 1; i < stage6b_allowedThreads; i++) {
            Task stage6b = group.subtask("QuickStepIsland Stage6b", new Runnable() {
                @Override
                public void run() {
                    dxQuickStepIsland_Stage6b(stage6CallContext);
                }
            });
            stage6b.submit();
        }
        dxQuickStepIsland_Stage6b(stage6CallContext);
    }


	private static void dxQuickStepIsland_Stage6_VelocityCheck(dxQuickStepperStage6CallContext stage6CallContext) {
		if (CHECK_VELOCITY_OBEYS_CONSTRAINT) {
			dxQuickStepperLocalContext localContext = stage6CallContext.m_localContext;
            /*
    	    int int m = localContext->m_m;
    	    if (m > 0) {
    	        const dxStepperProcessingCallContext *callContext = stage6CallContext->m_stepperCallContext;
    	        dxBody * const *body = callContext->m_islandBodiesStart;
    	        dReal *J = localContext->m_J;
    	        const dxJBodiesItem *jb = localContext->m_jb;
    
    	        dReal error = 0;
    	        const dReal* J_ptr = J;
    	        for (int i = 0; i < m; ++i) {
    	            int b1 = jb[i].first;
    	            int b2 = jb[i].second;
    	            dReal sum = 0;
    	            dxBody *bodycurr = body[(int)b1];
    	            for (int j = dSA__MIN; j != dSA__MAX; ++j) sum += J_ptr[JME__J1L_MIN + j] *
    	            bodycurr->lvel[dV3E__AXES_MIN + j] + J_ptr[JME__J1A_MIN + j] * bodycurr->avel[dV3E__AXES_MIN + j];
    	            if (b2 != -1) {
    	                dxBody *bodycurr = body[(int)b2];
    	                for (int k = dSA__MIN; k != dSA__MAX; ++k) sum += J_ptr[JME__J2L_MIN + k] *
    	                bodycurr->lvel[dV3E__AXES_MIN + k] + J_ptr[JME__J2A_MIN + k] * bodycurr->avel[dV3E__AXES_MIN +
    	                 k];
    	            }
    	            J_ptr += JME__MAX;
    	            error += dFabs(sum);
    	        }
    	        printf ("velocity error = %10.6e\n", error);
    	    }
        */
			throw new UnsupportedOperationException();
		}
	}

	private static
	void dxQuickStepIsland_Stage6b(dxQuickStepperStage6CallContext stage6CallContext)
	{
        DxStepperProcessingCallContext callContext = stage6CallContext.m_stepperCallContext;

	    double stepsize = callContext.m_stepSize();
	    DxBody[] bodyA = callContext.m_islandBodiesStartA();
        int bodyOfs = callContext.m_islandBodiesStartOfs();

	    // update the position and orientation from the new linear/angular velocity
	    // (over the given timestep)
	    int nb = callContext.m_islandBodiesCount();
	    int step_size = dxQUICKSTEPISLAND_STAGE6B_STEP;
	    int nb_steps = (nb + (step_size - 1)) / step_size;

	    int bi_step;
	    while ((bi_step = Atomics.ThrsafeIncrementIntUpToLimit(stage6CallContext.m_bi_6b, nb_steps)) != nb_steps) {
	        int bi = bi_step * step_size;
	        int bicnt = Math.min(step_size, nb - bi);

	        int bodycurr = bi;
	        int bodyend = bodycurr + bicnt;
	        while (true) {
	            DxBody b = bodyA[bodyOfs + bodycurr];
        		b.dxStepBody (stepsize);
	        	b.facc.setZero();
	        	b.tacc.setZero();
	            if (++bodycurr == bodyend) {
	                break;
	            }
	        }
	    }
	}

	@Override
	public int dxEstimateMemoryRequirements (
	        DxBody[] body, int bodyOfs, int nb, 
	        DxJoint[] _joint, int jointOfs, int _nj)
	{
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

	private DxQuickStep() {}
}