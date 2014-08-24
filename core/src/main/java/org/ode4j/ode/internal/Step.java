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
import static org.ode4j.ode.OdeMath.dMultiply0_133;
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply0_333;
import static org.ode4j.ode.OdeMath.dMultiply2_333;
import static org.ode4j.ode.OdeMath.dMultiplyAdd0_331;
import static org.ode4j.ode.OdeMath.dSetCrossMatrixMinus;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dIVERIFY;
import static org.ode4j.ode.internal.Common.dPAD;
import static org.ode4j.ode.internal.Common.dRecip;
import static org.ode4j.ode.internal.Matrix.dSetValue;
import static org.ode4j.ode.internal.Timer.dTimerEnd;
import static org.ode4j.ode.internal.Timer.dTimerNow;
import static org.ode4j.ode.internal.Timer.dTimerReport;
import static org.ode4j.ode.internal.Timer.dTimerStart;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stdout;
import static org.ode4j.ode.threading.ThreadingUtils.ThrsafeExchange;
import static org.ode4j.ode.threading.ThreadingUtils.ThrsafeIncrementIntUpToLimit;

import java.util.concurrent.atomic.AtomicInteger;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DJoint.DJointFeedback;
import org.ode4j.ode.internal.cpp4j.FILE;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dmaxcallcountestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemArena;
import org.ode4j.ode.threading.Threading_H.CallContext;
import org.ode4j.ode.threading.Threading_H.DCallReleasee;
import org.ode4j.ode.threading.Threading_H.dThreadedCallFunction;


class Step extends AbstractStepper implements dstepper_fn_t, dmemestimate_fn_t, 
dmaxcallcountestimate_fn_t {

	public static final Step INSTANCE = new Step();

	//#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
	//#define dMAX(A,B)  ((B)>(A) ? (B) : (A))
	//TX: not used
	//private static final int dMIN(int A, int B) { return ((A)>(B) ? (B) : (A)); }
	private static final int dMAX(int A, int B) { return ((B)>(A) ? (B) : (A)); }


	//****************************************************************************
	// misc defines

	//#define TIMING  /was commented out!
	private static final boolean TIMING = false;
	private static final void IFTIMING_dTimerStart(String name) {
		if (TIMING) {
			dTimerStart(name);
		}
	}
	private static final void IFTIMING_dTimerNow(String name) {
		if (TIMING) {
			dTimerNow(name);
		}
	}
	private static final void IFTIMING_dTimerEnd() {
		if (TIMING) {
			dTimerEnd();
		}
	}
	private static final void IFTIMING_dTimerReport (FILE fout, int average) {
		if (TIMING) {
			dTimerReport(fout, average);
		}
	}

	public static final boolean DIRECT_CHOLESKY = true;
	public static final boolean REPORT_ERROR = false;

	private static class dJointWithInfo1
	{
		DxJoint joint;
		final DxJoint.Info1 info = new DxJoint.Info1();
	}

	private static class dxStepperStage0Outputs
	{
		int                          ji_start;
		int                          ji_end;
		int                    m;
		int                    nub;
	}

	private static class dxStepperStage1CallContext implements CallContext
	{
		dxStepperStage1CallContext(final DxStepperProcessingCallContext stepperCallContext, 
				BlockPointer stageMemArenaState, double[] invI, 
				dJointWithInfo1[] jointinfosA, int jointinfosOfs)    {
			m_stepperCallContext = stepperCallContext;
			m_stageMemArenaState = stageMemArenaState; 
			m_invI = invI;
			m_jointinfosA = jointinfosA;
			m_jointinfosOfs = jointinfosOfs;
		}

		final DxStepperProcessingCallContext m_stepperCallContext;
		BlockPointer                    m_stageMemArenaState;
		double[]                        m_invI;
		dJointWithInfo1[]               m_jointinfosA;
		int				                m_jointinfosOfs;
		final dxStepperStage0Outputs          m_stage0Outputs = new dxStepperStage0Outputs();
	}

	private static class dxStepperStage0BodiesCallContext implements CallContext
	{
		dxStepperStage0BodiesCallContext(final DxStepperProcessingCallContext stepperCallContext, 
				double[] invI)
				{
			m_stepperCallContext = stepperCallContext;
			m_invI = invI; 
			m_tagsTaken = new AtomicInteger(0);
			m_gravityTaken = new AtomicInteger(0);
			//m_inertiaBodyIndex = 0;
				}

		final DxStepperProcessingCallContext m_stepperCallContext;
		double[]                        m_invI;
		final AtomicInteger                   m_tagsTaken;
		final AtomicInteger                   m_gravityTaken;
		//volatile int                    m_inertiaBodyIndex;
		final AtomicInteger                     m_inertiaBodyIndex = new AtomicInteger();
	}

	private static class dxStepperStage0JointsCallContext implements CallContext
	{
		dxStepperStage0JointsCallContext(final DxStepperProcessingCallContext stepperCallContext, 
				dJointWithInfo1[] jointinfosA, int jointinfosOfs, 
				dxStepperStage0Outputs stage0Outputs) {
			m_stepperCallContext = stepperCallContext;
			m_jointinfosA = jointinfosA;
			m_jointinfosOfs = jointinfosOfs;
			m_stage0Outputs = stage0Outputs;
		}

		final DxStepperProcessingCallContext m_stepperCallContext;
		dJointWithInfo1[]                 m_jointinfosA;
		int				                  m_jointinfosOfs;
		dxStepperStage0Outputs            m_stage0Outputs;
	}

	//forward declarations
	//static int dxStepIsland_Stage0_Bodies_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};
	//static int dxStepIsland_Stage0_Joints_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};
	//static int dxStepIsland_Stage1_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};

	//static void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext callContext) {/*TZ*/};
	//static void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext callContext) {/*TZ*/};
	//static void dxStepIsland_Stage1(dxStepperStage1CallContext callContext) {/*TZ*/};


	private static class dxStepperLocalContext
	{
		void Initialize(double[] invI, dJointWithInfo1[] jointinfosA,
				int jointinfosOfs,
				int nj, 
				int m, int nub, final int[] mindex, int[] findex, 
				double[] lo, double[] hi, double[] J, double[] A, double[] rhs)
		{
			m_invI = invI;
			m_jointinfosA = jointinfosA;
			m_jointinfosOfs = jointinfosOfs;
			m_nj = nj;
			m_m = m;
			m_nub = nub;
			m_mindex = mindex;
			m_findex = findex; 
			m_lo = lo;
			m_hi = hi;
			m_J = J;
			m_A = A;
			m_rhs = rhs;
		}

		double[]                           m_invI;
		dJointWithInfo1[]                 m_jointinfosA;
		int				                 m_jointinfosOfs;
		int                    m_nj;
		int                    m_m;
		int                    m_nub;
		int[]              m_mindex;
		int[]                              m_findex;
		double[]                           m_lo;
		double[]                           m_hi;
		double[]                           m_J;
		double[]                           m_A;
		double[]                           m_rhs;
	}

	private static class dxStepperStage3CallContext implements CallContext
	{
		void Initialize(final DxStepperProcessingCallContext callContext, 
				final dxStepperLocalContext localContext, 
				BlockPointer stage1MemArenaState)
		{
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			m_stage1MemArenaState = stage1MemArenaState;
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		dxStepperLocalContext     m_localContext;
		BlockPointer                           m_stage1MemArenaState;
	}

	private static class dxStepperStage2CallContext implements CallContext
	{
		void Initialize(final DxStepperProcessingCallContext callContext, 
				final dxStepperLocalContext localContext, 
				double[] JinvM, double[] rhs_tmp_or_cfm)
		{
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			m_JinvM = JinvM;
			m_rhs_tmp_or_cfm = rhs_tmp_or_cfm;
			//m_ji_J = 0;
			//m_ji_Ainit = 0;
			//m_ji_JinvM = 0;
			//m_ji_Aaddjb = 0;
			//m_bi_rhs_tmp = 0;
			//m_ji_rhs.set(0);
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		dxStepperLocalContext     m_localContext;
		double[]                        m_JinvM;
		double[]                        m_rhs_tmp_or_cfm;
		//volatile int               m_ji_J;
		final AtomicInteger               m_ji_J = new AtomicInteger();
		//volatile int               m_ji_Ainit;
		final AtomicInteger               m_ji_Ainit = new AtomicInteger();
		//volatile int               m_ji_JinvM;
		final AtomicInteger               m_ji_JinvM = new AtomicInteger();
		//volatile int               m_ji_Aaddjb;
		final AtomicInteger               m_ji_Aaddjb = new AtomicInteger();
		//volatile int               m_bi_rhs_tmp;
		final AtomicInteger               m_bi_rhs_tmp = new AtomicInteger();
		//volatile int               m_ji_rhs;
		final AtomicInteger              m_ji_rhs = new AtomicInteger();
	}

	//static int dxStepIsland_Stage2a_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};
	//static int dxStepIsland_Stage2aSync_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};
	//static int dxStepIsland_Stage2b_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};
	//static int dxStepIsland_Stage2bSync_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};
	//static int dxStepIsland_Stage2c_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};
	//static int dxStepIsland_Stage3_Callback(Object[] callContext, int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee) {/*TZ*/};

	//static void dxStepIsland_Stage2a(dxStepperStage2CallContext callContext) {/*TZ*/};
	//static void dxStepIsland_Stage2b(dxStepperStage2CallContext callContext) {/*TZ*/};
	//static void dxStepIsland_Stage2c(dxStepperStage2CallContext callContext) {/*TZ*/};
	//static void dxStepIsland_Stage3(dxStepperStage3CallContext callContext) {/*TZ*/};


	//****************************************************************************
	// special matrix multipliers

	// this assumes the 4th and 8th rows of B and C are zero.

	//TZ: not used...
//	private static void Multiply2_p8r (double[] A, int APos,
//			double[] B, int BPos, double[] C, int CPos,
//			int p, int r, int Askip)
//	{
//		dIASSERT (p>0 && r>0);
//		//		dAASSERT(A, B, C);
//		final int Askip_munus_r = Askip - r;
//		int aa = APos;//dReal *aa = A;
//		int bb = BPos;//const dReal *bb = B;
//		for (int i = p; i != 0; --i) {
//			int cc = CPos;//const dReal *cc = C;
//			for (int j = r; j != 0; --j) {
//				double sum;
//				sum  = B[bb] * C[cc]; //bb[0]*cc[0];
//				sum += B[bb+1] * C[cc+1]; //sum += bb[1]*cc[1];
//				sum += B[bb+2] * C[cc+2]; //sum += bb[2]*cc[2];
//				sum += B[bb+4] * C[cc+4]; //sum += bb[4]*cc[4];
//				sum += B[bb+5] * C[cc+5]; //sum += bb[5]*cc[5];
//				sum += B[bb+6] * C[cc+6]; //sum += bb[6]*cc[6];
//				A[aa++] = sum;//*(A++) = sum; 
//				cc +=8;//cc += 8;
//			}
//			bb += 8;
//			aa += Askip_munus_r;
//		}
//	}


	// this assumes the 4th and 8th rows of B and C are zero.

	private static void MultiplyAdd2_p8r (double[] A, int APos,
			double[] B, int BPos, double[] C, int CPos,
			int p, int r, int Askip)
	{
		dIASSERT (p>0 && r>0);
		//		dAASSERT(A, B, C);
		final int Askip_munus_r = Askip - r;
		dIASSERT(Askip >= r);
		int aa = APos;//  dReal *aa = A;
		int bb = BPos;//const dReal *bb = B;
		for (int i = p; i != 0; --i) {
			int cc = CPos;//const dReal *cc = C;
			for (int j = r; j != 0; --j) {
				double sum;
				sum  = B[bb] * C[cc]; //sum = bb[0]*cc[0];
				sum += B[bb+1] * C[cc+1]; //sum += bb[1]*cc[1];
				sum += B[bb+2] * C[cc+2]; //sum += bb[2]*cc[2];
				sum += B[bb+4] * C[cc+4]; //sum += bb[4]*cc[4];
				sum += B[bb+5] * C[cc+5]; //sum += bb[5]*cc[5];
				sum += B[bb+6] * C[cc+6]; //sum += bb[6]*cc[6];
				A[aa++] += sum;//*(aa++) += sum; 
				cc += 8;
			}
			bb += 8;
			aa += Askip_munus_r;
		}
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static void MultiplySub0_p81 (double[] A, final int APos,
			double[] B, final int BPos, double[] C, final int CPos, int p)
	{
		dIASSERT (p>0);
		//		dAASSERT(A, B, C);
		int aa = APos;//dReal *aa = A;
		int bb = BPos;//const dReal *bb = B;
		final double C_0 = C[0+CPos], C_1 = C[1+CPos], C_2 = C[2+CPos];
		final double C_4 = C[4+CPos], C_5 = C[5+CPos], C_6 = C[6+CPos];
		for (int i = p; i != 0; --i) {
			double sum;
			sum  = B[0+bb]*C_0;
			sum += B[1+bb]*C_1;
			sum += B[2+bb]*C_2;
			sum += B[4+bb]*C_4;
			sum += B[5+bb]*C_5;
			sum += B[6+bb]*C_6;
			A[aa++] -= sum;//*(aa++) = sum;
			bb += 8;//B += 8;
		}
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static void MultiplyAdd1_8q1 (double[] A, int APos, 
			double[] B, int BPos, double[] C, int CPos, int q)
	{
		dIASSERT (q>0);
		//	    dAASSERT(A, B, C);
		int bb = BPos;//const dReal *bb = B;
		double sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
		for (int k = 0; k < q; ++k) {
			final double C_k = C[k+CPos];
			sum0 += B[bb+0] * C_k;
			sum1 += B[bb+1] * C_k;
			sum2 += B[bb+2] * C_k;
			sum4 += B[bb+4] * C_k;
			sum5 += B[bb+5] * C_k;
			sum6 += B[bb+6] * C_k;
			bb += 8;
		}
		A[0+APos] += sum0;
		A[1+APos] += sum1;
		A[2+APos] += sum2;
		A[4+APos] += sum4;
		A[5+APos] += sum5;
		A[6+APos] += sum6;
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static void Multiply1_8q1 (double[] A, final int APos, 
			double[] B, final int BPos, double[] C, final int CPos, int q)
	{
		int bb = BPos;//const dReal *bb = B;
		double sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
		for (int k = 0; k < q; ++k) {
			final double C_k = C[k+CPos];
			sum0 += B[bb+0] * C_k;
			sum1 += B[bb+1] * C_k;
			sum2 += B[bb+2] * C_k;
			sum4 += B[bb+4] * C_k;
			sum5 += B[bb+5] * C_k;
			sum6 += B[bb+6] * C_k;
			bb += 8;
		}
		A[0+APos] = sum0;
		A[1+APos] = sum1;
		A[2+APos] = sum2;
		A[4+APos] = sum4;
		A[5+APos] = sum5;
		A[6+APos] = sum6;
	}

	//****************************************************************************

	/*extern */
	private void dxStepIsland(final DxStepperProcessingCallContext callContext)
	{
		IFTIMING_dTimerStart("preprocessing");

		DxWorldProcessMemArena memarena = callContext.m_stepperArena();
		DxWorld world = callContext.m_world();
		int nb = callContext.m_islandBodiesCount();
		int _nj = callContext.m_islandJointsCount();

		memarena.dummy();
		double[] invI = new double[3*4*nb];//memarena.AllocateArray<dReal> (3*4*(size_t)nb);
		// Reserve twice as much memory and start from the middle so that regardless of 
		// what direction the array grows to there would be sufficient room available.
		final int ji_reserve_count = 2 * _nj;
		memarena.dummy();
		dJointWithInfo1[] jointinfosA = new dJointWithInfo1[ji_reserve_count];//memarena.AllocateArray<dJointWithInfo1>(ji_reserve_count);
		//TZ: init
		for (int i = 0; i < jointinfosA.length; i++) {
			jointinfosA[i] = new dJointWithInfo1();
		}
		int jointinfosOfs = 0;

		final int allowedThreads = callContext.m_stepperAllowedThreads();
		dIASSERT(allowedThreads != 0);

		BlockPointer stagesMemArenaState = memarena.SaveState();

		//dxStepperStage1CallContext stage1CallContext = (dxStepperStage1CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage1CallContext));
		//new(stage1CallContext) dxStepperStage1CallContext(callContext, stagesMemArenaState, invI, jointinfos);
		memarena.dummy();
		final dxStepperStage1CallContext stage1CallContext = new dxStepperStage1CallContext(callContext, 
				stagesMemArenaState, invI, jointinfosA, jointinfosOfs);

		//dxStepperStage0BodiesCallContext *stage0BodiesCallContext = (dxStepperStage0BodiesCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0BodiesCallContext));
		//new(stage0BodiesCallContext) dxStepperStage0BodiesCallContext(callContext, invI);
		memarena.dummy();
		final dxStepperStage0BodiesCallContext stage0BodiesCallContext = 
				new dxStepperStage0BodiesCallContext(callContext, invI);

		//dxStepperStage0JointsCallContext *stage0JointsCallContext = (dxStepperStage0JointsCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0JointsCallContext));
		//new(stage0JointsCallContext) dxStepperStage0JointsCallContext(callContext, jointinfos, &stage1CallContext->m_stage0Outputs);
		memarena.dummy();
		final dxStepperStage0JointsCallContext stage0JointsCallContext = 
				new dxStepperStage0JointsCallContext(callContext, jointinfosA, jointinfosOfs, 
						stage1CallContext.m_stage0Outputs);

		if (allowedThreads == 1)
		{
			dxStepIsland_Stage0_Bodies(stage0BodiesCallContext);
			dxStepIsland_Stage0_Joints(stage0JointsCallContext);
			dxStepIsland_Stage1(stage1CallContext);
		}
		else
		{
			int bodyThreads = allowedThreads;
			int jointThreads = 1;

			Ref<DCallReleasee> stage1CallReleasee = new Ref<DCallReleasee>();
			world.threading().PostThreadedCallForUnawareReleasee(null, stage1CallReleasee, 
					bodyThreads + jointThreads, callContext.m_finalReleasee(), 
					null, dxStepIsland_Stage1_Callback, stage1CallContext, 0, 
					"StepIsland Stage1");

			world.threading().PostThreadedCallsGroup(null, bodyThreads, stage1CallReleasee.get(), 
					dxStepIsland_Stage0_Bodies_Callback, stage0BodiesCallContext, 
					"StepIsland Stage0-Bodies");

			world.threading().PostThreadedCall(null, null, 0, stage1CallReleasee.get(), null, 
					dxStepIsland_Stage0_Joints_Callback, stage0JointsCallContext, 0, 
					"StepIsland Stage0-Joints");
			dIASSERT(jointThreads == 1);
		}
//		{
//			//TODO
//			try {
//				dxStepIsland_Stage1(stage1CallContext);
//				ArrayList<Callable<Boolean>> tasks = new ArrayList<>(); 
//				for (int i = 0; i < DxQuickStep.THREADS; i++) {
//					tasks.add(new Callable<Boolean>() {
//						@Override
//						public Boolean call() {
//							//dxStepIsland_Stage1_Callback.run(call_context, i, this_releasee);
//							//dxStepperStage0BodiesCallContext callContext = (dxStepperStage0BodiesCallContext )_callContext;
//							dxStepIsland_Stage0_Bodies(stage0BodiesCallContext);
//							return Boolean.TRUE;
//						}
//					});
//				}
//				for (Future<Boolean> f: DxQuickStep.POOL.invokeAll(tasks, 1, TimeUnit.HOURS)) {
//					f.get();
//				}
//				dxStepIsland_Stage0_Joints(stage0JointsCallContext);
//			} catch (InterruptedException e) {
//				throw new RuntimeException(e);
//			} catch (ExecutionException e) {
//				throw new RuntimeException(e);
//			}
	}    

	private static dThreadedCallFunction dxStepIsland_Stage0_Bodies_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _callContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxStepperStage0BodiesCallContext callContext = (dxStepperStage0BodiesCallContext )_callContext;
			dxStepIsland_Stage0_Bodies(callContext);
			return true;
		}
	};

	private
	static 
	//void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext *callContext)
	void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext callContext)
	{
		DxBody[] bodyA = callContext.m_stepperCallContext.m_islandBodiesStartA();
		final int bOfs = callContext.m_stepperCallContext.m_islandBodiesStartOfs();
		int nb = callContext.m_stepperCallContext.m_islandBodiesCount();

		if (ThrsafeExchange(callContext.m_tagsTaken, 1) == 0)
		{
			// number all bodies in the body list - set their tag values
			for (int i=0; i<nb; i++) {
				bodyA[i+bOfs].tag = i;
			}
		}

		if (ThrsafeExchange(callContext.m_gravityTaken, 1) == 0)
		{
			DxWorld world = callContext.m_stepperCallContext.m_world();

			// add the gravity force to all bodies
			// since gravity does normally have only one component it's more efficient
			// to run three loops for each individual component
			double gravity_x = world.gravity.get0();
			if (gravity_x!=0) {
				for (int ii = bOfs; ii < bOfs+nb; ii++) {
					DxBody b = bodyA[ii];
					if (b.getGravityMode()) {
						b.facc.add(0, b.mass._mass * gravity_x);
					}
				}
			}
			double gravity_y = world.gravity.get1();
			if (gravity_y!=0) {
				for (int ii = bOfs; ii < bOfs+nb; ii++) {
					DxBody b = bodyA[ii];
					if (b.getGravityMode()) {
						b.facc.add(1, b.mass._mass * gravity_y);
					}
				}
			}
			double gravity_z = world.gravity.get2();
			if (gravity_z!=0) {
				for (int ii = bOfs; ii < bOfs+nb; ii++) {
					DxBody b = bodyA[ii];
					if (b.getGravityMode()) {
						b.facc.add(2, b.mass._mass * gravity_z);
					}
				}
			}
		}

		// for all bodies, compute the inertia tensor and its inverse in the global
		// frame, and compute the rotational force and add it to the torque
		// accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
		{
			int invIrowO = 0;
			double[] invIrowA = callContext.m_invI;
			int bodyIndex = ThrsafeIncrementIntUpToLimit(callContext.m_inertiaBodyIndex, nb);

			for (int i = 0; i != nb; invIrowO += 12, ++i) {
				if (i == bodyIndex) {
					DMatrix3 tmp = new DMatrix3();
					DxBody b = bodyA[i+bOfs];

					// compute inverse inertia tensor in global frame
					dMultiply2_333 (tmp,b.invI,b.posr().R());
					dMultiply0_333 (invIrowA, invIrowO,b.posr().R(),tmp);

					// Don't apply gyroscopic torques to bodies
					// if not flagged or the body is kinematic
					//if ((b.flags & DxBody.dxBodyGyroscopic)&& (b.invMass>0)) {
					if (b.isFlagsGyroscopic() && (b.invMass>0)) {
						DMatrix3 I = new DMatrix3();
						// compute inertia tensor in global frame
						dMultiply2_333 (tmp,b.mass._I,b.posr().R());
						dMultiply0_333 (I,b.posr().R(),tmp);
						// compute rotational force
						//#if 0
						//// Explicit computation
						//dMultiply0_331 (tmp,I,b.avel);
						//dSubtractVectorCross3(b.tacc,b.avel,tmp);
						//#else
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
						DMatrix3 Itild=new DMatrix3();//{0};  
						dSetCrossMatrixMinus(Itild,L);//,4);
						//for (int ii=0;ii<12;++ii) {
						//	Itild[ii]=Itild[ii]*h+I[ii];
						//}
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
							//	b.tacc[ii]+=tau0[ii];
							//}
							b.tacc.add(tau0);
						}
						//#endif  //if 0
					}

					bodyIndex = ThrsafeIncrementIntUpToLimit(callContext.m_inertiaBodyIndex, nb);
				}
			}
		}
	}

	private static dThreadedCallFunction dxStepIsland_Stage0_Joints_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _callContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxStepperStage0JointsCallContext callContext = (dxStepperStage0JointsCallContext)_callContext;
			dxStepIsland_Stage0_Joints(callContext);
			return true;
		}
	};

	private static 
	void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext callContext)
	{
		DxJoint[] _jointA = callContext.m_stepperCallContext.m_islandJointsStartA();
		int jOfs = callContext.m_stepperCallContext.m_islandJointsStartOfs();
		dJointWithInfo1[] jointinfosA = callContext.m_jointinfosA;
		int jiP = callContext.m_jointinfosOfs;
		int _nj = callContext.m_stepperCallContext.m_islandJointsCount();

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

		int ji_start, ji_end;

		{
			int mcurr = 0;
			int unb_start, mix_start, mix_end, lcp_end;
			unb_start = mix_start = mix_end = lcp_end = _nj;

			//	      dJointWithInfo1 *jicurr = jointiinfos + lcp_end;
			//	      dxJoint *const *const _jend = _joint + _nj;
			//	      dxJoint *const *_jcurr = _joint;
			dJointWithInfo1 jicurrO = jointinfosA.length > 0 ? jointinfosA[lcp_end+jiP] : null;
			int jicurrP = lcp_end;
			final int _jend = _nj; 
			int _jcurrP = 0;
			//DxJoint _jcurrO = null;
			while (true) {
				// -------------------------------------------------------------------------
				// Switch to growing array forward
				{
					boolean fwd_end_reached = false;
					dJointWithInfo1 jimixendO = jointinfosA.length > 0 ? jointinfosA[mix_end+jiP] : null;
					int jimixendP = mix_end;
					while (true) {  // jicurr=dest, _jcurr=src
						if (_jcurrP == _jend) {
							lcp_end = jicurrP;// - jointiinfos;
							fwd_end_reached = true;
							break;
						}
						DxJoint j = _jointA[jOfs+_jcurrP++];//*_jcurr++;
						//_jcurrO = _jointA[jOfs+_jcurrP];//TZ TODO?? Fails in Plane2DDemo
						j.getInfo1 (jicurrO.info);
						dIASSERT (/*jicurr->info.m >= 0 && */jicurrO.info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurrO.info.nub <= jicurrO.info.m);
						if (jicurrO.info.m > 0) {
							mcurr += jicurrO.info.m;
							if (jicurrO.info.nub == 0) { // A lcp info - a correct guess!!!
								jicurrO.joint = j;
								++jicurrP;
								jicurrO = (jicurrP+jiP) < jointinfosA.length ? jointinfosA[jicurrP+jiP] : null;//TZ
							} else if (jicurrO.info.nub < jicurrO.info.m) { // A mixed case
								if (unb_start == mix_start) { // no unbounded infos yet - just move to opposite side of mixed-s
									unb_start = mix_start = mix_start - 1;
									dJointWithInfo1 jimixstart = jointinfosA[mix_start+jiP];
									jimixstart.info.set( jicurrO.info);
									jimixstart.joint = j;
								} else if (jimixendP != jicurrP) { // have to swap to the tail of mixed-s
									DxJoint.Info1 tmp_info = jicurrO.info;
									jicurrP = jimixendP;
									jicurrO = jointinfosA[jicurrP+jiP];//TZ
									jimixendO.info.set(tmp_info);
									jimixendO.joint = j;
									++jimixendP; ++jicurrP;
									jicurrO = jointinfosA[jicurrP+jiP];//TZ
									jimixendO = jointinfosA[jimixendP+jiP];//TZ
								} else { // no need to swap as there are no LCP info-s yet
									jicurrO.joint = j;
									jimixendP = jicurrP = jicurrP + 1;
									jicurrO = jointinfosA[jicurrP+jiP];//TZ
									jimixendO = jointinfosA[jimixendP+jiP];//TZ
								}
							} else { // A purely unbounded case -- break out and proceed growing in opposite direction
								unb_start = unb_start - 1;
								dJointWithInfo1 jiunbstartO = jointinfosA[unb_start+jiP];
								int jiunbstartP = unb_start;
								jiunbstartO.info.set( jicurrO.info);
								jiunbstartO.joint = j;
								lcp_end = jicurrP;// - jointiinfos;
								mix_end = jimixendP;// - jointiinfos;
								jicurrP = jiunbstartP - 1;
								if (jicurrP >= 0) {
									jicurrO = jointinfosA[jicurrP+jiP];//TZ
								} else {
									//TODO TZ is this good??
									jicurrO = null;
								}
								break;
							}
						} else {
							j.tag = -1;
						}
					}
					if (fwd_end_reached) {
						break;
					}
				}
				// -------------------------------------------------------------------------
				// Switch to growing array backward
				{
					boolean bkw_end_reached = false;
					dJointWithInfo1 jimixstartO = jointinfosA[mix_start - 1 +jiP];
					int jimixstartP = mix_start - 1;
					while (true) {  // jicurr=dest, _jcurr=src
						if (_jcurrP == _jend) {
							unb_start = (jicurrP + 1);// - jointiinfos;
							mix_start = (jimixstartP + 1);// - jointiinfos;
							bkw_end_reached = true;
							break;
						}
						DxJoint j = _jointA[jOfs+_jcurrP++];//*_jcurr++;
						// _jcurrO = _jointA[jOfs+_jcurrP]; //TZ TODO?
						j.getInfo1 (jicurrO.info); //TZ
						dIASSERT (jicurrO.info.m >= 0 && jicurrO.info.m <= 6 && jicurrO.info.nub >= 0 && jicurrO.info.nub <= jicurrO.info.m);
						dIASSERT (/*jicurr->info.m >= 0 && */jicurrO.info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurrO.info.nub <= jicurrO.info.m);
						if (jicurrO.info.m > 0) {
							mcurr += jicurrO.info.m;
							if (jicurrO.info.nub == jicurrO.info.m) { // An unbounded info - a correct guess!!!
								jicurrO.joint = j;
								--jicurrP;
								if (jicurrP +jiP >=0) {
									jicurrO = jointinfosA[jicurrP+jiP]; //TZ
								} else {
									jicurrO = null;
								}
							} else if (jicurrO.info.nub != 0) { // A mixed case
								if (mix_end == lcp_end) { // no lcp infos yet - just move to opposite side of mixed-s
									dJointWithInfo1 jimixend = jointinfosA[mix_end+jiP];
									lcp_end = mix_end = mix_end + 1;
									jimixend.info.set( jicurrO.info);
									jimixend.joint = j;
								} else if (jimixstartP != jicurrP) { // have to swap to the head of mixed-s
									DxJoint.Info1 tmp_info = jicurrO.info;
									//*jicurr = *jimixstart;
									jicurrP = jimixstartP;
									jicurrO = jointinfosA[jicurrP+jiP]; //TZ
									jimixstartO.info.set( tmp_info);
									jimixstartO.joint = j;
									--jimixstartP; --jicurrP;
									jicurrO = jointinfosA[jicurrP+jiP]; //TZ
									jimixstartO = jointinfosA[jimixstartP+jiP]; //TZ
								} else { // no need to swap as there are no unbounded info-s yet
									jicurrO.joint = j;
									jimixstartP = jicurrP = jicurrP - 1;
									jicurrO = jointinfosA[jicurrP+jiP]; //TZ
									jimixstartO = jointinfosA[jimixstartP+jiP]; //TZ
								}
							} else { // A purely lcp case -- break out and proceed growing in opposite direction
								dJointWithInfo1 jilcpendO = jointinfosA[lcp_end+jiP];
								int jilcpendP = lcp_end;
								lcp_end = lcp_end + 1;
								jilcpendO.info.set( jicurrO.info);
								jilcpendO.joint = j;
								unb_start = (jicurrP + 1);// - jointiinfos;
								mix_start = (jimixstartP + 1);// - jointiinfos;
								jicurrP = jilcpendP + 1;
								jicurrO = jointinfosA[jicurrP+jiP]; //TZ
								break;
							}
						} else {
							j.tag = -1;
						}
					}
					if (bkw_end_reached) {
						break;
					}
				}
			}

			callContext.m_stage0Outputs.m = mcurr;
			callContext.m_stage0Outputs.nub = (mix_start - unb_start);
			//dIASSERT((size_t)(mix_start - unb_start) <= (size_t)UINT_MAX);
			ji_start = unb_start;
			ji_end = lcp_end;
		}

		{
			int jicurrP = ji_start;
			int jiend = ji_end;
			for (int i = 0; jicurrP != jiend; i++, ++jicurrP) {
				jointinfosA[jiP+jicurrP].joint.tag = i;
			}
			//	        final dJointWithInfo1 jicurr = jointinfos + ji_start;
			//	        final dJointWithInfo1 jiend = jointinfos + ji_end;
			//	        for (int i = 0; jicurr != jiend; i++, ++jicurr) {
			//	            jicurr.joint.tag = i;
			//	        }
			//			//TODO  < / <=  ?!?!?!
			//			for (int i = ji_start; i < ji_end; i++) {
			//				jointinfosA[i+jiP].joint.tag = i;
			//			}
		}

		callContext.m_stage0Outputs.ji_start = ji_start;
		callContext.m_stage0Outputs.ji_end = ji_end;
	}

	private static dThreadedCallFunction dxStepIsland_Stage1_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage1CallContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxStepperStage1CallContext stage1CallContext = (dxStepperStage1CallContext )_stage1CallContext;
			dxStepIsland_Stage1(stage1CallContext);
			return true;
		}
	};

	static 
	void dxStepIsland_Stage1(dxStepperStage1CallContext stage1CallContext)
	{
		final DxStepperProcessingCallContext callContext = stage1CallContext.m_stepperCallContext;
		dJointWithInfo1[] _jointinfosA = stage1CallContext.m_jointinfosA;
		int _jointinfosOfs = stage1CallContext.m_jointinfosOfs;
		double[] invI = stage1CallContext.m_invI;
		int ji_start = stage1CallContext.m_stage0Outputs.ji_start;
		int ji_end = stage1CallContext.m_stage0Outputs.ji_end;
		int m = stage1CallContext.m_stage0Outputs.m;
		int nub = stage1CallContext.m_stage0Outputs.nub;

		DxWorldProcessMemArena memarena = callContext.m_stepperArena();
		{
			memarena.RestoreState(stage1CallContext.m_stageMemArenaState);
			stage1CallContext = null; // WARNING! _stage1CallContext is not valid after this point!
			dIVERIFY(stage1CallContext == null); // To suppress compiler warnings about unused variable assignment

			//int _nj = callContext.m_islandJointsCount();
			//final int ji_reserve_count = 2 * _nj;
			//memarena.ShrinkArray<dJointWithInfo1>(jointiinfos, ji_reserve_count, ji_end);
			memarena.dummy();
		}

		DxWorld world = callContext.m_world();
		dJointWithInfo1[] jointinfosA = _jointinfosA;// + ji_start;
		int jiP = _jointinfosOfs + ji_start;
		int nj = ji_end - ji_start;
		//dIASSERT((size_t)(ji_end - ji_start) <= (size_t)UINT_MAX);

		int[] mindex = null;
		double[] lo = null, hi = null, J = null, A = null, rhs = null;
		int[] findex = null;

		// if there are constraints, compute cforce
		if (m > 0) {
			//mindex = memarena->AllocateArray<unsigned int>((size_t)(nj + 1));
			memarena.dummy();
			mindex = new int[nj + 1];
			{
				//unsigned int *mcurr = mindex;
				int[] mcurrA = mindex;
				int mcurrP = 0;
				int moffs = 0;
				mcurrA[mcurrP + 0] = moffs;
				mcurrP += 1;

				//	            dJointWithInfo1 jiend = jointinfos + nj;
				//	            for (dJointWithInfo1 *jicurr = jointinfos; jicurr != jiend; ++jicurr) {
				//	                //dxJoint *joint = jicurr->joint;
				//	                moffs += jicurr->info.m;
				//	                mcurr[0] = moffs;
				//	                mcurr += 1;
				//	            }
				int jiend = nj;
				for (int jicurrP = 0; jicurrP != jiend; ++jicurrP) {
					dJointWithInfo1 jicurrO = jointinfosA[jiP + jicurrP];
					//dxJoint *joint = jicurr->joint;
					moffs += jicurrO.info.m;
					mcurrA[mcurrP+0] = moffs;
					mcurrP += 1;
				}
			}

			//	        findex = memarena->AllocateArray<int>(m);
			//	        lo = memarena->AllocateArray<dReal>(m);
			//	        hi = memarena->AllocateArray<dReal>(m);
			//	        J = memarena->AllocateArray<dReal>(2 * 8 * (size_t)m);
			//	        A = memarena->AllocateArray<dReal>(m * (size_t)dPAD(m));
			//	        rhs = memarena->AllocateArray<dReal>(m);
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			memarena.dummy();
			findex = new int[m];
			lo = new double[m];
			hi = new double[m];
			J = new double[2 * 8 * m];
			A = new double[m * dPAD(m)];
			rhs = new double[m];
		}

		//dxStepperLocalContext *localContext = (dxStepperLocalContext *)memarena->AllocateBlock(sizeof(dxStepperLocalContext));
		memarena.dummy();
		dxStepperLocalContext localContext = new dxStepperLocalContext();
		localContext.Initialize(invI, jointinfosA, jiP, nj, m, nub, mindex, findex, lo, hi, J, A, rhs);

		BlockPointer stage1MemarenaState = memarena.SaveState();
		//dxStepperStage3CallContext *stage3CallContext = (dxStepperStage3CallContext*)memarena->AllocateBlock(sizeof(dxStepperStage3CallContext));
		memarena.dummy();
		dxStepperStage3CallContext stage3CallContext = new dxStepperStage3CallContext();
		stage3CallContext.Initialize(callContext, localContext, stage1MemarenaState);

		if (m > 0) {
			// create a constraint equation right hand side vector `c', a constraint
			// force mixing vector `cfm', and LCP low and high bound vectors, and an
			// 'findex' vector.
			//double[] JinvM = memarena->AllocateArray<dReal>(2 * 8 * (size_t)m);
			memarena.dummy();
			double[] JinvM = new double[2 * 8 * m];
			final int nb = callContext.m_islandBodiesCount();
			int cfm_elem = m, rhs_tmp_elem = nb*8;
			//dReal *cfm = memarena->AllocateArray<dReal>(dMAX(cfm_elem, rhs_tmp_elem));
			memarena.dummy();
			double[] cfm = new double[dMAX(cfm_elem, rhs_tmp_elem)];
			// dReal *rhs_tmp = cfm; // Reuse the same memory since rhs calculations start after cfm is not needed anymore

			//dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage2CallContext));
			memarena.dummy();
			dxStepperStage2CallContext stage2CallContext = new dxStepperStage2CallContext();
			stage2CallContext.Initialize(callContext, localContext, JinvM, cfm);

			int allowedThreads = callContext.m_stepperAllowedThreads();
			dIASSERT(allowedThreads != 0);

			if (allowedThreads == 1)
			{
				dxStepIsland_Stage2a(stage2CallContext);
				dxStepIsland_Stage2b(stage2CallContext);
				dxStepIsland_Stage2c(stage2CallContext);
				dxStepIsland_Stage3(stage3CallContext);
			}
			else
			{
				Ref<DCallReleasee> stage3CallReleasee = new Ref<DCallReleasee>();
				world.threading().PostThreadedCallForUnawareReleasee(null, stage3CallReleasee, 1, 
						callContext.m_finalReleasee(), 
						null, dxStepIsland_Stage3_Callback, stage3CallContext, 0, "StepIsland Stage3");

				Ref<DCallReleasee> stage2bSyncReleasee = new Ref<DCallReleasee>();
				world.threading().PostThreadedCall(null, stage2bSyncReleasee, 1, stage3CallReleasee.get(), 
						null, dxStepIsland_Stage2bSync_Callback, stage2CallContext, 0, "StepIsland Stage2b Sync");

				Ref<DCallReleasee> stage2aSyncReleasee = new Ref<DCallReleasee>();
				world.threading().PostThreadedCall(null, stage2aSyncReleasee, allowedThreads, stage2bSyncReleasee.get(), 
						null, dxStepIsland_Stage2aSync_Callback, stage2CallContext, 0, "StepIsland Stage2a Sync");

				world.threading().PostThreadedCallsGroup(null, allowedThreads, stage2aSyncReleasee.get(), 
						dxStepIsland_Stage2a_Callback, stage2CallContext, "StepIsland Stage2a");
			}
		}
		else {
			dxStepIsland_Stage3(stage3CallContext);
		}
	}


	//	static 
	//	int dxStepIsland_Stage2a_Callback(CallContext _stage2CallContext, 
	//			int /*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
	private static dThreadedCallFunction dxStepIsland_Stage2a_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, 
				int /*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxStepperStage2CallContext stage2CallContext = (dxStepperStage2CallContext )_stage2CallContext;
			dxStepIsland_Stage2a(stage2CallContext);
			return true;
		}
	};

	static 
	void dxStepIsland_Stage2a(dxStepperStage2CallContext stage2CallContext)
	{
		final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
		final dxStepperLocalContext localContext = stage2CallContext.m_localContext;
		dJointWithInfo1[] jointinfosA = localContext.m_jointinfosA;
		int jointinfosOfs = localContext.m_jointinfosOfs;
		int nj = localContext.m_nj;
		int[] mindex = localContext.m_mindex;

		final double stepsizeRecip = dRecip(callContext.m_stepSize());
		DxWorld world = callContext.m_world();

		{
			int[] findex = localContext.m_findex;
			double[] J = localContext.m_J;
			double[] cfm = stage2CallContext.m_rhs_tmp_or_cfm;
			double[] lo = localContext.m_lo;
			double[] hi = localContext.m_hi;
			double[] rhs = localContext.m_rhs;

			IFTIMING_dTimerNow ("create J");
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

			final double worldERP = world.getERP();

			DxJoint.Info2Descr Jinfo = new DxJoint.Info2Descr();
			Jinfo.setRowskip(8);
			Jinfo.setArrays(J, rhs, cfm, lo, hi, findex); //TZ

			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_J, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				final int J1rowP = 0 + 2*8*ofsi;//*const J1row = J + 2*8*(size_t)ofsi;
				Jinfo.J1lp = J1rowP;
				Jinfo.J1ap = J1rowP + 4;
				final int J2rowP = J1rowP + 8*infom;//dReal *const J2row = J1row + 8*(size_t)infom;
				Jinfo.J2lp = J2rowP;
				Jinfo.J2ap = J2rowP + 4;
				dSetZero (J, J1rowP, 2*8*infom);

				//              Jinfo.c = c + ofsi;
				//              Jinfo.cfm = cfm + ofsi;
				//              Jinfo.lo = lo + ofsi;
				//              Jinfo.hi = hi + ofsi;
				//              Jinfo.findex = findex + ofsi;
				Jinfo.setAllP(ofsi); //TZ

				//Jinfo.c = rhs + ofsi; 
				dSetZero(rhs, ofsi, infom);//dSetZero (rhs, Jinfo.c, infom);
				//Jinfo.cfm = cfm + ofsi;
				dSetValue(cfm, ofsi, infom, world.global_cfm);//dSetValue(Jinfo.cfm, infom, world.global_cfm);
				//Jinfo.lo = lo + ofsi;
				dSetValue (lo, ofsi, infom, -dInfinity);
	            //Jinfo.hi = hi + ofsi;
				dSetValue (hi, ofsi, infom, dInfinity);
	            //Jinfo.findex = findex + ofsi;
	            dSetValue(findex, ofsi, infom, -1);

				
				
				DxJoint joint = jointinfosA[jointinfosOfs+ji].joint;
				joint.getInfo2(stepsizeRecip, worldERP, Jinfo);

				//double[] rhs_row = Jinfo.c;
				for (int i = 0; i != infom; ++i) {
					//rhs_row[i] *= stepsizeRecip;
					//Jinfo.setC(i, Jinfo.getC(i) * stepsizeRecip);
					Jinfo.scaleC(i, stepsizeRecip);
				}

				// adjust returned findex values for global index numbering
				//int[] findex_row = Jinfo.findex;
				for (int j = infom; j != 0; ) {
					--j;
					int fival = Jinfo.getFindex(j);//findex_row[j];
					if (fival != -1) {
						//findex_row[j] = fival + ofsi;
						Jinfo.setFindex(j, fival + ofsi);
					}
				}
			}
		}
	}

	private static dThreadedCallFunction dxStepIsland_Stage2aSync_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			dxStepperStage2CallContext stage2CallContext = (dxStepperStage2CallContext )_stage2CallContext;
			final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
			DxWorld world = callContext.m_world();
			final int allowedThreads = callContext.m_stepperAllowedThreads();

			world.threading().AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads);
			world.threading().PostThreadedCallsGroup(null, allowedThreads, callThisReleasee, 
					dxStepIsland_Stage2b_Callback, stage2CallContext, "StepIsland Stage2b");

			return true;
		}
	};

	private static dThreadedCallFunction dxStepIsland_Stage2b_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxStepperStage2CallContext stage2CallContext = (dxStepperStage2CallContext )_stage2CallContext;
			dxStepIsland_Stage2b(stage2CallContext);
			return true;
		}
	};

	static 
	void dxStepIsland_Stage2b(dxStepperStage2CallContext stage2CallContext)
	{
		final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
		final dxStepperLocalContext localContext = stage2CallContext.m_localContext;
		dJointWithInfo1[] jointinfosA = localContext.m_jointinfosA;
		int jointinfosP = localContext.m_jointinfosOfs;
		int nj = localContext.m_nj;
		final int[] mindex = localContext.m_mindex;

		{
			// Warning!!!
			// This code depends on cfm elements and therefore must be in different sub-stage 
			// from Jacobian construction in Stage2a to ensure proper synchronization 
			// and avoid accessing numbers being modified.
			// Warning!!!
			final double stepsizeRecip = dRecip(callContext.m_stepSize());

			double[] A = localContext.m_A;
			final double[] cfm = stage2CallContext.m_rhs_tmp_or_cfm;
			final int m = localContext.m_m;

			final int mskip = dPAD(m);

			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_Ainit, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				double[] ArowA = A;
				int ArowP = mskip*ofsi;
				dSetZero(ArowA, ArowP, mskip*infom);
				double[] AdiagA = ArowA;
				int AdiagP = ArowP + ofsi;
				final double[] cfm_blockA = cfm;
				int cfm_blockP = ofsi;
				for (int i = 0; i != infom; AdiagP += mskip, ++i) {
					AdiagA[AdiagP+i] = cfm_blockA[cfm_blockP + i] * stepsizeRecip;
				}
			}
		}

		{
			// Warning!!!
			// This code depends on J elements and therefore must be in different sub-stage 
			// from Jacobian construction in Stage2a to ensure proper synchronization 
			// and avoid accessing numbers being modified.
			// Warning!!!
			double[] invI = localContext.m_invI;
			double[] J = localContext.m_J;
			double[] JinvM = stage2CallContext.m_JinvM;

			IFTIMING_dTimerNow ("compute JinvM");

			// compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
			// format as J so we just go through the constraints in J multiplying by
			// the appropriate scalars and matrices.
			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_JinvM, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				int Jdst = 2*8*ofsi;//JinvM + 2*8*ofsi;

				dSetZero(JinvM, Jdst, 2*8*infom);

				int Jsrc = 2*8*ofsi; //J + 2*8*ofsi;
				DxJoint joint = jointinfosA[jointinfosP+ji].joint;

				DxBody jb0 = joint.node[0].body;
				if (true) {// || jb0 != null) { // -- always true
					double body_invMass0 = jb0.invMass;
					int body_invI0 = jb0.tag*12; //invI + (int)jb0.tag*12;
					for (int j=infom; j>0;) {
						j--;
						//for (int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass0;
						for (int k=0; k<3; ++k) JinvM[Jdst+k] = J[Jsrc+k] * body_invMass0;
						dMultiply0_133 (JinvM,Jdst+4,J,Jsrc+4,invI,body_invI0);
						Jsrc += 8;
						Jdst += 8;
					}
				}

				DxBody jb1 = joint.node[1].body;
				if (jb1 != null) {
					double body_invMass1 = jb1.invMass;
					int body_invI1 = jb1.tag*12;//invI + (int)jb1.tag*12;
					for (int j=infom; j>0; ) {
						j--;
						//for (int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass1;
						for (int k=0; k<3; ++k) JinvM[Jdst+k] = J[Jsrc+k] * body_invMass1;
						dMultiply0_133 (JinvM,Jdst+4,J,Jsrc+4,invI,body_invI1);
						Jsrc += 8;
						Jdst += 8;
					}
				}
			}
		}

		{
			// Warning!!!
			// This code reads facc/tacc fields of body objects which (the fields)
			// may be modified by dxJoint::getInfo2(). Therefore the code must be
			// in different sub-stage from Jacobian construction in Stage2a 
			// to ensure proper synchronization and avoid accessing numbers being modified.
			// Warning!!!
			DxBody[] bodyA = callContext.m_islandBodiesStartA();
			int bodyP = callContext.m_islandBodiesStartOfs();
			final int nb = callContext.m_islandBodiesCount();
			final double[] invI = localContext.m_invI;
			double[] rhs_tmp = stage2CallContext.m_rhs_tmp_or_cfm;

			// compute the right hand side `rhs'
			IFTIMING_dTimerNow ("compute rhs_tmp");
			final double stepsizeRecip = dRecip(callContext.m_stepSize());

			// put v/h + invM*fe into rhs_tmp
			int bi;
			while ((bi = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_bi_rhs_tmp, nb)) != nb) {
				double[] tmp1currA = rhs_tmp;// + bi * 8;
				int tmp1currP = bi * 8;
				double[] invIrowA = invI;// + bi * 12;
				int invIrowP = bi * 12;
				DxBody b = bodyA[bodyP + bi];
				// dSetZero(tmp1curr, 8); -- not needed
				for (int j=0; j<3; ++j) tmp1currA[tmp1currP+j] = b.facc.get(j)*b.invMass + b.lvel.get(j)*stepsizeRecip;
				dMultiply0_331 (tmp1currA, tmp1currP+4, invIrowA, invIrowP, b.tacc);
				for (int k=0; k<3; ++k) tmp1currA[tmp1currP+4+k] += b.avel.get(k)*stepsizeRecip;
			}
		}
	}

	private static dThreadedCallFunction dxStepIsland_Stage2bSync_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, 
				int/*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			dxStepperStage2CallContext stage2CallContext = (dxStepperStage2CallContext)_stage2CallContext;
			final DxStepperProcessingCallContext callContext = stage2CallContext.m_stepperCallContext;
			DxWorld world = callContext.m_world();
			final int allowedThreads = callContext.m_stepperAllowedThreads();

			world.threading().AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads);
			world.threading().PostThreadedCallsGroup(null, allowedThreads, callThisReleasee, 
					dxStepIsland_Stage2c_Callback, stage2CallContext, "StepIsland Stage2c");

			return true;
		}
	};


	private static dThreadedCallFunction dxStepIsland_Stage2c_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage2CallContext, 
				int /*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxStepperStage2CallContext stage2CallContext = (dxStepperStage2CallContext)_stage2CallContext;
			dxStepIsland_Stage2c(stage2CallContext);
			return true;
		}
	};

	static 
	void dxStepIsland_Stage2c(dxStepperStage2CallContext stage2CallContext)
	{
		//const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
		final dxStepperLocalContext localContext = stage2CallContext.m_localContext;
		dJointWithInfo1[] jointinfosA = localContext.m_jointinfosA;
		int jointinfosP = localContext.m_jointinfosOfs;
		int nj = localContext.m_nj;
		final int[] mindex = localContext.m_mindex;

		{
			// Warning!!!
			// This code depends on A elements and JinvM elements and therefore 
			// must be in different sub-stage from A initialization and JinvM calculation in Stage2b 
			// to ensure proper synchronization and avoid accessing numbers being modified.
			// Warning!!!
			double[] A = localContext.m_A;
			final double[] JinvM = stage2CallContext.m_JinvM;
			double[] J = localContext.m_J;
			final int m = localContext.m_m;

			// now compute A = JinvM * J'. A's rows and columns are grouped by joint,
			// i.e. in the same way as the rows of J. block (i,j) of A is only nonzero
			// if joints i and j have at least one body in common. 
			final int mskip = dPAD(m);

			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_Aaddjb, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				double[] ArowA = A;// + (size_t)mskip*ofsi;
				int ArowP = mskip*ofsi;
				double[] JinvMrowA = JinvM;// + 2*8*(size_t)ofsi;
				int JinvMrowP = 2*8*ofsi;
				DxJoint joint = jointinfosA[jointinfosP+ji].joint;

				DxBody jb0 = joint.node[0].body;
				if (true) {// || jb0 != null) { // -- always true
					// compute diagonal block of A
					MultiplyAdd2_p8r (ArowA, ArowP + ofsi, JinvMrowA, JinvMrowP, 
							J, 2*8*ofsi, infom, infom, mskip);

					for (DxJointNode n0=(ji != 0 ? jb0.firstjoint.get() : null); n0!=null; n0=n0.next) {
						// if joint was tagged as -1 then it is an inactive (m=0 or disabled)
						// joint that should not be considered
						int j0 = n0.joint.tag;
						if (j0 != -1 && j0 < ji) {
							final int jiother_ofsi = mindex[j0];
							final int jiother_infom = mindex[j0 + 1] - jiother_ofsi;
							final dJointWithInfo1 jiother = jointinfosA[jointinfosP + j0];
							int ofsother = (jiother.joint.node[1].body == jb0) ? 8*jiother_infom : 0;
							// set block of A
							MultiplyAdd2_p8r (ArowA,ArowP + jiother_ofsi, JinvMrowA, JinvMrowP, 
									J, 2*8*jiother_ofsi + ofsother, infom, jiother_infom, mskip);
						}
					}
				}

				DxBody jb1 = joint.node[1].body;
				dIASSERT(jb1 != jb0);
				if (jb1!=null) {
					// compute diagonal block of A
					MultiplyAdd2_p8r (ArowA, ArowP + ofsi, JinvMrowA, JinvMrowP + 8*infom, 
							J, 2*8*ofsi + 8*infom, infom, infom, mskip);

					for (DxJointNode n1=(ji != 0 ? jb1.firstjoint.get() : null); n1!=null; n1=n1.next) {
						// if joint was tagged as -1 then it is an inactive (m=0 or disabled)
						// joint that should not be considered
						int j1 = n1.joint.tag;
						if (j1 != -1 && j1 < ji) {
							final int jiother_ofsi = mindex[j1];
							final int jiother_infom = mindex[j1 + 1] - jiother_ofsi;
							final dJointWithInfo1 jiother = jointinfosA[jointinfosP + j1];
							int ofsother = (jiother.joint.node[1].body == jb1) ? 8*jiother_infom : 0;
							// set block of A
							MultiplyAdd2_p8r (ArowA,ArowP + jiother_ofsi, JinvMrowA, JinvMrowP + 8*infom, 
									J, 2*8*jiother_ofsi + ofsother, infom, jiother_infom, mskip);
						}
					}
				}
			}
		}

		{
			// Warning!!!
			// This code depends on rhs_tmp elements and therefore must be in 
			// different sub-stage from rhs_tmp calculation in Stage2b to ensure 
			// proper synchronization and avoid accessing numbers being modified.
			// Warning!!!
			double[] J = localContext.m_J;
			double[] rhs_tmp = stage2CallContext.m_rhs_tmp_or_cfm;
			double[] rhs = localContext.m_rhs;

			// compute the right hand side `rhs'
			IFTIMING_dTimerNow ("compute rhs");

			// put J*rhs_tmp into rhs
			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_rhs, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				double[] rhscurrA = rhs;// + ofsi;
				int rhscurrP = ofsi;
				double[] JrowA = J;// + 2*8*(size_t)ofsi;
				int JrowP = 2*8*ofsi;

				DxJoint joint = jointinfosA[jointinfosP+ji].joint;

				DxBody jb0 = joint.node[0].body;
				if (true) {// || jb0 != null) { // -- always true
					MultiplySub0_p81 (rhscurrA, rhscurrP, JrowA, JrowP, rhs_tmp, 8*jb0.tag, infom);
				}

				DxBody jb1 = joint.node[1].body;
				if (jb1 != null) {
					MultiplySub0_p81 (rhscurrA, rhscurrP, JrowA, JrowP + 8*infom, rhs_tmp, 8*jb1.tag, infom);
				}
			}
		}
	}


	private static dThreadedCallFunction dxStepIsland_Stage3_Callback = new dThreadedCallFunction() {
		@Override
		public boolean run(CallContext _stage3CallContext, 
				int /*dcallindex_t*/ callInstanceIndex, DCallReleasee callThisReleasee)
		{
			//(void)callInstanceIndex; // unused
			//(void)callThisReleasee; // unused
			dxStepperStage3CallContext stage3CallContext = (dxStepperStage3CallContext)_stage3CallContext;
			dxStepIsland_Stage3(stage3CallContext);
			return true;
		}
	};

	static 
	void dxStepIsland_Stage3(dxStepperStage3CallContext stage3CallContext)
	{
		final DxStepperProcessingCallContext callContext = stage3CallContext.m_stepperCallContext;
		final dxStepperLocalContext localContext = stage3CallContext.m_localContext;

		DxWorldProcessMemArena memarena = callContext.m_stepperArena();
		memarena.RestoreState(stage3CallContext.m_stage1MemArenaState);
		stage3CallContext = null; // WARNING! stage3CallContext is not valid after this point!
		dIVERIFY(stage3CallContext == null); // To suppress unused variable assignment warnings

		double[] invI = localContext.m_invI;
		dJointWithInfo1[] jointinfosA = localContext.m_jointinfosA;
		int jointinfosP = localContext.m_jointinfosOfs;
		int nj = localContext.m_nj;
		int m = localContext.m_m;
		int nub = localContext.m_nub;
		//const unsigned int *mindex = localContext.m_mindex;
		int[] findex = localContext.m_findex;
		double[] lo = localContext.m_lo;
		double[] hi = localContext.m_hi;
		double[] J = localContext.m_J;
		double[] A = localContext.m_A;
		double[] rhs = localContext.m_rhs;

		//dxWorld *world = callContext->m_world;
		DxBody[] bodyA = callContext.m_islandBodiesStartA();
		int bodyP = callContext.m_islandBodiesStartOfs();
		int nb = callContext.m_islandBodiesCount();

		double[] lambda = null;

		if (m > 0) {
			//lambda = memarena->AllocateArray<dReal>(m);
			memarena.dummy();
			lambda = new double[m];


			BlockPointer lcpstate = memarena.BEGIN_STATE_SAVE();
			{
				IFTIMING_dTimerNow ("solving LCP problem");

				// solve the LCP problem and get lambda.
				// this will destroy A but that's OK
				DLCP.dSolveLCP (memarena, m, A, lambda, rhs, null, nub, lo, hi, findex);

			} 
			memarena.END_STATE_SAVE(lcpstate);
		}

		// this will be set to the force due to the constraints
		//dReal *cforce = memarena->AllocateArray<dReal>((size_t)nb * 8);
		memarena.dummy();
		double[] cforce = new double[nb * 8];
		//dSetZero (cforce,(size_t)nb*8);

		if (m > 0) {
			{
				IFTIMING_dTimerNow ("compute constraint force");

				// compute the constraint force `cforce'
				// compute cforce = J'*lambda
				int ofsi = 0;
				dJointWithInfo1 jicurrO = jointinfosA[jointinfosP];
				int jicurrP = 0;
				int jiend = jicurrP + nj;
				for (; jicurrP != jiend; ++jicurrP) {
					jicurrO = jointinfosA[jointinfosP+jicurrP];
					final int infom = jicurrO.info.m;
					DxJoint joint = jicurrO.joint;

					int JJ = 2*8*ofsi;//J + 
					int lambdarow = ofsi;//lambda + 

					DJointFeedback fb = joint.feedback;

					if (fb!=null) {
						// the user has requested feedback on the amount of force that this
						// joint is applying to the bodies. we use a slightly slower
						// computation that splits out the force components and puts them
						// in the feedback structure.
						double[] data=new double[8];
						Multiply1_8q1 (data, 0, J,JJ, lambda,lambdarow, infom);

						DxBody b1 = joint.node[0].body;
						int cf1 = 8*b1.tag;//cforce + 
						fb.f1.set(data[0],data[1],data[2]);
						fb.t1.set(data[4],data[5],data[6]);
						cforce[cf1+0] += data[0];
						cforce[cf1+1] += data[1];
						cforce[cf1+2] += data[2];
						cforce[cf1+4] += data[4];
						cforce[cf1+5] += data[5];
						cforce[cf1+6] += data[6]; 

						DxBody b2 = joint.node[1].body;
						if (b2!=null) {
							Multiply1_8q1 (data,0, J,JJ + 8*infom, lambda,lambdarow, infom);

							int cf2 = 8*b2.tag;//cforce + 
							fb.f2.set(data[0],data[1],data[2]);
							fb.t2.set(data[4],data[5],data[6]);
							cforce[cf2+0] += data[0];
							cforce[cf2+1] += data[1];
							cforce[cf2+2] += data[2];
							cforce[cf2+4] += data[4];
							cforce[cf2+5] += data[5];
							cforce[cf2+6] += data[6]; 
						}
					}
					else {
						// no feedback is required, let's compute cforce the faster way
						DxBody b1 = joint.node[0].body;
						int cf1 = 8*b1.tag;//cforce + 
						MultiplyAdd1_8q1 (cforce,cf1, J,JJ, lambda,lambdarow, infom);

						DxBody b2 = joint.node[1].body;
						if (b2!=null) {
							int cf2 = 8*b2.tag;//cforce + 
							MultiplyAdd1_8q1 (cforce, cf2, J,JJ + 8*infom, lambda,lambdarow, infom);
						}
					}

					ofsi += infom;
				}
			}
		} // if (m > 0)

		{
			// compute the velocity update
			IFTIMING_dTimerNow ("compute velocity update");

			double stepsize = callContext.m_stepSize();

			// add fe to cforce and multiply cforce by stepsize
			double[] data = new double[4];
			int invIrowP = 0;//invI;
			int cforcecurrP = 0;//cforce;
			//        dxBody *const *const bodyend = body + nb;
			//        for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow+=12, cforcecurr+=8, ++bodycurr) {
			//          dxBody *b = *bodycurr;
			for (int ii = bodyP; ii < bodyP+nb; ii++, invIrowP+=12, cforcecurrP+=8) {
				DxBody b = bodyA[ii];

				double body_invMass_mul_stepsize = stepsize * b.invMass;
				//for (int j=0; j<3; ++j) b.lvel[j] += (cforcecurr[j] + b.facc[j]) * body_invMass_mul_stepsize;
				b.lvel.add0( (cforce[cforcecurrP+0] + b.facc.get0()) * body_invMass_mul_stepsize);
				b.lvel.add1( (cforce[cforcecurrP+1] + b.facc.get1()) * body_invMass_mul_stepsize);
				b.lvel.add2( (cforce[cforcecurrP+2] + b.facc.get2()) * body_invMass_mul_stepsize);

				//for (int k=0; k<3; ++k) data[k] = (cforcecurr[4+k] + b.tacc[k]) * stepsize;
				data[0] = (cforce[cforcecurrP+4+0] + b.tacc.get0()) * stepsize;
				data[1] = (cforce[cforcecurrP+4+1] + b.tacc.get1()) * stepsize;
				data[2] = (cforce[cforcecurrP+4+2] + b.tacc.get2()) * stepsize;
				dMultiplyAdd0_331 (b.avel, invI, invIrowP, data, 0);
			}
		}

		{
			// update the position and orientation from the new linear/angular velocity
			// (over the given timestep)
			IFTIMING_dTimerNow ("update position");

			double stepsize = callContext.m_stepSize();

			//            dxBody *const *const bodyend = body + nb;
			//            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
			//                dxBody *b = *bodycurr;
			for (int ii = bodyP; ii < bodyP+nb; ii++) {
				DxBody b = bodyA[ii];
				b.dxStepBody (stepsize);
			}
		}

		{
			IFTIMING_dTimerNow ("tidy up");

			// zero all force accumulators
			//            dxBody *const *const bodyend = body + nb;
			//            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
			//                DxBody b = *bodycurr;
			for (int ii = bodyP; ii < bodyP+nb; ii++) {
				DxBody b = bodyA[ii];
				//          b->facc[0] = 0;
				//          b->facc[1] = 0;
				//          b->facc[2] = 0;
				//          b->facc[3] = 0;
				//          b->tacc[0] = 0;
				//          b->tacc[1] = 0;
				//          b->tacc[2] = 0;
				//          b->tacc[3] = 0;
				b.facc.setZero();
				b.tacc.setZero();
			}
		}

		IFTIMING_dTimerEnd();
		if (m > 0) IFTIMING_dTimerReport (stdout,1);

	}

	//****************************************************************************

	int dxEstimateStepMemoryRequirements (DxBody[] body, int nb, DxJoint[] _joint, int _nj)
	{
		//		(void)body; // unused
		//      int nj, m;
		//
		//      {
		//        int njcurr = 0, mcurr = 0;
		//        DxJoint.SureMaxInfo info;
		//        DxJoint *const *const _jend = _joint + _nj;
		//        for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; ++_jcurr) {  
		//          dxJoint *j = *_jcurr;
		//          j.getSureMaxInfo (info);
		//
		//          int jm = info.max_m;
		//          if (jm > 0) {
		//            njcurr++;
		//
		//            mcurr += jm;
		//          }
		//        }
		//        nj = njcurr; m = mcurr;
		//      }
		//
		//      size_t res = 0;
		//
		//	    res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb); // for invI
		//
		//	    {
		//	        size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * 2 * _nj); // for initial jointinfos
		//
		//	        // The array can't grow right more than by nj
		//	        size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * ((size_t)_nj + (size_t)nj)); // for shrunk jointinfos
		//	        sub1_res2 += dEFFICIENT_SIZE(sizeof(dxStepperLocalContext)); //for dxStepperLocalContext
		//	        if (m > 0) {
		//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(unsigned int) * (nj + 1)); // for mindex
		//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * m); // for J
		//	            unsigned int mskip = dPAD(m);
		//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * mskip * m); // for A
		//	            sub1_res2 += 3 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for lo, hi, rhs
		//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
		//	            {
		//	                size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dxStepperStage3CallContext)); //for dxStepperStage3CallContext
		//	                sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * m); // for JinvM
		//	                sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * dMAX((size_t)m, (size_t)nb * 8)); // for cfm and rhs_tmp
		//	                sub2_res1 += dEFFICIENT_SIZE(sizeof(dxStepperStage2CallContext)); // for dxStepperStage2CallContext
		//
		//	                size_t sub2_res2 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda
		//	                {
		//	                    size_t sub3_res1 = dEstimateSolveLCPMemoryReq(m, false);
		//
		//	                    size_t sub3_res2 = dEFFICIENT_SIZE(sizeof(dReal) * 8 * nb); // for cforce
		//
		//	                    sub2_res2 += dMAX(sub3_res1, sub3_res2);
		//	                }
		//
		//	                sub1_res2 += dMAX(sub2_res1, sub2_res2);
		//	            }
		//	        }
		//	        else {
		//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(dxStepperStage3CallContext)); // for dxStepperStage3CallContext
		//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 8 * nb); // for cforce
		//	        }
		//
		//	        size_t sub1_res12_max = dMAX(sub1_res1, sub1_res2);
		//	        size_t stage01_contexts = dEFFICIENT_SIZE(sizeof(dxStepperStage0BodiesCallContext))
		//	            + dEFFICIENT_SIZE(sizeof(dxStepperStage0JointsCallContext))
		//	            + dEFFICIENT_SIZE(sizeof(dxStepperStage1CallContext));
		//	        res += dMAX(sub1_res12_max, stage01_contexts);
		//      }
		//
		//      return res;
		return -1;
	}

	/*extern */
	private int dxEstimateStepMaxCallCount(
			int /*activeThreadCount*/ activeThreadCount, int allowedThreadCount)
	{
		int result = 1 // dxStepIsland itself
				+ (2 * allowedThreadCount + 2) // (dxStepIsland_Stage2a + dxStepIsland_Stage2b) * allowedThreadCount + 2 * dxStepIsland_Stage2?_Sync
				+ 1; // dxStepIsland_Stage3
		return result;
	}
	@Override
	public int run(int activeThreadCount, int allowedThreadCount) {
		return dxEstimateStepMaxCallCount(activeThreadCount, allowedThreadCount);
	}

	
	@Override
	public int dxEstimateMemoryRequirements(DxBody[] body, int bodyOfs, int nb,
			DxJoint[] _joint, int jointOfs, int _nj) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void run(DxStepperProcessingCallContext callContext) {
		dxStepIsland(callContext);
	}
}

