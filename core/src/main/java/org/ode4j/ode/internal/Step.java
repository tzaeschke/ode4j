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
import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.CommonEnums.*;
import static org.ode4j.ode.internal.DLCP.*;
import static org.ode4j.ode.internal.StepEnums.*;
import static org.ode4j.ode.internal.Timer.dTimerEnd;
import static org.ode4j.ode.internal.Timer.dTimerNow;
import static org.ode4j.ode.internal.Timer.dTimerReport;
import static org.ode4j.ode.internal.Timer.dTimerStart;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stdout;
import static org.ode4j.ode.internal.joints.JointEnums.*;
import static org.ode4j.ode.internal.processmem.DxUtil.EFFICIENT_ALIGNMENT;
import static org.ode4j.ode.threading.Atomics.*;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicIntegerArray;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DJoint.DJointFeedback;
import org.ode4j.ode.internal.cpp4j.FILE;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointNode;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dmaxcallcountestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxStepperProcessingCallContext.dstepper_fn_t;
import org.ode4j.ode.internal.processmem.DxUtil.BlockPointer;
import org.ode4j.ode.internal.processmem.DxWorldProcessIslandsInfo.dmemestimate_fn_t;
import org.ode4j.ode.internal.processmem.DxWorldProcessMemArena;

class Step extends AbstractStepper implements dstepper_fn_t, dmemestimate_fn_t, 
dmaxcallcountestimate_fn_t {

	public static final Step INSTANCE = new Step();

	//#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
	//#define dMAX(A,B)  ((B)>(A) ? (B) : (A))
	//TX: not used
	//private static final int dMIN(int A, int B) { return ((A)>(B) ? (B) : (A)); }
	private static int dMAX(int A, int B) { return ((B)>(A) ? (B) : (A)); }


	//****************************************************************************
	// misc defines

	//#define TIMING  /was commented out!
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
	private static void IFTIMING_dTimerReport (FILE fout, int average) {
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

	// #define AMATRIX_ALIGNMENT   dMAX(64, EFFICIENT_ALIGNMENT)
	// #define INVI_ALIGNMENT      dMAX(32, EFFICIENT_ALIGNMENT)
	// #define JINVM_ALIGNMENT     dMAX(64, EFFICIENT_ALIGNMENT)
	private static final int AMATRIX_ALIGNMENT = dMAX(64, EFFICIENT_ALIGNMENT);
	// private static final int INVI_ALIGNMENT    = dMAX(32, EFFICIENT_ALIGNMENT);
	private static final int JINVM_ALIGNMENT   = dMAX(64, EFFICIENT_ALIGNMENT);

	private static class dxStepperStage0Outputs
	{
		int                          ji_start;
		int                          ji_end;
		int                    m;
		int                    nub;
	}

	private static class dxStepperStage1CallContext
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

	private static class dxStepperStage0BodiesCallContext
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

	private static class dxStepperStage0JointsCallContext
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


	private static class dxStepperLocalContext {
		void Initialize(double[] invI, dJointWithInfo1[] jointinfosA,
						int jointinfosOfs,
						int nj,
						int m, int nub, final int[] mindex, int[] findex,
						double[] J, double[] A, double[] pairsRhsCfm, double[] pairsLoHi,
						AtomicIntegerArray bodyStartJoints, AtomicIntegerArray bodyJointLinks) {
			m_invI = invI;
			m_jointinfosA = jointinfosA;
			m_jointinfosOfs = jointinfosOfs;
			m_nj = nj;
			m_m = m;
			m_nub = nub;
			m_mindex = mindex;
			m_findex = findex;
			m_J = J;
			m_A = A;
			m_pairsRhsCfm = pairsRhsCfm;
			m_pairsLoHi = pairsLoHi;
			m_bodyStartJoints = bodyStartJoints;
			m_bodyJointLinks = bodyJointLinks;
		}

		double[]                           m_invI;
		dJointWithInfo1[]                 m_jointinfosA;
		int				                 m_jointinfosOfs;
		int                    m_nj;
		int                    m_m;
		int                    m_nub;
		int[]              m_mindex;
		int[]                              m_findex;
		double[]                           m_J;
		double[]                           m_A;
		double[]                           m_pairsRhsCfm;
		double[]                           m_pairsLoHi;
		AtomicIntegerArray                 m_bodyStartJoints;
		AtomicIntegerArray                 m_bodyJointLinks;
	}

	private static class dxStepperStage2CallContext
	{
		void Initialize(final DxStepperProcessingCallContext callContext, 
				final dxStepperLocalContext localContext, 
				double[] JinvM, double[] rhs_tmp)
		{
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			m_JinvM = JinvM;
			m_rhs_tmp = rhs_tmp;
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
		double[]                        m_rhs_tmp;
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

	private static class dxStepperStage3CallContext
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

	private static class dxStepperStage4CallContext
	{
		void Initialize(final DxStepperProcessingCallContext callContext,
						final dxStepperLocalContext localContext /*,
						BlockPointer stage1MemArenaState*/)
		{
			m_stepperCallContext = callContext;
			m_localContext = localContext;
			// m_stage1MemArenaState = stage1MemArenaState;
			m_bi_constrForce.set(0);
		}

		DxStepperProcessingCallContext m_stepperCallContext;
		dxStepperLocalContext     m_localContext;
		//BlockPointer                           m_stage1MemArenaState;
		final AtomicInteger m_bi_constrForce = new AtomicInteger();
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

	//static int dxStepIsland_Stage4_Callback(void *_stage4CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
	//static void dxStepIsland_Stage4(dxStepperStage4CallContext *stage4CallContext);


	//****************************************************************************
	// special matrix multipliers

	// this assumes the 4th and 8th rows of B and C are zero.

	private static void MultiplyAddJinvMxJToA(double[] ArowA, final int ArowOfs,
											  final double[] JinvMRowA, final int JinvMRowOfs,
											  final double[] JRowA, final int JRowOfs,
											  int infomJinvM, int infomJ, int mskip) {
		dIASSERT(infomJinvM > 0 && infomJ > 0); // && Arow && JinvMRow && JRow);
		final int mskip_munus_infomJ_plus_1 = mskip - infomJ + 1;
		dIASSERT(mskip >= infomJ);
		int currA = ArowOfs;
		int currJinvM = JinvMRowOfs;
		for (int i = infomJinvM; ; ) {
			double JiM0 = JinvMRowA[currJinvM + JIM_LX];
			double JiM1 = JinvMRowA[currJinvM + JIM_LY];
			double JiM2 = JinvMRowA[currJinvM + JIM_LZ];
			double JiM4 = JinvMRowA[currJinvM + JIM_AX];
			double JiM5 = JinvMRowA[currJinvM + JIM_AY];
			double JiM6 = JinvMRowA[currJinvM + JIM_AZ];
			int currJ = JRowOfs;
			for (int j = infomJ; ; ) {
				double sum = 0;
				sum = JiM0 * JRowA[currJ + JME_JLX];
				sum += JiM1 * JRowA[currJ + JME_JLY];
				sum += JiM2 * JRowA[currJ + JME_JLZ];
				sum += JiM4 * JRowA[currJ + JME_JAX];
				sum += JiM5 * JRowA[currJ + JME_JAY];
				sum += JiM6 * JRowA[currJ + JME_JAZ];
				ArowA[currA] += sum;//*currA += sum;
				if (--j == 0) {
					break;
				}
				++currA;
				currJ += JME__MAX;
			}
			if (--i == 0) {
				break;
			}
			currJinvM += JIM__MAX;
			currA += mskip_munus_infomJ_plus_1;
		}
	}


	// this assumes the 4th and 8th rows of B are zero.

	private static
	void MultiplySubJxRhsTmpFromRHS (double[] rowRhsCfmA, int rowRhsCfmOfs,
									 final double[] JRowA, int JRowOfs,
									 final double[] rowRhsTmpA, final int rowRhsTmpOfs,
									 int infom)
	{
		dIASSERT (infom > 0 );// && rowRhsCfm && JRow && rowRhsTmp);
		int currRhs = rowRhsCfmOfs + RCE_RHS; //dReal *currRhs = rowRhsCfm + RCE_RHS;
    	int currJ = JRowOfs; //const dReal *currJ = JRow;
    	final double RT_LX = rowRhsTmpA[rowRhsTmpOfs + dDA_LX], RT_LY = rowRhsTmpA[rowRhsTmpOfs + dDA_LY], RT_LZ = rowRhsTmpA[rowRhsTmpOfs + dDA_LZ];
		final double RT_AX = rowRhsTmpA[rowRhsTmpOfs + dDA_AX], RT_AY = rowRhsTmpA[rowRhsTmpOfs + dDA_AY], RT_AZ = rowRhsTmpA[rowRhsTmpOfs + dDA_AZ];
		for (int i = infom; ; ) {
		double sum = 0;
		sum  = JRowA[currJ + JME_JLX] * RT_LX;
		sum += JRowA[currJ + JME_JLY] * RT_LY;
		sum += JRowA[currJ + JME_JLZ] * RT_LZ;
		sum += JRowA[currJ + JME_JAX] * RT_AX;
		sum += JRowA[currJ + JME_JAY] * RT_AY;
		sum += JRowA[currJ + JME_JAZ] * RT_AZ;
        rowRhsCfmA[currRhs] -= sum; //*currRhs -= sum;
		if (--i == 0) {
			break;
		}
		currRhs += RCE__RHS_CFM_MAX;
		currJ += JME__MAX;
	}
	}


	private static
	void MultiplyAddJxLambdaToCForce(double[] cforce,
    final double[] JRowA, int JRowOfs, final double[] rowRhsLambdaA, final int rowRhsLambdaOfs, int infom,
									 DJointFeedback fb/*=NULL*/, int jointBodyIndex)
	{
		dIASSERT (infom > 0); // && cforce && JRow && rowRhsLambda);
		double sumLX = 0, sumLY = 0, sumLZ = 0, sumAX=0, sumAY = 0, sumAZ = 0;
    	int currJ = JRowOfs;
    	int currLambda = rowRhsLambdaOfs + RLE_LAMBDA;
		for (int k = infom; ; ) {
        final double lambda = rowRhsLambdaA[currLambda];
		sumLX += JRowA[currJ + JME_JLX] * lambda;
		sumLY += JRowA[currJ + JME_JLY] * lambda;
		sumLZ += JRowA[currJ + JME_JLZ] * lambda;
		sumAX += JRowA[currJ + JME_JAX] * lambda;
		sumAY += JRowA[currJ + JME_JAY] * lambda;
		sumAZ += JRowA[currJ + JME_JAZ] * lambda;
		if (--k == 0) {
			break;
		}
		currJ += JME__MAX;
		currLambda += RLE__RHS_LAMBDA_MAX;
	}
		if (fb != null) {
			if (jointBodyIndex == dJCB__MIN) {
//				fb.f1[dV3E_X] = sumLX;
//				fb.f1[dV3E_Y] = sumLY;
//				fb.f1[dV3E_Z] = sumLZ;
//				fb.t1[dV3E_X] = sumAX;
//				fb.t1[dV3E_Y] = sumAY;
//				fb.t1[dV3E_Z] = sumAZ;
				dAssertVec3Element();
				fb.f1.set(sumLX, sumLY, sumLZ);
				fb.t1.set(sumAX, sumAY, sumAZ);
			}
			else {
				dIASSERT(jointBodyIndex == dJCB__MIN + 1);
				dSASSERT(dJCB__MAX == 2);

//				fb.f2[dV3E_X] = sumLX;
//				fb.f2[dV3E_Y] = sumLY;
//				fb.f2[dV3E_Z] = sumLZ;
//				fb.t2[dV3E_X] = sumAX;
//				fb.t2[dV3E_Y] = sumAY;
//				fb.t2[dV3E_Z] = sumAZ;
				dAssertVec3Element();
				fb.f2.set(sumLX, sumLY, sumLZ);
				fb.t2.set(sumAX, sumAY, sumAZ);
			}
		}
		cforce[CFE_LX] += sumLX;
		cforce[CFE_LY] += sumLY;
		cforce[CFE_LZ] += sumLZ;
		cforce[CFE_AX] += sumAX;
		cforce[CFE_AY] += sumAY;
		cforce[CFE_AZ] += sumAZ;
	}

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


//	// this assumes the 4th and 8th rows of B and C are zero.
//
//	private static void MultiplyAdd2_p8r (double[] A, int APos,
//			double[] B, int BPos, double[] C, int CPos,
//			int p, int r, int Askip)
//	{
//		dIASSERT (p>0 && r>0);
//		//		dAASSERT(A, B, C);
//		final int Askip_munus_r = Askip - r;
//		dIASSERT(Askip >= r);
//		int aa = APos;//  dReal *aa = A;
//		int bb = BPos;//const dReal *bb = B;
//		for (int i = p; i != 0; --i) {
//			int cc = CPos;//const dReal *cc = C;
//			for (int j = r; j != 0; --j) {
//				double sum;
//				sum  = B[bb] * C[cc]; //sum = bb[0]*cc[0];
//				sum += B[bb+1] * C[cc+1]; //sum += bb[1]*cc[1];
//				sum += B[bb+2] * C[cc+2]; //sum += bb[2]*cc[2];
//				sum += B[bb+4] * C[cc+4]; //sum += bb[4]*cc[4];
//				sum += B[bb+5] * C[cc+5]; //sum += bb[5]*cc[5];
//				sum += B[bb+6] * C[cc+6]; //sum += bb[6]*cc[6];
//				A[aa++] += sum;//*(aa++) += sum;
//				cc += 8;
//			}
//			bb += 8;
//			aa += Askip_munus_r;
//		}
//	}
//
//
//	// this assumes the 4th and 8th rows of B are zero.
//
//	private static void MultiplySub0_p81 (double[] A, final int APos,
//			double[] B, final int BPos, double[] C, final int CPos, int p)
//	{
//		dIASSERT (p>0);
//		//		dAASSERT(A, B, C);
//		int aa = APos;//dReal *aa = A;
//		int bb = BPos;//const dReal *bb = B;
//		final double C_0 = C[0+CPos], C_1 = C[1+CPos], C_2 = C[2+CPos];
//		final double C_4 = C[4+CPos], C_5 = C[5+CPos], C_6 = C[6+CPos];
//		for (int i = p; i != 0; --i) {
//			double sum;
//			sum  = B[0+bb]*C_0;
//			sum += B[1+bb]*C_1;
//			sum += B[2+bb]*C_2;
//			sum += B[4+bb]*C_4;
//			sum += B[5+bb]*C_5;
//			sum += B[6+bb]*C_6;
//			A[aa++] -= sum;//*(aa++) = sum;
//			bb += 8;//B += 8;
//		}
//	}
//
//
//	// this assumes the 4th and 8th rows of B are zero.
//
//	private static void MultiplyAdd1_8q1 (double[] A, int APos,
//			double[] B, int BPos, double[] C, int CPos, int q)
//	{
//		dIASSERT (q>0);
//		//	    dAASSERT(A, B, C);
//		int bb = BPos;//const dReal *bb = B;
//		double sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
//		for (int k = 0; k < q; ++k) {
//			final double C_k = C[k+CPos];
//			sum0 += B[bb+0] * C_k;
//			sum1 += B[bb+1] * C_k;
//			sum2 += B[bb+2] * C_k;
//			sum4 += B[bb+4] * C_k;
//			sum5 += B[bb+5] * C_k;
//			sum6 += B[bb+6] * C_k;
//			bb += 8;
//		}
//		A[0+APos] += sum0;
//		A[1+APos] += sum1;
//		A[2+APos] += sum2;
//		A[4+APos] += sum4;
//		A[5+APos] += sum5;
//		A[6+APos] += sum6;
//	}
//
//
//	// this assumes the 4th and 8th rows of B are zero.
//
//	private static void Multiply1_8q1 (double[] A, final int APos,
//			double[] B, final int BPos, double[] C, final int CPos, int q)
//	{
//		int bb = BPos;//const dReal *bb = B;
//		double sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
//		for (int k = 0; k < q; ++k) {
//			final double C_k = C[k+CPos];
//			sum0 += B[bb+0] * C_k;
//			sum1 += B[bb+1] * C_k;
//			sum2 += B[bb+2] * C_k;
//			sum4 += B[bb+4] * C_k;
//			sum5 += B[bb+5] * C_k;
//			sum6 += B[bb+6] * C_k;
//			bb += 8;
//		}
//		A[0+APos] = sum0;
//		A[1+APos] = sum1;
//		A[2+APos] = sum2;
//		A[4+APos] = sum4;
//		A[5+APos] = sum5;
//		A[6+APos] = sum6;
//	}

	//****************************************************************************

	/*extern */
	private void dxStepIsland(final DxStepperProcessingCallContext callContext)
	{
		IFTIMING_dTimerStart("preprocessing");

		DxWorldProcessMemArena memarena = callContext.m_stepperArena();
		// DxWorld world = callContext.m_world();
		int nb = callContext.m_islandBodiesCount();
		int _nj = callContext.m_islandJointsCount();

		memarena.dummy();
		double[] invI = new double[dM3E__MAX * nb];//memarena.AllocateArray<dReal> (3*4*(size_t)nb);
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

//			dCallReleaseeID stage1CallReleasee;
//			world->PostThreadedCallForUnawareReleasee(NULL, &stage1CallReleasee, bodyThreads + jointThreads, callContext->m_finalReleasee,
//					NULL, &dxStepIsland_Stage1_Callback, stage1CallContext, 0, "StepIsland Stage1");
//
//			world->PostThreadedCallsGroup(NULL, bodyThreads, stage1CallReleasee, &dxStepIsland_Stage0_Bodies_Callback, stage0BodiesCallContext, "StepIsland Stage0-Bodies");
//
//			dxStepIsland_Stage0_Joints(stage0JointsCallContext);
//			world->AlterThreadedCallDependenciesCount(stage1CallReleasee, -1);
//			dIASSERT(jointThreads == 1);
			throw new UnsupportedOperationException("The stepper does not support multi-threading");
			// TODO CHECK-TZ
			// TODO Or does it? Remove Exception and see what happens
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
						b.facc.add(dV3E_X, b.mass._mass * gravity_x);
					}
				}
			}
			double gravity_y = world.gravity.get1();
			if (gravity_y!=0) {
				for (int ii = bOfs; ii < bOfs+nb; ii++) {
					DxBody b = bodyA[ii];
					if (b.getGravityMode()) {
						b.facc.add(dV3E_Y, b.mass._mass * gravity_y);
					}
				}
			}
			double gravity_z = world.gravity.get2();
			if (gravity_z!=0) {
				for (int ii = bOfs; ii < bOfs+nb; ii++) {
					DxBody b = bodyA[ii];
					if (b.getGravityMode()) {
						b.facc.add(dV3E_Z, b.mass._mass * gravity_z);
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

			for (int i = 0; i != nb; invIrowO += dM3E__MAX, ++i) {
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
						dAssertVec3Element();
						dSetCrossMatrixMinus(Itild,L);//,dV3E__MAX);
						//for (int ii=dV3E__MIN;ii<dV3E__MAX;++ii) {
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
							//Itild[dM3E_XX]-=1; Itild[dM3E_YY]-=1; Itild[dM3E_ZZ]-=1;
							dAssertVec3Element();
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
							//for (int ii=dSA__MIN;ii<dSA__MAX;++ii) {
							//	b.tacc[dV3E__AXES_MIN + ii]+=tau0[dV3E__AXES_MIN + ii];
							//}
							dAssertVec3Element();
							b.tacc.add(tau0);
						}
						//#endif  //if 0
					}

					bodyIndex = ThrsafeIncrementIntUpToLimit(callContext.m_inertiaBodyIndex, nb);
				}
			}
		}
	}

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

		dJointWithInfo1[] jointinfosA = _jointinfosA;// + ji_start;
		int jiP = _jointinfosOfs + ji_start;
		int nj = ji_end - ji_start;
		//dIASSERT((size_t)(ji_end - ji_start) <= (size_t)UINT_MAX);

		int[] mindex = null;
		double[] J = null, A = null, pairsRhsCfm = null, pairsLoHi = null;
		int[] findex = null;
		//atomicord32 *bodyStartJoints = NULL, *bodyJointLinks = NULL;
		AtomicIntegerArray bodyStartJoints = null, bodyJointLinks = null;


		// if there are constraints, compute constrForce
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

			// create a constraint equation right hand side vector `c', a constraint
			// force mixing vector `cfm', and LCP low and high bound vectors, and an
			// 'findex' vector.
			memarena.dummy();  // Well, dummy not really needed because we use the real thing...
			findex = memarena.AllocateArrayInt(m);
			J = memarena.AllocateArrayDReal(m * (2 * JME__MAX));
			A = memarena.AllocateOveralignedArrayDReal(m * dPAD(m), AMATRIX_ALIGNMENT);
			pairsRhsCfm = memarena.AllocateArrayDReal(m * RCE__RHS_CFM_MAX);
			pairsLoHi = memarena.AllocateArrayDReal(m * LHE__LO_HI_MAX);
	        final int nb = callContext.m_islandBodiesCount();
			bodyStartJoints = memarena.AllocateArrayAtomicord32(nb);
			bodyJointLinks = memarena.AllocateArrayAtomicord32(nj * dJCB__MAX);
			//dICHECK(nj < ~((atomicord32)0) / dJCB__MAX); // If larger joint counts are to be used, pointers (or size_t) need to be stored rather than atomicord32 indices
			dICHECK(nj < Integer.MAX_VALUE / dJCB__MAX); // If larger joint counts are to be used, pointers (or size_t) need to be stored rather than atomicord32 indices
		}

		//dxStepperLocalContext *localContext = (dxStepperLocalContext *)memarena->AllocateBlock(sizeof(dxStepperLocalContext));
		memarena.dummy();
		dxStepperLocalContext localContext = new dxStepperLocalContext();
		localContext.Initialize(invI, jointinfosA, jiP, nj, m, nub, mindex, findex, J, A, pairsRhsCfm, pairsLoHi, bodyStartJoints, bodyJointLinks);

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
			double[] JinvM = memarena.AllocateOveralignedArrayDReal(m * (2 * JIM__MAX), JINVM_ALIGNMENT);
			final int nb = callContext.m_islandBodiesCount();
			double[] rhs_tmp = memarena.AllocateArrayDReal(nb * dDA__MAX);

			//dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage2CallContext));
			memarena.dummy();
			dxStepperStage2CallContext stage2CallContext = new dxStepperStage2CallContext();
			stage2CallContext.Initialize(callContext, localContext, JinvM, rhs_tmp);

			int allowedThreads = callContext.m_stepperAllowedThreads();
			dIASSERT(allowedThreads != 0);

			if (allowedThreads == 1) {
				IFTIMING_dTimerNow("create J");
				dxStepIsland_Stage2a(stage2CallContext);
				IFTIMING_dTimerNow("compute Adiag, JinvM and rhs_tmp");
				dxStepIsland_Stage2b(stage2CallContext);
				IFTIMING_dTimerNow("compute A and rhs");
				dxStepIsland_Stage2c(stage2CallContext);
				dxStepIsland_Stage3(stage3CallContext);
			} else {
//				dxWorld *world = callContext->m_world;
//				dCallReleaseeID stage3CallReleasee;
//				world->PostThreadedCallForUnawareReleasee(NULL, &stage3CallReleasee, 1, callContext->m_finalReleasee,
//						NULL, &dxStepIsland_Stage3_Callback, stage3CallContext, 0, "StepIsland Stage3");
//
//				dCallReleaseeID stage2bSyncReleasee;
//				world->PostThreadedCall(NULL, &stage2bSyncReleasee, 1, stage3CallReleasee,
//						NULL, &dxStepIsland_Stage2bSync_Callback, stage2CallContext, 0, "StepIsland Stage2b Sync");
//
//				dCallReleaseeID stage2aSyncReleasee;
//				world->PostThreadedCall(NULL, &stage2aSyncReleasee, allowedThreads, stage2bSyncReleasee,
//						NULL, &dxStepIsland_Stage2aSync_Callback, stage2CallContext, 0, "StepIsland Stage2a Sync");
//
//				dIASSERT(allowedThreads > 1); /*if (allowedThreads > 1) */{
//				world->PostThreadedCallsGroup(NULL, allowedThreads - 1, stage2aSyncReleasee, &dxStepIsland_Stage2a_Callback, stage2CallContext, "StepIsland Stage2a");
//			}
//				dxStepIsland_Stage2a(stage2CallContext);
//				world->AlterThreadedCallDependenciesCount(stage2aSyncReleasee, -1);
				throw new UnsupportedOperationException();
			}
		}
		else {
			dxStepIsland_Stage3(stage3CallContext);
		}
	}


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
			final int[] findex = localContext.m_findex;
			final double[] J = localContext.m_J;
			final double[] pairsRhsCfm = localContext.m_pairsRhsCfm;
			final double[] pairsLoHi = localContext.m_pairsLoHi;
			final int JOfs = 0, pairsRhsCfmOfs = 0, pairsLoHiOfs = 0;

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
       		final double worldCFM = world.global_cfm;

			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_J, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				int JRowOfs = JOfs + ofsi * (2 * JME__MAX);
				int rowRhsCfmOfs = pairsRhsCfmOfs + ofsi * RCE__RHS_CFM_MAX;
				int rowLoHiOfs = pairsLoHiOfs + ofsi * LHE__LO_HI_MAX;
				{
					dSetZero (J, JRowOfs, infom * (2 * JME__MAX));

					final int endRhsCfmOfs = rowRhsCfmOfs + infom * RCE__RHS_CFM_MAX;
					for (int currRhsCfmOfs = rowRhsCfmOfs; currRhsCfmOfs != endRhsCfmOfs; currRhsCfmOfs += RCE__RHS_CFM_MAX) {
						pairsRhsCfm[currRhsCfmOfs + RCE_RHS] = 0.0;
						pairsRhsCfm[currRhsCfmOfs + RCE_CFM] = worldCFM;
					}

					int endLoHiOfs = rowLoHiOfs + infom * LHE__LO_HI_MAX;
					for (int currLoHiOfs = rowLoHiOfs; currLoHiOfs != endLoHiOfs; currLoHiOfs += LHE__LO_HI_MAX) {
						pairsLoHi[currLoHiOfs + LHE_LO] = -dInfinity;
						pairsLoHi[currLoHiOfs + LHE_HI] = dInfinity;
					}
				}
				int findexRowOfs = ofsi; //findex + ofsi;
				dSetValue(findex, findexRowOfs, infom, -1);

				DxJoint joint = jointinfosA[jointinfosOfs + ji].joint;
				joint.getInfo2(stepsizeRecip, worldERP, JME__MAX, J, JRowOfs + JME__J_MIN, J, JRowOfs + infom * JME__MAX + JME__J_MIN, RCE__RHS_CFM_MAX,
						pairsRhsCfm, rowRhsCfmOfs, pairsLoHi, rowLoHiOfs, findex, findexRowOfs);
				dSASSERT(LHE__LO_HI_MAX == RCE__RHS_CFM_MAX); // To make sure same step fits for both pairs in the call to dxJoint::getInfo2() above

				// findex iteration is compact and is not going to pollute caches - do it first
				{
					// adjust returned findex values for global index numbering
					final int findicesEnd = findexRowOfs + infom;
					for (int findexCurr = findexRowOfs; findexCurr != findicesEnd; ++findexCurr) {
					int fival = findex[findexCurr]; //*findexCurr;
					if (fival != -1) {
						findex[findexCurr] = fival + ofsi; //*findexCurr = fival + ofsi;
					}
				}
				}
				{
					int endRhsCfmOfs = rowRhsCfmOfs + infom * RCE__RHS_CFM_MAX;
					for (int currRhsCfmOfs = rowRhsCfmOfs; currRhsCfmOfs != endRhsCfmOfs; currRhsCfmOfs += RCE__RHS_CFM_MAX) {
						pairsRhsCfm[currRhsCfmOfs + RCE_RHS] *= stepsizeRecip;
						pairsRhsCfm[currRhsCfmOfs + RCE_CFM] *= stepsizeRecip;
					}
				}
			}
		}
	}

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
			double[] A = localContext.m_A;
			final double[] pairsRhsCfm = localContext.m_pairsRhsCfm;
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
				final double[] rowRfsCrmA = pairsRhsCfm;
				final int rowRfsCrmP = /*pairsRhsCfm + */ofsi * RCE__RHS_CFM_MAX;
				for (int i = 0; i != infom; AdiagP += mskip, ++i) {
					AdiagA[AdiagP+i] = rowRfsCrmA[rowRfsCrmP + i * RCE__RHS_CFM_MAX + RCE_CFM];
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

			// compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
			// format as J so we just go through the constraints in J multiplying by
			// the appropriate scalars and matrices.
			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_JinvM, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				int Jdst = ofsi * (2 * JIM__MAX);//JinvM + ofsi * (2 * JIM__MAX);

				dSetZero(JinvM, Jdst, infom * (2 * JIM__MAX));

				int Jsrc = ofsi * (2 * JME__MAX); //J + ofsi * (2 * JME__MAX);
				DxJoint joint = jointinfosA[jointinfosP + ji].joint;

				DxBody jb0 = joint.node[0].body;
				if (true) {// || jb0 != null) { // -- always true
					double body_invMass0 = jb0.invMass;
					int body_invI0 = jb0.tag * dM3E__MAX; //invI + (int)jb0.tag*12;
					for (int j = infom; j != 0; --j) {
						//for (int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass0;
						for (int k = dSA__MIN; k != dSA__MAX; ++k)
							JinvM[Jdst + JIM__L_AXES_MIN + k] = J[Jsrc + JME__JL_MIN + k] * body_invMass0;
						// TODO CHECK-TZ The second used to 4 4, now it is three...?
						dMultiply0_133(JinvM, Jdst + JIM__A_AXES_MIN, J, Jsrc + JME__JA_MIN, invI, body_invI0);
						// TODO CHECK-TZ This used to be '8'
						Jsrc += JME__MAX;
						Jdst += JIM__MAX;
					}
				}

				DxBody jb1 = joint.node[1].body;
				if (jb1 != null) {
					double body_invMass1 = jb1.invMass;
					int body_invI1 = jb1.tag * dM3E__MAX; //invI + (size_t)(unsigned int)jb1->tag * dM3E__MAX;
					for (int j = infom; j != 0; --j) {
						for (int k = dSA__MIN; k != dSA__MAX; ++k)
							JinvM[Jdst + JIM__L_AXES_MIN + k] = J[Jsrc + JME__JL_MIN + k] * body_invMass1;
						dMultiply0_133(JinvM, Jdst + JIM__A_AXES_MIN, J, Jsrc + JME__JA_MIN, invI, body_invI1);
						Jsrc += JME__MAX;
						Jdst += JIM__MAX;
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
			AtomicIntegerArray bodyStartJoints = localContext.m_bodyStartJoints;
			double[] rhs_tmp = stage2CallContext.m_rhs_tmp;

			// compute the right hand side `rhs'
			final double stepsizeRecip = dRecip(callContext.m_stepSize());

			// put v/h + invM*fe into rhs_tmp
			int bi;
			while ((bi = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_bi_rhs_tmp, nb)) != nb) {
				double[] tmp1currA = rhs_tmp;// + bi * 8;
				// TODO CHECK-TZ This used to be 8
				int tmp1currP = bi * dDA__MAX;
				double[] invIrowA = invI;// + bi * 12;
				int invIrowP = bi * dM3E__MAX;
				DxBody b = bodyA[bodyP + bi];
				// dSetZero(tmp1curr, 8); -- not needed
				for (int j = dSA__MIN; j != dSA__MAX; ++j)
					tmp1currA[tmp1currP + dDA__L_MIN + j] =
							b.facc.get(dV3E__AXES_MIN + j) * b.invMass +
									b.lvel.get(dV3E__AXES_MIN + j) * stepsizeRecip;
				dMultiply0_331(tmp1currA, tmp1currP + dDA__A_MIN, invIrowA, invIrowP, b.tacc);
				for (int k = dSA__MIN; k != dSA__MAX; ++k)
					tmp1currA[tmp1currP + dDA__A_MIN + k] += b.avel.get(dV3E__AXES_MIN + k) * stepsizeRecip;
				// Initialize body start joint indices -- this will be needed later for building body related joint
				// list in dxStepIsland_Stage2c
				bodyStartJoints.set(bi, 0);
			}
		}
	}

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
				double[] JinvMrowA = JinvM;// + ofsi * (2 * JIM__MAX);
				int JinvMrowP = ofsi * (2 * JIM__MAX);
				DxJoint joint = jointinfosA[jointinfosP+ji].joint;

				DxBody jb0 = joint.node[0].body;
				if (true) {// || jb0 != null) { // -- always true
					// compute diagonal block of A
              		int JRowP = ofsi * (2 * JME__MAX); //J + (size_t)ofsi * (2 * JME__MAX);
					MultiplyAddJinvMxJToA (ArowA, ArowP + ofsi, JinvMrowA, JinvMrowP, J, JRowP, infom, infom, mskip);

					for (DxJointNode n0=(ji != 0 ? jb0.firstjoint.get() : null); n0!=null; n0=n0.next) {
						// if joint was tagged as -1 then it is an inactive (m=0 or disabled)
						// joint that should not be considered
						int j0 = n0.joint.tag;
						if (j0 != -1 && j0 < ji) {
							final int jiother_ofsi = mindex[j0];
							final int jiother_infom = mindex[j0 + 1] - jiother_ofsi;
							final dJointWithInfo1 jiother = jointinfosA[jointinfosP + j0];
							int smart_infom = (jiother.joint.node[1].body == jb0) ? jiother_infom : 0;
							// set block of A
                        	final int JOtherP = /*J +*/ (jiother_ofsi * 2 + smart_infom) * JME__MAX;
							MultiplyAddJinvMxJToA (ArowA, ArowP + jiother_ofsi, JinvMrowA, JinvMrowP, J, JOtherP, infom, jiother_infom, mskip);
						}
					}
				}

				DxBody jb1 = joint.node[1].body;
				dIASSERT(jb1 != jb0);
				if (jb1!=null) {
					int JinvMOtherP = JinvMrowP + infom * JIM__MAX;
					// compute diagonal block of A
                	final int JRowP = /*J +*/ (ofsi * 2 + infom) * JME__MAX;
					MultiplyAddJinvMxJToA (ArowA, ArowP + ofsi, JinvMrowA, JinvMOtherP, J, JRowP, infom, infom, mskip);

					for (DxJointNode n1=(ji != 0 ? jb1.firstjoint.get() : null); n1!=null; n1=n1.next) {
						// if joint was tagged as -1 then it is an inactive (m=0 or disabled)
						// joint that should not be considered
						int j1 = n1.joint.tag;
						if (j1 != -1 && j1 < ji) {
							final int jiother_ofsi = mindex[j1];
							final int jiother_infom = mindex[j1 + 1] - jiother_ofsi;
							final dJointWithInfo1 jiother = jointinfosA[jointinfosP + j1];
							int smart_infom = (jiother.joint.node[1].body == jb1) ? jiother_infom : 0;
							// set block of A
                        	final int JOtherP = /*J +*/ (jiother_ofsi * 2 + smart_infom) * JME__MAX;
							MultiplyAddJinvMxJToA (ArowA, ArowP + jiother_ofsi, JinvMrowA, JinvMOtherP, J, JOtherP, infom, jiother_infom, mskip);
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
			double[] rhs_tmp = stage2CallContext.m_rhs_tmp;
			double[] pairsRhsCfm = localContext.m_pairsRhsCfm;
			AtomicIntegerArray bodyStartJoints = localContext.m_bodyStartJoints;
			AtomicIntegerArray bodyJointLinks = localContext.m_bodyJointLinks;


			// compute the right hand side `rhs'
			// put J*rhs_tmp into rhs
			int ji;
			while ((ji = ThrsafeIncrementIntUpToLimit(stage2CallContext.m_ji_rhs, nj)) != nj) {
				final int ofsi = mindex[ji];
				final int infom = mindex[ji + 1] - ofsi;

				double[] currRhsCfmA = pairsRhsCfm;
				int currRhsCfmP = ofsi * RCE__RHS_CFM_MAX;
				double[] JrowA = J;//  + ofsi * (2 * JME__MAX);
				int JrowP = ofsi * (2 * JME__MAX);

				DxJoint joint = jointinfosA[jointinfosP+ji].joint;

				DxBody jb0 = joint.node[0].body;
				if (true) {// || jb0 != null) { // -- always true
					int bodyIndex = jb0.tag;
					MultiplySubJxRhsTmpFromRHS (currRhsCfmA, currRhsCfmP, JrowA, JrowP, rhs_tmp, bodyIndex * dDA__MAX, infom);

					// Link joints connected to each body into a list to be used on results incorporation. The bodyStartJoints have been initialized in dxStepIsland_Stage2b.
                	final int linkIndex = (ji * dJCB__MAX + dJCB_FIRST_BODY); // It is asserted at links buffer allocation that the indices can't overflow atomicord32
					for (int oldStartIndex = bodyStartJoints.get(bodyIndex); ; oldStartIndex = bodyStartJoints.get(bodyIndex)) {
						bodyJointLinks.set(linkIndex, oldStartIndex);
						if (ThrsafeCompareExchange(bodyStartJoints, bodyIndex, oldStartIndex, linkIndex + 1)) { // The link index is stored incremented to allow 0 as end indicator
							break;
						}
					}
				}

				DxBody jb1 = joint.node[1].body;
				if (jb1 != null) {
					int bodyIndex = jb1.tag;
					MultiplySubJxRhsTmpFromRHS (currRhsCfmA, currRhsCfmP, JrowA, JrowP + infom * JME__MAX, rhs_tmp, bodyIndex * dDA__MAX, infom);

					// Link joints connected to each body into a list to be used on results incorporation. The bodyStartJoints have been initialized in dxStepIsland_Stage2b
                	final int linkIndex = ji * dJCB__MAX + dJCB_SECOND_BODY; // It is asserted at links buffer allocation that the indices can't overflow atomicord32
					for (int oldStartIndex = bodyStartJoints.get(bodyIndex); ; oldStartIndex = bodyStartJoints.get(bodyIndex)) {
						bodyJointLinks.set(linkIndex, oldStartIndex);
						if (ThrsafeCompareExchange(bodyStartJoints, bodyIndex, oldStartIndex, linkIndex + 1)) { // The link index is stored incremented to allow 0 as end indicator
							break;
						}
					}
				}
			}
		}
	}

	static 
	void dxStepIsland_Stage3(dxStepperStage3CallContext stage3CallContext)
	{
		final DxStepperProcessingCallContext callContext = stage3CallContext.m_stepperCallContext;
		final dxStepperLocalContext localContext = stage3CallContext.m_localContext;

		DxWorldProcessMemArena memarena = callContext.m_stepperArena();
		memarena.RestoreState(stage3CallContext.m_stage1MemArenaState);
		stage3CallContext = null; // WARNING! stage3CallContext is not valid after this point!
		dIVERIFY(stage3CallContext == null); // To suppress unused variable assignment warnings

		int m = localContext.m_m;
		int nub = localContext.m_nub;
		//const unsigned int *mindex = localContext.m_mindex;
		int[] findex = localContext.m_findex;
		double[] A = localContext.m_A;
		double[] pairsRhsLambda = localContext.m_pairsRhsCfm; // Reuse cfm buffer for lambdas as the former values are not needed any more
		double[] pairsLoHi = localContext.m_pairsLoHi;

		if (m > 0) {
			BlockPointer lcpstate = memarena.BEGIN_STATE_SAVE();
			{
				IFTIMING_dTimerNow ("solve LCP problem");

				// solve the LCP problem and get lambda.
				// this will destroy A but that's OK
				DLCP.dxSolveLCP (memarena, m, A, pairsRhsLambda, null, nub, pairsLoHi, findex);
				dSASSERT(RLE__RHS_LAMBDA_MAX == PBX__MAX && RLE_RHS == PBX_B && RLE_LAMBDA == PBX_X);
				dSASSERT(LHE__LO_HI_MAX == PLH__MAX && LHE_LO == PLH_LO && LHE_HI == PLH_HI);
			}
			memarena.END_STATE_SAVE(lcpstate);
		}

		// void *stage3MemarenaState = memarena->SaveState();

		// dxStepperStage4CallContext * stage4CallContext = (dxStepperStage4CallContext *)
		// memarena -> AllocateBlock(sizeof(dxStepperStage4CallContext));
		// stage4CallContext -> Initialize(callContext, localContext/*, stage3MemarenaState*/);
		memarena.dummy();
		dxStepperStage4CallContext stage4CallContext = new dxStepperStage4CallContext();
		stage4CallContext.Initialize(callContext, localContext/*, stage3MemarenaState*/);


    	final int allowedThreads = callContext.m_stepperAllowedThreads();
		dIASSERT(allowedThreads != 0);

		if (allowedThreads == 1) {
			IFTIMING_dTimerNow("compute and apply constraint force");
			dxStepIsland_Stage4(stage4CallContext);
			IFTIMING_dTimerEnd();

			if (m > 0) {
				IFTIMING_dTimerReport(stdout, 1);
			}
		} else {
			throw new UnsupportedOperationException();
			//			dCallReleaseeID finalReleasee = callContext.m_finalReleasee;
			//			DxWorld world = callContext.m_world;
			//			world.AlterThreadedCallDependenciesCount(finalReleasee, allowedThreads - 1);
			//			world.PostThreadedCallsGroup(NULL, allowedThreads - 1, finalReleasee, & dxStepIsland_Stage4_Callback, stage4CallContext, "StepIsland Stage4")
			//			;
			//			// Note: Adding another dependency for the finalReleasee is not necessary as it already depends on the current call
			//			dxStepIsland_Stage4(stage4CallContext);
		}
	}

//	private static
//	int dxStepIsland_Stage4_Callback(void *_stage4CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
//	{
//		(void)callInstanceIndex; // unused
//		(void)callThisReleasee; // unused
//		dxStepperStage4CallContext *stage4CallContext = (dxStepperStage4CallContext *)_stage4CallContext;
//		dxStepIsland_Stage4(stage4CallContext);
//		return 1;
//	}

	private static void dxStepIsland_Stage4(dxStepperStage4CallContext stage4CallContext) {
		final DxStepperProcessingCallContext callContext = stage4CallContext.m_stepperCallContext;
		final dxStepperLocalContext localContext = stage4CallContext.m_localContext;

		final double stepSize = callContext.m_stepSize();
		final DxBody[] bodiesA = callContext.m_islandBodiesStartA();
		final int bodiesP = callContext.m_islandBodiesStartOfs();
		double[] invI = localContext.m_invI;
		dJointWithInfo1[] jointInfosA = localContext.m_jointinfosA;
		final int jointInfosP = localContext.m_jointinfosOfs;
		double[] J = localContext.m_J;
		double[] pairsRhsLambda = localContext.m_pairsRhsCfm;
		final int[] mIndex = localContext.m_mindex;
		AtomicIntegerArray bodyStartJoints = localContext.m_bodyStartJoints;
		AtomicIntegerArray bodyJointLinks = localContext.m_bodyJointLinks;
		final int nb = callContext.m_islandBodiesCount();

		int bi = 0;
		while ((bi = ThrsafeIncrementIntUpToLimit(stage4CallContext.m_bi_constrForce, nb)) != nb) {
			DVector3 angularForceAccumulator = new DVector3();
			DxBody b = bodiesA[bodiesP + bi];
			double[] invIrowA = invI;// + bi * dM3E__MAX;
			int invIrowP = bi * dM3E__MAX;
			double body_invMass_mul_stepSize = stepSize * b.invMass;

			double[] bodyConstrForce = new double[CFE__MAX];
			boolean constrForceAvailable = false;

			int linkIndex = bodyStartJoints != null ? bodyStartJoints.get(bi) : 0;
			if (linkIndex != 0) {
				dSetZero(bodyConstrForce);//, dARRAY_SIZE(bodyConstrForce));
			}

			// compute the constraint force as constrForce = J'*lambda
			for (; linkIndex != 0; constrForceAvailable = true, linkIndex = bodyJointLinks.get(linkIndex - 1)) {
				int jointIndex = (linkIndex - 1) / dJCB__MAX;
				int jointBodyIndex = (linkIndex - 1) % dJCB__MAX;

				final dJointWithInfo1 currJointInfo = jointInfosA[jointInfosP + jointIndex];
				int ofsi = mIndex[jointIndex];
				dIASSERT(dIN_RANGE(jointIndex, 0, localContext.m_nj));

				double[] JRowA = J;// + (size_t)ofsi * (2 * JME__MAX);
				final int JRowP = ofsi * (2 * JME__MAX);
				double[] rowRhsLambdaA = pairsRhsLambda;// + (size_t)ofsi * RLE__RHS_LAMBDA_MAX;
				final int rowRhsLambdaP = ofsi * RLE__RHS_LAMBDA_MAX;

				DxJoint joint = currJointInfo.joint;
				final int infom = currJointInfo.info.m;

				// unsigned jRowExtraOffset = jointBodyIndex * infom * JME__MAX;
				int jRowExtraOffset = jointBodyIndex != dJCB__MIN ? infom * JME__MAX : 0;
				dSASSERT(dJCB__MAX == 2);

				DJointFeedback fb = joint.feedback;
				MultiplyAddJxLambdaToCForce(bodyConstrForce, JRowA, JRowP + jRowExtraOffset, rowRhsLambdaA, rowRhsLambdaP, infom, fb, jointBodyIndex);
			}

			// compute the velocity update
			if (constrForceAvailable) {
				// add fe to cforce and multiply cforce by stepSize
				for (int j = dSA__MIN; j != dSA__MAX; ++j) {
					double d =
							(bodyConstrForce[CFE__L_MIN + j] + b.facc.get(dV3E__AXES_MIN + j)) * body_invMass_mul_stepSize;
					b.lvel.add(dV3E__AXES_MIN + j, d);
				}
				for (int k = dSA__MIN; k != dSA__MAX; ++k) {
					double d = (bodyConstrForce[CFE__A_MIN + k] + b.tacc.get(dV3E__AXES_MIN + k)) * stepSize;
					angularForceAccumulator.set(dV3E__AXES_MIN + k, d);
				}
			} else {
				// add fe to cforce and multiply cforce by stepSize
				//dAddVectorScaledVector3(b.lvel, b.lvel, b.facc, body_invMass_mul_stepSize);
				b.lvel.addScaled(b.facc, body_invMass_mul_stepSize);
				//dCopyScaledVector3(angularForceAccumulator, b.tacc, stepSize);
				angularForceAccumulator.set(b.tacc).scale(stepSize);
			}

			dAssertVec3Element();
			dMultiplyAdd0_331(b.avel, invIrowA, invIrowP, angularForceAccumulator);// + dV3E__AXES_MIN);

			// update the position and orientation from the new linear/angular velocity
			// (over the given time step)
			b.dxStepBody(stepSize);

			// zero all force accumulators
			b.facc.setZero();
			b.tacc.setZero();
		}
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
		//	    res += dOVERALIGNED_SIZE(sizeof(dReal) * dM3E__MAX * nb, INVI_ALIGNMENT); // for invI
		//
		//	    {
		//	        size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * 2 * _nj); // for initial jointinfos
		//
		//	        // The array can't grow right more than by nj
		//	        size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * ((size_t)_nj + (size_t)nj)); // for shrunk jointinfos
		//	        sub1_res2 += dEFFICIENT_SIZE(sizeof(dxStepperLocalContext)); //for dxStepperLocalContext
		//	        if (m > 0) {
		//	            sub1_res2 += dEFFICIENT_SIZE(sizeof(unsigned int) * (nj + 1)); // for mindex
		//		sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
		//		sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * JME__MAX * m); // for J
		//		unsigned int mskip = dPAD(m);
		//		sub1_res2 += dOVERALIGNED_SIZE(sizeof(dReal) * mskip * m, AMATRIX_ALIGNMENT); // for A
		//		sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * RCE__RHS_CFM_MAX * m); // for pairsRhsCfm
		//		sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * LHE__LO_HI_MAX * m); // for pairsLoHi
		//		sub1_res2 += dEFFICIENT_SIZE(sizeof(atomicord32) * nb); // for bodyStartJoints
		//		sub1_res2 += dEFFICIENT_SIZE(sizeof(atomicord32)* dJCB__MAX * nj); // for bodyJointLinks
		//	}
		//
		//	{
		//		size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dxStepperStage3CallContext)); // for dxStepperStage3CallContext
		//
		//		size_t sub2_res2 = 0;
		//
		//		size_t sub2_res3 = dEFFICIENT_SIZE(sizeof(dxStepperStage4CallContext)); // for dxStepperStage4CallContext
		//
		//		if (m > 0) {
		//			sub2_res1 += dOVERALIGNED_SIZE(sizeof(dReal) * 2 * JIM__MAX * m, JINVM_ALIGNMENT); // for JinvM
		//			sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * dDA__MAX * nb); // for rhs_tmp
		//			sub2_res1 += dEFFICIENT_SIZE(sizeof(dxStepperStage2CallContext)); // for dxStepperStage2CallContext
		//
		//			sub2_res2 += dxEstimateSolveLCPMemoryReq(m, false);
		//		}
		//
		//		sub1_res2 += dMAX(sub2_res1, dMAX(sub2_res2, sub2_res3));
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

