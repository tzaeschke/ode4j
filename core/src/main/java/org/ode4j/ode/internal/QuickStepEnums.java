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

import static org.ode4j.ode.internal.Common.dSASSERT;
import static org.ode4j.ode.internal.CommonEnums.*;
import static org.ode4j.ode.internal.processmem.DxUtil.EFFICIENT_ALIGNMENT;
import static org.ode4j.ode.internal.processmem.DxUtil.dMAX;

import org.ode4j.ode.internal.joints.JointEnums;

class QuickStepEnums {

	// dxInvIRowElement
	public static final int IIE__MIN = 0;
	public static final int IIE__MATRIX_MIN = IIE__MIN;
	public static final int IIE__MATRIX_MAX = IIE__MATRIX_MIN + dM3E__MAX;
	public static final int IIE__MAX = IIE__MATRIX_MAX;

	// dxRHSCFMElement
	public static final int RCE_RHS = JointEnums.GI2_RHS;
	public static final int RCE_CFM = JointEnums.GI2_CFM;
	public static final int RCE__RHS_CFM_MAX = JointEnums.GI2__RHS_CFM_MAX;

	// dxLoHiElement
	public static final int LHE_LO = JointEnums.GI2_LO;
	public static final int LHE_HI = JointEnums.GI2_HI;
	public static final int LHE__LO_HI_MAX = JointEnums.GI2__LO_HI_MAX;

	// dxJacobiVectorElement
	public static final int JVE__MIN = 0;
	public static final int JVE__L_MIN = JVE__MIN + dDA__L_MIN;
	public static final int JVE_LX = JVE__MIN + dDA_LX;
    public static final int JVE_LY = JVE__MIN + dDA_LY;
    public static final int JVE_LZ = JVE__MIN + dDA_LZ;
    public static final int JVE__L_MAX = JVE__MIN + dDA__L_MAX;
    public static final int JVE__A_MIN = JVE__MIN + dDA__A_MIN;
    public static final int JVE_AX = JVE__MIN + dDA_AX;
    public static final int JVE_AY = JVE__MIN + dDA_AY;
    public static final int JVE_AZ = JVE__MIN + dDA_AZ;
    public static final int JVE__A_MAX = JVE__MIN + dDA__A_MAX;
    public static final int JVE__MAX = JVE__MIN + dDA__MAX;
    public static final int JVE__L_COUNT = JVE__L_MAX - JVE__L_MIN;
    public static final int JVE__A_COUNT = JVE__A_MAX - JVE__A_MIN;

	// dxJacobiMatrixElement
	public static final int JME__MIN = 0;
	public static final int JME__J1_MIN = JME__MIN;
	public static final int JME__J1L_MIN = JME__J1_MIN + JVE__L_MIN;
	public static final int JME_J1LX = JME__J1_MIN + JVE_LX;
	public static final int JME_J1LY = JME__J1_MIN + JVE_LY;
	public static final int JME_J1LZ = JME__J1_MIN + JVE_LZ;
	public static final int JME__J1L_MAX = JME__J1_MIN + JVE__L_MAX;
	public static final int JME__J1A_MIN = JME__J1_MIN + JVE__A_MIN;
	public static final int JME_J1AX = JME__J1_MIN + JVE_AX;
	public static final int JME_J1AY = JME__J1_MIN + JVE_AY;
	public static final int JME_J1AZ = JME__J1_MIN + JVE_AZ;
	public static final int JME__J1A_MAX = JME__J1_MIN + JVE__A_MAX;
	public static final int JME__J1_MAX = JME__J1_MIN + JVE__MAX;
	public static final int JME__RHS_CFM_MIN = JME__J1_MAX;
	public static final int JME_RHS = JME__RHS_CFM_MIN + RCE_RHS;
	public static final int JME_CFM = JME__RHS_CFM_MIN + RCE_CFM;
	public static final int JME__RHS_CFM_MAX = JME__RHS_CFM_MIN + RCE__RHS_CFM_MAX;
	public static final int JME__J2_MIN = JME__RHS_CFM_MAX;
	public static final int JME__J2L_MIN = JME__J2_MIN + JVE__L_MIN;
	public static final int JME_J2LX = JME__J2_MIN + JVE_LX;
	public static final int JME_J2LY = JME__J2_MIN + JVE_LY;
	public static final int JME_J2LZ = JME__J2_MIN + JVE_LZ;
	public static final int JME__J2L_MAX = JME__J2_MIN + JVE__L_MAX;
	public static final int JME__J2A_MIN = JME__J2_MIN + JVE__A_MIN;
	public static final int JME_J2AX = JME__J2_MIN + JVE_AX;
	public static final int JME_J2AY = JME__J2_MIN + JVE_AY;
	public static final int JME_J2AZ = JME__J2_MIN + JVE_AZ;
	public static final int JME__J2A_MAX = JME__J2_MIN + JVE__A_MAX;
	public static final int JME__J2_MAX = JME__J2_MIN + JVE__MAX;
	public static final int JME__LO_HI_MIN = JME__J2_MAX;
	public static final int JME_LO = JME__LO_HI_MIN + LHE_LO;
	public static final int JME_HI = JME__LO_HI_MIN + LHE_HI;
	public static final int JME__LO_HI_MAX = JME__LO_HI_MIN + LHE__LO_HI_MAX;
	public static final int JME__MAX = JME__LO_HI_MAX; // Is not that a luck to have 16 elements here? ;-)
	public static final int JME__J1_COUNT = JME__J1_MAX - JME__J1_MIN;
	public static final int JME__J2_COUNT = JME__J2_MAX - JME__J2_MIN;
	public static final int JME__J_COUNT = JVE__MAX;

	static {
		dSASSERT(JME__J_COUNT == JME__J1_COUNT);
		dSASSERT(JME__J_COUNT == JME__J2_COUNT);
	}

	// dxJacobiCopyElement
	public static final int JCE__MIN = 0;
	public static final int JCE__J1_MIN = JCE__MIN;
	public static final int JCE__J1L_MIN = JCE__J1_MIN;
	public static final int JCE_J1LX = JCE__J1L_MIN;
	public static final int JCE_J1LY = JCE_J1LX + 1;
	public static final int JCE_J1LZ = JCE_J1LY + 1;
	public static final int JCE__J1L_MAX = JCE_J1LZ + 1;
	public static final int JCE__J1A_MIN = JCE__J1L_MAX;
	public static final int JCE_J1AX = JCE__J1A_MIN;
	public static final int JCE_J1AY = JCE_J1AX + 1; 
	public static final int JCE_J1AZ = JCE_J1AY + 1;
	public static final int JCE__J1A_MAX = JCE_J1AZ + 1;
	public static final int JCE__J1_MAX = JCE__J1A_MAX;
	public static final int JCE__J2_MIN = JCE__J1_MAX;
	public static final int JCE__J2L_MIN = JCE__J2_MIN;
	public static final int JCE_J2LX = JCE__J2L_MIN;
	public static final int JCE_J2LY = JCE_J2LX + 1;
	public static final int JCE_J2LZ = JCE_J2LY + 1;
	public static final int JCE__J2L_MAX = JCE_J2LZ + 1;
	public static final int JCE__J2A_MIN = JCE__J2L_MAX;
	public static final int JCE_J2AX = JCE__J2A_MIN;
	public static final int JCE_J2AY = JCE_J2AX + 1;
	public static final int JCE_J2AZ = JCE_J2AY + 1;
	public static final int JCE__J2A_MAX = JCE_J2AZ + 1;
	public static final int JCE__J2_MAX = JCE__J2A_MAX;
	public static final int JCE__MAX = JCE__J2_MAX;
	public static final int JCE__J1_COUNT = JCE__J1_MAX - JCE__J1_MIN;
	public static final int JCE__J2_COUNT = JCE__J2_MAX - JCE__J2_MIN;
	public static final int JCE__JMAX_COUNT = Math.max(JCE__J1_COUNT, JCE__J2_COUNT);

	// dxInvMJTElement
	public static final int IMJ__MIN = 0;
	public static final int IMJ__1_MIN = IMJ__MIN;
	public static final int IMJ__1L_MIN = IMJ__1_MIN + JVE__L_MIN;
	public static final int IMJ_1LX = IMJ__1_MIN + JVE_LX;
	public static final int IMJ_1LY = IMJ__1_MIN + JVE_LY;
	public static final int IMJ_1LZ = IMJ__1_MIN + JVE_LZ;
	public static final int IMJ__1L_MAX = IMJ__1_MIN + JVE__L_MAX;
	public static final int IMJ__1A_MIN = IMJ__1_MIN + JVE__A_MIN;
	public static final int IMJ_1AX = IMJ__1_MIN + JVE_AX;
	public static final int IMJ_1AY = IMJ__1_MIN + JVE_AY;
	public static final int IMJ_1AZ = IMJ__1_MIN + JVE_AZ;
	public static final int IMJ__1A_MAX = IMJ__1_MIN + JVE__A_MAX;
	public static final int IMJ__1_MAX = IMJ__1_MIN + JVE__MAX;
	public static final int IMJ__2_MIN = IMJ__1_MAX;
	public static final int IMJ__2L_MIN = IMJ__2_MIN + JVE__L_MIN;
	public static final int IMJ_2LX = IMJ__2_MIN + JVE_LX;
	public static final int IMJ_2LY = IMJ__2_MIN + JVE_LY;
	public static final int IMJ_2LZ = IMJ__2_MIN + JVE_LZ;
	public static final int IMJ__2L_MAX = IMJ__2_MIN + JVE__L_MAX;
	public static final int IMJ__2A_MIN = IMJ__2_MIN + JVE__A_MIN;
	public static final int IMJ_2AX = IMJ__2_MIN + JVE_AX;
	public static final int IMJ_2AY = IMJ__2_MIN + JVE_AY;
	public static final int IMJ_2AZ = IMJ__2_MIN + JVE_AZ;
	public static final int IMJ__2A_MAX = IMJ__2_MIN + JVE__A_MAX;
	public static final int IMJ__2_MAX = IMJ__2_MIN + JVE__MAX;
	public static final int IMJ__MAX = IMJ__2_MAX;

	// dxContactForceElement
	public static final int CFE__MIN = 0;
	public static final int CFE__DYNAMICS_MIN = CFE__MIN;
	public static final int CFE__L_MIN = CFE__DYNAMICS_MIN + dDA__L_MIN;
	public static final int CFE_LX = CFE__DYNAMICS_MIN + dDA_LX;
	public static final int CFE_LY = CFE__DYNAMICS_MIN + dDA_LY;
	public static final int CFE_LZ = CFE__DYNAMICS_MIN + dDA_LZ;
	public static final int CFE__L_MAX = CFE__DYNAMICS_MIN + dDA__L_MAX;
	public static final int CFE__A_MIN = CFE__DYNAMICS_MIN + dDA__A_MIN;
	public static final int CFE_AX = CFE__DYNAMICS_MIN + dDA_AX;
	public static final int CFE_AY = CFE__DYNAMICS_MIN + dDA_AY;
	public static final int CFE_AZ = CFE__DYNAMICS_MIN + dDA_AZ;
	public static final int CFE__A_MAX = CFE__DYNAMICS_MIN + dDA__A_MAX;
	public static final int CFE__DYNAMICS_MAX = CFE__DYNAMICS_MIN + dDA__MAX;
	public static final int CFE__MAX = CFE__DYNAMICS_MAX;

	// dxRHSElement
	public static final int RHS__MIN = 0;
	public static final int RHS__DYNAMICS_MIN = RHS__MIN;
	public static final int RHS__L_MIN = RHS__DYNAMICS_MIN + dDA__L_MIN;
	public static final int RHS_LX = RHS__DYNAMICS_MIN + dDA_LX;
	public static final int RHS_LY = RHS__DYNAMICS_MIN + dDA_LY;
	public static final int RHS_LZ = RHS__DYNAMICS_MIN + dDA_LZ;
	public static final int RHS__L_MAX = RHS__DYNAMICS_MIN + dDA__L_MAX;
	public static final int RHS__A_MIN = RHS__DYNAMICS_MIN + dDA__A_MIN;
	public static final int RHS_AX = RHS__DYNAMICS_MIN + dDA_AX;
	public static final int RHS_AY = RHS__DYNAMICS_MIN + dDA_AY;
	public static final int RHS_AZ = RHS__DYNAMICS_MIN + dDA_AZ;
	public static final int RHS__A_MAX = RHS__DYNAMICS_MIN + dDA__A_MAX;
	public static final int RHS__DYNAMICS_MAX = RHS__DYNAMICS_MIN + dDA__MAX;
	public static final int RHS__MAX = RHS__DYNAMICS_MAX;

	//	#define JACOBIAN_ALIGNMENT  dMAX(JME__MAX * sizeof(dReal), EFFICIENT_ALIGNMENT)
	//		dSASSERT(((JME__MAX - 1) & JME__MAX) == 0); // Otherwise there is no reason to over-align the Jacobian
	//
	//	#define JCOPY_ALIGNMENT    dMAX(32, EFFICIENT_ALIGNMENT)
	//	#define INVI_ALIGNMENT     dMAX(32, EFFICIENT_ALIGNMENT)
	//	#define INVMJ_ALIGNMENT    dMAX(32, EFFICIENT_ALIGNMENT)
	public static final int JACOBIAN_ALIGNMENT = dMAX(JME__MAX /* * sizeof(dReal)*/, EFFICIENT_ALIGNMENT);
	static {
		dSASSERT(((JME__MAX - 1) & JME__MAX) == 0); // Otherwise there is no reason to over-align the Jacobian
	}
	public static final int JCOPY_ALIGNMENT = dMAX(32, EFFICIENT_ALIGNMENT);
	public static final int INVI_ALIGNMENT  = dMAX(32, EFFICIENT_ALIGNMENT);
	public static final int INVMJ_ALIGNMENT = dMAX(32, EFFICIENT_ALIGNMENT);
}
