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

import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.JointEnums;

import static org.ode4j.ode.internal.CommonEnums.*;

class StepEnums {


    // dxRHSCFMElement
    public static final int RCE_RHS = JointEnums.GI2_RHS;
    public static final int RCE_CFM = JointEnums.GI2_CFM;
    // Elements for array reuse
    public static final int RLE_RHS = RCE_RHS;
    public static final int RLE_LAMBDA = RCE_CFM;
    public static final int RCE__RHS_CFM_MAX = JointEnums.GI2__RHS_CFM_MAX;
    public static final int RLE__RHS_LAMBDA_MAX = RCE__RHS_CFM_MAX;
    // dxLoHiElement
    public static final int LHE_LO = JointEnums.GI2_LO;
    public static final int LHE_HI = JointEnums.GI2_HI;
    public static final int LHE__LO_HI_MAX = JointEnums.GI2__LO_HI_MAX;


    // dxJacobiVectorElement
    public static final int JVE__MIN = 0;

    public static final int JVE__L_MIN = JVE__MIN + dDA__L_MIN;

    public static final int JVE_LX = JVE__L_MIN + dSA_X;
    public static final int JVE_LY = JVE__L_MIN + dSA_Y;
    public static final int JVE_LZ = JVE__L_MIN + dSA_Z;

    public static final int JVE__L_MAX = JVE__L_MIN + dSA__MAX;

    public static final int JVE__A_MIN = JVE__MIN + dDA__A_MIN;

    public static final int JVE_AX = JVE__A_MIN + dSA_X;
    public static final int JVE_AY = JVE__A_MIN + dSA_Y;
    public static final int JVE_AZ = JVE__A_MIN + dSA_Z;

    public static final int JVE__A_MAX = JVE__A_MIN + dSA__MAX;

    public static final int JVE__MAX = JVE__MIN + dDA__MAX;

    public static final int JVE__L_COUNT = JVE__L_MAX - JVE__L_MIN;
    public static final int JVE__A_COUNT = JVE__A_MAX - JVE__A_MIN;


    // dxJacobiMatrixElement
    public static final int JME__MIN = 0;

    public static final int JME__J_MIN = JME__MIN;
    public static final int JME__JL_MIN = JME__J_MIN + JVE__L_MIN;
    public static final int JME_JLX = JME__J_MIN + JVE_LX;
    public static final int JME_JLY = JME__J_MIN + JVE_LY;
    public static final int JME_JLZ = JME__J_MIN + JVE_LZ;

    public static final int JME__JL_MAX = JME__J_MIN + JVE__L_MAX;

    public static final int JME__JA_MIN = JME__J_MIN + JVE__A_MIN;

    public static final int JME_JAX = JME__J_MIN + JVE_AX;
    public static final int JME_JAY = JME__J_MIN + JVE_AY;
    public static final int JME_JAZ = JME__J_MIN + JVE_AZ;

    public static final int JME__JA_MAX = JME__J_MIN + JVE__A_MAX;
    public static final int JME__J_MAX = JME__J_MIN + JVE__MAX;

    public static final int JME__MAX = JME__J_MAX;

    public static final int JME__J_COUNT = JME__J_MAX - JME__J_MIN;


    // dxJInvMElement
    public static final int JIM__MIN = 0;

    public static final int JIM__L_MIN = JIM__MIN + dMD_LINEAR * dV3E__MAX;

    public static final int JIM__L_AXES_MIN = JIM__L_MIN + dV3E__AXES_MIN;

    public static final int JIM_LX = JIM__L_MIN + dV3E_X;
    public static final int JIM_LY = JIM__L_MIN + dV3E_Y;
    public static final int JIM_LZ = JIM__L_MIN + dV3E_Z;

    public static final int JIM__L_AXES_MAX = JIM__L_MIN + dV3E__AXES_MAX;

    public static final int JIM_LPAD = JIM__L_MIN + dV3E_PAD;

    public static final int JIM__L_MAX = JIM__L_MIN + dV3E__MAX;

    public static final int JIM__A_MIN = JIM__MIN + dMD_ANGULAR * dV3E__MAX;

    public static final int JIM__A_AXES_MIN = JIM__A_MIN + dV3E__AXES_MIN;

    public static final int JIM_AX = JIM__A_MIN + dV3E_X;
    public static final int JIM_AY = JIM__A_MIN + dV3E_Y;
    public static final int JIM_AZ = JIM__A_MIN + dV3E_Z;

    public static final int JIM__A_AXES_MAX = JIM__A_MIN + dV3E__AXES_MAX;

    public static final int JIM_APAD = JIM__A_MIN + dV3E_PAD;

    public static final int JIM__A_MAX = JIM__A_MIN + dV3E__MAX;

    public static final int JIM__MAX = JIM__MIN + dMD__MAX * dV3E__MAX;


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

}
