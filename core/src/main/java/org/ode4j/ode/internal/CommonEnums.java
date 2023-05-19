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

public class CommonEnums {

	// dSpaceAxis
	public static final int dSA__MIN = 0;
	public static final int dSA_X = dSA__MIN;
	public static final int dSA_Y = dSA_X + 1; 
	public static final int dSA_Z = dSA_Y + 1;
	public static final int dSA__MAX = dSA_Z + 1;
	// dMotionDynamics
	public static final int dMD__MIN = 0;
	public static final int dMD_LINEAR = dMD__MIN;
	public static final int dMD_ANGULAR = dMD_LINEAR + 1; 
	public static final int dMD__MAX = dMD_ANGULAR + 1;
	// dDynamicsAxis
	public static final int dDA__MIN = 0;
	public static final int dDA__L_MIN = dDA__MIN + dMD_LINEAR * dSA__MAX;
	public static final int dDA_LX = dDA__L_MIN + dSA_X; 
	public static final int dDA_LY = dDA__L_MIN + dSA_Y;
	public static final int dDA_LZ = dDA__L_MIN + dSA_Z;
	public static final int dDA__L_MAX = dDA__L_MIN + dSA__MAX;
	public static final int dDA__A_MIN = dDA__MIN + dMD_ANGULAR * dSA__MAX;
	public static final int dDA_AX = dDA__A_MIN + dSA_X;
	public static final int dDA_AY = dDA__A_MIN + dSA_Y;
	public static final int dDA_AZ = dDA__A_MIN + dSA_Z;
	public static final int dDA__A_MAX = dDA__A_MIN + dSA__MAX;
	public static final int dDA__MAX = dDA__MIN + dMD__MAX * dSA__MAX;
	// dVec3Element
	public static final int dV3E__MIN = 0;
	public static final int dV3E__AXES_MIN = dV3E__MIN;
	public static final int dV3E_X = dV3E__AXES_MIN + dSA_X;
	public static final int dV3E_Y = dV3E__AXES_MIN + dSA_Y;
	public static final int dV3E_Z = dV3E__AXES_MIN + dSA_Z;
	public static final int dV3E__AXES_MAX = dV3E__AXES_MIN + dSA__MAX;
	public static final int dV3E_PAD = dV3E__AXES_MAX;
	public static final int dV3E__MAX = dV3E_PAD + 1;
	// dVec4Element
	public static final int dV4E__MIN = 0;
	public static final int dV4E_X = dV4E__MIN + dSA_X;
	public static final int dV4E_Y = dV4E__MIN + dSA_Y;
	public static final int dV4E_Z = dV4E__MIN + dSA_Z;
	public static final int dV4E_O = dV4E__MIN + dSA__MAX;
	public static final int dV4E__MAX = dV4E_O + 1;
	// dMat3Element
	public static final int dM3E__MIN = 0;
	public static final int dM3E__X_MIN = dM3E__MIN + dSA_X * dV3E__MAX;
	public static final int dM3E__X_AXES_MIN = dM3E__X_MIN + dV3E__AXES_MIN;
	public static final int dM3E_XX = dM3E__X_MIN + dV3E_X;
	public static final int dM3E_XY = dM3E__X_MIN + dV3E_Y;
	public static final int dM3E_XZ = dM3E__X_MIN + dV3E_Z;
	public static final int dM3E__X_AXES_MAX = dM3E__X_MIN + dV3E__AXES_MAX;
	public static final int dM3E_XPAD = dM3E__X_MIN + dV3E_PAD;
	public static final int dM3E__X_MAX = dM3E__X_MIN + dV3E__MAX;
	public static final int dM3E__Y_MIN = dM3E__MIN + dSA_Y * dV3E__MAX;
	public static final int dM3E__Y_AXES_MIN = dM3E__Y_MIN + dV3E__AXES_MIN;
	public static final int dM3E_YX = dM3E__Y_MIN + dV3E_X;
	public static final int dM3E_YY = dM3E__Y_MIN + dV3E_Y;
	public static final int dM3E_YZ = dM3E__Y_MIN + dV3E_Z;
	public static final int dM3E__Y_AXES_MAX = dM3E__Y_MIN + dV3E__AXES_MAX;
	public static final int dM3E_YPAD = dM3E__Y_MIN + dV3E_PAD;
	public static final int dM3E__Y_MAX = dM3E__Y_MIN + dV3E__MAX;
	public static final int dM3E__Z_MIN = dM3E__MIN + dSA_Z * dV3E__MAX;
	public static final int dM3E__Z_AXES_MIN = dM3E__Z_MIN + dV3E__AXES_MIN;
	public static final int dM3E_ZX = dM3E__Z_MIN + dV3E_X;
	public static final int dM3E_ZY = dM3E__Z_MIN + dV3E_Y;
	public static final int dM3E_ZZ = dM3E__Z_MIN + dV3E_Z;
	public static final int dM3E__Z_AXES_MAX = dM3E__Z_MIN + dV3E__AXES_MAX;
	public static final int dM3E_ZPAD = dM3E__Z_MIN + dV3E_PAD;
	public static final int dM3E__Z_MAX = dM3E__Z_MIN + dV3E__MAX;
	public static final int dM3E__MAX = dM3E__MIN + dSA__MAX * dV3E__MAX;
	// dMat4Element
	public static final int dM4E__MIN = 0;
	public static final int dM4E__X_MIN = dM4E__MIN + dV4E_X * dV4E__MAX;
    public static final int dM4E_XX = dM4E__X_MIN + dV4E_X;
    public static final int dM4E_XY = dM4E__X_MIN + dV4E_Y;
    public static final int dM4E_XZ = dM4E__X_MIN + dV4E_Z;
    public static final int dM4E_XO = dM4E__X_MIN + dV4E_O;
	public static final int dM4E__X_MAX = dM4E__X_MIN + dV4E__MAX;
	public static final int dM4E__Y_MIN = dM4E__MIN + dV4E_Y * dV4E__MAX;
	public static final int dM4E_YX = dM4E__Y_MIN + dV4E_X;
	public static final int dM4E_YY = dM4E__Y_MIN + dV4E_Y;
	public static final int dM4E_YZ = dM4E__Y_MIN + dV4E_Z;
	public static final int dM4E_YO = dM4E__Y_MIN + dV4E_O;
	public static final int dM4E__Y_MAX = dM4E__Y_MIN + dV4E__MAX;
	public static final int dM4E__Z_MIN = dM4E__MIN + dV4E_Z * dV4E__MAX;
	public static final int dM4E_ZX = dM4E__Z_MIN + dV4E_X;
	public static final int dM4E_ZY = dM4E__Z_MIN + dV4E_Y;
	public static final int dM4E_ZZ = dM4E__Z_MIN + dV4E_Z;
	public static final int dM4E_ZO = dM4E__Z_MIN + dV4E_O;
	public static final int dM4E__Z_MAX = dM4E__Z_MIN + dV4E__MAX;
	public static final int dM4E__O_MIN = dM4E__MIN + dV4E_O * dV4E__MAX;
    public static final int dM4E_OX = dM4E__O_MIN + dV4E_X;
    public static final int dM4E_OY = dM4E__O_MIN + dV4E_Y;
    public static final int dM4E_OZ = dM4E__O_MIN + dV4E_Z;
    public static final int dM4E_OO = dM4E__O_MIN + dV4E_O;
    public static final int dM4E__O_MAX = dM4E__O_MIN + dV4E__MAX;
    public static final int dM4E__MAX = dM4E__MIN + dV4E__MAX * dV4E__MAX;
    // dQuatElement
	public static final int dQUE__MIN = 0;
	public static final int dQUE_R = dQUE__MIN;
    public static final int dQUE__AXIS_MIN = dQUE_R + 1;
    public static final int dQUE_I = dQUE__AXIS_MIN + dSA_X;
    public static final int dQUE_J = dQUE__AXIS_MIN + dSA_Y;
    public static final int dQUE_K = dQUE__AXIS_MIN + dSA_Z;
    public static final int dQUE__AXIS_MAX = dQUE__AXIS_MIN + dSA__MAX;
    public static final int dQUE__MAX = dQUE__AXIS_MAX;

	public static void dAssertVec3Element() {
		// TZ: Using this elaborate construct of constants to access an array is inefficient with
		//     ode4j's DVector3 implementation. Therefore, we do not use these constants but
		//     assert() that they are as we expect them to be.
		assert(dV3E_X == 0 && dV3E_Y == 1 && dV3E_Z == 2);
	}

	private CommonEnums() {}
}
