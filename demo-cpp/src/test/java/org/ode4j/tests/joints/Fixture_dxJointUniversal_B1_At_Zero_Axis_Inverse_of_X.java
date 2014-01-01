/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zï¿½schke      *
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
package org.ode4j.tests.joints;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateUniversal;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAngle1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis1Offset;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Rotation.dRFromAxisAndAngle;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointUniversal;

// Only one body body1 at (0,0,0)
// The joint is an Universal Joint.
// Axis the inverse of the X axis
// Anchor at (0, 0, 0)
//
//       ^Y
//       |
//       |
//       |
//       |
//       |
// Z <-- X
public class Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X
{
    private static double d2r(double degree) {
        return degree * (M_PI / 180.0);
    }
//    private static double r2d(double degree) {
//        return degree * (180.0/M_PI);
//    }

    public Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X()
	{
		wId = dWorldCreate();

		bId1 = dBodyCreate (wId);
		dBodySetPosition (bId1, 0, 0, 0);

		jId   = dJointCreateUniversal (wId, null);
		joint = (DxJointUniversal) jId;


		dJointAttach (jId, bId1, null);
		dJointSetUniversalAnchor (jId, 0, 0, 0);

		//TZ: moved to initialization
		//            axis[0] = -1;
		//            axis[1] = 0;
		//            axis[2] = 0;
		dJointSetUniversalAxis1(jId, axis.get0(), axis.get1(), axis.get2());
	}

	//        ~Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy (wId);
	}

	static DWorld wId;

	DBody bId1;


	DUniversalJoint jId;
	DxJointUniversal joint;

	DVector3C axis = new DVector3(-1, 0, 0);



	// Rotate B1 by 90deg around X then back to original position
	//
	//   ^
	//   |  => <---
	//   |
	//  B1      B1
	//
	// Start with a Delta of 90deg
	//            ^
	//  <---  =>  |
	//            |
	//   B1      B1
	//TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X,
	@Test public void test_dJointSetUniversalAxisOffset_1Body_B1_90Deg()
	{
		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);

		DVector3 axis = new DVector3();
		dJointGetUniversalAxis1(jId, axis);

		double ang1 = d2r((90.0));

		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang1);
		dBodySetRotation (bId1, R);

		CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);

		dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(), ang1, 0);
		CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);

		R.setIdentity(); // Set the rotation of the body to be zero
		dBodySetRotation (bId1, R);

		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
	}

	// Rotate B1 by -0.23rad around X then back to original position
	//
	//   ^         ^
	//   |  =>    /
	//   |       /
	//  B1      B1
	//
	// Start with a Delta of -0.23rad
	//     ^     ^
	//    /  =>  |
	//   /       |
	//   B1     B1
	//TEST_FIXTURE (Fixture_DxJointUniversal_B1_At_Zero_Axis_Inverse_of_X,
	@Test public void test_dJointSetUniversalAxisOffset_1Body_B1_Minus0_23rad()
	{
		CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
		dBodySetRotation (bId1, R);

		CHECK_CLOSE ((0.23), dJointGetUniversalAngle1 (jId), 1e-4);

		DVector3 axis = new DVector3();
		dJointGetUniversalAxis1 (jId, axis);
		dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),  (0.23), 0);
		CHECK_CLOSE ((0.23), dJointGetUniversalAngle1 (jId), 1e-4);

		dRFromAxisAndAngle (R, 1, 0, 0, 0);
		dBodySetRotation (bId1, R);

		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
	}

}