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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAngle2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAngles;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis2Offset;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Rotation.dRFromAxisAndAngle;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;

// Only one body body2 at (0,0,0)
// The joint is an Universal Joint.
// Axis1 is along the X axis.
// Axis2 is along the Y axis.
// Anchor at (0, 0, 0)
//
//       ^Y
//       |
//       |
//       |
//       |
//       |
// Z <-- X
public class Fixture_dxJointUniversal_B2_At_Zero_Default_Axes
{
    static double d2r(double degree) {
        return degree * (M_PI / 180.0);
    }
    static double r2d(double degree) {
        return degree * (180.0/M_PI);
    }

	public Fixture_dxJointUniversal_B2_At_Zero_Default_Axes()
	{
		wId = dWorldCreate();

		bId2 = dBodyCreate (wId);
		dBodySetPosition (bId2, 0, 0, 0);

		jId   = dJointCreateUniversal (wId, null);


		dJointAttach (jId, null, bId2);
		dJointSetUniversalAnchor (jId, 0, 0, 0);
	}

	//        ~Fixture_dxJointUniversal_B2_At_Zero_Default_Axes()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy (wId);
	}

	static DWorld wId;

	DBody bId2;

	DUniversalJoint jId;


	// =========================================================================
	// Test ONE BODY behavior
	// =========================================================================


	// Test when there is only one body at position two on the joint
	//TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
	@Test public void test_dJointGetUniversalAngle1_1Body_B2()
	{
		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

		RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (0, angle1.d, 1e-4);
		CHECK_CLOSE (0, angle2.d, 1e-4);

		DVector3 axis1 = new DVector3();
		dJointGetUniversalAxis1 (jId, axis1);

		DVector3 axis2 = new DVector3();
		dJointGetUniversalAxis2 (jId, axis2);

		DMatrix3 R = new DMatrix3();

		double ang1 = (0.0);

		double ang2 = (0.23);
		dRFromAxisAndAngle (R, axis2.get0(), axis2.get1(), axis2.get2(), ang2);
		dBodySetRotation (bId2, R);



		CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (ang1, angle1.d, 1e-4);
		CHECK_CLOSE (-ang2, angle2.d, 1e-4);



		DMatrix3 I = new DMatrix3();
		I.setIdentity(); // Set the rotation of the body to be the Identity (i.e. zero)
		dBodySetRotation (bId2, I);

		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


		dJointSetUniversalAxis2 (jId, -axis2.get0(), -axis2.get1(), -axis2.get2());

		dBodySetRotation (bId2, R);

		CHECK_CLOSE (-ang1, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (-ang1, angle1.d, 1e-4);
		CHECK_CLOSE (ang2, angle2.d, 1e-4);

		// Test the same rotation, when axis1 is inverted and axis2 is default
		dBodySetRotation (bId2, I);

		dJointSetUniversalAxis1 (jId, -axis1.get0(), -axis1.get1(), -axis1.get2());
		dJointSetUniversalAxis2 (jId, axis2.get0(), axis2.get1(), axis2.get2());


		dBodySetRotation (bId2, R);

		CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (ang1, angle1.d, 1e-4);
		CHECK_CLOSE (-ang2, angle2.d, 1e-4);
	}

	// Rotate B2 by 90deg around X then back to original position
	//
	//   ^
	//   |  => <---
	//   |
	//  B2      B2
	//
	// Start with a Delta of 90deg
	//            ^
	//  <---  =>  |
	//            |
	//   B2      B2
	//TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
	@Test public void test_dJointSetUniversalAxisOffset_1Body_B2_90Deg()
	{
		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


		DVector3 axis = new DVector3();
		dJointGetUniversalAxis2 (jId, axis);

		double ang2 = d2r((90.0));

		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
		dBodySetRotation (bId2, R);

		CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

		dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(), 0, -ang2);
		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


		R.setIdentity(); // Set the rotation of the body to be zero
		dBodySetRotation (bId2, R);

		CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);
	}

	// Rotate B2 by -0.23rad around Y then back to original position
	//
	//   ^         ^
	//   |  =>    /
	//   |       /
	//  B2      B2
	//
	// Start with an offset of -0.23rad
	//     ^     ^
	//    /  =>  |
	//   /       |
	//   B2     B2
	//TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
	@Test public void test_dJointSetUniversalAxis2Offset_1Body_B2_Minus0_23rad()
	{
		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

		RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (0, angle1, 1e-4);
		CHECK_CLOSE (0, angle2, 1e-4);

		DVector3 axis = new DVector3();
		dJointGetUniversalAxis2 (jId, axis);

		double ang1 = 0;
		double ang2 = (-0.23);


		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
		dBodySetRotation (bId2, R);


		CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (ang1, angle1, 1e-4);
		CHECK_CLOSE (-ang2, angle2, 1e-4);


		dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(),
				ang1, -ang2);
		CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

		R.setIdentity(); // Set the rotation of the body to be zero
		dBodySetRotation (bId2, R);

		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (0, angle1, 1e-4);
		CHECK_CLOSE (0, angle2, 1e-4);
	}


}