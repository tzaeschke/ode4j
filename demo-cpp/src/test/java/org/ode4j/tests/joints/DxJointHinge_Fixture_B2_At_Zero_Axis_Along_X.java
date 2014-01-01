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
package org.ode4j.tests.joints;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateHinge;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetHingeAngle;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHingeAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHingeAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHingeAxisOffset;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointHinge;

/**
 * Only one body body2 at (0,0,0)
// The joint is an Hinge Joint.
// Axis is along the X axis
// Anchor at (0, 0, 0)
//
//       ^Y
//       |
//       |
//       |
//       |
//       |
// Z <-- X
 */
public class DxJointHinge_Fixture_B2_At_Zero_Axis_Along_X {
	public DxJointHinge_Fixture_B2_At_Zero_Axis_Along_X()
	{
		wId = dWorldCreate();

		bId2 = dBodyCreate (wId);
		dBodySetPosition (bId2, 0, 0, 0);

		jId   = dJointCreateHinge (wId, null);
		joint = (DxJointHinge) jId;


		dJointAttach (jId, null, bId2);
		dJointSetHingeAnchor (jId, 0, 0, 0);

		axis.set( 1, 0, 0 );
	}

	//~dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	static DWorld wId;

	DBody bId2;


	DHingeJoint jId;
	DxJointHinge joint;

	DVector3 axis = new DVector3();
	//  };

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
	//TEST_FIXTURE (dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X,
	@Test public void test_dJointSetHingeAxisOffset_1Body_B2_90Deg() {
		DMatrix3 R = new DMatrix3();

		dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

		CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

		dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);

		CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

		dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  -M_PI/2.0);
		CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

		dRFromAxisAndAngle (R, 1, 0, 0, 0);
		dBodySetRotation (bId2, R);

		CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
	}

	// Rotate B2 by -0.23rad around X then back to original position
	//
	//   ^         ^
	//   |  =>    /
	//   |       /
	//  B2      B2
	//
	// Start with a Delta of -0.23rad
	//     ^     ^
	//    /  =>  |
	//   /       |
	//   B2     B2
	//TEST_FIXTURE (dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X,
	@Test public void test_dJointSetHingeAxisOffset_1Body_B2_Minus0_23rad() {
		DMatrix3 R = new DMatrix3();

		dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

		CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

		dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
		dBodySetRotation (bId2, R);

		CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

		dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  (0.23));
		CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

		dRFromAxisAndAngle (R, 1, 0, 0, 0);
		dBodySetRotation (bId2, R);

		CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
	}
}