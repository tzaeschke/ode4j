/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zäschke      *
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
package org.ode4j.tests;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetLinearVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePR;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRPositionRate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAnchor;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointPR;

////////////////////////////////////////////////////////////////////////////////
// Fixture for testing the PositionRate
//
// The second body is at 90deg w.r.t. the first body
//
//
// Default Position
//                       ^Z
//                       |
//                       |
//
//                     +---+
//                     |   |Body2
//                     |   |
//                     |   |
//                     +---+
//                       |
//                       |
//                       |
//                       |            Body1
//                     R _      -    +-----------+
//                      (_)----|-----|           |  ----->Y
//                              -    +-----------+
//
// N.B. X is comming out of the page
////////////////////////////////////////////////////////////////////////////////
public class PRGetInfo1_Fixture_4  {
	public PRGetInfo1_Fixture_4() {
		wId = dWorldCreate();

		bId1 = dBodyCreate(wId);
		dBodySetPosition(bId1, 0, 1, 0);

		bId2 = dBodyCreate(wId);
		dBodySetPosition(bId2, 0, 0, 1);

		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
		dBodySetRotation (bId2, R);


		jId = dJointCreatePR(wId, null);
		joint = (DxJointPR)jId;

		dJointAttach(jId, bId1, bId2);
		dJointSetPRAnchor (jId, (0.0), (0.0), (0.0));

		dBodySetLinearVel(bId1, (0.0), (0.0), (0.0));
		dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

		dBodySetLinearVel(bId2, (0.0), (0.0), (0.0));
		dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

	}

	//~PRGetInfo1_Fixture_4() {
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	DPRJoint jId;
	DxJointPR joint;

	static DWorld wId;

	DBody bId1;
	DBody bId2;

	DxJoint.Info1 info = new DxJoint.Info1();
	//};


	////////////////////////////////////////////////////////////////////////////////
	// Position Body1 [0,  1, 0]
	// Position Body2 [0,  0, 1]
	// Axis of the prismatic [0, 1, 0]
	// Axis of the rotoide   [1, 0, 0]
	//
	// Only the first body moves
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PRGetInfo1_Fixture_4,
	@Test public void GetPRPositionRate_Bodies_at90deg_B1_moves()
	{
		dBodySetLinearVel(bId1, (3.33), (0.0), (0.0)); // This is impossible but ...
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

		// The length was at zero and this will give an negative length
		dBodySetLinearVel(bId1, (0.0), (3.33), (0.0));
		CHECK_EQUAL((3.33), dJointGetPRPositionRate (jId) );

		dBodySetLinearVel(bId1, (0.0), (0.0), (3.33));     // This is impossible but ...
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );


		// Only angular velocity
		dBodySetAngularVel(bId1, (1.22), (0.0), (0.0));
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

		dBodySetAngularVel(bId1, (0.0), (2.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

		dBodySetAngularVel(bId1, (0.0), (0.0), (5.55));
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );
	}

	////////////////////////////////////////////////////////////////////////////////
	// Position Body1 [0, 1, 0]
	// Position Body2 [0, 0, 1]
	// Axis of the prismatic [0, 1, 0]
	// Axis of the rotoide   [1, 0, 0]
	//
	// Only the second body moves
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PRGetInfo1_Fixture_4,
	@Test public void  GetPRPositionRate_Bodies_at90deg_B2_moves()
	{
		dBodySetLinearVel(bId2, (3.33), (0.0), (0.0)); // This is impossible but ...
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

		dBodySetLinearVel(bId2, (0.0), (3.33), (0.0));
		CHECK_EQUAL((-3.33), dJointGetPRPositionRate (jId) );

		dBodySetLinearVel(bId2, (0.0), (0.0), (3.33));     // This is impossible but ...
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );


		// Only angular velocity
		dBodySetAngularVel(bId2, (1.22), (0.0), (0.0));
		CHECK_EQUAL((-1.0*1.22), dJointGetPRPositionRate (jId) );

		dBodySetAngularVel(bId2, (0.0), (2.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

		dBodySetAngularVel(bId2, (0.0), (0.0), (5.55));
		CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );
	}

}