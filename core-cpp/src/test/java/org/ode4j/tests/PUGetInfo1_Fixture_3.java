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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePU;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUPositionRate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAnchor;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointPU;

////////////////////////////////////////////////////////////////////////////////
// Texture for testing the PositionRate
//
// Default Position:
//   Position Body1 (3, 0, 0)
//   Position Body2 (1, 0, 0)
//   Angchor        (2, 0, 0)
//   Axis1          (0, 1, 0)
//   Axis2          (0, 0, 1)
//   AxisP1         (1, 0, 0)
//
// Default velocity:
//   Body 1 lvel=( 0, 0, 0)    avel=( 0, 0, 0)
//   Body 2 lvel=( 0, 0, 0)    avel=( 0, 0, 0)
//
//
//               Y                ^ Axis2
//              ^                 |
//             /                  |     ^ Axis1
// Z^         /                   |    /
//  |        / Body 2             |   /         Body 1
//  |       /  +---------+        |  /          +-----------+
//  |      /  /         /|        | /          /           /|
//  |     /  /         / +        _/     -    /           / +
//  |    /  /         /-/--------(_)----|--- /-----------/-------> AxisP
//  |   /  +---------+ /                 -  +-----------+ /
//  |  /   |         |/                     |           |/
//  | /    +---------+                      +-----------+
//  |/
//  .-----------------------------------------> X
//             |----------------->
//             Anchor2           <--------------|
//                               Anchor1
//
////////////////////////////////////////////////////////////////////////////////
public class PUGetInfo1_Fixture_3  {
	public PUGetInfo1_Fixture_3() {
		wId = dWorldCreate();

		bId1 = dBodyCreate(wId);
		dBodySetPosition(bId1, 3, 0, 0);

		bId2 = dBodyCreate(wId);
		dBodySetPosition(bId2, 1, 0, 0);


		jId = dJointCreatePU(wId, null);
		joint = (DxJointPU)jId;

		dJointAttach(jId, bId1, bId2);
		dJointSetPUAnchor (jId, 2, 0, 0);

		dBodySetLinearVel (bId1, (0.0), (0.0), (0.0));
		dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

		dBodySetLinearVel (bId2, (0.0), (0.0), (0.0));
		dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

	}

	//~PUGetInfo1_Fixture_3() {
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	DPUJoint jId;
	DxJointPU joint;

	static DWorld wId;

	DBody bId1;
	DBody bId2;

	DxJoint.Info1 info = new DxJoint.Info1();
	//};

	////////////////////////////////////////////////////////////////////////////////
	// Position Body1 [3, 0, 0]
	// Position Body2 [1, 0, 0]
	// Axis of the prismatic [1, 0, 0]
	// Axis1                 [0, 1, 0]
	// Axis2                 [0, 0, 1]
	//
	// Move at the same speed
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PUGetInfo1_Fixture_3,
	@Test public void test1_GetPUPositionRate()
	{
		// They move with the same linear speed
		// Angular speed == 0
		dBodySetLinearVel(bId1, (0.0), (3.33), (0.0));
		dBodySetLinearVel(bId2, (0.0), (3.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetLinearVel(bId1, (1.11), (3.33), (0.0));
		dBodySetLinearVel(bId2, (1.11), (3.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetLinearVel(bId1, (1.11), (3.33), (2.22));
		dBodySetLinearVel(bId2, (1.11), (3.33), (2.22));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );


		// Reset for the next set of test.
		dBodySetLinearVel(bId1, (0.0), (0.0), (0.0));
		dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

		dBodySetLinearVel(bId2, (0.0), (0.0), (0.0));
		dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));


		// They move with the same angular speed
		// linear speed == 0

		dBodySetAngularVel(bId1, (1.22), (0.0), (0.0));
		dBodySetAngularVel(bId2, (1.22), (0.0), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetAngularVel(bId1, (1.22), (2.33), (0.0));
		dBodySetAngularVel(bId2, (1.22), (2.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetAngularVel(bId1, (1.22), (2.33), (3.44));
		dBodySetAngularVel(bId2, (1.22), (2.33), (3.44));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );
	}


	////////////////////////////////////////////////////////////////////////////////
	// Position Body1 [3, 0, 0]
	// Position Body2 [1, 0, 0]
	// Axis of the prismatic [1, 0, 0]
	// Axis1                 [0, 1, 0]
	// Axis2                 [0, 0, 1]
	//
	// Only the first body moves
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PUGetInfo1_Fixture_3,
	@Test public void GetPUPositionRate_Bodies_in_line_B1_moves()
	{
		dBodySetLinearVel(bId1, (3.33), (0.0), (0.0)); // This is impossible but ...
		CHECK_EQUAL((3.33), dJointGetPUPositionRate (jId) );

		dBodySetLinearVel(bId1, (0.0), (3.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetLinearVel(bId1, (0.0), (0.0), (3.33));     // This is impossible but ...
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );


		// Only the first body as angular velocity
		dBodySetAngularVel(bId1, (1.22), (0.0), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetAngularVel(bId1, (0.0), (2.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetAngularVel(bId1, (0.0), (0.0), (5.55));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );
	}

	////////////////////////////////////////////////////////////////////////////////
	// Position Body1 [3, 0, 0]
	// Position Body2 [1, 0, 0]
	// Axis of the prismatic [1, 0, 0]
	// Axis1                 [0, 1, 0]
	// Axis2                 [0, 0, 1]
	//
	// Only the second body moves
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PUGetInfo1_Fixture_3,
	@Test public void GetPUPositionRate_Bodies_in_line_B2_moves()
	{
		// The length was at zero and this will give an negative length
		dBodySetLinearVel(bId2, (3.33), (0.0), (0.0));
		CHECK_EQUAL((-3.33), dJointGetPUPositionRate (jId) );

		dBodySetLinearVel(bId2, (0.0), (3.33), (0.0));      // This is impossible but ...
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetLinearVel(bId2, (0.0), (0.0), (3.33));     // This is impossible but ...
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );


		// Only angular velocity
		dBodySetAngularVel(bId2, (1.22), (0.0), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetAngularVel(bId2, (0.0), (2.33), (0.0));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

		dBodySetAngularVel(bId2, (0.0), (0.0), (5.55));
		CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );
	}
}