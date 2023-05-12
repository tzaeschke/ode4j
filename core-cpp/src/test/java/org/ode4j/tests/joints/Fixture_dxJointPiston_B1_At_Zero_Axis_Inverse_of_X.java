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

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddForce;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePiston;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPistonPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPistonPositionRate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPistonAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPistonAnchorOffset;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPistonAxis;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointPiston;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

/**
 * Only body 1
 * The body are positionned at (0, 0, 0), with no rotation
 * The joint is a Piston Joint
 * Axis is in the oppsite X axis
 * Anchor at (0, 0, 0)
 */
public class Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X extends TestSuperClass
{
	public Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X()
	{
		wId = dWorldCreate();

		bId1 = dBodyCreate (wId);
		dBodySetPosition (bId1, 0, 0, 0);

		jId   = dJointCreatePiston (wId, null);
		joint = (DxJointPiston) jId;


		dJointAttach (jId, bId1, null);

		dJointSetPistonAxis (jId, axis.get0(), axis.get1(), axis.get2());
	}

	//        ~Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X()
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy (wId);
	}

	static DWorld wId;

	DBody bId1;

	DPistonJoint jId;
	DxJointPiston joint;

	static DVector3C axis = new DVector3(-1, 0, 0);

	static double offset = 3.1;
	//    };
	//    const dVector3 Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X::axis =
	//    {
	//        -1, 0, 0
	//    };
	//    const dReal    Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X::offset =  (3.1);

	// Move 1st body offset unit in the X direction
	//
	//  X------->       X--------->  <--- Axis
	//  B1          =>     B1
	//
	// Start with a Offset of offset unit
	//
	//  X------->       X--------->  <--- Axis
	//     B1       =>  B1
	//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
	@Test public void test_dJointSetPistonAxisOffset_B1_OffsetUnit()
	{
		dJointSetPistonAnchor (jId, 0, 0, 0);

		CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

		dBodySetPosition (bId1, offset, 0, 0);

		CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

		dJointSetPistonAnchorOffset (jId, 0, 0, 0,
				-offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
		CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

		dBodySetPosition (bId1, 0, 0, 0);
		CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
	}

	// Move 1st body offset unit in the opposite X direction
	//
	//  X------->          X--------->   <--- Axis
	//  B1          =>  B1
	//
	// Start with a Offset of -offset unit
	//
	//      X------->      X--------->   <--- Axis
	//  B1            =>   B1
	//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
	@Test public void test_dJointSetPistonAxisOffset_B1_Minus_OffsetUnit()
	{
		dJointSetPistonAnchor (jId, 0, 0, 0);

		CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

		dBodySetPosition (bId1, -offset, 0, 0);

		CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

		dJointSetPistonAnchorOffset (jId, 0, 0, 0,
				offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
		CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

		dBodySetPosition (bId1, 0, 0, 0);
		CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
	}
	// ==========================================================================
	// Test Position Rate
	// ==========================================================================

	// Apply force on 1st body in the X direction also the Axis direction
	//
	//  X------->       X---------> <-- Axis
	//  B1  F->      =>     B1
	//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
	@Test public void test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B1()
	{
		CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

		dBodyAddForce (bId1, 1.0, 0, 0);
		dWorldQuickStep (wId, 1.0);

		CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
	}

	// Apply force on 1st body in the inverse X direction
	//
	//  X------->           X---------> <-- Axis
	//  B1  <-F      => B1
	//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
	@Test public void test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B1()
	{
		CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

		dBodyAddForce (bId1, -1.0, 0, 0);
		dWorldQuickStep (wId, 1.0);

		CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
	}


}