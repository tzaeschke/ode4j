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

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointPiston;

import static org.ode4j.cpp.internal.ApiCppBody.*;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop1;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop1;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

////////////////////////////////////////////////////////////////////////////////
// Default Position:
// Position Body1 (1, 0, 0)
// Position Body2 (3, 0, 0)
// Angchor        (2, 0, 0)
// AxisR          (0, 1, 0)
// Axis2          (0, 0, 1)
// AxisP1         (1, 0, 0)
//
/// <PRE>
///^Z                             |- Anchor point
/// |     Body_1                  |                       Body_2
/// |     +---------------+       V                       +------------------+
/// |    /               /|                             /                  /|
/// |   /               / +       |--      ______      /                  / +
/// .- /      x        /./........x.......(_____()..../         x        /.......> axis
///   +---------------+ /         |--                +------------------+ /        X
///   |               |/                             |                  |/
///   +---------------+                              +------------------+
///          |                                                 |
///          |                                                 |
///          |------------------> <----------------------------|
///              anchor1                  anchor2
///
///
/// Axis Y is going into the page
////////////////////////////////////////////////////////////////////////////////
//  SUITE(JointPiston)
//  {
public class PistonGetInfo1_Fixture_1  {
	public PistonGetInfo1_Fixture_1() {
		wId = dWorldCreate();

		bId1 = dBodyCreate(wId);
		dBodySetPosition(bId1, 1, 0, 0);

		bId2 = dBodyCreate(wId);
		dBodySetPosition(bId2, 3, 0, 0);


		jId = dJointCreatePiston(wId, null);
		joint = (DxJointPiston)jId;

		dJointAttach(jId, bId1, bId2);

		dJointSetPistonAnchor (jId, 2, 0, 0);
	}

	//~PistonGetInfo1_Fixture_1() {
	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	DPistonJoint jId;
	DxJointPiston joint;

	static DWorld wId;

	DBody bId1;
	DBody bId2;

	DxJoint.Info1 info = new DxJoint.Info1();
	//};


	////////////////////////////////////////////////////////////////////////////////
	// Test when there is no limits.
	// The 2 bodies stay aligned.
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PistonGetInfo1_Fixture_1,
	@Test public void test1_SetPistonParam()
	{
		dJointSetPistonParam(jId, dParamLoStop1, -dInfinity);
		dJointSetPistonParam(jId, dParamHiStop1,  dInfinity);
		dJointSetPistonParam(jId, dParamLoStop2, -M_PI);
		dJointSetPistonParam(jId, dParamHiStop2 , M_PI);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(4, info.m);
	}


	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// The Body 2 is moved -100 unit then at 100
	//
	// Default value for axisR1 = 1,0,0
	// Default value for axisP1 = 0,1,0
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PistonGetInfo1_Fixture_1,
	@Test public void test1_GetPistonParam()
	{
		dJointSetPistonParam(jId, dParamLoStop1, -10);
		dJointSetPistonParam(jId, dParamHiStop1,  10);

		dBodySetPosition(bId2, (-100.0), (0.0), (0.0));

		joint.getInfo1(info);

		CHECK_EQUAL(2, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(5, info.m);


		dBodySetPosition(bId2, (100.0), (0.0), (0.0));

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(1, joint.limotP.limit);
		CHECK_EQUAL(5, info.m);

		// Reset Position and test
		dBodySetPosition(bId2, 1, 0, 0);
		DMatrix3 R_final = new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1 );
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);


		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(4, info.m);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// and the rotoide at -45deg and 45deg.
	// The Body 2 is rotated by 90deg around the axis
	//
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PistonGetInfo1_Fixture_1,
	@Test public void test2_PistonGetInfo1()
	{
		dJointSetPistonParam(jId, dParamLoStop1, -10);
		dJointSetPistonParam(jId, dParamHiStop1,  10);
		dJointSetPistonParam(jId, dParamLoStop2, -M_PI/4.0);
		dJointSetPistonParam(jId, dParamHiStop2, M_PI/4.0);

		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);

		joint.getInfo1(info);

		CHECK_EQUAL(1, joint.limotR.limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(5, info.m);

		// Reset Position and test
		DMatrix3 R_final =  new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1 );
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(4, info.m);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// and for rotoide at -45deg and 45deg.
	// The Body 2 is rotated by 90deg around the axis
	// Body1 is moved at X=100
	//
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PistonGetInfo1_Fixture_1,
	@Test public void test3_PistonGetInfo1()
	{
		dJointSetPistonParam(jId, dParamLoStop1, -10);
		dJointSetPistonParam(jId, dParamHiStop1,  10);
		dJointSetPistonParam(jId, dParamLoStop2, -M_PI/4.0);
		dJointSetPistonParam(jId, dParamHiStop2, M_PI/4.0);


		dBodySetPosition (bId1, (100.0), (0.0), (0.0));

		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);

		joint.getInfo1(info);

		CHECK_EQUAL(2, joint.limotP.limit);
		CHECK_EQUAL(1, joint.limotR.limit);

		CHECK_EQUAL(6, info.m);

		// Reset Position and test
		dBodySetPosition(bId1, 1, 0, 0);

		DMatrix3 R_final =  new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1 );
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(4, info.m);
	}
}