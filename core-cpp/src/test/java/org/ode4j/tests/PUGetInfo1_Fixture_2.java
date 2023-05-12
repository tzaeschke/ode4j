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
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePU;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamBounce3;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamCFM2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFudgeFactor1;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop1;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop3;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop1;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop3;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamStopERP2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamVel;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointPU;

////////////////////////////////////////////////////////////////////////////////
// Default Position:
// Position Body1 (3, 0, 0)
// Position Body2 (1, 0, 0)
// Angchor        (2, 0, 0)
// Axis1          (0, 1, 0)
// Axis2          (0, 0, 1)
// AxisP1         (1, 0, 0)
//
// The motor on axis1 is now powered. (i.e. joint->getLimot1()->fmax > 0
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
//  .------------------------------------.---> X
//             |----------------->
//             Anchor2           <--------------|
//                               Anchor1
//
////////////////////////////////////////////////////////////////////////////////
public class PUGetInfo1_Fixture_2  {
	public PUGetInfo1_Fixture_2() {
		wId = dWorldCreate();

		bId1 = dBodyCreate(wId);
		dBodySetPosition(bId1, 3, 0, 0);

		bId2 = dBodyCreate(wId);
		dBodySetPosition(bId2, 1, 0, 0);


		jId = dJointCreatePU(wId, null);
		joint = (DxJointPU)jId;

		dJointAttach(jId, bId1, bId2);

		dJointSetPUAnchor (jId, 2, 0, 0);

		joint.getLimot1().fmax = 1;
	}

	//~PUGetInfo1_Fixture_2() {
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
	// Test when there is no limits.
	// The 2 bodies stay aligned.
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PUGetInfo1_Fixture_2,
	@Test public void test0_PUGetInfo1()
	{
		dJointSetPUParam(jId, dParamLoStop1, -M_PI);
		dJointSetPUParam(jId, dParamHiStop1 , M_PI);
		dJointSetPUParam(jId, dParamLoStop2, -M_PI);
		dJointSetPUParam(jId, dParamHiStop2,  M_PI);
		dJointSetPUParam(jId, dParamLoStop3, -dInfinity);
		dJointSetPUParam(jId, dParamHiStop3,  dInfinity);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(4, info.m);
	}


	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// The Body 2 is moved -100 unit then at 100
	//
	// Default value for axisR1 = 1,0,0
	// Default value for axisP1 = 0,1,0
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PUGetInfo1_Fixture_2,
	@Test public void test1_PUGetInfo1()
	{
		dJointSetPUParam(jId, dParamLoStop3, -10);
		dJointSetPUParam(jId, dParamHiStop3,  10);

		dBodySetPosition(bId2, (-100.0), (0.0), (0.0));

		joint.getInfo1(info);


		CHECK_EQUAL(0, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(2, joint.limotP.limit);
		CHECK_EQUAL(5, info.m);


		dBodySetPosition(bId2, (100.0), (0.0), (0.0));

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(1, joint.limotP.limit);
		CHECK_EQUAL(5, info.m);

		// Reset Position and test
		dBodySetPosition(bId2, 1, 0, 0);
		DMatrix3 R_final = new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1 );
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);


		CHECK_EQUAL(0, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(4, info.m);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// and for Axis1 and Axis2 at -45deg and 45deg.
	// The Body 2 is rotated by 90deg around Axis1
	//
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PUGetInfo1_Fixture_2,
	@Test public void test2_PUGetInfo1()
	{
		dJointSetPUParam(jId, dParamLoStop1, -M_PI/4.0);
		dJointSetPUParam(jId, dParamHiStop1, M_PI/4.0);
		dJointSetPUParam(jId, dParamLoStop2, -M_PI/4.0);
		dJointSetPUParam(jId, dParamHiStop2, M_PI/4.0);
		dJointSetPUParam(jId, dParamLoStop3, -10);
		dJointSetPUParam(jId, dParamHiStop3,  10);


		DMatrix3 R = new DMatrix3();
		dBodySetPosition (bId2, 0, 0, 1);
		dRFromAxisAndAngle (R, 0, 1, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);

		joint.getInfo1(info);

		CHECK_EQUAL(1, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(4, info.m);

		// Reset Position and test
		dBodySetPosition(bId2, 1, 0, 0);
		DMatrix3 R_final = new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1 );
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(4, info.m);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// and for Axis1 and Axis2 at -45deg and 45deg.
	// The Body 2 is rotated by 90deg around Axis1 and
	// Body1 is moved at X=100
	//
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PUGetInfo1_Fixture_2,
	@Test public void test3_PUGetInfo1()
	{
		dJointSetPUParam(jId, dParamLoStop1, -M_PI/4.0);
		dJointSetPUParam(jId, dParamHiStop1, M_PI/4.0);
		dJointSetPUParam(jId, dParamLoStop2, -M_PI/4.0);
		dJointSetPUParam(jId, dParamHiStop2, M_PI/4.0);
		dJointSetPUParam(jId, dParamLoStop3, -10);
		dJointSetPUParam(jId, dParamHiStop3,  10);


		dBodySetPosition (bId1, (100.0), (0.0), (0.0));

		DMatrix3 R = new DMatrix3();
		dBodySetPosition (bId2, 0, 0, 1);
		dRFromAxisAndAngle (R, 0, 1, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);

		joint.getInfo1(info);

		CHECK_EQUAL(1, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(2, joint.limotP.limit);
		CHECK_EQUAL(5, info.m);

		// Reset Position and test
		dBodySetPosition(bId1, 3, 0, 0);

		dBodySetPosition(bId2, 1, 0, 0);
		DMatrix3 R_final = new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1 );
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.getLimot1().limit);
		CHECK_EQUAL(0, joint.getLimot2().limit);
		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(4, info.m);
	}



	//TEST_FIXTURE(PUGetInfo1_Fixture_2,
	@Test public void test_SetPUParam()
	{
		dJointSetPUParam(jId, dParamHiStop, (5.0) );
		CHECK_EQUAL((5.0), joint.getLimot1().histop);

		dJointSetPUParam(jId, dParamVel, (7.0) );
		CHECK_EQUAL((7.0), joint.getLimot1().vel);

		//TZ #ifdef D_PARAM_NAMES.dParamFudgeFactor1
		//TODO			dJointSetPUParam(jId, dParamFudgeFactor1, (5.5) );
		//			//TZ CHECK_EQUAL((5.5), joint.getLimot1().D_PARAM_NAMES.dParamFudgeFactor);
		//			CHECK_EQUAL((5.5), joint.getLimot1().fudge_factor);
		// #endif

		dJointSetPUParam(jId, dParamCFM2, (9.0) );
		CHECK_EQUAL((9.0), joint.getLimot2().normal_cfm);

		dJointSetPUParam(jId, dParamStopERP2, (11.0) );
		CHECK_EQUAL((11.0), joint.getLimot2().stop_erp);


		dJointSetPUParam(jId, dParamBounce3, (13.0) );
		CHECK_EQUAL((13.0), joint.limotP.bounce);
	}



	//TEST_FIXTURE(PUGetInfo1_Fixture_1,
	@Test public void test_GetPUParam()
	{
		joint.limotP.histop = (5.0);
		CHECK_EQUAL(joint.getLimot1().histop,
				dJointGetPUParam(jId, dParamHiStop) );

		joint.limotP.vel = (7.0);

		CHECK_EQUAL(joint.getLimot1().vel,
				dJointGetPUParam(jId, dParamVel) );

		//TZ #ifdef D_PARAM_NAMES.dParamFudgeFactor1
		//TZ joint.limotP.D_PARAM_NAMES.dParamFudgeFactor =  (5.5);
		joint.limotP.fudge_factor =  5.5;

		//			CHECK_EQUAL(joint.getLimot1().D_PARAM_NAMES.dParamFudgeFactor,
		//					dJointGetPUParam(jId, D_PARAM_NAMES.dParamFudgeFactor1) );
		CHECK_EQUAL(joint.getLimot1().fudge_factor,
				dJointGetPUParam(jId, dParamFudgeFactor1) );
		//#endif

		joint.getLimot2().normal_cfm = (9.0);
		CHECK_EQUAL(joint.getLimot2().normal_cfm,
				dJointGetPUParam(jId, dParamCFM2) );

		joint.getLimot2().stop_erp = (11.0);
		CHECK_EQUAL(joint.getLimot2().stop_erp,
				dJointGetPUParam(jId, dParamStopERP2) );

		joint.limotP.bounce = (13.0);
		CHECK_EQUAL(joint.limotP.bounce,
				dJointGetPUParam(jId, dParamBounce3) );
	}
}