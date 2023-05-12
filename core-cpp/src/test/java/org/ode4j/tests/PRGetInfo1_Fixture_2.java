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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePR;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamCFM2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop2;
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
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointPR;

// The joint is now powered. (i.e. info.fmax > 0
public class PRGetInfo1_Fixture_2  {
	public PRGetInfo1_Fixture_2() {
		wId = dWorldCreate();

		bId1 = dBodyCreate(wId);
		dBodySetPosition(bId1, 0, -1, 0);

		bId2 = dBodyCreate(wId);
		dBodySetPosition(bId2, 0, 1, 0);


		jId = dJointCreatePR(wId, null);
		joint = (DxJointPR)jId;

		dJointAttach(jId, bId1, bId2);
		dJointSetPRAnchor (jId, (0.0), (0.0), (0.0));

		joint.limotP.fmax = 1;
	}

	//~PRGetInfo1_Fixture_2() {
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


	////////////////////////////////////////////////////////////////////////////////
	// Test when there is no limits.
	// The 2 bodies stay align.
	//
	// Default value for axisR1 = 1,0,0
	// Default value for axisP1 = 0,1,0
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PRGetInfo1_Fixture_2,
	@Test public void test1_PRGetInfo1()
	{
		dJointSetPRParam(jId, dParamLoStop, -dInfinity);
		dJointSetPRParam(jId, dParamHiStop,  dInfinity);
		dJointSetPRParam(jId, dParamLoStop2, -M_PI);
		dJointSetPRParam(jId, dParamHiStop2, M_PI);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(5, info.m);
	}


	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// The Body 2 is moved -100 unit then at 100
	//
	// Default value for axisR1 = 1,0,0
	// Default value for axisP1 = 0,1,0
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PRGetInfo1_Fixture_2,
	@Test public void test2_PRGetInfo1()
	{

		dJointSetPRParam(jId, dParamLoStop, -10);
		dJointSetPRParam(jId, dParamHiStop,  10);
		dJointSetPRParam(jId, dParamLoStop2, -M_PI);
		dJointSetPRParam(jId, dParamHiStop2, M_PI);


		dBodySetPosition(bId2, 0, -100, 0);

		joint.getInfo1(info);

		CHECK_EQUAL(2, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(5, info.m);


		dBodySetPosition(bId2, 0, 100, 0);

		joint.getInfo1(info);

		CHECK_EQUAL(1, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(5, info.m);

		// Reset Position and test
		dBodySetPosition(bId2, 0, 1, 0);
		DMatrix3 R_final = new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1);
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(5, info.m);
	}

	////////////////////////////////////////////////////////////////////////////////
	// Test when there is limits for the prismatic at -10 and 10
	// and for the rotoide at -45deg and 45deg
	// The Body 2 is only rotated by 90deg since the rotoide limits are not
	// used this should not change the limit value.
	//
	// Default value for axisR1 = 1,0,0
	// Default value for axisP1 = 0,1,0
	//
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PRGetInfo1_Fixture_2,
	@Test public void test3_PRGetInfo1()
	{

		dJointSetPRParam(jId, dParamLoStop, -10);
		dJointSetPRParam(jId, dParamHiStop,  10);
		dJointSetPRParam(jId, dParamLoStop2, -M_PI/4.0);
		dJointSetPRParam(jId, dParamHiStop2, M_PI/4.0);


		DMatrix3 R = new DMatrix3();
		dBodySetPosition (bId2, 0, 0, 100);
		dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(1, joint.limotR.limit);
		CHECK_EQUAL(6, info.m);

		// Reset Position and test
		dBodySetPosition(bId2, 0, 1, 0);
		DMatrix3 R_final = new DMatrix3( 1,0,0,
				0,1,0,
				0,0,1 );
		dBodySetRotation (bId2, R_final);

		joint.getInfo1(info);

		CHECK_EQUAL(0, joint.limotP.limit);
		CHECK_EQUAL(0, joint.limotR.limit);
		CHECK_EQUAL(5, info.m);
	}


	////////////////////////////////////////////////////////////////////////////////
	// Test the setting and getting of parameters
	////////////////////////////////////////////////////////////////////////////////
	//TEST_FIXTURE(PRGetInfo1_Fixture_1,
	@Test public void test_SetPRParam()
	{
		dJointSetPRParam(jId, dParamHiStop, (5.0) );
		CHECK_EQUAL((5.0), joint.limotP.histop);

		dJointSetPRParam(jId, dParamVel, (7.0) );
		CHECK_EQUAL((7.0), joint.limotP.vel);

		// TODO ? TZ: Probably not,
		//#ifdef D_PARAM_NAMES.dParamFudgeFactor1
		//			if (D_PARAM_NAMES.dParamFudgeFactor1) {
		//		dJointSetPRParam(jId, D_PARAM_NAMES.dParamFudgeFactor1, (5.5) );
		//		CHECK_EQUAL((5.5), joint.limotP.get(D_PARAM_NAMES.dParamFudgeFactor));
		//			} //#endif  //TZ unnecessary

		dJointSetPRParam(jId, dParamCFM2, (9.0) );
		CHECK_EQUAL((9.0), joint.limotR.normal_cfm);

		dJointSetPRParam(jId, dParamStopERP2, (11.0) );
		CHECK_EQUAL((11.0), joint.limotR.stop_erp);
	}

	//TEST_FIXTURE(PRGetInfo1_Fixture_1,
	@Test public void test_GetPRParam()
	{
		joint.limotP.histop = (5.0);
		CHECK_EQUAL(joint.limotP.histop,
				dJointGetPRParam(jId, dParamHiStop) );

		joint.limotP.vel = (7.0);

		CHECK_EQUAL(joint.limotP.vel,
				dJointGetPRParam(jId, dParamVel) );

		//#ifdef D_PARAM_NAMES.dParamFudgeFactor1  //TZ unnecessary
		//		joint.limotP.set(D_PARAM_NAMES.dParamFudgeFactor, 5.5) ;//) =  (5.5);

		//		CHECK_EQUAL(joint.limotP.get(D_PARAM_NAMES.dParamFudgeFactor),
		//				dJointGetPRParam(jId, D_PARAM_NAMES.dParamFudgeFactor1) );
		//#endif

		joint.limotR.normal_cfm = (9.0);
		CHECK_EQUAL(joint.limotR.normal_cfm,
				dJointGetPRParam(jId, dParamCFM2) );

		joint.limotR.stop_erp = (11.0);
		CHECK_EQUAL(joint.limotR.stop_erp,
				dJointGetPRParam(jId, dParamStopERP2) );
	}

}