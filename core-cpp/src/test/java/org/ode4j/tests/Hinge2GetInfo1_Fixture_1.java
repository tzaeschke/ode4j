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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateHinge2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHinge2Anchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetHinge2Param;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointHinge2;

////////////////////////////////////////////////////////////////////////////////
// Testing the Hinge2 Joint
//
//  public class SUITE_JointHinge2
//  {
public class Hinge2GetInfo1_Fixture_1 {
	public Hinge2GetInfo1_Fixture_1() {
		wId = dWorldCreate();

		bId1 = dBodyCreate(wId);
		dBodySetPosition(bId1, 0, -1, 0);

		bId2 = dBodyCreate(wId);
		dBodySetPosition(bId2, 0, 1, 0);


		jId = dJointCreateHinge2(wId, null);

		dJointAttach(jId, bId1, bId2);

		dJointSetHinge2Anchor (jId, 0.0, 0.0, 0.0);
	}

	@AfterClass
	public static void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	DHinge2Joint jId;

	static DWorld wId;

	DBody bId1;
	DBody bId2;

	DxJoint.Info1 info;
	//		};

	//		void TEST_FIXTURE(Hinge2GetInfo1_Fixture_1, test_hinge2GetInfo1)
	@Test
	public void test_hinge2GetInfo1()
	{
	    /*
		//       ^Y
		//     |---|                             HiStop
		//     |   |                     ^Y         /
		//     |B_2|                     |       /
		//     |---|                     |    /
		//       |               -----  | /
		// Z <-- *            Z<--|B_2|--*
		//     / | \              -----  | \
		//    /|---|\                  |---| \
		//   / |   | \                 |   |   \
		//  /  |B_1|  \                |B_1|     \
		// /   |---|   \               |---|       \
		//LoStop        HiStop                   LoStop
		//
		//
		//
		//
		 */
		DMatrix3 R = new DMatrix3();

		dJointSetHinge2Param(jId, dParamLoStop, -M_PI/4.0);
		dJointSetHinge2Param(jId, dParamHiStop, M_PI/4.0);

		DxJoint.Info1 info = new DxJoint.Info1();


		DxJointHinge2 joint = (DxJointHinge2)jId;

		// Original position inside the limits
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(4, info.m);



		// Move the body outside the Lo limits
		dBodySetPosition (bId2, 0, 0, 1);
		dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);
		joint.getInfo1(info);
		CHECK_EQUAL(1, joint.getLimot1().getLimit());
		CHECK_EQUAL(5, info.m);


		// Return to original position
		// Keep the limits
		dBodySetPosition (bId2, 0, 1, 0);
		dRFromAxisAndAngle (R, 1, 0.0, 0.0, 0.0);
		dBodySetRotation (bId2, R);
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(4, info.m);


		// Move the body outside the Lo limits
		dBodySetPosition (bId2, 0, 0, 1);
		dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
		dBodySetRotation (bId2, R);
		joint.getInfo1(info);
		CHECK_EQUAL(1, joint.getLimot1().getLimit());
		CHECK_EQUAL(5, info.m);



		// Return to original position
		// and remove the limits
		dBodySetPosition (bId2, 0, 1, 0);
		dRFromAxisAndAngle (R, 1, 0.0, 0.0, 0.0);
		dBodySetRotation (bId2, R);
		dJointSetHinge2Param(jId, dParamLoStop, -2*M_PI);
		dJointSetHinge2Param(jId, dParamHiStop,  2*M_PI);
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(4, info.m);


		// Set the limits
		// Move pass the Hi limits
		dJointSetHinge2Param(jId, dParamLoStop, -M_PI/4.0);
		dJointSetHinge2Param(jId, dParamHiStop,  M_PI/4.0);
		dBodySetPosition (bId2, 0, 0, 1);
		dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
		dBodySetRotation (bId2, R);
		joint.getInfo1(info);
		CHECK_EQUAL(2, joint.getLimot1().getLimit());
		CHECK_EQUAL(5, info.m);


		// Return to original position
		// Keep the limits
		dBodySetPosition (bId2, 0, 1, 0);
		dRFromAxisAndAngle (R, 1, 0.0, 0.0, 0.0);
		dBodySetRotation (bId2, R);
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(4, info.m);


		// Move the pass the Hi limit
		dBodySetPosition (bId2, 0, 0, 1);
		dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
		dBodySetRotation (bId2, R);
		joint.getInfo1(info);
		CHECK_EQUAL(2, joint.getLimot1().getLimit());
		CHECK_EQUAL(5, info.m);


		// Return to original position
		// and remove the limits
		dBodySetPosition (bId2, 0, 1, 0);
		dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
		dBodySetRotation (bId2, R);
		dJointSetHinge2Param(jId, dParamLoStop, -2*M_PI);
		dJointSetHinge2Param(jId, dParamHiStop,  2*M_PI);
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(4, info.m);


		/// Motorize the first joint angle
		dJointSetHinge2Param(jId, dParamFMax, 2);
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(5, info.m);


		/// Motorize the second joint angle
		dJointSetHinge2Param(jId, dParamFMax2, 2);
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(6, info.m);

		/// Unmotorize the first joint angle
		dJointSetHinge2Param(jId, dParamFMax, 0);
		joint.getInfo1(info);
		CHECK_EQUAL(0, joint.getLimot1().getLimit());
		CHECK_EQUAL(5, info.m);
	}
	//	} // End of SUITE(JointHinge2)D_PARAM_NAMES
}