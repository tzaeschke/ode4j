/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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

import org.ode4j.ode.internal.joints.DxJoint;
import org.ode4j.ode.internal.joints.DxJointHinge2;
import org.ode4j.ode.internal.joints.DxJointPiston;
import org.ode4j.ode.internal.joints.DxJointPR;
import org.ode4j.ode.internal.joints.DxJointPU;
import org.ode4j.ode.internal.joints.DxJointUniversal;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.DWorld;

import org.junit.AfterClass;
import org.junit.Test;
import org.junit.experimental.runners.Enclosed;
import org.junit.runner.RunWith;

import static org.ode4j.cpp.OdeCpp.*;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.*;
import static org.ode4j.ode.OdeMath.*;

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joint.cpp
//
//
////////////////////////////////////////////////////////////////////////////////
// #include <UnitTest++.h>
// #include <ode/ode.h>
// #include "../ode/src/joints/joints.h"

@RunWith(Enclosed.class)
public class JointTest {


	////////////////////////////////////////////////////////////////////////////////
	// Testing the Hinge2 Joint
	//
	//	public class SUITE_JointHinge2
	//	{
	public static class Hinge2GetInfo1_Fixture_1 {
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

	////////////////////////////////////////////////////////////////////////////////
	// Testing the Universal Joint
	//
	//	public class SUITE_JointUniversal
	//	{

	public static class UniversalGetInfo1_Fixture_1  {
		public UniversalGetInfo1_Fixture_1() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 0, -1, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 0, 1, 0);


			jId = dJointCreateUniversal(wId, null);

			dJointAttach(jId, bId1, bId2);

			dJointSetUniversalAnchor (jId, 0.0, 0.0, 0.0);
		}

		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy(wId);
		}

		DUniversalJoint jId;

		static DWorld wId;

		DBody bId1;
		DBody bId2;

		DxJoint.Info1 info;
		//			};

		//TEST_FIXTURE(UniversalGetInfo1_Fixture_1, test_hinge2GetInfo1_RotAroundX)
		@Test public void TEST_FIXTURE_UniversalX()
		{
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
			DMatrix3 R = new DMatrix3();

			dJointSetUniversalParam(jId, dParamLoStop, -M_PI/4.0);
			dJointSetUniversalParam(jId, dParamHiStop,  M_PI/4.0);
			dJointSetUniversalParam(jId, dParamLoStop2, -M_PI/4.0);
			dJointSetUniversalParam(jId, dParamHiStop2,  M_PI/4.0);

			DxJoint.Info1 info = new DxJoint.Info1();


			DxJointUniversal joint = (DxJointUniversal)jId;

			// Original position inside the limits
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(4, info.m);

			// Move the body outside the Lo limits
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(1, joint.getLimot1().limit);
			CHECK_EQUAL(5, info.m);


			// Return to original position
			// Keep the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 1, 0.0, 0.0, 0.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(4, info.m);


			// Move the body outside the Lo limits
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(1, joint.getLimot1().limit);
			CHECK_EQUAL(5, info.m);



			// Return to original position
			// and remove the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 1, 0.0, 0.0, 0.0);
			dBodySetRotation (bId2, R);
			dJointSetUniversalParam(jId, dParamLoStop, -2*M_PI);
			dJointSetUniversalParam(jId, dParamHiStop,  2*M_PI);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(4, info.m);


			// Set the limits
			// Move pass the Hi limits
			dJointSetUniversalParam(jId, dParamLoStop, -M_PI/4.0);
			dJointSetUniversalParam(jId, dParamHiStop,  M_PI/4.0);
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(2, joint.getLimot1().limit);
			CHECK_EQUAL(5, info.m);


			// Return to original position
			// Keep the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 1, 0.0, 0.0, 0.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(4, info.m);


			// Move the pass the Hi limit
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(2, joint.getLimot1().limit);
			CHECK_EQUAL(5, info.m);


			// Return to original position
			// and remove the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);
			dJointSetUniversalParam(jId, dParamLoStop, -2*M_PI);
			dJointSetUniversalParam(jId, dParamHiStop,  2*M_PI);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(4, info.m);


			/// Motorize the first joint angle
			dJointSetUniversalParam(jId, dParamFMax, 2);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(5, info.m);


			/// Motorize the second joint angle
			dJointSetUniversalParam(jId, dParamFMax2, 2);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(6, info.m);

			/// Unmotorize the first joint angle
			dJointSetUniversalParam(jId, dParamFMax, 0);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(5, info.m);
		}
		//
		//		TEST_FIXTURE(UniversalGetInfo1_Fixture_1, test_hinge2GetInfo1_RotAroundY)
		@Test public void TEST_FIXTURE_UniversalY()
		{
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
			DMatrix3 R = new DMatrix3();;

			dJointSetUniversalParam(jId, dParamLoStop, -M_PI/4.0);
			dJointSetUniversalParam(jId, dParamHiStop,  M_PI/4.0);
			dJointSetUniversalParam(jId, dParamLoStop2, -M_PI/4.0);
			dJointSetUniversalParam(jId, dParamHiStop2,  M_PI/4.0);

			DxJoint.Info1 info = new DxJoint.Info1();


			DxJointUniversal joint = (DxJointUniversal)jId;

			// Original position inside the limits
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(4, info.m);

			// Move the body outside the Lo limits
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 0, 1, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(1, joint.getLimot2().limit);
			CHECK_EQUAL(5, info.m);


			// Return to original position
			// Keep the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 0, 1, 0, 0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(4, info.m);


			// Move the body outside the Lo limits
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 0, 1, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(1, joint.getLimot2().limit);
			CHECK_EQUAL(5, info.m);



			// Return to original position
			// and remove the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 0, 1, 0, 0);
			dBodySetRotation (bId2, R);
			dJointSetUniversalParam(jId, dParamLoStop2, -2*M_PI);
			dJointSetUniversalParam(jId, dParamHiStop2,  2*M_PI);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(4, info.m);


			// Set the limits
			// Move pass the Hi limits
			dJointSetUniversalParam(jId, dParamLoStop2, -M_PI/4.0);
			dJointSetUniversalParam(jId, dParamHiStop2,  M_PI/4.0);
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 0, 1, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(2, joint.getLimot2().limit);
			CHECK_EQUAL(5, info.m);


			// Return to original position
			// Keep the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 0, 1, 0, 0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(4, info.m);


			// Move the pass the Hi limit
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 0, 1, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);
			joint.getInfo1(info);
			CHECK_EQUAL(2, joint.getLimot2().limit);
			CHECK_EQUAL(5, info.m);


			// Return to original position
			// and remove the limits
			dBodySetPosition (bId2, 0, 1, 0);
			dRFromAxisAndAngle (R, 0, 1, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);
			dJointSetUniversalParam(jId, dParamLoStop2, -2*M_PI);
			dJointSetUniversalParam(jId, dParamHiStop2,  2*M_PI);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(4, info.m);


			/// Motorize the first joint angle
			dJointSetUniversalParam(jId, dParamFMax, 2);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(5, info.m);


			/// Motorize the second joint angle
			dJointSetUniversalParam(jId, dParamFMax2, 2);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(6, info.m);

			/// Unmotorize the first joint angle
			dJointSetUniversalParam(jId, dParamFMax, 0);
			joint.getInfo1(info);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(5, info.m);
		}
		//	} // End of SUITE(JointUniversal)

	}

	// // //
	// Testing the PR Joint
	//
	//		public static class SUITE_JointPR
	//		{
	public static class PRGetInfo1_Fixture_1  {
		public PRGetInfo1_Fixture_1() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 0, -1, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 0, 1, 0);


			jId = dJointCreatePR(wId, null);
			joint = (DxJointPR)jId;

			dJointAttach(jId, bId1, bId2);

			dJointSetPRAnchor (jId, (0.0), (0.0), (0.0));
		}

		//~PRGetInfo1_Fixture_1() {
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
		// Test when there is no limits.
		// The 2 bodies stay aligned.
		//
		// Default value for axisR1 = 1,0,0
		// Default value for axisP1 = 0,1,0
		////////////////////////////////////////////////////////////////////////////////
		//				@Test public void TEST_FIXTURE(PRGetInfo1_Fixture_1, test1_PRGetInfo1_)
		@Test public void test1_PRGetInfo1()
		{
			dJointSetPRParam(jId, dParamLoStop, -dInfinity);
			dJointSetPRParam(jId, dParamHiStop,  dInfinity);
			dJointSetPRParam(jId, dParamLoStop2, -M_PI);
			dJointSetPRParam(jId, dParamHiStop2, M_PI);

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
		//public void TEST_FIXTURE(PRGetInfo1_Fixture_1,
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
			DMatrix3 R_final = new DMatrix3( 1.,0.,0.,0.,
					0.,1.,0.,0.,
					0.,0.,1.,0. );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(4, info.m);
		}

		////////////////////////////////////////////////////////////////////////////////
		// Test when there is limits for the prismatic at -10 and 10
		// and for the rotoide at -45deg and 45deg.
		// The Body 2 is only rotated by 90deg since the rotoide limits are not
		// used this should not change the limit value.
		//
		// Default value for axisR1 = 1,0,0
		// Default value for axisP1 = 0,1,0
		//
		////////////////////////////////////////////////////////////////////////////////
		//public void TEST_FIXTURE(PRGetInfo1_Fixture_1,
		@Test public void test3_PRGetInfo1()
		{
			dJointSetPRParam(jId, dParamLoStop, -10);
			dJointSetPRParam(jId, dParamHiStop,  10);
			dJointSetPRParam(jId, dParamLoStop2, -M_PI/4.0);
			dJointSetPRParam(jId, dParamHiStop2, M_PI/4.0);


			DMatrix3 R = new DMatrix3();
			dBodySetPosition (bId2, 0, 0, 1);
			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(1, joint.limotR.limit);
			CHECK_EQUAL(5, info.m);

			// Reset Position and test
			dBodySetPosition(bId2, 0, 1, 0);
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(4, info.m);
		}
	}


	// The joint is now powered. (i.e. info.fmax > 0
	public static class PRGetInfo1_Fixture_2  {
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
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

			//#ifdef D_PARAM_NAMES.dParamFudgeFactor1
			//			if (D_PARAM_NAMES.dParamFudgeFactor1) {
			System.err.println("FIXME: test_SetPRParam()");
			//TODO		dJointSetPRParam(jId, D_PARAM_NAMES.dParamFudgeFactor1, (5.5) );
			//TODO		CHECK_EQUAL((5.5), joint.limotP.get(D_PARAM_NAMES.dParamFudgeFactor));
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
			System.err.println("FIXME: test_GetPRParam()");
			//TODO		joint.limotP.set(D_PARAM_NAMES.dParamFudgeFactor, 5.5) ;//) =  (5.5);

			//TODO		CHECK_EQUAL(joint.limotP.get(D_PARAM_NAMES.dParamFudgeFactor),
			//TODO				dJointGetPRParam(jId, D_PARAM_NAMES.dParamFudgeFactor1) );
			//#endif

			joint.limotR.normal_cfm = (9.0);
			CHECK_EQUAL(joint.limotR.normal_cfm,
					dJointGetPRParam(jId, dParamCFM2) );

			joint.limotR.stop_erp = (11.0);
			CHECK_EQUAL(joint.limotR.stop_erp,
					dJointGetPRParam(jId, dParamStopERP2) );
		}

	}

	////////////////////////////////////////////////////////////////////////////////
	// Fixture for testing the PositionRate
	//
	// Default Position
	//                       ^Z
	//                       |
	//                       |
	//
	//    Body2              R            Body1
	//   +---------+         _      -    +-----------+
	//   |         |--------(_)----|-----|           |  ----->Y
	//   +---------+                -    +-----------+
	//
	// N.B. X is comming out of the page
	////////////////////////////////////////////////////////////////////////////////
	public static class PRGetInfo1_Fixture_3  {
		public PRGetInfo1_Fixture_3() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 0,  1, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 0, -1, 0);


			jId = dJointCreatePR(wId, null);
			joint = (DxJointPR)jId;

			dJointAttach(jId, bId1, bId2);
			dJointSetPRAnchor (jId, (0.0), (0.0), (0.0));

			dBodySetLinearVel (bId1, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

			dBodySetLinearVel (bId2, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

		}

		//~PRGetInfo1_Fixture_3() {
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
		// Position Body2 [0, -1, 0]
		// Axis of the prismatic [0, 1, 0]
		// Axis of the rotoide   [1, 0, ]0
		//
		// Move at the same speed
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PRGetInfo1_Fixture_3,
		@Test public void test_GetPRPositionRate_1()
		{
			// They move with the same linear speed
			// Angular speed == 0
			dBodySetLinearVel(bId1, (0.0), (3.33), (0.0));
			dBodySetLinearVel(bId2, (0.0), (3.33), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetLinearVel(bId1, (1.11), (3.33), (0.0));
			dBodySetLinearVel(bId2, (1.11), (3.33), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetLinearVel(bId1, (1.11), (3.33), (2.22));
			dBodySetLinearVel(bId2, (1.11), (3.33), (2.22));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );


			// Reset for the next set of test.
			dBodySetLinearVel(bId1, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

			dBodySetLinearVel(bId2, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));


			// They move with the same angular speed
			// linear speed == 0

			dBodySetAngularVel(bId1, (1.22), (0.0), (0.0));
			dBodySetAngularVel(bId2, (1.22), (0.0), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetAngularVel(bId1, (1.22), (2.33), (0.0));
			dBodySetAngularVel(bId2, (1.22), (2.33), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetAngularVel(bId1, (1.22), (2.33), (3.44));
			dBodySetAngularVel(bId2, (1.22), (2.33), (3.44));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );
		}


		////////////////////////////////////////////////////////////////////////////////
		// Position Body1 [0,  1, 0]
		// Position Body2 [0, -1, 0]
		// Axis of the prismatic [0, 1, 0]
		// Axis of the rotoide   [1, 0, ]0
		//
		// Only the first body moves
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PRGetInfo1_Fixture_3,
		@Test public void GetPRPositionRate_Bodies_in_line_B1_moves()
		{
			dBodySetLinearVel(bId1, (3.33), (0.0), (0.0)); // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetLinearVel(bId1, (0.0), (3.33), (0.0));
			CHECK_EQUAL((3.33), dJointGetPRPositionRate (jId) );

			dBodySetLinearVel(bId1, (0.0), (0.0), (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );


			// Only the first body as angular velocity
			dBodySetAngularVel(bId1, (1.22), (0.0), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetAngularVel(bId1, (0.0), (2.33), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetAngularVel(bId1, (0.0), (0.0), (5.55));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );
		}

		////////////////////////////////////////////////////////////////////////////////
		// Position Body1 [0,  1, 0]
		// Position Body2 [0, -1, 0]
		// Axis of the prismatic [0, 1, 0]
		// Axis of the rotoide   [1, 0, ]0
		//
		// Only the second body moves
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PRGetInfo1_Fixture_3,
		@Test public void GetPRPositionRate_Bodies_in_line_B2_moves()
		{
			dBodySetLinearVel(bId2, (3.33), (0.0), (0.0)); // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			// The length was at zero and this will give an negative length
			dBodySetLinearVel(bId2, (0.0), (3.33), (0.0));
			CHECK_EQUAL((-3.33), dJointGetPRPositionRate (jId) );

			dBodySetLinearVel(bId2, (0.0), (0.0), (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );


			// Only angular velocity
			dBodySetAngularVel(bId2, (1.22), (0.0), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetAngularVel(bId2, (0.0), (2.33), (0.0));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );

			dBodySetAngularVel(bId2, (0.0), (0.0), (5.55));
			CHECK_EQUAL((0.0), dJointGetPRPositionRate (jId) );
		}
	}

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
	public static class PRGetInfo1_Fixture_4  {
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

	} // End of SUITE(JointPR)





	// // //
	// Testing the PU Joint
	//
	// //
	////////////////////////////////////////////////////////////////////////////////
	// Default Position:
	// Position Body1 (3, 0, 0)
	// Position Body2 (1, 0, 0)
	// Angchor        (2, 0, 0)
	// Axis1          (0, 1, 0)
	// Axis2          (0, 0, 1)
	// AxisP1         (1, 0, 0)
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
	//	SUITE(JointPU)
	//	{
	public static class PUGetInfo1_Fixture_1  {
		public PUGetInfo1_Fixture_1() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 3, 0, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 1, 0, 0);


			jId = dJointCreatePU(wId, null);
			joint = (DxJointPU)jId;

			dJointAttach(jId, bId1, bId2);

			dJointSetPUAnchor (jId, 2, 0, 0);
		}

		//~PUGetInfo1_Fixture_1() {
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
		//TEST_FIXTURE(PUGetInfo1_Fixture_1,
		@Test public void test1_SetPUParam()
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
			CHECK_EQUAL(3, info.m);
		}


		////////////////////////////////////////////////////////////////////////////////
		// Test when there is limits for the prismatic at -10 and 10
		// The Body 2 is moved -100 unit then at 100
		//
		// Default value for axisR1 = 1,0,0
		// Default value for axisP1 = 0,1,0
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PUGetInfo1_Fixture_1,
		@Test public void test1_GetPUParam()
		{
			dJointSetPUParam(jId, dParamLoStop3, -10);
			dJointSetPUParam(jId, dParamHiStop3,  10);

			dBodySetPosition(bId2, (-100.0), (0.0), (0.0));

			joint.getInfo1(info);


			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(2, joint.limotP.limit);
			CHECK_EQUAL(4, info.m);


			dBodySetPosition(bId2, (100.0), (0.0), (0.0));

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(1, joint.limotP.limit);
			CHECK_EQUAL(4, info.m);

			// Reset Position and test
			dBodySetPosition(bId2, 1, 0, 0);
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);


			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(3, info.m);
		}

		////////////////////////////////////////////////////////////////////////////////
		// Test when there is limits for the prismatic at -10 and 10
		// and for Axis1 and Axis2 at -45deg and 45deg.
		// The Body 2 is rotated by 90deg around Axis1
		//
		//
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PUGetInfo1_Fixture_1,
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(3, info.m);
		}

		////////////////////////////////////////////////////////////////////////////////
		// Test when there is limits for the prismatic at -10 and 10
		// and for Axis1 and Axis2 at -45deg and 45deg.
		// The Body 2 is rotated by 90deg around Axis1 and
		// Body1 is moved at X=100
		//
		//
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PUGetInfo1_Fixture_1,
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.getLimot1().limit);
			CHECK_EQUAL(0, joint.getLimot2().limit);
			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(3, info.m);
		}
	}

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
	public static class PUGetInfo1_Fixture_2  {
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
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
	public static class PUGetInfo1_Fixture_3  {
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

	////////////////////////////////////////////////////////////////////////////////
	// Fixture for testing the PositionRate
	//
	// Default Position:
	// Position Body1 (3, 0, 0)
	// Position Body2 (0, 0, 1)
	// Angchor        (2, 0, 0)
	// Axis1          (0, 1, 0)
	// Axis2          (1, 0, 0)
	// AxisP          (1, 0, 0)
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
	//                       |      ^ Axis1
	//                       |     /
	//                       |    /
	//                       |   /         Body1
	//                     R _      -    +-----------+
	//                      (_)----|-----|           |  ----->X  AxisP, Axis2
	//                              -    +-----------+
	//
	// N.B. Y is going into the page
	////////////////////////////////////////////////////////////////////////////////
	public static class PUGetInfo1_Fixture_4  {
		public PUGetInfo1_Fixture_4() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 3, 0, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 0, 0, 1);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);


			jId = dJointCreatePU(wId, null);
			joint = (DxJointPU)jId;

			dJointAttach(jId, bId1, bId2);
			dJointSetPUAnchor (jId, 2, 0, 0);
			dJointSetPUAxis1 (jId, 0, 1, 0);
			dJointSetPUAxis2 (jId, 1, 0, 0);
			dJointSetPUAxisP (jId, 1, 0, 0);


			dBodySetLinearVel(bId1, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

			dBodySetLinearVel(bId2, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

		}

		//~PUGetInfo1_Fixture_4() {
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
		// Position Body1 (3, 0, 0)
		// Position Body2 (1, 0, 0)
		// Angchor        (2, 0, 0)
		// Axis1          (0, 1, 0)
		// Axis2          (0, 0, 1)
		// AxisP1         (1, 0, 0)
		//
		// Only the first body moves
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PUGetInfo1_Fixture_4,
		@Test public void GetPUPositionRate_Bodies_at90deg_B1_moves()
		{
			dBodySetLinearVel(bId1, (3.33), (0.0), (0.0)); // This is impossible but ...
			CHECK_EQUAL((3.33), dJointGetPUPositionRate (jId) );

			dBodySetLinearVel(bId1, (0.0), (3.33), (0.0));
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

			dBodySetLinearVel(bId1, (0.0), (0.0), (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );


			// Only angular velocity
			dBodySetAngularVel(bId1, (1.22), (0.0), (0.0));
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

			dBodySetAngularVel(bId1, (0.0), (2.33), (0.0));
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

			dBodySetAngularVel(bId1, (0.0), (0.0), (5.55));
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );
		}

		////////////////////////////////////////////////////////////////////////////////
		// Position Body1 (3, 0, 0)
		// Position Body2 (1, 0, 0)
		// Angchor        (2, 0, 0)
		// Axis1          (0, 1, 0)
		// Axis2          (0, 0, 1)
		// AxisP1         (1, 0, 0)
		//
		// Only the second body moves
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PUGetInfo1_Fixture_4,
		@Test public void GetPUPositionRate_Bodies_at90deg_B2_moves()
		{
			// The length was at zero and this will give an negative length
			dBodySetLinearVel(bId2, (3.33), (0.0), (0.0));
			CHECK_EQUAL((-3.33), dJointGetPUPositionRate (jId) );

			dBodySetLinearVel(bId2, (0.0), (3.33), (0.0));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

			dBodySetLinearVel(bId2, (0.0), (0.0), (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );


			// Only angular velocity
			dBodySetAngularVel(bId2, (1.22), (0.0), (0.0));
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );

			dBodySetAngularVel(bId2, (0.0), (2.33), (0.0));
			CHECK_EQUAL((-1.0*2.330), dJointGetPUPositionRate (jId) );

			dBodySetAngularVel(bId2, (0.0), (0.0), (5.55));
			CHECK_EQUAL((0.0), dJointGetPUPositionRate (jId) );
		}

	} // End of SUITE(JointPU)


	// =============================================================================
	// =============================================================================
	//
	// Testing the Piston Joint
	//
	// =============================================================================
	// =============================================================================

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
	//	SUITE(JointPiston)
	//	{
	public static class PistonGetInfo1_Fixture_1  {
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
			DMatrix3 R_final = new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
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
			DMatrix3 R_final =  new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
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

			DMatrix3 R_final =  new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(4, info.m);
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	// Default Position:
	// Position Body1 (1, 0, 0)
	// Position Body2 (3, 0, 0)
	// Angchor        (2, 0, 0)
	// AxisR          (0, 1, 0)
	// Axis2          (0, 0, 1)
	// AxisP1         (1, 0, 0)
	//
	// The motor on axis1 is now powered. (i.e. joint->getLimot1()->fmax > 0
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
	public static class PistonGetInfo1_Fixture_2  {
		public PistonGetInfo1_Fixture_2() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 1, 0, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 3, 0, 0);


			jId = dJointCreatePiston(wId, null);
			joint = (DxJointPiston)jId;

			dJointAttach(jId, bId1, bId2);

			dJointSetPistonAnchor (jId, 2, 0, 0);

			joint.limotP.fmax = 1;
		}

		//~PistonGetInfo1_Fixture_2() {
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
		//TEST_FIXTURE(PistonGetInfo1_Fixture_2,
		@Test public void test0_PistonGetInfo1()
		{
			dJointSetPistonParam(jId, dParamLoStop1, -dInfinity);
			dJointSetPistonParam(jId, dParamHiStop1,  dInfinity);
			dJointSetPistonParam(jId, dParamLoStop2, -M_PI);
			dJointSetPistonParam(jId, dParamHiStop2,  M_PI);


			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(5, info.m);
		}


		////////////////////////////////////////////////////////////////////////////////
		// Test when there is limits for the prismatic at -10 and 10
		// The Body 2 is moved -100 unit then at 100
		//
		// Default value for axis = 1,0,0
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PistonGetInfo1_Fixture_2,
		@Test public void test1_PistonGetInfo1()
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

			CHECK_EQUAL(1, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(5, info.m);

			// Reset Position and test
			dBodySetPosition(bId2, 3, 0, 0);
			DMatrix3 R_final =  new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(5, info.m);
		}

		////////////////////////////////////////////////////////////////////////////////
		// Test when there is limits for the prismatic at -10 and 10
		// and for the rotoide at -45deg and 45deg.
		// The Body 2 is rotated by 90deg around the axis
		//
		//
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PistonGetInfo1_Fixture_2,
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

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(1, joint.limotR.limit);
			CHECK_EQUAL(6, info.m);

			// Reset Position and test
			DMatrix3 R_final =  new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(5, info.m);
		}

		////////////////////////////////////////////////////////////////////////////////
		// Test when there is limits for the prismatic at -10 and 10
		// and for the rotoide axuis at -45deg and 45deg.
		// The Body 2 is rotated by 90deg around the axis and
		// Body1 is moved at X=100
		//
		//
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PistonGetInfo1_Fixture_2,
		@Test public void test3_PistonGetInfo1()
		{
			dJointSetPistonParam(jId, dParamLoStop1, -10);
			dJointSetPistonParam(jId, dParamHiStop1,  10);
			dJointSetPistonParam(jId, dParamLoStop2, -M_PI/4.0);
			dJointSetPistonParam(jId, dParamHiStop2, M_PI/4.0);



			dBodySetPosition (bId1, (100.0), (0.0), (0.0));

			DMatrix3 R =  new DMatrix3();
			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);

			joint.getInfo1(info);

			CHECK_EQUAL(2, joint.limotP.limit);
			CHECK_EQUAL(1, joint.limotR.limit);
			CHECK_EQUAL(6, info.m);

			// Reset Position and test
			dBodySetPosition(bId1, 1, 0, 0);

			dBodySetPosition(bId2, 3, 0, 0);
			DMatrix3 R_final =  new DMatrix3( 1,0,0,0,
					0,1,0,0,
					0,0,1,0 );
			dBodySetRotation (bId2, R_final);

			joint.getInfo1(info);

			CHECK_EQUAL(0, joint.limotP.limit);
			CHECK_EQUAL(0, joint.limotR.limit);
			CHECK_EQUAL(5, info.m);
		}



		//TEST_FIXTURE(PistonGetInfo1_Fixture_2,
		@Test public void test_SetPistonParam()
		{
			dJointSetPistonParam(jId, dParamHiStop, (5.0) );
			CHECK_EQUAL((5.0), joint.limotP.histop);

			dJointSetPistonParam(jId, dParamVel, (7.0) );
			CHECK_EQUAL((7.0), joint.limotP.vel);

			//#ifdef D_PARAM_NAMES.dParamFudgeFactor1  //TZ unnecessary !?!?
			//TODO			dJointSetPistonParam(jId, dParamFudgeFactor1, (5.5) );
			////			CHECK_EQUAL((5.5), joint.limotP.D_PARAM_NAMES.dParamFudgeFactor);
			//			CHECK_EQUAL(5.5, joint.limotP.fudge_factor);
			//#endif

			dJointSetPistonParam(jId, dParamCFM2, (9.0) );
			CHECK_EQUAL((9.0), joint.limotR.normal_cfm);

			dJointSetPistonParam(jId, dParamStopERP2, (11.0) );
			CHECK_EQUAL((11.0), joint.limotR.stop_erp);
		}



		//TEST_FIXTURE(PistonGetInfo1_Fixture_1,
		@Test public void test_GetPistonParam()
		{
			joint.limotP.histop = (5.0);
			CHECK_EQUAL(joint.limotP.histop,
					dJointGetPistonParam(jId, dParamHiStop) );

			joint.limotP.vel = (7.0);

			CHECK_EQUAL(joint.limotP.vel,
					dJointGetPistonParam(jId, dParamVel) );

			//#ifdef D_PARAM_NAMES.dParamFudgeFactor1 //TZ unnecessary ?!?
			//joint.limotP.D_PARAM_NAMES.dParamFudgeFactor =  (5.5);
			joint.limotP.fudge_factor = 5.5;

			//			CHECK_EQUAL(joint.limotP.D_PARAM_NAMES.dParamFudgeFactor,
			//					dJointGetPistonParam(jId, D_PARAM_NAMES.dParamFudgeFactor1) );
			CHECK_EQUAL(joint.limotP.fudge_factor,
					dJointGetPistonParam(jId, dParamFudgeFactor1) );
			//#endif

			joint.limotR.normal_cfm = (9.0);
			CHECK_EQUAL(joint.limotR.normal_cfm,
					dJointGetPistonParam(jId, dParamCFM2) );

			joint.limotR.stop_erp = (11.0);
			CHECK_EQUAL(joint.limotR.stop_erp,
					dJointGetPistonParam(jId, dParamStopERP2) );
		}
	}


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
	public static class PistonGetInfo1_Fixture_3  {
		public PistonGetInfo1_Fixture_3() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 3, 0, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 1, 0, 0);


			jId = dJointCreatePiston(wId, null);

			dJointAttach(jId, bId1, bId2);
			dJointSetPistonAnchor (jId, 2, 0, 0);

			dBodySetLinearVel (bId1, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

			dBodySetLinearVel (bId2, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

		}

		//~PistonGetInfo1_Fixture_3() {
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy(wId);
		}

		DPistonJoint jId;

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
		//TEST_FIXTURE(PistonGetInfo1_Fixture_3,
		@Test public void test1_GetPistonPositionRate()
		{
			// They move with the same linear speed
			// Angular speed == 0
			dBodySetLinearVel(bId1, 0, (3.33), 0);
			dBodySetLinearVel(bId2, 0, (3.33), 0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId1, (1.11), (3.33), 0);
			dBodySetLinearVel(bId2, (1.11), (3.33), 0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId1, (1.11), (3.33), (2.22));
			dBodySetLinearVel(bId2, (1.11), (3.33), (2.22));
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );


			// Reset for the next set of test.
			dBodySetLinearVel(bId1, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

			dBodySetLinearVel(bId2, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));


			// They move with the same angular speed
			// linear speed == 0

			dBodySetAngularVel(bId1, (1.22), 0.0, 0.0);
			dBodySetAngularVel(bId2, (1.22), 0.0, 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId1, (1.22), (2.33), 0.0);
			dBodySetAngularVel(bId2, (1.22), (2.33), 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId1, (1.22), (2.33), (3.44));
			dBodySetAngularVel(bId2, (1.22), (2.33), (3.44));
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );
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
		//TEST_FIXTURE(PistonGetInfo1_Fixture_3,
		@Test public void GetPistonPositionRate_Bodies_in_line_B1_moves()
		{
			dBodySetLinearVel(bId1, (3.33), 0.0, 0.0); // This is impossible but ...
			CHECK_EQUAL((3.33), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId1, 0, (3.33), 0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId1, 0, 0, (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );


			// Only the first body as angular velocity
			dBodySetAngularVel(bId1, (1.22), 0.0, 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId1, 0.0, (2.33), 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId1, 0.0, 0.0, (5.55));
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );
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
		//TEST_FIXTURE(PistonGetInfo1_Fixture_3,
		@Test public void GetPistonPositionRate_Bodies_in_line_B2_moves()
		{
			// The length was at zero and this will give an negative length
			dBodySetLinearVel(bId2, (3.33), 0.0, 0.0);
			CHECK_EQUAL((-3.33), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId2, 0, (3.33), 0);      // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId2, 0, 0, (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );


			// Only angular velocity
			dBodySetAngularVel(bId2, (1.22), 0.0, 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId2, 0.0, (2.33), 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId2, 0.0, 0.0, (5.55));
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	// Fixture for testing the PositionRate
	//
	// Default Position:
	// Position Body1 (3, 0, 0)
	// Position Body2 (0, 0, 1)
	// Angchor        (2, 0, 0)
	// Axis1          (0, 1, 0)
	// Axis2          (1, 0, 0)
	// AxisP          (1, 0, 0)
	//
	// The second body is at 90deg w.r.t. the first body
	// From
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
	// To
	//
	//               Y                ^ Axis2
	//              ^                 |
	//             /  Body 2          |     ^ Axis1
	// Z^          +----------+       |    /
	//  |        //          /|       |   /         Body 1
	//  |       /+----------+ |       |  /          +-----------+
	//  |      / |          | |       | /          /           /|
	//  |     /  |          | |       _/     -    /           / +
	//  |    /   |          |-|------(_)----|--- /-----------/-------> AxisP
	//  |   /    |          | |              -  +-----------+ /
	//  |  /     |          | |                 |           |/
	//  | /      |          | +                 +-----------+
	//  |/       |          |/
	//  .--------+----------+--------------------> X
	//             |---------------->
	//             Anchor2           <--------------|
	//                               Anchor1
	// Default Position
	//
	// N.B. Y is going into the page
	////////////////////////////////////////////////////////////////////////////////
	public static class PistonGetInfo1_Fixture_4  {
		public PistonGetInfo1_Fixture_4() {
			wId = dWorldCreate();

			bId1 = dBodyCreate(wId);
			dBodySetPosition(bId1, 3, 0, 0);

			bId2 = dBodyCreate(wId);
			dBodySetPosition(bId2, 0, 0, 1);

			DMatrix3 R =  new DMatrix3();
			dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);


			jId = dJointCreatePiston(wId, null);

			dJointAttach(jId, bId1, bId2);
			dJointSetPistonAnchor (jId, 2, 0, 0);


			dBodySetLinearVel(bId1, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

			dBodySetLinearVel(bId2, (0.0), (0.0), (0.0));
			dBodySetAngularVel(bId1, (0.0), (0.0), (0.0));

		}

		//~PistonGetInfo1_Fixture_4() {
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy(wId);
		}

		DPistonJoint jId;

		static DWorld wId;

		DBody bId1;
		DBody bId2;

		DxJoint.Info1 info = new DxJoint.Info1();
		//};


		////////////////////////////////////////////////////////////////////////////////
		// Position Body1 (3, 0, 0)
		// Position Body2 (1, 0, 0)
		// Angchor        (2, 0, 0)
		// Axis1          (0, 1, 0)
		// Axis2          (0, 0, 1)
		// AxisP1         (1, 0, 0)
		//
		// Only the first body moves
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PistonGetInfo1_Fixture_4,
		@Test public void GetPistonPositionRate_Bodies_at90deg_B1_moves()
		{
			dBodySetLinearVel(bId1, (3.33), 0.0, 0.0); // This is impossible but ...
			CHECK_EQUAL((3.33), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId1, 0, (3.33), 0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId1, 0, 0, (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );


			// Only angular velocity
			dBodySetAngularVel(bId1, (1.22), 0.0, 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId1, 0.0, (2.33), 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId1, 0.0, 0.0, (5.55));
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );
		}

		////////////////////////////////////////////////////////////////////////////////
		// Position Body1 (3, 0, 0)
		// Position Body2 (1, 0, 0)
		// Angchor        (2, 0, 0)
		// Axis1          (0, 1, 0)
		// Axis2          (0, 0, 1)
		// AxisP1         (1, 0, 0)
		//
		// Only the second body moves
		////////////////////////////////////////////////////////////////////////////////
		//TEST_FIXTURE(PistonGetInfo1_Fixture_4,
		@Test public void GetPistonPositionRate_Bodies_at90deg_B2_moves()
		{
			// The length was at zero and this will give an negative length
			dBodySetLinearVel(bId2, (3.33), 0.0, 0.0);
			CHECK_EQUAL((-3.33), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId2, 0, (3.33), 0);     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetLinearVel(bId2, 0, 0, (3.33));     // This is impossible but ...
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );


			// Only angular velocity
			dBodySetAngularVel(bId2, (1.22), 0.0, 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId2, 0.0, (2.33), 0.0);
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );

			dBodySetAngularVel(bId2, 0.0, 0.0, (5.55));
			CHECK_EQUAL((0.0), dJointGetPistonPositionRate (jId) );
		}



		public static class Fixture_Simple_Hinge
		{
			public Fixture_Simple_Hinge ()
			{
				wId = dWorldCreate();

				bId1 = dBodyCreate(wId);
				dBodySetPosition(bId1, 0, -1, 0);

				bId2 = dBodyCreate(wId);
				dBodySetPosition(bId2, 0, 1, 0);


				jId = dJointCreateHinge(wId, null);

				dJointAttach(jId, bId1, bId2);
			}

			//		        ~Fixture_Simple_Hinge()
			@AfterClass
			public static void DESTRUCTOR() {
				dWorldDestroy(wId);
			}

			DHingeJoint jId;

			static DWorld wId;

			DBody bId1;
			DBody bId2;
			//};

			// Test that it is possible to have joint without a body
			//TEST_FIXTURE(Fixture_Simple_Hinge, 
			@Test public void test_dJointAttach()
			{
				boolean only_body1_OK = true;
				try {
					dJointAttach(jId, bId1, null);
					dWorldStep (wId, 1);
				}
				catch (Throwable t) {
					//OK
					only_body1_OK = false;
				}
				CHECK_EQUAL(true, only_body1_OK);

				boolean only_body2_OK = true;
				try {
					dJointAttach(jId, null, bId2);
					dWorldStep (wId, 1);
				}
				catch (Throwable t) {
					//OK
					only_body2_OK = false;
				}
				CHECK_EQUAL(true, only_body2_OK);

				boolean no_body_OK = true;
				try {
					dJointAttach(jId, null, null);
					dWorldStep (wId, 1);
				}
				catch (Throwable t) {
					//OK
					no_body_OK = false;
				}
				CHECK_EQUAL(true, no_body_OK);
			}

		}

	} // End of SUITE(JointPiston)
}

