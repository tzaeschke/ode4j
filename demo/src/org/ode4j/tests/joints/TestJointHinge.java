/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests.joints;

import org.junit.AfterClass;
import org.junit.Test;
import org.junit.experimental.runners.Enclosed;
import org.junit.runner.RunWith;


import org.ode4j.ode.internal.joints.DxJointHinge;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DWorld;

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.cpp.OdeCpp.*;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.*;


//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joinst/hinge.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

//#include <UnitTest++.h>
//#include <ode/ode.h>
//
//#include "../../ode/src/joints/hinge.h"

//@RunWith(org.junit.runners.Enclosed.class)
@RunWith(Enclosed.class)
public class TestJointHinge {

	//SUITE (TestdxJointHinge)
	//{
	// The 2 bodies are positionned at (0, 0, 0), with no rotation
	// The joint is an Hinge Joint
	// Axis is along the X axis
	// Anchor at (0, 0, 0)
	//         ^Y
	//         |
	//         |
	//         |
	//         |
	//         |
	// Z <---- . (X going out of the page)
	//@RunWith(Enclosed.class)
	public static class dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X {
		public dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);

			jId   = dJointCreateHinge (wId, null);
			joint = (DxJointHinge) jId;


			dJointAttach (jId, bId1, bId2);
			dJointSetHingeAnchor (jId, 0, 0, 0);

			axis.set( 1, 0, 0);
		}

		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy(wId);
		}

		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DHingeJoint jId;
		DxJointHinge joint;

		DVector3 axis = new DVector3();
		//  };

		// Rotate 2nd body 90deg around X then back to original position
		//
		//   ^  ^       ^
		//   |  |  =>   |  <---
		//   |  |       |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 90deg
		//   ^           ^   ^
		//   | <---  =>  |   |
		//   |           |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE {// (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetHingeAxisOffset_B2_90deg() {
			DMatrix3 R = new DMatrix3();

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  -M_PI/2.0);
			CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}


		// Rotate 2nd body -90deg around X then back to original position
		//
		//   ^  ^       ^
		//   |  |  =>   |  --->
		//   |  |       |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 90deg
		//   ^           ^   ^
		//   | --->  =>  |   |
		//   |           |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetHingeAxisOffset_B2_Minus90deg() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  M_PI/2.0);
			CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}


		// Rotate 1st body 0.23rad around X then back to original position
		//
		//   ^  ^     ^      ^
		//   |  |  =>  \     |
		//   |  |       \    |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 0.23rad
		// ^    ^        ^   ^
		//  \   | =>     |   |
		//   \  |        |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetHingeAxisOffset_B1_0_23rad() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, (0.23) );
			dBodySetRotation (bId1, R);

			CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  (0.23));
			CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}


		// Rotate 1st body -0.23rad around Z then back to original position
		//
		//   ^  ^         ^  ^
		//   |  |  =>    /   |
		//   |  |       /    |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 0.23rad
		//     ^ ^        ^   ^
		//    /  | =>     |   |
		//   /   |        |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetHingeAxisOffset_B1_Minus0_23rad() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (-(0.23), dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  -(0.23));
			CHECK_CLOSE (-(0.23), dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}
	}

	// The 2 bodies are positionned at (0, 0, 0), with no rotation
	// The joint is an Hinge Joint.
	// Axis in the inverse direction of the X axis
	// Anchor at (0, 0, 0)
	//         ^Y
	//         |
	//         |
	//         |
	//         |
	//         |
	// Z <---- x (X going out of the page)
	public static class dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X {
		public dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, -1, 0);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 1, 0);

			jId   = dJointCreateHinge (wId, null);
			joint = (DxJointHinge) jId;


			dJointAttach (jId, bId1, bId2);
			dJointSetHingeAnchor (jId, 0, 0, 0);

			axis.set( -1, 0, 0 );
		}

		//    ~dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy(wId);
		}

		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DHingeJoint jId;
		DxJointHinge joint;

		DVector3 axis = new DVector3();
		//  };

		// Rotate 2nd body 90deg around X then back to original position
		//
		//   ^  ^       ^
		//   |  |  =>   |  <---
		//   |  |       |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 90deg
		//   ^           ^   ^
		//   | <---  =>  |   |
		//   |           |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetHingeAxisOffset_B2_90Deg() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  M_PI/2.0);
			CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}


		// Rotate 2nd body -90deg around X then back to original position
		//
		//   ^  ^       ^
		//   |  |  =>   |  --->
		//   |  |       |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 90deg
		//   ^           ^   ^
		//   | --->  =>  |   |
		//   |           |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetHingeAxisOffset_B2_Minus90Deg() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  -M_PI/2.0);
			CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}


		// Rotate 1st body 0.23rad around X then back to original position
		//
		//   ^  ^     ^      ^
		//   |  |  =>  \     |
		//   |  |       \    |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 0.23rad
		// ^    ^        ^   ^
		//  \   | =>     |   |
		//   \  |        |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetHingeAxisOffset_B1_0_23rad() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, (0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (-(0.23), dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  -(0.23));
			CHECK_CLOSE (-(0.23), dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}


		// Rotate 2nd body -0.23rad around Z then back to original position
		//
		//   ^  ^         ^  ^
		//   |  |  =>    /   |
		//   |  |       /    |
		//  B1  B2     B1   B2
		//
		// Start with a Delta of 0.23rad
		//     ^ ^        ^   ^
		//    /  | =>     |   |
		//   /   |        |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetHingeAxisOffset_B1_Minus0_23rad() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  (0.23));
			CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
		}
	}

	// Only one body body1 at (0,0,0)
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
	public static class dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X {
		public dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			jId   = dJointCreateHinge (wId, null);
			joint = (DxJointHinge) jId;


			dJointAttach (jId, bId1, null);
			dJointSetHingeAnchor (jId, 0, 0, 0);

			axis.set( 1, 0, 0 );
		}

		//~dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy(wId);
		}

		static DWorld wId;

		DBody bId1;


		DHingeJoint jId;
		DxJointHinge joint;

		DVector3 axis = new DVector3();
		//  };

		// Rotate B1 by 90deg around X then back to original position
		//
		//   ^
		//   |  => <---
		//   |
		//  B1      B1
		//
		// Start with a Delta of 90deg
		//            ^
		//  <---  =>  |
		//            |
		//   B1      B1
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetHingeAxisOffset_1Body_B1_90Deg() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  M_PI/2.0);
			CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
		}

		// Rotate B1 by -0.23rad around X then back to original position
		//
		//   ^         ^
		//   |  =>    /
		//   |       /
		//  B1      B1
		//
		// Start with a Delta of -0.23rad
		//     ^     ^
		//    /  =>  |
		//   /       |
		//   B1     B1
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetHingeAxisOffset_1Body_B1_Minus0_23rad() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (-(0.23), dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  -(0.23));
			CHECK_CLOSE (-(0.23), dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
		}
	}


	// Only one body body1 at (0,0,0)
	// The joint is an Hinge Joint.
	// Axis the inverse of the X axis
	// Anchor at (0, 0, 0)
	//
	//       ^Y
	//       |
	//       |
	//       |
	//       |
	//       |
	// Z <-- X
	public static class dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X {
		public dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			jId   = dJointCreateHinge (wId, null);
			joint = (DxJointHinge) jId;


			dJointAttach (jId, bId1, null);
			dJointSetHingeAnchor (jId, 0, 0, 0);

			axis.set( -1, 0, 0 );
		}

		//~dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy(wId);
		}

		static DWorld wId;

		DBody bId1;


		DHingeJoint jId;
		DxJointHinge joint;

		DVector3 axis = new DVector3();
		//  };

		// Rotate B1 by 90deg around X then back to original position
		//
		//   ^
		//   |  => <---
		//   |
		//  B1      B1
		//
		// Start with a Delta of 90deg
		//            ^
		//  <---  =>  |
		//            |
		//   B1      B1
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetHingeAxisOffset_1Body_B1_90Deg() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  -M_PI/2.0);
			CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
		}

		// Rotate B1 by -0.23rad around X then back to original position
		//
		//   ^         ^
		//   |  =>    /
		//   |       /
		//  B1      B1
		//
		// Start with a Delta of -0.23rad
		//     ^     ^
		//    /  =>  |
		//   /       |
		//   B1     B1
		//TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetHingeAxisOffset_1Body_B1_Minus0_23rad() {
			DMatrix3 R = new DMatrix3();

			dJointSetHingeAxis (jId, axis.get0(), axis.get1(), axis.get2());

			CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

			dJointSetHingeAxisOffset (jId, axis.get0(), axis.get1(), axis.get2(),  (0.23));
			CHECK_CLOSE ((0.23), dJointGetHingeAngle (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
		}
	}




	// Only one body body2 at (0,0,0)
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
	public static class dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X {
		public dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X()
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

	// Create 2 bodies attached by a Hinge joint
	// Axis is along the X axis (Default value
	// Anchor at (0, 0, 0)      (Default value)
	//
	//       ^Y
	//       |
	//       * Body2
	//       |
	//       |
	// Body1 |
	// *     Z-------->
	public static class dxJointHinge_Test_Initialization {
		public dxJointHinge_Test_Initialization()
		{
			wId = dWorldCreate();

			// Remove gravity to have the only force be the force of the joint
			dWorldSetGravity(wId, 0,0,0);

			for (int j=0; j<2; ++j) {
				bId[j][0] = dBodyCreate (wId);
				dBodySetPosition (bId[j][0], -1, -2, -3);

				bId[j][1] = dBodyCreate (wId);
				dBodySetPosition (bId[j][1], 11, 22, 33);


				DMatrix3 R = new DMatrix3();
				DVector3 axis = new DVector3(); // Random axis

				axis.set0( 0.53);
				axis.set1(-0.71);
				axis.set2( 0.43);
				dNormalize3(axis);
				dRFromAxisAndAngle (R, axis, (0.47123)); // 27deg
				dBodySetRotation (bId[j][0], R);


				axis.set0( 1.2);
				axis.set1( 0.87);
				axis.set2(-0.33);
				dNormalize3(axis);
				dRFromAxisAndAngle (R, axis,
						(0.47123)); // 27deg
				dBodySetRotation (bId[j][1], R);

				jId[j]   = dJointCreateHinge (wId, null);
				dJointAttach (jId[j], bId[j][0], bId[j][1]);
				//        dJointSetHingeParam(jId[j], dParamLoStop, 1);
				//        dJointSetHingeParam(jId[j], dParamHiStop, 2);
				//        dJointSetHingeParam(jId[j], dParamFMax, 200);
			}
		}

		//~dxJointHinge_Test_Initialization()
		@AfterClass
		public static void DESTRUCTOR()
		{
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody[][] bId = new DBody[2][2];


		DHingeJoint[] jId = new DHingeJoint[2];

		//		  };


		// Test if setting a Hinge with its default values
		// will behave the same as a default Hinge joint
		//TEST_FIXTURE (dxJointHinge_Test_Initialization,
		@Test public void test_Hinge_Initialization() {
			//using namespace std;

			DVector3 axis = new DVector3();
			dJointGetHingeAxis(jId[1], axis);
			dJointSetHingeAxis(jId[1], axis.get0(), axis.get1(), axis.get2());


			DVector3 anchor = new DVector3();
			dJointGetHingeAnchor(jId[1], anchor);
			dJointSetHingeAnchor(jId[1], anchor.get0(), anchor.get1(), anchor.get2());


			for (int b=0; b<2; ++b) {
				// Compare body b of the first joint with its equivalent on the
				// second joint
				DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
				DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
				CHECK_CLOSE (qA.get0(), qB.get0(), 1e-6);
				CHECK_CLOSE (qA.get1(), qB.get1(), 1e-6);
				CHECK_CLOSE (qA.get2(), qB.get2(), 1e-6);
				CHECK_CLOSE (qA.get3(), qB.get3(), 1e-6);
			}

			dWorldStep (wId,0.5);
			dWorldStep (wId,0.5);
			dWorldStep (wId,0.5);
			dWorldStep (wId,0.5);

			for (int b=0; b<2; ++b) {
				// Compare body b of the first joint with its equivalent on the
				// second joint
				DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
				DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
				CHECK_CLOSE (qA.get0(), qB.get0(), 1e-6);
				CHECK_CLOSE (qA.get1(), qB.get1(), 1e-6);
				CHECK_CLOSE (qA.get2(), qB.get2(), 1e-6);
				CHECK_CLOSE (qA.get3(), qB.get3(), 1e-6);


				DVector3C posA = dBodyGetPosition(bId[0][b]);
				DVector3C posB = dBodyGetPosition(bId[1][b]);
				CHECK_CLOSE (posA.get0(), posB.get0(), 1e-6);
				CHECK_CLOSE (posA.get1(), posB.get1(), 1e-6);
				CHECK_CLOSE (posA.get2(), posB.get2(), 1e-6);
				//CHECK_CLOSE (posA.get3(), posB.get3(), 1e-6);
			}
		}


	} // End of SUITE TestdxJointHinge
}

