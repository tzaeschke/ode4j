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

import static org.ode4j.cpp.OdeCpp.*;
import static org.ode4j.ode.OdeMath.*;

import org.cpp4j.java.RefDouble;
import org.junit.AfterClass;
import org.junit.Test;
import org.junit.experimental.runners.Enclosed;
import org.junit.runner.RunWith;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointUniversal;

import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.*;


/**
 * This file create unit test for some of the functions found in:
 * ode/src/joinst/universal.cpp
 */

//#include <iostream>
//
//#include <UnitTest++.h>
//#include <ode/ode.h>
//
//#include "../../ode/src/joints/universal.h"

//SUITE (TestdxJointUniversal)
@RunWith(Enclosed.class)
public class TestJointUniversal
{


	static double d2r(double degree)
	{
		return degree * (double)(M_PI / 180.0);
	}
	static double r2d(double degree)
	{
		return degree * (double)(180.0/M_PI);
	}

	// The 2 bodies are positionned at (0, 0, 0)
	// The bodis have no rotation.
	// The joint is a Universal Joint
	// Axis1 is along the X axis
	// Axis2 is along the Y axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y
	{
		public Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y()
		{

			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);


			jId   = dJointCreateUniversal (wId, null);
			joint = (DxJointUniversal) jId;


			dJointAttach (jId, bId1, bId2);
		}

		//        ~Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DUniversalJoint jId;
		DxJointUniversal joint;


		// Test is dJointGetUniversalAngles versus
		// dJointGetUniversalAngle1 and dJointGetUniversalAngle2 dJointGetUniversalAxis
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetGetUniversalAngles_Versus_Angle1_and_Angle2()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);

			DMatrix3 R = new DMatrix3();
			double ang1, ang2;


			DVector3 axis1 = new DVector3();
			dJointGetUniversalAxis1 (jId, axis1);

			DVector3 axis2 = new DVector3();
			dJointGetUniversalAxis2 (jId, axis2);

			ang1 = d2r((23.0));
			dRFromAxisAndAngle (R, axis1.get0(), axis1.get1(), axis1.get2(), ang1);
			dBodySetRotation (bId1, R);

			ang2 = d2r((17.0));
			dRFromAxisAndAngle (R, axis2.get0(), axis2.get1(), axis2.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);






			// ax1 and ax2 are pseudo-random axis. N.B. They are NOT the axis of the joints.
			DVector3 ax1 = new DVector3();
			ax1.set(	(0.2),
					-(0.67),
					-(0.81));
			dNormalize3(ax1);

			DVector3 ax2 = new DVector3();
			ax2.set(	(0.62),
					(0.31),
					(0.43));
			dNormalize3(ax2);


			ang1 = d2r((23.0));
			dRFromAxisAndAngle (R, ax1.get0(), ax1.get1(), ax1.get2(), ang1);
			dBodySetRotation (bId1, R);

			ang2 = d2r((0.0));

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle1.d, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (angle2.d, dJointGetUniversalAngle2 (jId), 1e-4);



			ang1 = d2r((0.0));

			ang2 = d2r((23.0));
			dRFromAxisAndAngle (R, ax2.get0(), ax2.get1(), ax2.get2(), ang2);
			dBodySetRotation (bId1, R);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle1.d, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (angle2.d, dJointGetUniversalAngle2 (jId), 1e-4);


			ang1 = d2r((38.0));
			dRFromAxisAndAngle (R, ax1.get0(), ax1.get1(), ax1.get2(), ang2);
			dBodySetRotation (bId1, R);

			ang2 = d2r((-43.0));
			dRFromAxisAndAngle (R, ax2.get0(), ax2.get1(), ax2.get2(), ang2);
			dBodySetRotation (bId1, R);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle1.d, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (angle2.d, dJointGetUniversalAngle2 (jId), 1e-4);


			// Try with random axis for the axis of the joints
			dRSetIdentity(R);
			dBodySetRotation (bId1, R);
			dBodySetRotation (bId1, R);

			axis1.set(	(0.32),
					-(0.57),
					(0.71));
			dNormalize3(axis1);

			axis2.set(	-(0.26),
					-(0.31),
					(0.69));
			dNormalize3(axis2);

			DVector3 cross = new DVector3();
			dCROSS(cross, OP.EQ, axis1, axis2);
			dJointSetUniversalAxis1(jId, axis1.get0(), axis1.get1(), axis1.get2());
			dJointSetUniversalAxis2(jId, cross.get0(), cross.get1(), cross.get2());


			ang1 = d2r((23.0));
			dRFromAxisAndAngle (R, ax1.get0(), ax1.get1(), ax1.get2(), ang1);
			dBodySetRotation (bId1, R);

			ang2 = d2r((0.0));

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle1.d, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (angle2.d, dJointGetUniversalAngle2 (jId), 1e-4);



			ang1 = d2r((0.0));

			ang2 = d2r((23.0));
			dRFromAxisAndAngle (R, ax2.get0(), ax2.get1(), ax2.get2(), ang2);
			dBodySetRotation (bId1, R);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle1.d, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (angle2.d, dJointGetUniversalAngle2 (jId), 1e-4);


			ang1 = d2r((38.0));
			dRFromAxisAndAngle (R, ax1.get0(), ax1.get1(), ax1.get2(), ang2);
			dBodySetRotation (bId1, R);

			ang2 = d2r((-43.0));
			dRFromAxisAndAngle (R, ax2.get0(), ax2.get1(), ax2.get2(), ang2);
			dBodySetRotation (bId1, R);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle1.d, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (angle2.d, dJointGetUniversalAngle2 (jId), 1e-4);
		}


		// ==========================================================================
		// Testing the offset
		// TODO:
		// - Test Axis1Offset(...., 0, ang2);
		// ==========================================================================


		//  Rotate first body 90deg around X (Axis1) then back to original position
		//
		//    ^  ^           ^           Z ^
		//    |  |  => <---  |             |
		//    |  |           |             |
		//   B1  B2     B1   B2            .----->Y
		//                                /
		//                               /
		//                              v X    (N.B. X is going out of the screen)
		//
		//  Set Axis1 with an Offset of 90deg
		//       ^        ^   ^
		//  <--- |  =>    |   |
		//       |        |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetUniversalAxis1Offset_B1_90deg()
		{
			DMatrix3 R = new DMatrix3();

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);


			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);

			double ang1 = d2r((90.0));
			double ang2 = d2r((0.0));
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang1);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (ang2, angle2.d, 1e-4);



			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (ang2, angle2.d, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);
		}

		//  Rotate 2nd body 90deg around (Axis2) then back to original position
		//  Offset when setting axis1
		//
		//    ^  ^           ^           Z ^
		//    |  |  => <---  |             |
		//    |  |           |             |
		//   B1  B2     B1   B2            .----->Y
		//                                /
		//                               /
		//                              v X    (N.B. X is going out of the screen)
		//
		//  Set Axis1 with an Offset of 90deg
		//       ^        ^   ^
		//  <--- |  =>    |   |
		//       |        |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetUniversalAxis1Offset_B2_90deg()
		{
			DMatrix3 R = new DMatrix3();

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);


			DVector3 ax1 = new DVector3(), ax2 = new DVector3();
			dJointGetUniversalAxis1 (jId, ax1);
			dJointGetUniversalAxis2 (jId, ax2);

			double ang1 = d2r((0.0));
			double ang2 = d2r((90.0));
			dRFromAxisAndAngle (R, ax2.get0(), ax2.get1(), ax2.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);



			dJointSetUniversalAxis1Offset (jId, ax1.get0(), ax1.get1(), ax1.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);
		}

		//  Rotate second body 90deg around Y (Axis2) then back to original position
		//
		//    ^  ^       ^           Z ^
		//    |  |  =>   |   .         |
		//    |  |       |             |
		//   B1  B2     B1   B2        .----->Y
		//                            /
		//                           /
		//                          v X    (N.B. X is going out of the screen)
		//
		//  Set Axis2 with an Offset of 90deg
		//   ^           ^   ^
		//   |   .  =>   |   |
		//   |           |   |
		//   B1  B2     B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetUniversalAxisOffset_B2_90deg()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang1 = d2r((0.0));
			double ang2 = d2r((90.0));

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);


			dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);
		}



		//  Rotate 2nd body -90deg around Y (Axis2) then back to original position
		//
		//    ^  ^       ^               Z ^
		//    |  |  =>   |   x             |
		//    |  |       |                 |
		//   B1  B2     B1   B2          X .----> Y
		//                               N.B. X is going out of the screen
		//  Start with a Delta of 90deg
		//    ^           ^   ^
		//    |  x  =>    |   |
		//    |           |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetUniversalAxisOffset_B2_Minus90deg()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
			CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang1 = d2r((0.0));
			double ang2 = d2r((90.0));

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), -ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (ang2, angle2.d, 1e-4);



			dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (ang2, angle2.d, 1e-4);


			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), 0);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);
		}


		//  Rotate 1st body 0.23rad around X (Axis1) then back to original position
		//
		//    ^  ^     ^      ^           Z ^
		//    |  |  =>  \     |             |
		//    |  |       \    |             |
		//   B1  B2     B1   B2             .-------> Y
		//                                 /
		//                                /
		//                               v X  (N.B. X is going out of the screen)
		//
		//  Start with a Delta of 0.23rad
		//  ^    ^        ^   ^
		//   \   | =>     |   |
		//    \  |        |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetUniversalAxis1Offset_B1_0_23rad()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);

			double ang1 = (0.23);
			double ang2 = (0.0);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang1);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (ang2, angle2, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}

		//  Rotate 2nd body 0.23rad around Y (Axis2) then back to original position
		//
		//    ^  ^      ^     ^           Z ^   ^ Y (N.B. Y is going in the screen)
		//    |  |  =>  |    /              |  /
		//    |  |      |   /               | /
		//   B1  B2     B1  B2              .-------> X
		//
		//  Start with a Delta of 0.23rad
		//   ^     ^    ^   ^
		//   |    /  => |   |
		//   |   /      |   |
		//   B1  B2     B1  B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetUniversalAxisOffset_B2_0_23rad()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang1 = (0.0);
			double ang2 = (0.23);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}


		// Rotate 1st body 0.23rad around X axis and 2nd body 0.37rad around Y (Axis2)
		// then back to their original position.
		// The Axis offset are set one at a time
		//
		//    ^  ^    ^         ^          Z ^   ^ Y (N.B. Y is going in the screen)
		//    |  |  => \      /             |  /
		//    |  |      \   /               | /
		//   B1  B2     B1  B2              .-------> X
		//
		//  Start with a Delta of 0.23rad
		// ^         ^  ^   ^
		//  \      / => |   |
		//   \   /      |   |
		//   B1  B2     B1  B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y,
		@Test public void test_dJointSetUniversalAxisOffset_B1_0_23rad_B2_0_37rad_One_by_One()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis1 = new DVector3();
			dJointGetUniversalAxis1 (jId, axis1);
			DVector3 axis2 = new DVector3();
			dJointGetUniversalAxis2 (jId, axis2);

			DMatrix3 R = new DMatrix3();

			double ang1 = (0.23);
			dRFromAxisAndAngle (R, axis1.get0(), axis1.get1(), axis1.get2(), ang1);
			dBodySetRotation (bId1, R);

			double ang2 = (0.37);
			dRFromAxisAndAngle (R, axis2.get0(), axis2.get1(), axis2.get2(), ang2);
			dBodySetRotation (bId2, R);


			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);


			dJointSetUniversalAxis1Offset (jId, axis1.get0(), axis1.get1(), axis1.get2(),
					ang1, -ang2 );
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);

			dJointGetUniversalAxis1 (jId, axis1);
			dJointGetUniversalAxis2 (jId, axis2);

			dRFromAxisAndAngle (R, axis2.get0(), axis2.get1(), axis2.get2(), ang2);
			dBodySetRotation (bId2, R);

			dJointSetUniversalAxis2Offset (jId, axis2.get0(), axis2.get1(), axis2.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}
	}


	// The 2 bodies are positionned at (-1, -2, -3),  and (11, 22, 33)
	// The bodis have rotation of 27deg around some axis.
	// The joint is a Universal Joint
	// Axis is along the X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X
	{
		public Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, -1, -2, -3);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 11, 22, 33);

			DMatrix3 R = new DMatrix3();

			DVector3 axis = new DVector3();

			axis.set(	(0.53),
					-(0.71),
					(0.43));
			dNormalize3(axis);
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
					(0.47123)); // 27deg
					dBodySetRotation (bId1, R);


					axis.set(	(1.2),
							(0.87),
							-(0.33));
					dNormalize3(axis);
					dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
							(0.47123)); // 27deg
					dBodySetRotation (bId2, R);

					jId   = dJointCreateUniversal (wId, null);
					joint = (DxJointUniversal) jId;


					dJointAttach (jId, bId1, bId2);
		}

		//        ~Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DUniversalJoint jId;
		DxJointUniversal joint;

		// Test is dJointSetUniversalAxis and dJointGetUniversalAxis return same value
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Random_Axis_Along_X,
		@Test public void test_dJointSetGetUniversalAxis()
		{
			DVector3 axisOrig = new DVector3(), axis = new DVector3();


			dJointGetUniversalAxis1 (jId, axisOrig);
			dJointGetUniversalAxis1 (jId, axis);
			dJointSetUniversalAxis1 (jId, axis.get0(), axis.get1(), axis.get2());
			dJointGetUniversalAxis1 (jId, axis);
			CHECK_CLOSE (axis.get0(), axisOrig.get0() , 1e-4);
			CHECK_CLOSE (axis.get1(), axisOrig.get1() , 1e-4);
			CHECK_CLOSE (axis.get2(), axisOrig.get2() , 1e-4);


			dJointGetUniversalAxis2 (jId, axisOrig);
			dJointGetUniversalAxis2(jId, axis);
			dJointSetUniversalAxis2 (jId, axis.get0(), axis.get1(), axis.get2());
			dJointGetUniversalAxis2 (jId, axis);
			CHECK_CLOSE (axis.get0(), axisOrig.get0() , 1e-4);
			CHECK_CLOSE (axis.get1(), axisOrig.get1() , 1e-4);
			CHECK_CLOSE (axis.get2(), axisOrig.get2() , 1e-4);


			DVector3 anchor1 = new DVector3(), anchor2 = new DVector3();
			DVector3 anchorOrig1 = new DVector3(), anchorOrig2 = new DVector3();
			dJointGetUniversalAnchor (jId, anchorOrig1);
			dJointGetUniversalAnchor (jId, anchor1);
			dJointGetUniversalAnchor2 (jId, anchorOrig2);
			dJointGetUniversalAnchor2 (jId, anchor2);

			dJointSetUniversalAnchor (jId, anchor1.get0(), anchor1.get1(), anchor1.get2());
			dJointGetUniversalAnchor (jId, anchor1);
			dJointGetUniversalAnchor2 (jId, anchor2);
			//TODO
			//            CHECK_CLOSE (anchor1[0], anchorOrig1[0] , 1e-4);
			//            CHECK_CLOSE (anchor1[0], anchorOrig1[0] , 1e-4);
			//            CHECK_CLOSE (anchor1[0], anchorOrig1[0] , 1e-4);
			CHECK_CLOSE (anchor1.get0(), anchorOrig1.get0() , 1e-4);
			CHECK_CLOSE (anchor1.get1(), anchorOrig1.get1() , 1e-4);
			CHECK_CLOSE (anchor1.get2(), anchorOrig1.get2() , 1e-4);

			//TODO
			//            CHECK_CLOSE (anchor2[0], anchorOrig2[0] , 1e-4);
			//            CHECK_CLOSE (anchor2[0], anchorOrig2[0] , 1e-4);
			//            CHECK_CLOSE (anchor2[0], anchorOrig2[0] , 1e-4);
			CHECK_CLOSE (anchor2.get0(), anchorOrig2.get0() , 1e-4);
			CHECK_CLOSE (anchor2.get1(), anchorOrig2.get1() , 1e-4);
			CHECK_CLOSE (anchor2.get2(), anchorOrig2.get2() , 1e-4);
		}
	}


	// Only one body body1 at (0,0,0)
	// The joint is an Universal Joint.
	// Axis1 is along the X axis
	// Axis2 is along the Y axis
	// Anchor at (0, 0, 0)
	//
	//       ^Y
	//       |
	//       |
	//       |
	//       |
	//       |
	// Z <-- X
	public static class Fixture_dxJointUniversal_B1_At_Zero_Default_Axes
	{
		public Fixture_dxJointUniversal_B1_At_Zero_Default_Axes()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			jId   = dJointCreateUniversal (wId, null);


			dJointAttach (jId, bId1, null);
			dJointSetUniversalAnchor (jId, 0, 0, 0);
		}

		//        ~Fixture_dxJointUniversal_B1_At_Zero_Default_Axes()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;


		DUniversalJoint jId;


		// =========================================================================
		// Test ONE BODY behavior
		// =========================================================================


		// Test when there is only one body at position one on the joint
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Default_Axes,
		@Test public void test_dJointGetUniversalAngle1_1Body_B1()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);

			DVector3 axis1 = new DVector3();
			dJointGetUniversalAxis1 (jId, axis1);
			DVector3 axis2 = new DVector3();
			dJointGetUniversalAxis2 (jId, axis2);

			DMatrix3 R = new DMatrix3();

			double ang1 = (0.23);
			dRFromAxisAndAngle (R, axis1.get0(), axis1.get1(), axis1.get2(), ang1);
			dBodySetRotation (bId1, R);

			double ang2 = (0.0);


			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);



			DMatrix3 I = new DMatrix3();
			dRSetIdentity(I); // Set the rotation of the body to be the Identity (i.e. zero)
			dBodySetRotation (bId1, I);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


			// Test the same rotation, when axis1 is inverted
			dJointSetUniversalAxis1 (jId, -axis1.get0(), -axis1.get1(), -axis1.get2());

			dBodySetRotation (bId1, R);

			CHECK_CLOSE (-ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (-ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);


			// Test the same rotation, when axis1 is default and axis2 is inverted
			dBodySetRotation (bId1, I);

			dJointSetUniversalAxis1 (jId, axis1.get0(), axis1.get1(), axis1.get2());
			dJointSetUniversalAxis2 (jId, -axis2.get0(), -axis2.get1(), -axis2.get2());


			dBodySetRotation (bId1, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (ang2, angle2.d, 1e-4);
		}
		// Rotate the body by 90deg around X then back to original position.
		// The body is attached at the second position of the joint:
		// dJointAttache(jId, 0, bId);
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
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Default_Axes,
		@Test public void test_dJointSetUniversalAxisOffset_1Body_B1_90Deg()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (M_PI/2.0, dJointGetUniversalAngle1 (jId), 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);
			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),  M_PI/2.0, 0);
			CHECK_CLOSE (M_PI/2.0, dJointGetUniversalAngle1 (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		}

		// Rotate the body by -0.23rad around X then back to original position.
		// The body is attached at the second position of the joint:
		// dJointAttache(jId, 0, bId);
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
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Default_Axes,
		@Test public void test_dJointSetUniversalAxisOffset_1Body_B1_Minus0_23rad()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (-(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);
			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),  -(0.23), 0);
			CHECK_CLOSE (-(0.23), dJointGetUniversalAngle1 (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		}

	}



	// Only one body body2 at (0,0,0)
	// The joint is an Universal Joint.
	// Axis1 is along the X axis.
	// Axis2 is along the Y axis.
	// Anchor at (0, 0, 0)
	//
	//       ^Y
	//       |
	//       |
	//       |
	//       |
	//       |
	// Z <-- X
	public static class Fixture_dxJointUniversal_B2_At_Zero_Default_Axes
	{
		public Fixture_dxJointUniversal_B2_At_Zero_Default_Axes()
		{
			wId = dWorldCreate();

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);

			jId   = dJointCreateUniversal (wId, null);


			dJointAttach (jId, null, bId2);
			dJointSetUniversalAnchor (jId, 0, 0, 0);
		}

		//        ~Fixture_dxJointUniversal_B2_At_Zero_Default_Axes()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId2;

		DUniversalJoint jId;


		// =========================================================================
		// Test ONE BODY behavior
		// =========================================================================


		// Test when there is only one body at position two on the joint
		//TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
		@Test public void test_dJointGetUniversalAngle1_1Body_B2()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1.d, 1e-4);
			CHECK_CLOSE (0, angle2.d, 1e-4);

			DVector3 axis1 = new DVector3();
			dJointGetUniversalAxis1 (jId, axis1);

			DVector3 axis2 = new DVector3();
			dJointGetUniversalAxis2 (jId, axis2);

			DMatrix3 R = new DMatrix3();

			double ang1 = (0.0);

			double ang2 = (0.23);
			dRFromAxisAndAngle (R, axis2.get0(), axis2.get1(), axis2.get2(), ang2);
			dBodySetRotation (bId2, R);



			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);



			DMatrix3 I = new DMatrix3();
			dRSetIdentity(I); // Set the rotation of the body to be the Identity (i.e. zero)
			dBodySetRotation (bId2, I);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis2 (jId, -axis2.get0(), -axis2.get1(), -axis2.get2());

			dBodySetRotation (bId2, R);

			CHECK_CLOSE (-ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (-ang1, angle1.d, 1e-4);
			CHECK_CLOSE (ang2, angle2.d, 1e-4);

			// Test the same rotation, when axis1 is inverted and axis2 is default
			dBodySetRotation (bId2, I);

			dJointSetUniversalAxis1 (jId, -axis1.get0(), -axis1.get1(), -axis1.get2());
			dJointSetUniversalAxis2 (jId, axis2.get0(), axis2.get1(), axis2.get2());


			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1.d, 1e-4);
			CHECK_CLOSE (-ang2, angle2.d, 1e-4);
		}

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
		//TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
		@Test public void test_dJointSetUniversalAxisOffset_1Body_B2_90Deg()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);


			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang2 = d2r((90.0));

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(), 0, -ang2);
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);
		}

		// Rotate B2 by -0.23rad around Y then back to original position
		//
		//   ^         ^
		//   |  =>    /
		//   |       /
		//  B2      B2
		//
		// Start with an offset of -0.23rad
		//     ^     ^
		//    /  =>  |
		//   /       |
		//   B2     B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B2_At_Zero_Default_Axes,
		@Test public void test_dJointSetUniversalAxis2Offset_1Body_B2_Minus0_23rad()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang1 = 0;
			double ang2 = (-0.23);


			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
			dBodySetRotation (bId2, R);


			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);


			dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}


	}





	// =========================================================================
	//
	// =========================================================================


	// Create 2 bodies attached by a Universal joint
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
	public static class dxJointUniversal_Test_Initialization
	{
		public dxJointUniversal_Test_Initialization()
		{
			wId = dWorldCreate();

			// Remove gravity to have the only force be the force of the joint
			dWorldSetGravity(wId, 0,0,0);

			for (int j=0; j<2; ++j)
			{
				bId[j][0] = dBodyCreate (wId);
				dBodySetPosition (bId[j][0], -1, -2, -3);

				bId[j][1] = dBodyCreate (wId);
				dBodySetPosition (bId[j][1], 11, 22, 33);


				DMatrix3 R = new DMatrix3();
				DVector3 axis = new DVector3(); // Random axis

				axis.set(	(0.53),
						-(0.71),
						(0.43));
				dNormalize3(axis);
				dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
						(0.47123)); // 27deg
						dBodySetRotation (bId[j][0], R);


						axis.set(	(1.2),
								(0.87),
								-(0.33));
						dNormalize3(axis);
						dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
								(0.47123)); // 27deg
								dBodySetRotation (bId[j][1], R);

								jId[j]   = dJointCreateUniversal (wId, null);
								dJointAttach (jId[j], bId[j][0], bId[j][1]);
								dJointSetUniversalParam(jId[j], dParamLoStop, 1);
								dJointSetUniversalParam(jId[j], dParamHiStop, 2);
								dJointSetUniversalParam(jId[j], dParamFMax, 200);
			}
		}

		//        ~dxJointUniversal_Test_Initialization()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody[][] bId = new DBody[2][2];


		DUniversalJoint[] jId = new DUniversalJoint[2];

		//    };


		// Test if setting a Universal with its default values
		// will behave the same as a default Universal joint
		//TEST_FIXTURE (dxJointUniversal_Test_Initialization,
		@Test public void test_Universal_Initialization()
		{
			//using namespace std;

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1(jId[1], axis);
			dJointSetUniversalAxis1(jId[1], axis.get0(), axis.get1(), axis.get2());

			dJointGetUniversalAxis2(jId[1], axis);
			dJointSetUniversalAxis2(jId[1], axis.get0(), axis.get1(), axis.get2());

			DVector3 anchor = new DVector3();
			dJointGetUniversalAnchor(jId[1], anchor);
			dJointSetUniversalAnchor(jId[1], anchor.get0(), anchor.get1(), anchor.get2());


			for (int b=0; b<2; ++b)
			{
				// Compare body b of the first joint with its equivalent on the
				// second joint
				DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
				DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
				CHECK_CLOSE (qA.get0(), qB.get0(), 1e-4);
				CHECK_CLOSE (qA.get1(), qB.get1(), 1e-4);
				CHECK_CLOSE (qA.get2(), qB.get2(), 1e-4);
				CHECK_CLOSE (qA.get3(), qB.get3(), 1e-4);
			}

			dWorldStep (wId,0.5);
			dWorldStep (wId,0.5);
			dWorldStep (wId,0.5);
			dWorldStep (wId,0.5);

			for (int b=0; b<2; ++b)
			{
				// Compare body b of the first joint with its equivalent on the
				// second joint
				DQuaternionC qA = dBodyGetQuaternion(bId[0][b]);
				DQuaternionC qB = dBodyGetQuaternion(bId[1][b]);
				CHECK_CLOSE (qA.get0(), qB.get0(), 1e-4);
				CHECK_CLOSE (qA.get1(), qB.get1(), 1e-4);
				CHECK_CLOSE (qA.get2(), qB.get2(), 1e-4);
				CHECK_CLOSE (qA.get3(), qB.get3(), 1e-4);


				DVector3C posA = dBodyGetPosition(bId[0][b]);
				DVector3C posB = dBodyGetPosition(bId[1][b]);
				CHECK_CLOSE (posA.get0(), posB.get0(), 1e-4);
				CHECK_CLOSE (posA.get1(), posB.get1(), 1e-4);
				CHECK_CLOSE (posA.get2(), posB.get2(), 1e-4);
				//CHECK_CLOSE (posA[3], posB[3], 1e-4);
			}
		}
	}



	//  The 2 bodies are positionned at (0, 0, 0), with no rotation
	//  The joint is an Universal Joint.
	//  Axis in the inverse direction of the X axis
	//  Anchor at (0, 0, 0)
	//          ^Y
	//          |
	//          |
	//          |
	//          |
	//          |
	//  Z <---- x (X going out of the page)
	public static class Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X
	{
		public Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);

			jId   = dJointCreateUniversal (wId, null);
			joint = (DxJointUniversal) jId;


			dJointAttach (jId, bId1, bId2);
			dJointSetUniversalAnchor (jId, 0, 0, 0);

			//TZ: moved to initialisation
			//            axis[0] = -1;
			//            axis[1] = 0;
			//            axis[2] = 0;
			dJointSetUniversalAxis1(jId, axis.get0(), axis.get1(), axis.get2());
		}

		//        ~Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DUniversalJoint jId;
		DxJointUniversal joint;

		DVector3C axis = new DVector3(-1, 0, 0);
		//    };


		// No offset when setting the Axis1 offset
		// x is a Symbol for lines pointing into the screen
		// . is a Symbol for lines pointing out of the screen
		//
		//    In 2D                   In 3D
		//    ^  ^      ^    ^        Z ^   ^ Y
		//    |  |  =>  |    |          |  /
		//    |  |      |    |          | /
		//   B1  B2     B1   B2         .-------> X     <-- Axis1
		//
		//  Start with a Delta of 90deg
		//    ^  ^         ^   ^
		//    |  |    =>  |   |
		//    |  |        |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxis1Offset_No_Offset_Axis1_Inverse_of_X()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
			CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);

			double ang1 = (0.0);
			double ang2 = (0.0);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang1);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (ang2, angle2, 1e-4);

			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}



		//  Rotate 1st body 90deg around axis1 then back to original position
		//  x is a Symbol for lines pointing into the screen
		//  . is a Symbol for lines pointing out of the screen
		//
		//    In 2D                   In 3D
		//    ^  ^           ^        Z ^   ^ Y
		//    |  |  =>   x   |          |  /
		//    |  |           |          | /
		//   B1  B2     B1   B2         .-------> X     <-- Axis1
		//
		//  Start with a Delta of 90deg
		//       ^         ^   ^
		//    x  |    =>  |   |
		//       |        |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxis1Offset_B1_90Deg_Axis1_Inverse_of_X()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
			CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);

			double ang1 = d2r(90);
			double ang2 = (0.0);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang1);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (ang2, angle2, 1e-4);

			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}



		// No offset when setting the Axis 2 offset
		// x is a Symbol for lines pointing into the screen
		// . is a Symbol for lines pointing out of the screen
		//
		//    In 2D                   In 3D
		//    ^  ^       ^   ^        Z ^   ^ Y             ^ Axis2
		//    |  |  =>   |   |          |  /               /
		//    |  |       |   |          | /               /
		//   B1  B2     B1   B2         . ------->    <-- Axis1
		//
		//  Start with a Delta of 90deg
		//    ^  ^        ^   ^
		//    |  |    =>  |   |
		//    |  |        |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxis2Offset_No_Offset_Axis2_Inverse_of_X()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
			CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang1 = d2r((0.0));
			double ang2 = d2r((0.0));

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);

			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}

		//  Rotate 2nd body 90deg around axis2 then back to original position
		//
		//    In 2D                   In 3D
		//    ^  ^       ^            Z ^   ^ Y             ^ Axis2
		//    |  |  =>   |   -->        |  /               /
		//    |  |       |              | /               /
		//   B1  B2     B1   B2         . ------->    <-- Axis1
		//
		//  Start with a Delta of 90deg
		//    ^           ^   ^
		//    | <---  =>  |   |
		//    |           |   |
		//   B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxisOffset_B2_90Deg()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
			CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang1 = d2r((0.0));
			double ang2 = d2r((90.0));

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis2Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);

			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}


		//  Rotate 2nd body -90deg around axis2 then back to original position
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
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxis1Offset_B2_Minus90Deg()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);
			CHECK_CLOSE (dJointGetUniversalAngle2 (jId), 0, 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis2 (jId, axis);

			double ang1 = d2r(0.0);
			double ang2 = d2r((-90.0));


			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang2);
			dBodySetRotation (bId2, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);


			dJointGetUniversalAxis1 (jId, axis);
			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),
					ang1, -ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId2, R);


			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
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
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxis1Offset_B1_0_23rad()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);

			double ang1 = (0.23);
			double ang2 = (0.0);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang1);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);


			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),  ang1, ang2);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (-ang2, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (ang1, angle1, 1e-4);
			CHECK_CLOSE (-ang2, angle2, 1e-4);


			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
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
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxisOffset_B1_Minus0_23rad()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE ((0.23), dJointGetUniversalAngle1 (jId), 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);
			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),  (0.23), 0);
			CHECK_CLOSE ((0.23), dJointGetUniversalAngle1 (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		}
	}





	// Only one body body1 at (0,0,0)
	// The joint is an Universal Joint.
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
	public static class Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X
	{
		public Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			jId   = dJointCreateUniversal (wId, null);
			joint = (DxJointUniversal) jId;


			dJointAttach (jId, bId1, null);
			dJointSetUniversalAnchor (jId, 0, 0, 0);

			//TZ: moved to initialization
			//            axis[0] = -1;
			//            axis[1] = 0;
			//            axis[2] = 0;
			dJointSetUniversalAxis1(jId, axis.get0(), axis.get1(), axis.get2());
		}

		//        ~Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;


		DUniversalJoint jId;
		DxJointUniversal joint;

		DVector3C axis = new DVector3(-1, 0, 0);



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
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxisOffset_1Body_B1_90Deg()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1(jId, axis);

			double ang1 = d2r((90.0));

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), ang1);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);

			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(), ang1, 0);
			CHECK_CLOSE (ang1, dJointGetUniversalAngle1 (jId), 1e-4);

			dRSetIdentity(R); // Set the rotation of the body to be zero
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
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
		//TEST_FIXTURE (Fixture_DxJointUniversal_B1_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetUniversalAxisOffset_1Body_B1_Minus0_23rad()
		{
			CHECK_CLOSE (dJointGetUniversalAngle1 (jId), 0, 1e-4);

			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, 1, 0, 0, -(0.23));
			dBodySetRotation (bId1, R);

			CHECK_CLOSE ((0.23), dJointGetUniversalAngle1 (jId), 1e-4);

			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);
			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(),  (0.23), 0);
			CHECK_CLOSE ((0.23), dJointGetUniversalAngle1 (jId), 1e-4);

			dRFromAxisAndAngle (R, 1, 0, 0, 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		}

	}

















	// The 2 bodies are positionned at (0,0,0),  and (0,0,0)
	// The bodis have no rotation.
	// The joint is a Universal Joint
	// The axis of the joint are at random (Still at 90deg w.r.t each other)
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointUniversal_B1_and_B2_Axis_Random
	{
		public Fixture_dxJointUniversal_B1_and_B2_Axis_Random()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, -1, -2, -3);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 11, 22, 33);


			jId   = dJointCreateUniversal (wId, null);


			dJointAttach (jId, bId1, bId2);

			DVector3 axis1 = new DVector3();
			axis1.set(	(0.53),
					-(0.71),
					(0.43));
			dNormalize3(axis1);

			DVector3 axis = new DVector3();
			axis.set(	(1.2),
					(0.87),
					-(0.33));

			DVector3 axis2 = new DVector3();
			dCROSS(axis2, OP.EQ, axis1, axis);

			dJointSetUniversalAxis1(jId, axis1.get0(), axis1.get1(), axis1.get2());
			dJointSetUniversalAxis2(jId, axis2.get0(), axis2.get1(), axis2.get2());
		}

		//        ~Fixture_dxJointUniversal_B1_and_B2_Axis_Random()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}


		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DUniversalJoint jId;
		//    };


		// Rotate first body 90deg around Axis1 then back to original position
		//
		//   ^  ^           ^       Z ^
		//   |  |  => <---  |         |
		//   |  |           |         |
		//  B1  B2     B1   B2      X .----->Y
		//                          N.B. X is going out of the screen
		// Set Axis1 with an Offset of 90deg
		//      ^        ^   ^
		// <--- |  =>    |   |
		//      |        |   |
		//  B1  B2      B1   B2
		//TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_Axis_Random,
		@Test public void test_dJointSetUniversalAxisOffset_B1_90deg_Axis_Random()
		{
			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);
			RefDouble angle1 = new RefDouble(), angle2 = new RefDouble();
			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);


			DVector3 axis = new DVector3();
			dJointGetUniversalAxis1 (jId, axis);

			double angle = d2r(90);
			DMatrix3 R = new DMatrix3();
			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), angle);
			dBodySetRotation (bId1, R);


			CHECK_CLOSE (angle, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);



			dJointSetUniversalAxis1Offset (jId, axis.get0(), axis.get1(), axis.get2(), angle, 0);
			CHECK_CLOSE (angle, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (angle, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);


			dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(), 0);
			dBodySetRotation (bId1, R);

			CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
			CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

			dJointGetUniversalAngles(jId, angle1, angle2);
			CHECK_CLOSE (0, angle1, 1e-4);
			CHECK_CLOSE (0, angle2, 1e-4);
		}
	}
} // End of SUITE TestdxJointUniversal

