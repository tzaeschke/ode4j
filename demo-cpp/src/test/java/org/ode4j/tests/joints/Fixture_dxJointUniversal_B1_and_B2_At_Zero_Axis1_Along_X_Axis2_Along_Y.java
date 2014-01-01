/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zï¿½schke      *
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

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateUniversal;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAngle1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAngle2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAngles;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetUniversalAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis1Offset;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis2Offset;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.ode.OdeMath.dCalcVectorCross3;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Rotation.dRFromAxisAndAngle;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.AfterClass;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DUniversalJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.joints.DxJointUniversal;

// The 2 bodies are positionned at (0, 0, 0)
// The bodis have no rotation.
// The joint is a Universal Joint
// Axis1 is along the X axis
// Axis2 is along the Y axis
// Anchor at (0, 0, 0)
public class Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis1_Along_X_Axis2_Along_Y
{
    private static double d2r(double degree) {
        return degree * (M_PI / 180.0);
    }
//    private static double r2d(double degree) {
//        return degree * (180.0/M_PI);
//    }

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
		R.setIdentity();
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
		dCalcVectorCross3(cross, axis1, axis2);
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


		R.setIdentity(); // Set the rotation of the body to be zero
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


		R.setIdentity(); // Set the rotation of the body to be zero
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


		R.setIdentity(); // Set the rotation of the body to be zero
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


		R.setIdentity(); // Set the rotation of the body to be zero
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


		R.setIdentity(); // Set the rotation of the body to be zero
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


		R.setIdentity(); // Set the rotation of the body to be zero
		dBodySetRotation (bId1, R);
		dBodySetRotation (bId2, R);

		CHECK_CLOSE (0, dJointGetUniversalAngle1 (jId), 1e-4);
		CHECK_CLOSE (0, dJointGetUniversalAngle2 (jId), 1e-4);

		dJointGetUniversalAngles(jId, angle1, angle2);
		CHECK_CLOSE (0, angle1, 1e-4);
		CHECK_CLOSE (0, angle2, 1e-4);
	}
}