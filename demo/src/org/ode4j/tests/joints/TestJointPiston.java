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
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.joints.DxJointPiston;

import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.*;

//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

/**
 * This file create unit test for some of the functions found in:
 * ode/src/joinst/piston.cpp
 */

//#include <UnitTest++.h>
//#include <ode/ode.h>
//
//#include "../../ode/src/joints/piston.h"

//SUITE (TestdxJointPiston)
@RunWith(Enclosed.class)
public class TestJointPiston
{
	// The 2 bodies are positionned at (0, 0, 0), with no rotation
	// The joint is a Piston Joint
	// Axis is along the X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X
	{
		public Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);

			jId   = dJointCreatePiston (wId, null);
			joint = (DxJointPiston) jId;


			dJointAttach (jId, bId1, bId2);

			dJointSetPistonAxis (jId, axis.get0(), axis.get1(), axis.get2());
		}

		//        ~Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DPistonJoint jId;
		DxJointPiston joint;

		static DVector3C axis = new DVector3(1, 0, 0);

		static double offset = 3.1;
		//    };
		//TODO
		//    const dVector3 Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X::axis =
		//    {
		//        1, 0, 0
		//    };
		//    const dReal    Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X::offset =  (3.1);

		// Move 1st body offset unit in the X direction
		//
		//  X------->       X---------> Axis -->
		//  B1          =>     B1
		//  B2              B2
		//
		// Start with a Offset of offset unit
		//
		//  X------->       X---------> Axis -->
		//     B1       =>  B1
		//  B2              B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B1_3Unit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, offset, 0, 0);

			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			// Only here to test a deprecated warning
			//TZ commented out because it's deprecated
			//dJointSetPistonAxisDelta (jId, 1, 0, 0, 0, 0, 0);
		}

		// Move 1st body offset unit in the opposite X direction
		//
		//  X------->          X---------> Axis -->
		//  B1          =>  B1
		//  B2                 B2
		//
		// Start with a Offset of -offset unit
		//
		//      X------->      X---------> Axis -->
		//  B1            =>   B1
		//      B2             B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B1_Minus_3Unit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, -offset, 0, 0);

			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					-offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}

		// Move 2nd body offset unit in the X direction
		//
		//  X------->       X---------> Axis -->
		//  B1          =>  B1
		//  B2                 B2
		//
		// Start with a Offset of offset unit
		//
		//  X------->       X---------> Axis -->
		//  B1          =>  B1
		//     B2           B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_3Unit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, offset, 0, 0);

			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					-offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}

		// Move 2nd body offset unit in the opposite X direction
		//
		//  X------->          X---------> Axis -->
		//  B1          =>     B1
		//  B2              B2
		//
		// Start with a Offset of -offset unit
		//
		//     X------->    X---------> Axis -->
		//     B1       =>  B1
		//  B2              B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_Minus_3Unit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, -offset, 0, 0);

			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}
		// ==========================================================================
		// Test Position Rate
		// ==========================================================================

		// Apply force on 1st body in the X direction also the Axis direction
		//
		//  X------->       X---------> Axis -->
		//  B1  F->      =>     B1
		//  B2              B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Along_Axis_on_B1()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId1, 1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
		}

		// Apply force on 1st body in the inverse X direction
		//
		//  X------->           X---------> Axis -->
		//  B1  <-F      => B1
		//  B2                  B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B1()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId1, -1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
		}


		// Apply force on 1st body in the X direction also the Axis direction
		//
		//  X------->       X---------> Axis -->
		//  B1          =>  B1
		//  B2 F->             B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Along_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, 1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
		}

		// Apply force on 1st body in the inverse X direction
		//
		//  X------->           X---------> Axis -->
		//  B1           =>     B1
		//  B2  <-F          B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, -1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
		}


	}


	// The 2 bodies are positionned at (0, 0, 0), with no rotation
	// The joint is a Piston Joint
	// Axis is the opposite of the X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X
	{
		public Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);

			jId   = dJointCreatePiston (wId, null);
			joint = (DxJointPiston) jId;


			dJointAttach (jId, bId1, bId2);


			dJointSetPistonAxis (jId, axis.get0(), axis.get1(), axis.get2());
		}

		//        ~Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;
		DBody bId2;


		DPistonJoint jId;
		DxJointPiston joint;

		static DVector3C axis = new DVector3(-1, 0, 0);
		static double offset = 3.1;
		//    };
		//TODO
		//    const dVector3 Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X::axis =
		//    {
		//        -1, 0, 0
		//    };
		//    const dReal    Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X::offset =  (3.1);

		// Move 1st body offset unit in the X direction
		//
		//  X------->       X--------->   <-- Axis
		//  B1          =>     B1
		//  B2              B2
		//
		// Start with a Offset of offset unit
		//
		//  X------->       X--------->  <-- Axis
		//     B1       =>  B1
		//  B2              B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonAxisOffset_B1_3Unit()
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
		//  X------->          X--------->  <-- Axis
		//  B1          =>  B1
		//  B2                 B2
		//
		// Start with a Offset of offset unit
		//
		//     X------->  X--------->      <-- Axis
		//  B1       =>   B1
		//     B2         B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonAxisOffset_B1_Minus_3Unit()
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

		// Move 2nd body offset unit in the X direction
		//
		//  X------->       X--------->  <-- Axis
		//  B1          =>  B1
		//  B2                 B2
		//
		// Start with a Offset of offset unit
		//
		//  X------->       X--------->  <-- Axis
		//  B1          =>  B1
		//     B2           B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_3Unit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, offset, 0, 0);

			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}

		// Move 2nd body offset unit in the opposite X direction
		//
		//  X------->          X--------->  <-- Axis
		//  B1          =>     B1
		//  B2              B2
		//
		// Start with a Offset of -offset unit
		//
		//     X------->    X--------->     <-- Axis
		//     B1       =>  B1
		//  B2              B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_Minus_3Unit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, -offset, 0, 0);

			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					-offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}
		// ==========================================================================
		// Test Position Rate
		// ==========================================================================

		// Apply force on 1st body in the X direction also the Axis direction
		//
		//  X------->       X---------> <-- Axis
		//  B1  F->      =>     B1
		//  B2              B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
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
		//  B2                  B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B1()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId1, -1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
		}

		// Apply force on 1st body in the X direction also the Axis direction
		//
		//  X------->       X---------> <-- Axis
		//  B1          =>  B1
		//  B2 F->             B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, 1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
		}

		// Apply force on 1st body in the inverse X direction
		//
		//  X------->           X---------> <-- Axis
		//  B1          =>      B1
		//  B2 <-F           B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, -1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
		}



	}

	// Only body 1
	// The body are positionned at (0, 0, 0), with no rotation
	// The joint is a Piston Joint
	// Axis is along the X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X
	{
		public Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X()
		{
			wId = dWorldCreate();

			bId1 = dBodyCreate (wId);
			dBodySetPosition (bId1, 0, 0, 0);

			jId   = dJointCreatePiston (wId, null);
			joint = (DxJointPiston) jId;


			dJointAttach (jId, bId1, null);

			dJointSetPistonAxis (jId, axis.get0(), axis.get1(), axis.get2());
		}

		//        ~Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1;

		DPistonJoint jId;
		DxJointPiston joint;

		static DVector3C axis = new DVector3(1, 0, 0);

		static double offset = 3.1;
		//};
		//TODO
		//    const dVector3 Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X::axis =
		//    {
		//        1, 0, 0
		//    };
		//    const dReal    Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X::offset =  (3.1);

		// Move 1st body offset unit in the X direction
		//
		//  X------->       X---------> Axis -->
		//  B1          =>     B1
		//
		// Start with a Offset of offset unit
		//
		//  X------->       X---------> Axis -->
		//     B1       =>  B1
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B1_OffsetUnit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, offset, 0, 0);

			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}

		// Move 1st body offset unit in the opposite X direction
		//
		//  X------->          X---------> Axis -->
		//  B1          =>  B1
		//
		// Start with a Offset of -offset unit
		//
		//      X------->      X---------> Axis -->
		//  B1            =>   B1
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B1_Minus_OffsetUnit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, -offset, 0, 0);

			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					-offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId1, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}
		// ==========================================================================
		// Test Position Rate
		// ==========================================================================

		// Apply force on 1st body in the X direction also the Axis direction
		//
		//  X------->       X---------> Axis -->
		//  B1  F->      =>     B1
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Along_Axis_on_B1()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId1, 1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
		}

		// Apply force on 1st body in the inverse X direction
		//
		//  X------->           X---------> Axis -->
		//  B1  <-F      => B1
		//TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B1()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId1, -1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
		}


	}

	// Only body 1
	// The body are positionned at (0, 0, 0), with no rotation
	// The joint is a Piston Joint
	// Axis is in the oppsite X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X
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
		//TODO
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








	// Only body 2
	// The body are positionned at (0, 0, 0), with no rotation
	// The joint is a Piston Joint
	// Axis is along the X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X
	{
		public Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X()
		{
			wId = dWorldCreate();

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);

			jId   = dJointCreatePiston (wId, null);
			joint = (DxJointPiston) jId;


			dJointAttach (jId, null, bId2);

			dJointSetPistonAxis (jId, axis.get0(), axis.get1(), axis.get2());
		}

		//        ~Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId2;

		DPistonJoint jId;
		DxJointPiston joint;

		static DVector3C axis = new DVector3(1, 0, 0);

		static double offset = 3.1;
		//    };
		//TODO
		//    const dVector3 Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X::axis =
		//    {
		//        1, 0, 0
		//    };
		//    const dReal    Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X::offset =  (3.1);

		// Move 2nd body offset unit in the X direction
		//
		//  X------->       X---------> Axis -->
		//  B2          =>     B2
		//
		// Start with a Offset of offset unit
		//
		//  X------->       X---------> Axis -->
		//     B2       =>  B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_OffsetUnit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, offset, 0, 0);

			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					-offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}

		// Move 2nd body offset unit in the opposite X direction
		//
		//  X------->          X---------> Axis -->
		//  B2          =>  B2
		//
		// Start with a Offset of -offset unit
		//
		//      X------->      X---------> Axis -->
		//  B2            =>   B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_Minus_OffsetUnit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, -offset, 0, 0);

			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}
		// ==========================================================================
		// Test Position Rate
		// ==========================================================================

		// Apply force on body 2 in the X direction also the Axis direction
		//
		//  X------->       X---------> Axis -->
		//  B2 F->             B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Along_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, 1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
		}

		// Apply force on body 2 in the inverse X direction
		//
		//  X------->           X---------> Axis -->
		//  B2  <-F          B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, -1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
		}


	}

	// Only body 2
	// The body are positionned at (0, 0, 0), with no rotation
	// The joint is a Piston Joint
	// Axis is in the opposite X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X
	{
		public Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X()
		{
			wId = dWorldCreate();

			bId2 = dBodyCreate (wId);
			dBodySetPosition (bId2, 0, 0, 0);

			jId   = dJointCreatePiston (wId, null);
			joint = (DxJointPiston) jId;


			dJointAttach (jId, null, bId2);

			dJointSetPistonAxis (jId, axis.get0(), axis.get1(), axis.get2());
		}

		//        ~Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId2;

		DPistonJoint jId;
		DxJointPiston joint;

		static DVector3C axis = new DVector3(-1, 0, 0);

		static double offset = 3.1;
		//    };
		//TODO
		//    const dVector3 Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X::axis =
		//    {
		//        -1, 0, 0
		//    };
		//    const dReal    Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X::offset =  (3.1);

		// Move 2nd body offset unit in the X direction
		//
		//  X------->       X--------->  <--- Axis
		//  B2          =>     B2
		//
		// Start with a Offset of offset unit
		//
		//  X------->       X--------->  <--- Axis
		//     B2       =>  B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_OffsetUnit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, offset, 0, 0);

			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					offset*axis.get0(),offset*axis.get1(),offset*axis.get2());
			CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			//TZ: commented out, because it's deprecated
			//dJointSetPistonAxisDelta (jId, 1, 0, 0, 0, 0, 0);
		}

		// Move 1st body offset unit in the opposite X direction
		//
		//  X------->          X--------->   <--- Axis
		//  B2          =>  B2
		//
		// Start with a Offset of -offset unit
		//
		//      X------->      X--------->   <--- Axis
		//  B2            =>   B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonAxisOffset_B2_Minus_OffsetUnit()
		{
			dJointSetPistonAnchor (jId, 0, 0, 0);

			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, -offset, 0, 0);

			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dJointSetPistonAnchorOffset (jId, 0, 0, 0,
					-offset*axis.get0(),-offset*axis.get1(),-offset*axis.get2());
			CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

			dBodySetPosition (bId2, 0, 0, 0);
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
		}

		// ==========================================================================
		// Test Position Rate
		// ==========================================================================

		// Apply force on body 2 in the X direction also the Axis direction
		//
		//  X------->       X---------> <-- Axis
		//  B2 F->             B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, 1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
		}

		// Apply force on body 2 in the inverse X direction
		//
		//  X------->           X---------> <-- Axis
		//  B2 <-F           B2
		//TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
		@Test public void test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B2()
		{
			CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
			CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

			dBodyAddForce (bId2, -1.0, 0, 0);
			dWorldQuickStep (wId, 1.0);

			CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
		}
	}



	// Create 2 bodies attached by a Piston joint
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
	public static class dxJointPiston_Test_Initialization
	{
		public dxJointPiston_Test_Initialization()
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

				axis.set( (0.53), 
						-(0.71),
						(0.43));
				dNormalize3(axis);
				dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
						(0.47123)); // 27deg
						dBodySetRotation (bId[j][0], R);


						axis.set( (1.2),
								(0.87),
								-(0.33));
						dNormalize3(axis);
						dRFromAxisAndAngle (R, axis.get0(), axis.get1(), axis.get2(),
								(0.47123)); // 27deg
								dBodySetRotation (bId[j][1], R);

								jId[j] = dJointCreatePiston (wId, null);
								dJointAttach (jId[j], bId[j][0], bId[j][1]);
			}
		}

		//        ~dxJointPiston_Test_Initialization()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody[][] bId = new DBody[2][2];


		DPistonJoint[] jId = new DPistonJoint[2];

		//    };


		// Test if setting a Piston with its default values
		// will behave the same as a default Piston joint
		//TEST_FIXTURE (dxJointPiston_Test_Initialization,
		@Test public void test_Piston_Initialization()
		{
			//using namespace std;

			DVector3 axis = new DVector3();
			dJointGetPistonAxis(jId[1], axis);
			dJointSetPistonAxis(jId[1], axis.get0(), axis.get1(), axis.get2());


			DVector3 anchor = new DVector3();
			dJointGetPistonAnchor(jId[1], anchor);
			dJointSetPistonAnchor(jId[1], anchor.get0(), anchor.get1(), anchor.get2());


			for (int b=0; b<2; ++b)
			{
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

			for (int b=0; b<2; ++b)
			{
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
				//CHECK_CLOSE (posA[3], posB[3], 1e-6);
			}


		}
	}




	// Compare only one body to 2 bodies with one fixed.
	//
	// The body are positionned at (0, 0, 0), with no rotation
	// The joint is a Piston Joint
	// Axis is along the X axis
	// Anchor at (0, 0, 0)
	public static class Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X
	{
		public Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X()
		{
			wId = dWorldCreate();

			bId1_12 = dBodyCreate (wId);
			dBodySetPosition (bId1_12, 0, 0, 0);

			bId2_12 = dBodyCreate (wId);
			dBodySetPosition (bId2_12, 0, 0, 0);
			// The force will be added in the function since it is not
			// always on the same body

			jId_12 = dJointCreatePiston (wId, null);
			dJointAttach(jId_12, bId1_12, bId2_12);

			fixed = dJointCreateFixed (wId, null);



			bId = dBodyCreate (wId);
			dBodySetPosition (bId, 0, 0, 0);

			dBodyAddForce (bId, 4, 0, 0);

			jId = dJointCreatePiston (wId, null);
		}

		//        ~Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X()
		@AfterClass
		public static void DESTRUCTOR() {
			dWorldDestroy (wId);
		}

		static DWorld wId;

		DBody bId1_12;
		DBody bId2_12;

		DPistonJoint jId_12; // Joint with 2 bodies

		DFixedJoint fixed;



		DBody  bId;
		DPistonJoint jId;    // Joint with one body
		//    };

		// This test compare the result of a slider with 2 bodies where body body 2 is
		// fixed to the world to a slider with only one body at position 1.
		//
		// Test the limits [-1, 0.25] when only one body at is attached to the joint
		// using dJointAttache(jId, bId, 0);
		//
		//TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
		@Test public void test_Limit_minus1_025_One_Body_on_left()
		{
			dBodyAddForce (bId1_12, 4, 0, 0);

			dJointAttach(jId_12, bId1_12, bId2_12);
			dJointSetPistonParam(jId_12, dParamLoStop, -1);
			dJointSetPistonParam(jId_12, dParamHiStop, 0.25);

			dJointAttach(fixed, null, bId2_12);
			dJointSetFixed(fixed);

			dJointAttach(jId, bId, null);
			dJointSetPistonParam(jId, dParamLoStop, -1);
			dJointSetPistonParam(jId, dParamHiStop, 0.25);


			for (int i=0; i<50; ++i)
				dWorldStep(wId, 1.0);


			DVector3C pos1_12 = dBodyGetPosition(bId1_12);
			DVector3C pos = dBodyGetPosition(bId);

			CHECK_CLOSE (pos1_12.get0(), pos.get0(), 1e-2);
			CHECK_CLOSE (pos1_12.get1(), pos.get1(), 1e-2);
			CHECK_CLOSE (pos1_12.get2(), pos.get2(), 1e-2);

			DQuaternionC q1_12 = dBodyGetQuaternion(bId1_12);
			DQuaternionC q = dBodyGetQuaternion(bId);

			CHECK_CLOSE (q1_12.get0(), q.get0(), 1e-4);
			CHECK_CLOSE (q1_12.get1(), q.get1(), 1e-4);
			CHECK_CLOSE (q1_12.get2(), q.get2(), 1e-4);
			CHECK_CLOSE (q1_12.get3(), q.get3(), 1e-4);
		}



		// This test compare the result of a slider with 2 bodies where body body 1 is
		// fixed to the world to a slider with only one body at position 2.
		//
		// Test the limits [-1, 0.25] when only one body at is attached to the joint
		// using dJointAttache(jId, 0, bId);
		//
		//TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
		@Test public void test_Limit_minus1_025_One_Body_on_right()
		{
			dBodyAddForce (bId2_12, 4, 0, 0);

			dJointAttach(jId_12, bId1_12, bId2_12);
			dJointSetPistonParam(jId_12, dParamLoStop, -1);
			dJointSetPistonParam(jId_12, dParamHiStop, 0.25);

			dJointAttach(fixed, bId1_12, null);
			dJointSetFixed(fixed);


			dJointAttach(jId, null, bId);
			dJointSetPistonParam(jId, dParamLoStop, -1);
			dJointSetPistonParam(jId, dParamHiStop, 0.25);

			for (int i=0; i<50; ++i)
				dWorldStep(wId, 1.0);


			DVector3C pos2_12 = dBodyGetPosition(bId2_12);
			DVector3C pos = dBodyGetPosition(bId);

			CHECK_CLOSE (pos2_12.get0(), pos.get0(), 1e-2);
			CHECK_CLOSE (pos2_12.get1(), pos.get1(), 1e-2);
			CHECK_CLOSE (pos2_12.get2(), pos.get2(), 1e-2);


			DQuaternionC q2_12 = dBodyGetQuaternion(bId2_12);
			DQuaternionC q = dBodyGetQuaternion(bId);

			CHECK_CLOSE (q2_12.get0(), q.get0(), 1e-4);
			CHECK_CLOSE (q2_12.get1(), q.get1(), 1e-4);
			CHECK_CLOSE (q2_12.get2(), q.get2(), 1e-4);
			CHECK_CLOSE (q2_12.get3(), q.get3(), 1e-4);
		}



		// This test compare the result of a slider with 2 bodies where body body 2 is
		// fixed to the world to a slider with only one body at position 1.
		//
		// Test the limits [0, 0] when only one body at is attached to the joint
		// using dJointAttache(jId, bId, 0);
		//
		// The body should not move since their is no room between the two limits
		//
		//TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
		@Test public void test_Limit_0_0_One_Body_on_left()
		{
			dBodyAddForce (bId1_12, 4, 0, 0);

			dJointAttach(jId_12, bId1_12, bId2_12);
			dJointSetPistonParam(jId_12, dParamLoStop, 0);
			dJointSetPistonParam(jId_12, dParamHiStop, 0);

			dJointAttach(fixed, null, bId2_12);
			dJointSetFixed(fixed);


			dJointAttach(jId, bId, null);
			dJointSetPistonParam(jId, dParamLoStop, 0);
			dJointSetPistonParam(jId, dParamHiStop, 0);

			for (int i=0; i<500; ++i)
				dWorldStep(wId, 1.0);


			DVector3C pos1_12 = dBodyGetPosition(bId1_12);
			DVector3C pos = dBodyGetPosition(bId);

			CHECK_CLOSE (pos1_12.get0(), pos.get0(), 1e-4);
			CHECK_CLOSE (pos1_12.get1(), pos.get1(), 1e-4);
			CHECK_CLOSE (pos1_12.get2(), pos.get2(), 1e-4);

			CHECK_CLOSE (0, pos.get0(), 1e-4);
			CHECK_CLOSE (0, pos.get1(), 1e-4);
			CHECK_CLOSE (0, pos.get2(), 1e-4);


			DQuaternionC q1_12 = dBodyGetQuaternion(bId1_12);
			DQuaternionC q = dBodyGetQuaternion(bId);

			CHECK_CLOSE (q1_12.get0(), q.get0(), 1e-4);
			CHECK_CLOSE (q1_12.get1(), q.get1(), 1e-4);
			CHECK_CLOSE (q1_12.get2(), q.get2(), 1e-4);
			CHECK_CLOSE (q1_12.get3(), q.get3(), 1e-4);
		}


		// This test compare the result of a slider with 2 bodies where body body 1 is
		// fixed to the world to a slider with only one body at position 2.
		//
		// Test the limits [0, 0] when only one body at is attached to the joint
		// using dJointAttache(jId, 0, bId);
		//
		// The body should not move since their is no room between the two limits
		//
		//TEST_FIXTURE(Fixture_dxJointPiston_Compare_Body_At_Zero_Axis_Along_X,
		@Test public void test_Limit_0_0_One_Body_on_right()
		{
			dBodyAddForce (bId2_12, 4, 0, 0);

			dJointAttach(jId_12, bId1_12, bId2_12);
			dJointSetPistonParam(jId_12, dParamLoStop, 0);
			dJointSetPistonParam(jId_12, dParamHiStop, 0);

			dJointAttach(fixed, bId1_12, null);
			dJointSetFixed(fixed);


			dJointAttach(jId, null, bId);
			dJointSetPistonParam(jId, dParamLoStop, 0);
			dJointSetPistonParam(jId, dParamHiStop, 0);

			for (int i=0; i<500; ++i)
				dWorldStep(wId, 1.0);

			DVector3C pos2_12 = dBodyGetPosition(bId2_12);
			DVector3C pos = dBodyGetPosition(bId);

			CHECK_CLOSE (pos2_12.get0(), pos.get0(), 1e-4);
			CHECK_CLOSE (pos2_12.get1(), pos.get1(), 1e-4);
			CHECK_CLOSE (pos2_12.get2(), pos.get2(), 1e-4);

			CHECK_CLOSE (0, pos.get0(), 1e-4);
			CHECK_CLOSE (0, pos.get1(), 1e-4);
			CHECK_CLOSE (0, pos.get2(), 1e-4);


			DQuaternionC q2_12 = dBodyGetQuaternion(bId2_12);
			DQuaternionC q = dBodyGetQuaternion(bId);

			CHECK_CLOSE (q2_12.get0(), q.get0(), 1e-4);
			CHECK_CLOSE (q2_12.get1(), q.get1(), 1e-4);
			CHECK_CLOSE (q2_12.get2(), q.get2(), 1e-4);
			CHECK_CLOSE (q2_12.get3(), q.get3(), 1e-4);
		}

	}
} // End of SUITE TestdxJointPiston
