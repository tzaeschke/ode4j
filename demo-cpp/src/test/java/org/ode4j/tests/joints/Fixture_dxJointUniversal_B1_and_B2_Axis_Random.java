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
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis1Offset;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetUniversalAxis2;
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

// The 2 bodies are positionned at (0,0,0),  and (0,0,0)
// The bodis have no rotation.
// The joint is a Universal Joint
// The axis of the joint are at random (Still at 90deg w.r.t each other)
// Anchor at (0, 0, 0)
public class Fixture_dxJointUniversal_B1_and_B2_Axis_Random
{
    private static double d2r(double degree) {
        return degree * (M_PI / 180.0);
    }
//    private static double r2d(double degree) {
//        return degree * (180.0/M_PI);
//    }

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
		dCalcVectorCross3(axis2, axis1, axis);

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