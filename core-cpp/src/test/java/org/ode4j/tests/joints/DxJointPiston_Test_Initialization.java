/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
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
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetQuaternion;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePiston;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPistonAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPistonAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPistonAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPistonAxis;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.After;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

/**
 *  Create 2 bodies attached by a Piston joint
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
 */
public class DxJointPiston_Test_Initialization extends TestSuperClass
{
	public DxJointPiston_Test_Initialization()
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
	@After
	public void DESTRUCTOR() {
		dWorldDestroy (wId);
	}

	private static DWorld wId;

	private DBody[][] bId = new DBody[2][2];


	private DPistonJoint[] jId = new DPistonJoint[2];

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