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
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests.joints;

import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetAMotorAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetAMotorAxisRel;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetAMotorNumAxes;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetAMotorAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetAMotorNumAxes;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.OdeMath;
import org.ode4j.tests.joints.DxJointAMotor.FixtureBase;

//TEST_FIXTURE(FixtureBase, sanity_check)
public class DxJointAMotor_FixtureBase_sanity_check extends FixtureBase
{
	@Test public void test_DxJointAMotor_FixtureBase_sanity_check()
	{
		DMatrix3 R = new DMatrix3();
		dRFromAxisAndAngle(R, 1, 1, 1, 10*OdeMath.M_PI/180);
		dBodySetRotation(body, R);

		DVector3 res = new DVector3();

		joint.attach(body, null);
		dJointSetAMotorNumAxes(joint, 3);
		CHECK_EQUAL(3, dJointGetAMotorNumAxes(joint));

		// axes relative to world
		dJointSetAMotorAxis(joint, 0, 0, 1, 0, 0);
		dJointGetAMotorAxis(joint, 0, res);
		CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 0));
		CHECK_CLOSE(1, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 1, 0, 0, 1, 0);
		dJointGetAMotorAxis(joint, 1, res);
		CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 1));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 2, 0, 0, 0, 1);
		dJointGetAMotorAxis(joint, 2, res);
		CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 2));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get2(), DxJointAMotor.tol);

		// axes relative to body1
		dJointSetAMotorAxis(joint, 0, 1, 1, 0, 0);
		dJointGetAMotorAxis(joint, 0, res);
		CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 0));
		CHECK_CLOSE(1, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 1, 1, 0, 1, 0);
		dJointGetAMotorAxis(joint, 1, res);
		CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 1));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 2, 1, 0, 0, 1);
		dJointGetAMotorAxis(joint, 2, res);
		CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 2));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get2(), DxJointAMotor.tol);

		// axes relative to body2
		dJointSetAMotorAxis(joint, 0, 2, 1, 0, 0);
		dJointGetAMotorAxis(joint, 0, res);
		CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 0));
		CHECK_CLOSE(1, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 1, 2, 0, 1, 0);
		dJointGetAMotorAxis(joint, 1, res);
		CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 1));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 2, 2, 0, 0, 1);
		dJointGetAMotorAxis(joint, 2, res);
		CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 2));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get2(), DxJointAMotor.tol);

		// reverse attachment to force internal reversal
		joint.attach(null, body);
		// axes relative to world
		dJointSetAMotorAxis(joint, 0, 0, 1, 0, 0);
		dJointGetAMotorAxis(joint, 0, res);
		CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 0));
		CHECK_CLOSE(1, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 1, 0, 0, 1, 0);
		dJointGetAMotorAxis(joint, 1, res);
		CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 1));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 2, 0, 0, 0, 1);
		dJointGetAMotorAxis(joint, 2, res);
		CHECK_EQUAL(0, dJointGetAMotorAxisRel(joint, 2));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get2(), DxJointAMotor.tol);

		// axes relative to body1
		dJointSetAMotorAxis(joint, 0, 1, 1, 0, 0);
		dJointGetAMotorAxis(joint, 0, res);
		CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 0));
		CHECK_CLOSE(1, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 1, 1, 0, 1, 0);
		dJointGetAMotorAxis(joint, 1, res);
		CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 1));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 2, 1, 0, 0, 1);
		dJointGetAMotorAxis(joint, 2, res);
		CHECK_EQUAL(1, dJointGetAMotorAxisRel(joint, 2));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get2(), DxJointAMotor.tol);

		// axes relative to body2
		dJointSetAMotorAxis(joint, 0, 2, 1, 0, 0);
		dJointGetAMotorAxis(joint, 0, res);
		CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 0));
		CHECK_CLOSE(1, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 1, 2, 0, 1, 0);
		dJointGetAMotorAxis(joint, 1, res);
		CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 1));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get2(), DxJointAMotor.tol);

		dJointSetAMotorAxis(joint, 2, 2, 0, 0, 1);
		dJointGetAMotorAxis(joint, 2, res);
		CHECK_EQUAL(2, dJointGetAMotorAxisRel(joint, 2));
		CHECK_CLOSE(0, res.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, res.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(1, res.get2(), DxJointAMotor.tol);
	}
}