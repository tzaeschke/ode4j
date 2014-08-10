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

import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.ode.OdeMath;
import org.ode4j.tests.joints.DxJointAMotor.FixtureXUser;

//TEST_FIXTURE(FixtureXUser, rotate_x)
public class DxJointAMotor_FixtureXUser_rotate_x extends FixtureXUser
{
	@Test public void test_DxJointAMotor_FixtureXUser_rotate_x()
	{
		final double h = 1;
		final double v = 1;
		DMatrix3 identity = new DMatrix3(
				1, 0, 0,// 0,
				0, 1, 0,// 0,
				0, 0, 1);//, 0);
		dBodySetRotation(body, identity);
		dBodySetAngularVel(body, v, 0, 0);
		dWorldQuickStep(world, h);
		DMatrix3C rot = dBodyGetRotation(body);
		CHECK_CLOSE(1, rot.get00(), DxJointAMotor.tol);
		CHECK_CLOSE(0, rot.get10(), DxJointAMotor.tol);
		CHECK_CLOSE(0, rot.get20(), DxJointAMotor.tol);

		CHECK_CLOSE(0, rot.get01(), DxJointAMotor.tol);
		CHECK_CLOSE(OdeMath.dCos(v*h), rot.get11(), DxJointAMotor.tol);
		CHECK_CLOSE(OdeMath.dSin(v*h), rot.get21(), DxJointAMotor.tol);

		CHECK_CLOSE(0, rot.get02(), DxJointAMotor.tol);
		CHECK_CLOSE(-OdeMath.dSin(v*h), rot.get12(), DxJointAMotor.tol);
		CHECK_CLOSE( OdeMath.dCos(v*h), rot.get22(), DxJointAMotor.tol);
	}
}