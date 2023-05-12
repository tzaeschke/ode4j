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

import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetAMotorAxis;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;

import org.junit.Test;
import org.ode4j.math.DVector3;
import org.ode4j.tests.joints.DxJointAMotor.FixtureXEuler;

//TEST_FIXTURE(FixtureXEuler, check_axes)
public class DxJointAMotor_FixtureXEuler_check_axes extends FixtureXEuler
{
	@Test public void test_DxJointAMotor_FixtureXEuler_check_axes()
	{
		// test patch #181 bug fix
		DVector3 axis_x = new DVector3();
		dJointGetAMotorAxis(joint, 0, axis_x);
		CHECK_CLOSE(1, axis_x.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, axis_x.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, axis_x.get2(), DxJointAMotor.tol);

		DVector3 axis_y = new DVector3();
		dJointGetAMotorAxis(joint, 1, axis_y);
		CHECK_CLOSE(0, axis_y.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(1, axis_y.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(0, axis_y.get2(), DxJointAMotor.tol);

		DVector3 axis_z = new DVector3();
		dJointGetAMotorAxis(joint, 2, axis_z);
		CHECK_CLOSE(0, axis_z.get0(), DxJointAMotor.tol);
		CHECK_CLOSE(0, axis_z.get1(), DxJointAMotor.tol);
		CHECK_CLOSE(1, axis_z.get2(), DxJointAMotor.tol);
	}
}