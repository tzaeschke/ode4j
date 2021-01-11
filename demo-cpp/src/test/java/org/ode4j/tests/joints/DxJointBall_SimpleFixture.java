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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DWorld;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

import static org.ode4j.cpp.internal.ApiCppBody.*;
import static org.ode4j.cpp.internal.ApiCppJoint.*;
import static org.ode4j.cpp.internal.ApiCppWorld.*;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.ode.internal.Common.dSqrt;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_CLOSE;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

public class DxJointBall_SimpleFixture extends TestSuperClass {
	
	@Before
	public void DxJointBall_SimpleFixture_init()
	{
		w = dWorldCreate();
		b1 = dBodyCreate(w);
		b2 = dBodyCreate(w);
		j = dJointCreateBall(w, null);
		dJointAttach(j, b1, b2);
	}

	@After
	public void DESTRUCTOR() {

		j.destroy();
		dBodyDestroy(b1);
		dBodyDestroy(b2);
		dWorldDestroy(w);
	}

	private	DWorld w;
	private DBody b1, b2;
	private DBallJoint j;
	//  };

	// TEST_FIXTURE(SimpleFixture, testTargetDistance)
	@Test public void testTargetDistance()
	{
		dBodySetPosition(b1, -1, -2, -3);
		dBodySetPosition(b2, 3, 5, 7);
		dJointAttach(j, b1, b2); // this recomputes the deduced target distance
		CHECK_CLOSE(dJointGetBallDistance(j), dSqrt(165.0), 1e-4);

		// moving body should not change target distance
		dBodySetPosition(b1, 2,3,4);
		CHECK_CLOSE(dJointGetBallDistance(j), dSqrt(165.0), 1e-4);

		// setting target distance manually should override the deduced one
		dJointSetBallDistance(j, 6.0);
		CHECK_EQUAL(dJointGetBallDistance(j), 6.0);
	}
}