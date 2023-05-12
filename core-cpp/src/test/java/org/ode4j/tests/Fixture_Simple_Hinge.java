/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zäschke      *
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
package org.ode4j.tests;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateHinge;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.CHECK_EQUAL;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;

public class Fixture_Simple_Hinge extends TestSuperClass
{
	@Before
	public void Fixture_Simple_Hinge_init ()
	{
		wId = dWorldCreate();

		bId1 = dBodyCreate(wId);
		dBodySetPosition(bId1, 0, -1, 0);

		bId2 = dBodyCreate(wId);
		dBodySetPosition(bId2, 0, 1, 0);


		jId = dJointCreateHinge(wId, null);

		dJointAttach(jId, bId1, bId2);
	}

	//		        ~Fixture_Simple_Hinge()
	@After
	public void DESTRUCTOR() {
		dWorldDestroy(wId);
	}

	private DHingeJoint jId;

	private static DWorld wId;

	private DBody bId1;
	private DBody bId2;
	//};

	// Test that it is possible to have joint without a body
	//TEST_FIXTURE(Fixture_Simple_Hinge, 
	@Test public void test_dJointAttach()
	{
		boolean only_body1_OK = true;
		try {
			dJointAttach(jId, bId1, null);
			dWorldStep (wId, 1);
		}
		catch (Throwable t) {
			//OK
			t.printStackTrace();
			only_body1_OK = false;
		}
		CHECK_EQUAL(true, only_body1_OK);

		boolean only_body2_OK = true;
		try {
			dJointAttach(jId, null, bId2);
			dWorldStep (wId, 1);
		}
		catch (Throwable t) {
			//OK
			only_body2_OK = false;
		}
		CHECK_EQUAL(true, only_body2_OK);

		boolean no_body_OK = true;
		try {
			dJointAttach(jId, null, null);
			dWorldStep (wId, 1);
		}
		catch (Throwable t) {
			//OK
			no_body_OK = false;
		}
		CHECK_EQUAL(true, no_body_OK);
	}

}