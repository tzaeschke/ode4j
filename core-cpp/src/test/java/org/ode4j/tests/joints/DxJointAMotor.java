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

import static org.ode4j.cpp.internal.ApiCppBody.*;

import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.tests.UnitTestPlusPlus.TestSuperClass;


//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joinst/fixed.cpp
//
//
////////////////////////////////////////////////////////////////////////////////


//SUITE (TestdxJointAMotor)
class DxJointAMotor
{

	static final double tol = 1e-5;

	static class FixtureBase extends TestSuperClass {
		DWorld world;
		DBody body;
		DAMotorJoint joint;

		FixtureBase()
		{
			world = OdeHelper.createWorld();
			body = OdeHelper.createBody(world);
			joint = OdeHelper.createAMotorJoint(world);
		}

		//~FixtureBase()
		void DESTRUCTOR()
		{
			joint.destroy();
			body.destroy();
			world.destroy();
		}
	}


	static class FixtureXUser extends FixtureBase {
		FixtureXUser()
		{
			// body only allowed to rotate around X axis
			dBodySetFiniteRotationMode(body, true);
			dBodySetFiniteRotationAxis(body, 1, 0, 0);
			joint.attach(body, null);
			dJointSetAMotorNumAxes(joint, 2);
			dJointSetAMotorAxis(joint, 0, 2, 0, 1, 0);
			dJointSetAMotorAxis(joint, 1, 2, 0, 0, 1);
			dJointSetAMotorParam(joint, dParamVel, 0);
			dJointSetAMotorParam(joint, dParamFMax, OdeConstants.dInfinity);
			dJointSetAMotorParam(joint, dParamVel2, 0);
			dJointSetAMotorParam(joint, dParamFMax2, OdeConstants.dInfinity);
		}
	}

	static class FixtureXEuler extends FixtureBase {
		FixtureXEuler()
		{
			// body only allowed to rotate around X axis
			joint.attach(null, body);
			joint.setMode(DAMotorJoint.AMotorMode.dAMotorEuler);
			joint.setAxis(0, 0, 1, 0, 0);
			joint.setAxis(2, 0, 0, 0, 1);
		}
	}

} // End of SUITE TestdxJointAMotor
