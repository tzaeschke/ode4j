/*************************************************************************
 *                                                                       *
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
package org.ode4j.ode;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.ode.ragdoll.DRagdoll;
import org.ode4j.ode.test.geoms.DxDefaultHumanRagdollConfig;

public class TestIssue0031_RagdollNPE {

	/**
	 * The call to ragdoll.destroy() used to cause an NPE.
	 */
	@Test
	public void testIssue31() {
		DWorld world = OdeHelper.createWorld();
		world.setGravity(0,0,-9.8);
		world.setDamping(1e-4, 1e-5);
		//    dWorldSetERP(world, 1);
		DSpace space = OdeHelper.createSimpleSpace();
		OdeHelper.createPlane( space, 0, 0, 1, 0 );

		DRagdoll ragdoll = OdeHelper.createRagdoll(world, space, new DxDefaultHumanRagdollConfig());
		ragdoll.setAngularDamping(0.1);
		DQuaternion q = new DQuaternion(1, 0, 0, 0);
		DRotation.dQFromAxisAndAngle(q, new DVector3(1, 0, 0), -0.5 * Math.PI);
		for (DRagdoll.DRagdollBody bone : ragdoll.getBoneIter()) {
			DGeom g = OdeHelper.createCapsule(space, bone.getRadius(), bone.getLength());
			DBody body = bone.getBody();
			DQuaternion qq = new DQuaternion();
			OdeMath.dQMultiply1(qq, q, body.getQuaternion());
			body.setQuaternion(qq);
			DMatrix3 R = new DMatrix3();
			OdeMath.dRfromQ(R, q);
			DVector3 v = new DVector3();
			OdeMath.dMultiply0_133(v, body.getPosition(), R);
			body.setPosition(v.get0(), v.get1(), v.get2());
			g.setBody(body);
		}

		ragdoll.destroy();
		
	}
	
	
}
