/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import org.ode4j.tests.bugs.BugsTest;
import org.ode4j.tests.joints.TestJointBall;
import org.ode4j.tests.joints.TestJointFixed;
import org.ode4j.tests.joints.TestJointHinge;
import org.ode4j.tests.joints.TestJointHinge2;
import org.ode4j.tests.joints.TestJointPR;
import org.ode4j.tests.joints.TestJointPU;
import org.ode4j.tests.joints.TestJointPiston;
import org.ode4j.tests.joints.TestJointSlider;
import org.ode4j.tests.joints.TestJointUniversal;
import org.ode4j.tests.math.OdeMathTZ;
import org.ode4j.tests.math.TestDMatrix3;
import org.ode4j.tests.math.TestDQuaternion;
import org.ode4j.tests.math.TestDVector3;
import org.ode4j.tests.math.TestDVector6;

@RunWith(Suite.class)
@Suite.SuiteClasses( { 
	TestOdeMath.class, 
	OdeMathTZ.class,
	TestDVector3.class,
	TestDVector6.class,
	TestDQuaternion.class,
	TestDMatrix3.class,
	CollisionTest.class,
	JointTest.class, 
	TestJointBall.class,
	TestJointFixed.class, 
	TestJointHinge.class,
	TestJointHinge2.class,
	TestJointPiston.class,
	TestJointPR.class,
	TestJointPU.class,
	TestJointSlider.class,
	TestJointUniversal.class,
	BugsTest.class
})
public class AllTests {
	public static void main(String[] args) {
		
	}
}
