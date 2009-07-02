package org.ode4j.tests;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import org.ode4j.tests.joints.TestJointFixed;
import org.ode4j.tests.joints.TestJointHinge;
import org.ode4j.tests.joints.TestJointSlider;
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
	TestJointFixed.class, 
	TestJointHinge.class,
	TestJointSlider.class 
})
public class AllTests2 {
	public static void main(String[] args) {
		
	}
}
