package org.ode4j.tests;

import org.junit.Test;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

public class TestIssue0017_NpeInSetTorques {

	@Test
	public void test() {
		DWorld w = OdeHelper.createWorld();
		DAMotorJoint j = OdeHelper.createAMotorJoint(w);
		j.setNumAxes(3);
		j.addTorques( 1, 2, 3 );
	}
	
}
