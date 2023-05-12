package org.ode4j.ode;

import org.junit.Test;

public class TestIssue0017_NpeInSetTorques {

	@Test
	public void test() {
		DWorld w = OdeHelper.createWorld();
		DAMotorJoint j = OdeHelper.createAMotorJoint(w);
		j.attach(OdeHelper.createBody(w), OdeHelper.createBody(w));
		j.setNumAxes(3);
		j.addTorques( 1, 2, 3 );
	}
	
}
