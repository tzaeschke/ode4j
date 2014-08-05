package org.ode4j.tests.UnitTestPlusPlus;

import org.junit.After;
import org.junit.Before;
import org.ode4j.ode.internal.OdeInit;

public class TestSuperClass {

	@Before
	public void beforeEachTest() {
		OdeInit.dInitODE();
	}
	
	@After
	public void afterEachTest() {
		OdeInit.dCloseODE();
	}
	
}
