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
package org.ode4j.tests.math;

import junit.framework.TestCase;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector4;
import org.ode4j.math.DVector6;
import org.ode4j.math.DMatrix3.DVector3ColView;

public abstract class OdeTestCase extends TestCase {

	protected void assertEquals(double exp, double val) {
		//TODO use proper EPSILON!!!
		double eps = 10e-9;
		if (Math.abs(exp - val) > eps) {
			fail("Argmuments not equal: Expected=" + exp + ";  Actual=" + val);
		}
	}
	
	protected void assertEquals(DVector3 exp, DVector3 val) {
		//TODO use proper EPSILON!!!
		double eps = 10e-9;
		for (int i = 0; i < exp.dim(); i++) {
			if (Math.abs(exp.get(i)-val.get(i)) > eps) {
				fail("Argmuments not equal: Expected=" + exp + ";  Actual=" + val);
			}
		}
	}
	
	protected void assertEquals(DVector3 exp, DVector3ColView val) {
		//TODO use proper EPSILON!!!
		double eps = 10e-9;
		for (int i = 0; i < exp.dim(); i++) {
			if (Math.abs(exp.get(i)-val.get(i)) > eps) {
				fail("Argmuments not equal: Expected=" + exp + ";  Actual=" + val);
			}
		}
	}
	
	protected void assertEquals(DVector4 exp, DVector4 val) {
		//TODO use proper EPSILON!!!
		double eps = 10e-9;
		for (int i = 0; i < exp.dim(); i++) {
			if (Math.abs(exp.get(i)-val.get(i)) > eps) {
				fail("Argmuments not equal: Expected=" + exp + ";  Actual=" + val);
			}
		}
	}
	
	protected void assertEquals(DQuaternion exp, DQuaternion val) {
		//TODO use proper EPSILON!!!
		double eps = 10e-9;
		for (int i = 0; i < exp.dim(); i++) {
			if (Math.abs(exp.get(i)-val.get(i)) > eps) {
				fail("Argmuments not equal: Expected=" + exp + ";  Actual=" + val);
			}
		}
	}
	
	protected void assertEquals(DVector6 exp, DVector6 val) {
		//TODO use proper EPSILON!!!
		double eps = 10e-9;
		for (int i = 0; i < exp.dim(); i++) {
			if (Math.abs(exp.get(i)-val.get(i)) > eps) {
				fail("Argmuments not equal: Expected=" + exp + ";  Actual=" + val);
			}
		}
	}

	protected void assertEquals(DMatrix3 exp, DMatrix3 val) {
		//TODO use proper EPSILON!!!
		double eps = 10e-9;
		for (int i = 0; i < exp.dimI(); i++) {
			for (int j = 0; j < exp.dimJ(); j++) {
				if (Math.abs(exp.get(i,j)-val.get(i,j)) > eps) {
					fail("Argmuments not equal: Expected=" + exp + ";  Actual=" + val);
				}
			}
		}
	}
	
}