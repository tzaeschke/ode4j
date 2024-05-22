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

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Issue #133: ODE INTERNAL ERROR in FastLSolve.solveL1Straight().
 */
public class TestIssue0139_MultiDestroy {

    private final DContactBuffer contacts = new DContactBuffer(4);
    private DWorld world;
    private DJointGroup contactGroup;

    @Before
    public void beforeTest() {
        OdeHelper.initODE2(0);
    }

    @After
    public void afterTest() {
        OdeHelper.closeODE();
    }

    @Test
    public void test() {
        double ballRadius = 5.0;
        double ballMass = 23.0;
        contactGroup = OdeHelper.createJointGroup();
        world = OdeHelper.createWorld();
        world.setGravity(0, 0, -9.81);
        DSpace space = OdeHelper.createSimpleSpace();
        DBody ballBody = OdeHelper.createBody(world);
        DGeom ballGeom = OdeHelper.createSphere(space, ballRadius);
        ballGeom.setBody(ballBody);
        DMass m = OdeHelper.createMass();
        m.setSphereTotal(ballMass, ballRadius);
        ballBody.setMass(m);
        DPlane plane = OdeHelper.createPlane(space, 0, 0, 1, 0);

        plane.destroy();
        space.destroy();
        world.destroy();
        ballBody.destroy();
        ballGeom.destroy();

        // Test duplicate calls to destroy()
        plane.destroy();
        space.destroy();
        world.destroy();
        ballBody.destroy();
        ballGeom.destroy();
    }
}
