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
public class TestIssue0133_LcpInternalError {

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

    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        DBody b1 = o1.getBody();
        DBody b2 = o2.getBody();
        if (b1 != null && b2 != null && OdeHelper.areConnected(b1, b2)) return;

        int n = OdeHelper.collide(o1, o2, OdeConstants.dContactBounce, contacts.getGeomBuffer());
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                DContact contact = contacts.get(i);
                DJoint contactJoint = OdeHelper.createContactJoint(world, contactGroup, contact);
                contactJoint.attach(o1.getBody(), o2.getBody());
            }
        }
    }

    /**
     * This crashed due to a bug in DLCP.solve1() with: <br>
     * java.lang.RuntimeException: #1: assertion failed
     * at org.ode4j/org.ode4j.ode.internal.ErrorHdl.dDebug(ErrorHdl.java:146)
     * at org.ode4j/org.ode4j.ode.internal.ErrorHandler.dDebug(ErrorHandler.java:101)
     * at org.ode4j/org.ode4j.ode.internal.Common.dIASSERT(Common.java:128)
     * at org.ode4j/org.ode4j.ode.internal.FastLSolve.solveL1Straight(FastLSolve.java:49)
     * at org.ode4j/org.ode4j.ode.internal.DLCP.solve1(DLCP.java:935)
     * at org.ode4j/org.ode4j.ode.internal.DLCP.solve1(DLCP.java:545)
     */
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
        ballBody.setPosition(0, 0, ballRadius * 3);
        OdeHelper.createPlane(space, 0, 0, 1, 0);

        for (int i = 0; i < 10000; i++) {
            OdeHelper.spaceCollide(space, null, (data, o1, o2) -> nearCallback(data, o1, o2));
            world.step(0.01);
            contactGroup.empty();
        }
    }
}
