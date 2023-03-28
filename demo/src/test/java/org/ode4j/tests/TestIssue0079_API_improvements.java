/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2023 Tilmann Zaeschke     *
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
package org.ode4j.tests;

import org.junit.Test;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;

import static org.junit.Assert.assertEquals;

public class TestIssue0079_API_improvements {

    /**
     * DBody::addLinearVelocity.
     */
    @Test
    public void testIssue79_addLinearVelocity() {
        DWorld world = OdeHelper.createWorld();
        DBody b = OdeHelper.createBody(world);
        b.addLinearVel(1, 3, 5);
        assertEquals(b.getLinearVel(), new DVector3(1, 3, 5));

        b.addLinearVel(new DVector3(2, 3, 4));
        assertEquals(b.getLinearVel(), new DVector3(3, 6, 9));
    }

    /**
     * DVector3C::reAdd().
     */
    @Test
    public void testIssue79_DVector3C_reAdd() {
        DVector3C v1 = new DVector3(1, 2, 3);
        DVector3C v2 = v1.reAdd(2, 3, 4);
        assertEquals(v2, new DVector3(3, 5, 7));
        DVector3C v3 = v2.reAdd(2, 3, 4);
        assertEquals(v1, new DVector3(1, 2, 3));
        assertEquals(v2, new DVector3(3, 5, 7));
        assertEquals(v3, new DVector3(5, 8, 11));
    }

    /**
     * DQuaternion::setIdentity().
     */
    @Test
    public void testIssue79_DQuaternion_setIdentity() {
        DQuaternion q1 = new DQuaternion(1, 2, 3, 4);
        DQuaternionC q2 = DQuaternion.IDENTITY;
        assertEquals(q2, new DQuaternion(1, 0, 0, 0));
        q1.setIdentity();
        assertEquals(q1, new DQuaternion(1, 0, 0, 0));
        assertEquals(q1, DQuaternion.IDENTITY);
    }

    /**
     * DQuaternion::setIdentity().
     */
    @Test
    public void testIssue79_DQuaternion_setZero() {
        DQuaternion q1 = new DQuaternion(1, 2, 3, 4);
        DQuaternionC q2 = DQuaternion.ZERO;
        assertEquals(q2, new DQuaternion(0, 0, 0, 0));
        q1.setZero();
        assertEquals(q1, new DQuaternion(0, 0, 0, 0));
        assertEquals(q1, DQuaternion.ZERO);
    }
}
