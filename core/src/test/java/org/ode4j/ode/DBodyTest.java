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
package org.ode4j.ode;

import org.junit.Assert;
import org.junit.Test;
import org.ode4j.math.DVector3;

import static org.junit.Assert.assertEquals;

public class DBodyTest {

    /**
     * DBody::addLinearVelocity.
     */
    @Test
    public void testAddLinearVelocity() {
        DWorld world = OdeHelper.createWorld();
        DBody b = OdeHelper.createBody(world);
        b.addLinearVel(1, 3, 5);
        Assert.assertEquals(b.getLinearVel(), new DVector3(1, 3, 5));

        b.addLinearVel(new DVector3(2, 3, 4));
        assertEquals(b.getLinearVel(), new DVector3(3, 6, 9));
    }
}
