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

import org.junit.Test;
import org.ode4j.math.DVector3;

import java.util.Iterator;

import static org.junit.Assert.*;

public class DBodyTest {

    /**
     * DBody::addLinearVelocity.
     */
    @Test
    public void testAddLinearVelocity() {
        DWorld world = OdeHelper.createWorld();
        DBody b = OdeHelper.createBody(world);
        b.addLinearVel(1, 3, 5);
        assertTrue(b.getLinearVel().isEq(1, 3, 5, 0));

        b.addLinearVel(new DVector3(2, 3, 4));
        assertTrue(b.getLinearVel().isEq(3, 6, 9, 0));
    }

    /**
     * DBody::getGeomIterator().
     */
    @Test
    public void testGetGeomIterator() {
        DWorld world = OdeHelper.createWorld();
        DBody b = OdeHelper.createBody(world);

        DGeom g1 = OdeHelper.createBox(1, 1, 1);
        g1.setBody(b);

        Iterator<DGeom> it1 = b.getGeomIterator();
        assertEquals(g1, it1.next());
        assertFalse(it1.hasNext());

        DGeom g2 = OdeHelper.createBox(1, 1, 1);
        g2.setBody(b);
        Iterator<DGeom> it2 = b.getGeomIterator();
        // The following order may in fact change! -> Fix test to ignore ordering!
        assertEquals(g2, it2.next());
        assertEquals(g1, it2.next());
        assertFalse(it2.hasNext());
    }
}
