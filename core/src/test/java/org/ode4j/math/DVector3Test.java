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
package org.ode4j.math;

import org.junit.Test;

import static org.junit.Assert.*;

public class DVector3Test {

    @Test
    public void testReAdd() {
        DVector3C v1 = new DVector3(1, 2, 3);
        DVector3C v2 = v1.reAdd(2, 3, 4);
        assertEquals(v2, new DVector3(3, 5, 7));
        DVector3C v3 = v2.reAdd(2, 3, 4);
        assertEquals(v1, new DVector3(1, 2, 3));
        assertEquals(v2, new DVector3(3, 5, 7));
        assertEquals(v3, new DVector3(5, 8, 11));
    }

    @Test
    public void testToDegrees() {
        DVector3 v1 = new DVector3(Math.PI, Math.PI/2, Math.PI/4);
        v1.eqToDegrees();
        assertEquals(new DVector3(180, 90, 45), v1);
        v1.eqToRadians();
        assertEquals(new DVector3(Math.PI, Math.PI/2, Math.PI/4), v1);

        DVector3 v2 = new DVector3(Math.PI, Math.PI/2, Math.PI/4);
        DVector3 v2b = v2.eqToDegrees();
        assertSame(v2, v2b);
        DVector3 v2c = v2.eqToRadians();
        assertSame(v2, v2c);
    }
}
