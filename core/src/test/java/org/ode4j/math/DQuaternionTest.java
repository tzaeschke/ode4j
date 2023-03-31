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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DQuaternionTest {

    /**
     * DQuaternion::setIdentity().
     */
    @Test
    public void testSetIdentity() {
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
    public void testSetZero() {
        DQuaternion q1 = new DQuaternion(1, 2, 3, 4);
        DQuaternionC q2 = DQuaternion.ZERO;
        assertEquals(q2, new DQuaternion(0, 0, 0, 0));
        q1.setZero();
        assertEquals(q1, new DQuaternion(0, 0, 0, 0));
        assertEquals(q1, DQuaternion.ZERO);
        assertTrue(q1.isZero());
    }

    /**
     * DQuaternion::toEuler().
     * DQuaternion::fromEuler().
     */
    @Test
    public void testToEuler() {
        DQuaternionC q1 = new DQuaternion(1, 2, 3, 4);
        DVector3 v1 = q1.toEuler();
        DQuaternion q1b = DQuaternion.fromEuler(v1);

        DQuaternion q1n = new DQuaternion(q1);
        q1n.normalize();
        for (int i = 0; i < DQuaternion.LEN; ++i) {
            assertEquals(q1n.get(i), q1b.get(i), 0.0000000001);
        }
    }

    /**
     * DQuaternion::toEuler().
     * DQuaternion::fromEuler().
     */
    @Test
    public void testToEulerDegrees() {
        DQuaternionC q1 = new DQuaternion(1, 2, 3, 4);
        DVector3 v1 = q1.toEulerDegrees();
        DQuaternion q1b = DQuaternion.fromEulerDegrees(v1);

        DQuaternion q1n = new DQuaternion(q1);
        q1n.normalize();
        for (int i = 0; i < DQuaternion.LEN; ++i) {
            assertEquals(q1n.get(i), q1b.get(i), 0.0000000001);
        }

        // compare with radians
        DVector3 vD = new DVector3(30, 60, 90);
        DVector3 vR = new DVector3(Math.toRadians(30), Math.toRadians(60), Math.toRadians(90));
        DQuaternionC qD = DQuaternion.fromEulerDegrees(vD);
        DQuaternionC qR = DQuaternion.fromEuler(vR);
        for (int i = 0; i < DQuaternion.LEN; ++i) {
            assertEquals(qD.get(i), qR.get(i), 0.0000000001);
        }
    }

    /**
     * DQuaternion::invert().
     */
    @Test
    public void testInverse() {
        DQuaternionC q1 = new DQuaternion(1, 1, 1, 1);
        DQuaternion q1i = new DQuaternion(q1);
        q1i.eqInverse();
        assertEquals(new DQuaternion(0.25, -0.25, -0.25, -0.25), q1i);

        DQuaternionC q2 = new DQuaternion(1, 2, 3, 4);
        DQuaternion q2i = new DQuaternion(q2);
        q2i.eqInverse();
        q2i.eqInverse();
        for (int i = 0; i < DQuaternion.LEN; ++i) {
            assertEquals(q2.get(i), q2i.get(i), 0.0000000001);
        }
    }

    /**
     * DQuaternion::reInverse().
     */
    @Test
    public void testReInverse() {
        DQuaternionC q1 = new DQuaternion(1, 1, 1, 1);
        DQuaternion q1i = q1.reInverse();
        assertEquals(new DQuaternion(0.25, -0.25, -0.25, -0.25), q1i);
        assertEquals(new DQuaternion(1, 1, 1, 1), q1);

        DQuaternion q2 = new DQuaternion(1, 2, 3, 4);
        DQuaternion q2i = q2.reInverse();
        DQuaternion q2ii = q2i.reInverse();
        for (int i = 0; i < DQuaternion.LEN; ++i) {
            assertEquals(q2.get(i), q2ii.get(i), 0.0000000001);
        }
    }
}
