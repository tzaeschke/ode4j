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

public class DQuaternionTest {

    private static final double eps = 1e-9;

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
    @SuppressWarnings("deprecation")
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
    @SuppressWarnings("deprecation")
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

    @Test
    public void testGet(){
        DQuaternion x = new DQuaternion(1, 2, 3, 4);
        assertEquals(1., x.get0(), 0);
        assertEquals(2., x.get1(), 0);
        assertEquals(3., x.get2(), 0);
        assertEquals(4., x.get3(), 0);
        assertEquals(1., x.get(0), 0);
        assertEquals(2., x.get(1), 0);
        assertEquals(3., x.get(2), 0);
        assertEquals(4., x.get(3), 0);
    }

    @Test
    public void testEqual(){
        DQuaternion x = new DQuaternion(1, 2, 3, 4);
        DQuaternion xx = new DQuaternion(1, 2, 3, 4);
        DQuaternion x1 = new DQuaternion(0, 2, 3, 4);
        DQuaternion x2 = new DQuaternion(1, 0, 3, 4);
        DQuaternion x3 = new DQuaternion(1, 2, 0, 4);
        DQuaternion x4 = new DQuaternion(1, 2, 3, 0);
        assertTrue(x.isEq(xx, 0));
        assertFalse(x.isEq(x1, 0));
        assertFalse(x.isEq(x2, 0));
        assertFalse(x.isEq(x3, 0));
        assertFalse(x.isEq(x4, 0));
    }

    @Test
    public void testSet(){
        DQuaternion x = new DQuaternion(1, 2, 3, 4);
        DQuaternion x2 = new DQuaternion(1, 2, 3, 4);
        DQuaternion y = new DQuaternion(5, 6, 7, 8);
        DQuaternion z = new DQuaternion(9, 10, 11, 12);
        x.set0(9);
        assertEquals(9., x.get0(), 0);
        x.set1(10);
        assertEquals(10., x.get1(), 0);
        x.set2(11);
        assertEquals(11., x.get2(), 0);
        x.set3(12);
        assertEquals(12., x.get3(), 0);
        assertEquals(x, z);

        x.set(0, 5);
        assertEquals(5., x.get0(), 0);
        x.set(1, 6);
        assertEquals(6., x.get1(), 0);
        x.set(2, 7);
        assertEquals(7., x.get2(), 0);
        x.set(3, 8);
        assertEquals(8., x.get3(), 0);
        assertEquals(x, y);

        x.set(1, 2, 3, 4);
        assertEquals(x, x2);

        x.set(y);
        assertEquals(x, y);
    }

    @Test
    public void testInit(){
        DQuaternion x = new DQuaternion(1, 2, 3, 4);
        DQuaternion y = new DQuaternion();
        DQuaternion z = new DQuaternion(x);
        assertTrue(x.isEq(z, 0));
        assertFalse(x.isEq(y, 0));
        assertEquals(0., y.get0(), 0);
        assertEquals(0., y.get1(), 0);
        assertEquals(0., y.get2(), 0);
        assertEquals(0., y.get3(), 0);

        assertEquals(1., z.get0(), 0);
        assertEquals(2., z.get1(), 0);
        assertEquals(3., z.get2(), 0);
        assertEquals(z.get3(), 4., 0);
    }

    @Test
    public void testAdd(){
        DQuaternion x = new DQuaternion(1, 2, 3, 4);
        DQuaternion y = new DQuaternion(4, 8, -1, -7);
        DQuaternion t = new DQuaternion();
        assertFalse(x.isEq(y, 0));

        t.add(x);
        assertTrue(t.isEq(x, 0));
        t.add(3, 6, -4, -11);
        assertTrue(t.isEq(y, 0));
    }

    @Test
    public void testSub(){
        DQuaternion x = new DQuaternion(1, 2, 3, 4);
        DQuaternion y = new DQuaternion(4, 8, -1, -7);
        DQuaternion t = new DQuaternion();
        assertFalse(x.isEq(y, 0));

        t.add(x);
        t.add(x);
    }

    @Test
    public void testScale(){
        DQuaternion y = new DQuaternion(4, 10, -6, -13);
        DQuaternion t = new DQuaternion();

        t.set(y);
        t.scale(0.5);
        assertTrue(t.isEq( new DQuaternion(2, 5, -3, -6.5), 0 ));
    }

    @Test
    public void testOther(){
        DQuaternion t = new DQuaternion();

        //TODO remove dSafeNormalize3()?
        try {
            t.set(0, 0, 0, 0).normalize();
            fail();
        } catch (IllegalStateException e) {
            // Good!
        }
        assertEquals(new DQuaternion(1, 0, 0, 0), t);

        t.set(3, 4, -18, -6.5);
        t.normalize();
        assertEquals(new DQuaternion(0.15166804174966758, 0.20222405566622345, -0.9100082504980056, -0.32861409045761314), t);

        t.set(3, 4, -5, -2);
        assertEquals(Math.sqrt(54), t.length(), eps);
        assertEquals(54.0, t.lengthSquared(), eps);
    }

    @Test
    public void testDot() {
        DQuaternion x = new DQuaternion(1, 2, 3, 5);
        DQuaternion y = new DQuaternion(4, 8, -1, -2);

        assertEquals(4 + 16 - 3 - 10, x.dot(y), eps);

        // * 1 if rotation1 == rotation2
        // * 0 if rotation1 and rotation2 are perpendicular
        // * -1 if rotation1 and rotation2 are opposite

        // q1 == q2
        DQuaternion q1 = x.copy().normalize();
        DQuaternion q2 = y.copy().normalize();
        assertEquals(1, q1.dot(q1), 0.0001);
        assertEquals(1, q2.dot(q2), 0.0001);

        // opposite directions
        DQuaternion q1i = q1.copy().scale(-1);
        DQuaternion q2i = q2.copy().scale(-1);
        assertEquals(-1, q1i.dot(q1), 0.0001);
        assertEquals(-1, q2i.dot(q2), 0.0001);
        assertEquals(-1, q1.dot(q1i), 0.0001);
        assertEquals(-1, q2.dot(q2i), 0.0001);

        // perpendicular
        DQuaternion x2 = new DQuaternion(1, 0, 0, 0).normalize();
        DQuaternion y2 = new DQuaternion(0, 1, 0, 0).normalize();
        assertEquals(0, x2.dot(y2), 0.0001);
        assertEquals(0, y2.dot(x2), 0.0001);
    }
}
