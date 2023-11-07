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
import org.ode4j.ode.OdeMath;

import static org.junit.Assert.*;

public class DVector3Test {

    private static final double eps = 1e-9;

    @Test
    public void testReAdd() {
        DVector3C v1 = new DVector3(1, 2, 3);
        DVector3C v2 = v1.reAdd(2, 3, 4);
        assertTrue(v2.isEq(new DVector3(3, 5, 7), 0));
        DVector3C v3 = v2.reAdd(2, 3, 4);
        assertTrue(v1.isEq(new DVector3(1, 2, 3), 0));
        assertTrue(v2.isEq(new DVector3(3, 5, 7), 0));
        assertTrue(v3.isEq(new DVector3(5, 8, 11), 0));
    }

    @Test
    public void testToDegrees() {
        DVector3 v1 = new DVector3(Math.PI, Math.PI/2, Math.PI/4);
        v1.eqToDegrees();
        assertTrue(new DVector3(180, 90, 45).isEq(v1, 0));
        v1.eqToRadians();
        assertTrue(new DVector3(Math.PI, Math.PI/2, Math.PI/4).isEq(v1, 0));

        DVector3 v2 = new DVector3(Math.PI, Math.PI/2, Math.PI/4);
        DVector3 v2b = v2.eqToDegrees();
        assertSame(v2, v2b);
        DVector3 v2c = v2.eqToRadians();
        assertSame(v2, v2c);
    }

    @Test
    public void testFromDoubleArray(){
        DVector3 x = new DVector3(10, 20, 30);
        double[] a = {10, 20, 30};
        DVector3 y = DVector3.fromDoubleArray(a);
        assertTrue(x.isEq(y, 0));
    }

    @Test
    public void testFromFloatArray(){
        DVector3 x = new DVector3(10, 20, 30);
        float[] a = {10, 20, 30};
        DVector3 y = DVector3.fromFloatArray(a);
        assertTrue(x.isEq(y, 0));
    }

    @Test
    public void testToDoubleArray(){
        DVector3 x = new DVector3(10, 20, 30);
        double[] a = x.toDoubleArray();
        assertEquals(3, a.length);
        assertEquals(10, a[0], 0);
        assertEquals(20, a[1], 0);
        assertEquals(30, a[2], 0);
    }

    @Test
    public void testToFloatArray(){
        DVector3 x = new DVector3(10, 20, 30);
        float[] a = x.toFloatArray();
        assertEquals(3, a.length);
        assertEquals(10, a[0], 0);
        assertEquals(20, a[1], 0);
        assertEquals(30, a[2], 0);
    }

    @Test
    public void testGet(){
        DVector3 x = new DVector3(1, 2, 3);
        assertEquals(1., x.get0(), 0);
        assertEquals(2., x.get1(), 0);
        assertEquals(3., x.get2(), 0);
        assertEquals(1., x.get(0), 0);
        assertEquals(2., x.get(1), 0);
        assertEquals(3., x.get(2), 0);
    }

    @Test
    public void testEqual(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 xx = new DVector3(1, 2, 3);
        DVector3 x1 = new DVector3(0, 2, 3);
        DVector3 x2 = new DVector3(1, 0, 3);
        DVector3 x3 = new DVector3(1, 2, 0);
        assertTrue(x.isEq(xx, 0));
        assertFalse(x.isEq(x1, 0));
        assertFalse(x.isEq(x2, 0));
        assertFalse(x.isEq(x3, 0));
    }

    @Test
    public void testSet(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 x2 = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(4, 5, 6);
        DVector3 z = new DVector3(7, 8, 9);
        x.set0(7);
        assertEquals(7., x.get0(), 0);
        x.set1(8);
        assertEquals(8., x.get1(), 0);
        x.set2(9);
        assertEquals(9., x.get2(), 0);
        assertTrue(x.isEq(z, 0));

        x.set(0, 4);
        assertEquals(4., x.get0(), 0);
        x.set(1, 5);
        assertEquals(5., x.get1(), 0);
        x.set(2, 6);
        assertEquals(6., x.get2(), 0);
        assertTrue(x.isEq(y, 0));

        x.set(1, 2, 3);
        assertTrue(x.isEq(x2, 0));

        x.set(y);
        assertTrue(x.isEq(y, 0));

        //This ",0" should be removed at some point (?)
        x.set( new double[]{ 8, 9, 11, 0} );
        assertTrue(x.get0()==8 && x.get1()==9 && x.get2()==11);

//		x.setValues(2.5);
//		assertTrue(x.get0()==2.5 && x.get1()==2.5 && x.get2()==2.5);

        assertFalse(x.isEq(x2, 0));
        assertFalse(x.isEq(y, 0));
        assertFalse(x.isEq(z, 0));
    }

    @Test
    public void testInit(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3();
        DVector3 z = new DVector3(x);
        assertTrue(x.isEq(z, 0));
        assertFalse(x.isEq(y, 0));
        assertEquals(0., y.get0(), 0);
        assertEquals(0., y.get1(), 0);
        assertEquals(0., y.get2(), 0);

        assertEquals(1., z.get0(), 0);
        assertEquals(2., z.get1(), 0);
        assertEquals(3., z.get2(), 0);
    }

    @Test
    public void testAdd(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(4, 8, -1);
        DVector3 t = new DVector3();
        assertFalse(x.isEq(y, 0));

        t.add(x);
        assertTrue(t.isEq(x, 0));
        t.add(3, 6, -4);
        assertTrue(t.isEq(y, 0));

        t.add(0, -3);
        t.add(1, -6);
        t.add(2, 4);
        assertTrue(t.isEq(x, 0));

        t.add0(3);
        t.add1(6);
        t.add2(-4);
        assertTrue(t.isEq(y, 0));
    }

    @Test
    public void testAddScale(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(1.5, 3, 4.5);
        DVector3 t = new DVector3();
        assertFalse(x.isEq(y, 0));

        t.addScaled(x, 1);
        assertTrue(t.isEq(x, 0));

        t.setZero();
        t.addScaled(x, -5);
        t.addScaled(x,  6);
        assertTrue(t.isEq(x, 0));
        t.addScaled(x,  0);
        assertTrue(t.isEq(x, 0));

        t.setZero();
        t.addScaled(x, 1.5);
        assertTrue(t.isEq(y, 0));
    }

    @Test
    public void testCross1(){
        DVector3C x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(1.5, 3, 2);

        DVector3 t = new DVector3(x).cross(y);
        DVector3 t2 = new DVector3();
        OdeMath.dCalcVectorCross3(t2, x, y);
        assertTrue(t2.isEq(t, 0));
    }

    @Test
    public void testCross2(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(1.5, 3, 2);
        DVector3 t = new DVector3();
        assertFalse(x.isEq(y, 0));


        t.eqCross(x, y);
        DVector3 t2 = new DVector3();
        OdeMath.dCalcVectorCross3(t2, x, y);
        assertTrue(t2.isEq(t, 0));
    }

    @Test
    public void testSub(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(4, 8, -1);
        DVector3 t = new DVector3();
        assertFalse(x.isEq(y, 0));

        t.add(x);
        t.add(x);
        t.sub(x);
        assertTrue(t.isEq(x, 0));
        t.sub(-3, -6, 4);
        assertTrue(t.isEq(y, 0));
    }

    @Test
    public void testScale(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(4, 10, -6);
        DVector3 t = new DVector3();

        t.set(x);
        t.scale(4, 5, -2);
        assertTrue(t.isEq(y, 0));
        t.scale(0.5);
        assertTrue(t.isEq( new DVector3(2, 5, -3), 0 ));
    }

    @Test
    public void testCopy() {
        DVector3 y = new DVector3(4, 8, -1);
        DVector3 t = y.copy();
        assertTrue( y.isEq(t, 0) );
        t.set0(1);
        assertFalse( y.isEq(t, 0) );
    }

    @Test
    public void testOther(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(4, 8, -1);
        DVector3 t = new DVector3();

        try {
            t.set(0, 0, 0).normalize();
            fail();
        } catch (RuntimeException e) {
            //Good!
        }
        t.set(0, 0, 0).safeNormalize();
        assertTrue(new DVector3(0, 0, 0).isEq(t, 0));

        t.set(3, 4, -18);
        t.normalize();
        assertTrue(new DVector3(0.16058631827165676, 0.21411509102887566, -0.9635179096299405).isEq(t, 0));

        try {
            t.set(0, 0, 0).normalize();
            assertTrue(new DVector3(1, 0, 0).isEq(t, 0));
            fail(t.toString());
        } catch (RuntimeException e) {
            //Ignore
        }

        t.set(3, 4, -18);
        t.normalize();
        assertTrue(new DVector3(0.16058631827165676, 0.21411509102887566, -0.9635179096299405).isEq(t, 0));


        t.set(3, 4, -5);
        assertEquals(Math.sqrt(50), t.length(), eps);
        assertEquals(50.0, t.lengthSquared(), eps);

        t.set(-3, -4, -5);
        t.eqAbs();
        assertTrue(new DVector3(3, 4, 5).isEq(t, 0));

        t.eqDiff(x, y);
        assertTrue(new DVector3(-3, -6, 4).isEq(t, 0));
    }

    @Test
    public void testDot(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(4, 8, -1);

        assertEquals( 4+16-3 , x.dot(y), eps);
    }

    @Test
    public void testSums(){
        DVector3 x = new DVector3(1, 2, 3);
        DVector3 y = new DVector3(4, 8, -1);
        DVector3 t = new DVector3();


        t.add(x);
        t.add(x);
        t.sub(x);
        assertTrue(t.isEq(x, 0));
        t.sub(-3, -6, 4);
        assertTrue(t.isEq(y, 0));
    }


    @Test
    public void testDots() {
        DVector3 x = new DVector3(21, 22, 23);
        DVector3 y = new DVector3(31, 32, 33);
        DMatrix3 m = new DMatrix3(11, 12, 13, 14, 15, 16, 17, 18, 19);

        double d, ex;

        //check dot
        d = x.dot(y);
        ex = 21*31 + 22*32 + 23*33;
        assertEquals(ex, d, eps);

        //check dotCol
        d = x.dotCol(m, 0);
        ex = 21*11 + 22*14 + 23*17;
        assertEquals(ex, d, eps);

        d = x.dotCol(m, 1);
        ex = 21*12 + 22*15 + 23*18;
        assertEquals(ex, d, eps);

        d = x.dotCol(m, 2);
        ex = 21*13 + 22*16 + 23*19;
        assertEquals(ex, d, eps);

        //check illegal arguments
        try {
            x.dotCol(m, -1);
            fail();
        } catch (IllegalArgumentException e) {
            //good
        }
        try {
            x.dotCol(m, 3);
            fail();
        } catch (IllegalArgumentException e) {
            //good
        }

        //Check col/row views
        d = x.dot(m.viewCol(0));
        ex = 21*11 + 22*14 + 23*17;
        assertEquals(ex, d, eps);

        d = x.dot(m.viewRowT(0));
        ex = 21*11 + 22*12 + 23*13;
        assertEquals(ex, d, eps);
    }
}
