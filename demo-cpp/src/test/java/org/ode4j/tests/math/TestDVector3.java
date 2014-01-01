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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests.math;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.OdeMath;

public class TestDVector3 extends OdeTestCase {

	@Test public void main() {
		
	}
	
	@Test
	public void testGet(){
		DVector3 x = new DVector3(1, 2, 3);
		assertEquals(x.get0(), 1.);
		assertEquals(x.get1(), 2.);
		assertEquals(x.get2(), 3.);
		assertEquals(x.get(0), 1.);
		assertEquals(x.get(1), 2.);
		assertEquals(x.get(2), 3.);
	}		
		
	@Test
	public void testEqual(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 xx = new DVector3(1, 2, 3);
		DVector3 x1 = new DVector3(0, 2, 3);
		DVector3 x2 = new DVector3(1, 0, 3);
		DVector3 x3 = new DVector3(1, 2, 0);
		assertTrue(x.isEq(xx));
		assertFalse(x.isEq(x1));
		assertFalse(x.isEq(x2));
		assertFalse(x.isEq(x3));
	}		
		
	@Test
	public void testSet(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 x2 = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(4, 5, 6);
		DVector3 z = new DVector3(7, 8, 9);
		x.set0(7);
		assertEquals(x.get0(), 7.);
		x.set1(8);
		assertEquals(x.get1(), 8.);
		x.set2(9);
		assertEquals(x.get2(), 9.);
		assertTrue(x.isEq(z));
		
		x.set(0, 4);
		assertEquals(x.get0(), 4.);
		x.set(1, 5);
		assertEquals(x.get1(), 5.);
		x.set(2, 6);
		assertEquals(x.get2(), 6.);
		assertTrue(x.isEq(y));

		x.set(1, 2, 3);
		assertTrue(x.isEq(x2));
		
		x.set(y);
		assertTrue(x.isEq(y));

		//This ",0" should be removed at some point (?)
		x.set( new double[]{ 8, 9, 11, 0} );
		assertTrue(x.get0()==8 && x.get1()==9 && x.get2()==11);

//		x.setValues(2.5);
//		assertTrue(x.get0()==2.5 && x.get1()==2.5 && x.get2()==2.5);

		assertFalse(x.isEq(x2));
		assertFalse(x.isEq(y));
		assertFalse(x.isEq(z));
	}		
		
	@Test
	public void testInit(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3();
		DVector3 z = new DVector3(x);
		assertTrue(x.isEq(z));
		assertFalse(x.isEq(y));
		assertEquals(y.get0(), 0.);
		assertEquals(y.get1(), 0.);
		assertEquals(y.get2(), 0.);

		assertEquals(z.get0(), 1.);
		assertEquals(z.get1(), 2.);
		assertEquals(z.get2(), 3.);
	}		
	
	@Test
	public void testAdd(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(4, 8, -1);
		DVector3 t = new DVector3();
		assertFalse(x.isEq(y));
		
		t.add(x);
		assertTrue(t.isEq(x));
		t.add(3, 6, -4);
		assertTrue(t.isEq(y));

		t.add(0, -3);
		t.add(1, -6);
		t.add(2, 4);
		assertTrue(t.isEq(x));

		t.add0(3);
		t.add1(6);
		t.add2(-4);
		assertTrue(t.isEq(y));
	}		
	
	@Test
	public void testAddScale(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(1.5, 3, 4.5);
		DVector3 t = new DVector3();
		assertFalse(x.isEq(y));
		
		t.addScaled(x, 1);
		assertTrue(t.isEq(x));
		
		t.setZero();
		t.addScaled(x, -5);
		t.addScaled(x,  6);
		assertTrue(t.isEq(x));
		t.addScaled(x,  0);
		assertTrue(t.isEq(x));

		t.setZero();
		t.addScaled(x, 1.5);
		assertTrue(t.isEq(y));
	}		
	
	@Test
	public void testCross(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(1.5, 3, 4.5);
		DVector3 t = new DVector3();
		assertFalse(x.isEq(y));
		
		
		t.eqCross(x, y);
		DVector3 t2 = new DVector3();
		OdeMath.dCalcVectorCross3(t2, x, y);
		assertEquals(t2, t);
	}		
	
	@Test
	public void testSum(){
		//TODO
//		dVector3 x = new dVector3(1, 2, 3);
//		dVector3 y = new dVector3(4, 8, -1);
//		dVector3 t = new dVector3();
//		assertFalse(x.isEq(y));
//		
//		t.add(x);
//		assertTrue(t.isEq(x));
//		t.add(3, 6, -4);
//		assertTrue(t.isEq(y));
//
//		t.add(0, -3);
//		t.add(1, -6);
//		t.add(2, 4);
//		assertTrue(t.isEq(x));
//
//		t.add0(3);
//		t.add1(6);
//		t.add2(-4);
//		assertTrue(t.isEq(y));
	}		
	
	@Test
	public void testSub(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(4, 8, -1);
		DVector3 t = new DVector3();
		assertFalse(x.isEq(y));
		
		t.add(x);
		t.add(x);
		t.sub(x);
		assertTrue(t.isEq(x));
		t.sub(-3, -6, 4);
		assertTrue(t.isEq(y));

//		t.sub(0, 3);
//		t.sub(1, 6);
//		t.sub(2, -4);
//		assertTrue(t.isEq(x));
	}		
	
	@Test
	public void testScale(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(4, 10, -6);
		DVector3 t = new DVector3();
		
		t.set(x);
		t.scale(4, 5, -2);
		assertTrue(t.isEq(y));
		t.scale(0.5);
		assertTrue(t.isEq( new DVector3(2, 5, -3) ));

//		t.sub(0, 3);
//		t.sub(1, 6);
//		t.sub(2, -4);
//		assertTrue(t.isEq(x));
	}		
	
	@Test
	public void testClone() {
		DVector3 y = new DVector3(4, 8, -1);
		DVector3 t = y.clone();
		assertTrue( y.isEq(t) );
		t.set0(1);
		assertFalse( y.isEq(t) );
	}
	
	@Test
	public void testOther(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(4, 8, -1);
		DVector3 t = new DVector3();

		try {
			t.set(0, 0, 0).normalize();
			fail();
		} catch (IllegalStateException e) {
			//Good!
		}
		assertEquals(new DVector3(1, 0, 0), t);

		t.set(3, 4, -18);
		t.normalize();
		assertEquals(new DVector3(0.16058631827165676, 0.21411509102887566, -0.9635179096299405), t);

		try {
			t.set(0, 0, 0).normalize();
			//assertEquals(new dVector3(1, 0, 0), t);
			fail(t.toString());
		} catch (IllegalStateException e) {
			//Ignore
		}

		t.set(3, 4, -18);
		t.normalize();
		assertEquals(new DVector3(0.16058631827165676, 0.21411509102887566, -0.9635179096299405), t);
		

		t.set(3, 4, -5);
		assertEquals(Math.sqrt(50), t.length());
		assertEquals(50.0, t.lengthSquared());
		
		t.set(-3, -4, -5);
		t.eqAbs();
		assertEquals(new DVector3(3, 4, 5), t);
		
		t.eqDiff(x, y);
		assertEquals(new DVector3(-3, -6, 4), t);
	}		
	
	@Test
	public void testDot(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(4, 8, -1);
		
		assertEquals( 4+16-3 , x.dot(y));
	}		
	
//	@Test
//	public void testMul(){
	//TODO ?!?!?!?
//		dVector3 x = new dVector3(1, 2, 3);
//		dVector3 y = new dVector3(4, 8, -1);
//		dVector3 t = new dVector3();
//		
//		dMatrix3 B = new dMatrix3(0.10, 0.11, 0.12,   1.10, 1.11, 1.12,  2.10, 2.11, 2.22);
//		dVector3 c = new dVector3(-1, 2.5, -11.7);
//		
//		t.eqMul(B, c);
//		double x1 = ;
//		double x2 = ;
//		double x3 = ;
//	}		
	
	@Test
	public void testSums(){
		DVector3 x = new DVector3(1, 2, 3);
		DVector3 y = new DVector3(4, 8, -1);
		DVector3 t = new DVector3();
		
		
		t.add(x);
		t.add(x);
		t.sub(x);
		assertTrue(t.isEq(x));
		t.sub(-3, -6, 4);
		assertTrue(t.isEq(y));
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
		assertEquals(ex, d);
		
		//check dotCol
		d = x.dotCol(m, 0);
		ex = 21*11 + 22*14 + 23*17;
		assertEquals(ex, d);
		
		d = x.dotCol(m, 1);
		ex = 21*12 + 22*15 + 23*18;
		assertEquals(ex, d);
		
		d = x.dotCol(m, 2);
		ex = 21*13 + 22*16 + 23*19;
		assertEquals(ex, d);

		//check illegal arguments
		try {
			d = x.dotCol(m, -1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = x.dotCol(m, 3);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		
		
		//Check col/row views
		d = x.dot(m.viewCol(0));
		ex = 21*11 + 22*14 + 23*17;
		assertEquals(ex, d);
		
		d = x.dot(m.viewRowT(0));
		ex = 21*11 + 22*12 + 23*13;
		assertEquals(ex, d);
	}
}