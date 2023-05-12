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
import org.ode4j.math.DQuaternion;

public class TestDQuaternion extends OdeTestCase {

	@Test public void main() {
		
	}
	
	@Test
	public void testGet(){
		DQuaternion x = new DQuaternion(1, 2, 3, 4);
		assertEquals(x.get0(), 1.);
		assertEquals(x.get1(), 2.);
		assertEquals(x.get2(), 3.);
		assertEquals(x.get3(), 4.);
		assertEquals(x.get(0), 1.);
		assertEquals(x.get(1), 2.);
		assertEquals(x.get(2), 3.);
		assertEquals(x.get(3), 4.);
	}		
		
	@Test
	public void testEqual(){
		DQuaternion x = new DQuaternion(1, 2, 3, 4);
		DQuaternion xx = new DQuaternion(1, 2, 3, 4);
		DQuaternion x1 = new DQuaternion(0, 2, 3, 4);
		DQuaternion x2 = new DQuaternion(1, 0, 3, 4);
		DQuaternion x3 = new DQuaternion(1, 2, 0, 4);
		DQuaternion x4 = new DQuaternion(1, 2, 3, 0);
		assertTrue(x.isEq(xx));
		assertFalse(x.isEq(x1));
		assertFalse(x.isEq(x2));
		assertFalse(x.isEq(x3));
		assertFalse(x.isEq(x4));
	}		
		
	@Test
	public void testSet(){
		DQuaternion x = new DQuaternion(1, 2, 3, 4);
		DQuaternion x2 = new DQuaternion(1, 2, 3, 4);
		DQuaternion y = new DQuaternion(5, 6, 7, 8);
		DQuaternion z = new DQuaternion(9, 10, 11, 12);
		x.set0(9);
		assertEquals(x.get0(), 9.);
		x.set1(10);
		assertEquals(x.get1(), 10.);
		x.set2(11);
		assertEquals(x.get2(), 11.);
		x.set3(12);
		assertEquals(x.get3(), 12.);
		assertEquals(x, z);
		
		x.set(0, 5);
		assertEquals(x.get0(), 5.);
		x.set(1, 6);
		assertEquals(x.get1(), 6.);
		x.set(2, 7);
		assertEquals(x.get2(), 7.);
		x.set(3, 8);
		assertEquals(x.get3(), 8.);
		assertEquals(x, y);

		x.set(1, 2, 3, 4);
		assertEquals(x, x2);
		
		x.set(y);
		assertEquals(x, y);

//		x.set( new double[]{ 8, 9, 11, -12} );
//		assertTrue(x.get0()==8 && x.get1()==9 && x.get2()==11 && x.get3()==-12);
//
//		x.setValues(2.5);
//		assertTrue(x.get0()==2.5 && x.get1()==2.5 && x.get2()==2.5 && x.get3()==2.5);
//
//		assertFalse(x.equals(x2));
//		assertFalse(x.equals(y));
//		assertFalse(x.equals(z));
	}		
		
	@Test
	public void testInit(){
		DQuaternion x = new DQuaternion(1, 2, 3, 4);
		DQuaternion y = new DQuaternion();
		DQuaternion z = new DQuaternion(x);
		assertTrue(x.isEq(z));
		assertFalse(x.isEq(y));
		assertEquals(y.get0(), 0.);
		assertEquals(y.get1(), 0.);
		assertEquals(y.get2(), 0.);
		assertEquals(y.get3(), 0.);

		assertEquals(z.get0(), 1.);
		assertEquals(z.get1(), 2.);
		assertEquals(z.get2(), 3.);
		assertEquals(z.get3(), 4.);
	}		
	
	@Test
	public void testAdd(){
		DQuaternion x = new DQuaternion(1, 2, 3, 4);
		DQuaternion y = new DQuaternion(4, 8, -1, -7);
		DQuaternion t = new DQuaternion();
		assertFalse(x.isEq(y));
		
		t.add(x);
		assertTrue(t.isEq(x));
		t.add(3, 6, -4, -11);
		assertTrue(t.isEq(y));

//		t.add(0, -3);
//		t.add(1, -6);
//		t.add(2, 4);
//		t.add(3, 11);
//		assertTrue(t.equals(x));

//		t.add0(3);
//		t.add1(6);
//		t.add2(-4);
//		assertTrue(t.equals(y));
	}		
	
	@Test
	public void testSum(){
		//TODO
//		dQuaternion x = new dQuaternion(1, 2, 3, 4);
//		dQuaternion y = new dQuaternion(4, 8, -1, -7);
//		dQuaternion t = new dQuaternion();
//		assertFalse(x.equals(y));
//		
//		t.add(x);
//		assertTrue(t.equals(x));
//		t.add(3, 6, -4, -11);
//		assertTrue(t.equals(y));
//
//		t.add(0, -3);
//		t.add(1, -6);
//		t.add(2, 4);
//		t.add(3, 11);
//		assertTrue(t.equals(x));
//
////		t.add0(3);
////		t.add1(6);
////		t.add2(-4);
////		assertTrue(t.equals(y));
	}		
	
	@Test
	public void testSub(){
		DQuaternion x = new DQuaternion(1, 2, 3, 4);
		DQuaternion y = new DQuaternion(4, 8, -1, -7);
		DQuaternion t = new DQuaternion();
		assertFalse(x.isEq(y));
		
		t.add(x);
		t.add(x);
//		t.sub(x);
//		assertTrue(t.equals(x));
//		t.sub(-3, -6, 4);
//		assertTrue(t.equals(y));

//		t.sub(0, 3);
//		t.sub(1, 6);
//		t.sub(2, -4);
//		assertTrue(t.isEq(x));
	}		
	
	@Test
	public void testScale(){
		DQuaternion y = new DQuaternion(4, 10, -6, -13);
		DQuaternion t = new DQuaternion();
		
		t.set(y);
		t.scale(0.5);
		assertTrue(t.isEq( new DQuaternion(2, 5, -3, -6.5) ));
	}		
	
//	@Test
//	public void testClone() {
//		dQuaternion y = new dQuaternion(4, 8, -1, -7);
//		dQuaternion t = y.clone();
//		assertTrue( y.equals(t) );
//		t.set0(1);
//		assertFalse( y.equals(t) );
//	}
	
	@Test
	public void testOther(){
//		DQuaternion x = new DQuaternion(1, 2, 3, 4);
//		DQuaternion y = new DQuaternion(4, 8, -1, -7);
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

//		try {
//			t.set(0, 0, 0, 0).normalize();
//			//assertEquals(new dQuaternion(1, 0, 0), t);
//			fail(t.toString());
//		} catch (IllegalStateException e) {
//			//Ignore
//		}
//
//		t.set(3, 4, -18, -6.5);
//		t.normalize();
//		assertEquals(new dQuaternion(0.16058631827165676, 0.21411509102887566, -0.9635179096299405, 0.1), t);
		

		t.set(3, 4, -5, -2);
		assertEquals(Math.sqrt(54), t.length());
		assertEquals(54.0, t.lengthSquared());
		
//		t.set(-3, -4, -5);
//		t.eqAbs();
//		assertEquals(new dQuaternion(3, 4, 5), t);
//		
//		t.eqDiff(x, y);
//		assertEquals(new dQuaternion(-3, -6, 4), t);
	}		
	
//	@Test
//	public void testDot(){
//		dQuaternion x = new dQuaternion(1, 2, 3, 4);
//		dQuaternion y = new dQuaternion(4, 8, -1, -7);
//		dQuaternion t = new dQuaternion();
//		
//		assertEquals( 4+16-3 , t.eqDot(x, y));
//	}		
	
//	@Test
//	public void testMul(){
	//TODO ?!?!?!?
//		dQuaternion x = new dQuaternion(1, 2, 3);
//		dQuaternion y = new dQuaternion(4, 8, -1);
//		dQuaternion t = new dQuaternion();
//		
//		dMatrix3 B = new dMatrix3(0.10, 0.11, 0.12,   1.10, 1.11, 1.12,  2.10, 2.11, 2.22);
//		dQuaternion c = new dQuaternion(-1, 2.5, -11.7);
//		
//		t.eqMul(B, c);
//		double x1 = ;
//		double x2 = ;
//		double x3 = ;
//	}		
}