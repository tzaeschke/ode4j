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
import org.ode4j.math.DVector6;

public class TestDVector6 extends OdeTestCase {

	@Test public void main() {
		
	}
	
	@Test
	public void testGet(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		assertEquals(x.get0(), 1.);
		assertEquals(x.get1(), 2.);
		assertEquals(x.get2(), 3.);
		assertEquals(x.get3(), 4.);
		assertEquals(x.get4(), 5.);
		assertEquals(x.get5(), 6.);
		assertEquals(x.get(0), 1.);
		assertEquals(x.get(1), 2.);
		assertEquals(x.get(2), 3.);
		assertEquals(x.get(3), 4.);
		assertEquals(x.get(4), 5.);
		assertEquals(x.get(5), 6.);
	}		
		
	@Test
	public void testEqual(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 xx = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 x1 = new DVector6(0, 2, 3, 4, 5, 6);
		DVector6 x2 = new DVector6(1, 0, 3, 4, 5, 6);
		DVector6 x3 = new DVector6(1, 2, 0, 4, 5, 6);
		DVector6 x4 = new DVector6(1, 2, 3, 0, 5, 6);
		DVector6 x5 = new DVector6(1, 2, 3, 4, 0, 6);
		DVector6 x6 = new DVector6(1, 2, 3, 4, 5, 0);
		assertTrue(x.isEq(xx));
		assertFalse(x.isEq(x1));
		assertFalse(x.isEq(x2));
		assertFalse(x.isEq(x3));
		assertFalse(x.isEq(x4));
		assertFalse(x.isEq(x5));
		assertFalse(x.isEq(x6));
	}		
		
	@Test
	public void testSet(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 x2 = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6(7, 8, 9, 10, 11, 12);
		DVector6 z = new DVector6(13, 14, 15, 16, 17, 18);
//		x.set0(9);
//		assertEquals(x.get0(), 9.);
//		x.set1(10);
//		assertEquals(x.get1(), 10.);
//		x.set2(11);
//		assertEquals(x.get2(), 11.);
//		x.set3(12);
//		assertEquals(x.get3(), 12.);
//		assertEquals(x, z);
		
		x.set(0, 7);
//		assertEquals(x.get0(), 5.);
		x.set(1, 8);
//		assertEquals(x.get1(), 6.);
		x.set(2, 9);
//		assertEquals(x.get2(), 7.);
		x.set(3, 10);
//		assertEquals(x.get3(), 8.);
		x.set(4, 11);
//		assertEquals(x.get3(), 8.);
		x.set(5, 12);
//		assertEquals(x.get3(), 8.);
		assertEquals(x, y);

		x.set(1, 2, 3, 4, 5, 6);
		assertEquals(x, x2);
		
		x.set(y);
		assertEquals(x, y);

		//This ",0" should be removed at some point (?)
		x.set( new double[]{ 8, 9, 11, 7, 6, 5} );
		assertTrue(x.get0()==8 && x.get1()==9 && x.get2()==11 && x.get3()==7 && x.get4()==6 && x.get5()==5);

//		x.setValues(2.5);
//		assertTrue(x.get0()==2.5 && x.get1()==2.5 && x.get2()==2.5 && x.get3()==2.5 && x.get4()==2.5 && x.get5()==2.5);

		assertFalse(x.isEq(x2));
		assertFalse(x.isEq(y));
		assertFalse(x.isEq(z));
	}		
		
	@Test
	public void testInit(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6();
		DVector6 z = new DVector6(x);
		assertTrue(x.isEq(z));
		assertFalse(x.isEq(y));
		assertEquals(y.get0(), 0.);
		assertEquals(y.get1(), 0.);
		assertEquals(y.get2(), 0.);
		assertEquals(y.get3(), 0.);
		assertEquals(y.get4(), 0.);
		assertEquals(y.get5(), 0.);

		assertEquals(z.get0(), 1.);
		assertEquals(z.get1(), 2.);
		assertEquals(z.get2(), 3.);
		assertEquals(z.get3(), 4.);
		assertEquals(z.get4(), 5.);
		assertEquals(z.get5(), 6.);
	}		
	
	@Test
	public void testAdd(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6(4, 8, -1, -7, 17, 22);
		DVector6 t = new DVector6();
		assertFalse(x.isEq(y));
		
//		t.add(x);
//		assertTrue(t.equals(x));
//		t.add(3, 6, -4, -11);
//		assertTrue(t.equals(y));

		t.set(y);
		t.add(0, -3);
		t.add(1, -6);
		t.add(2, 4);
		t.add(3, 11);
		t.add(4, -12);
		t.add(5, -16);
		assertTrue(t.isEq(x));

//		t.add0(3);
//		t.add1(6);
//		t.add2(-4);
//		assertTrue(t.equals(y));
	}		
	
	@Test
	public void testSub(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6(4, 8, -1, -7, 17, 22);
		//DVector6 t = new DVector6();
		assertFalse(x.isEq(y));
		
//		t.add(x);
//		t.add(x);
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
//		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
//		DVector6 y = new DVector6(4, 10, -6, -13, 7.3, -2.2);
//		DVector6 t = new DVector6();
		
//		t.set(x);
//		t.scale(4, 5, -2);
//		assertTrue(t.equals(y));
//		t.set(y);
//		t.scale(0.5);
//		assertTrue(t.equals( new dVector6(2, 5, -3, -6.5) ));

//		t.sub(0, 3);
//		t.sub(1, 6);
//		t.sub(2, -4);
//		assertTrue(t.isEq(x));
	}		
	
//	@Test
//	public void testClone() {
//		dVector6 y = new dVector6(4, 8, -1, -7, 17, 22);
//		dVector6 t = y.clone();
//		assertTrue( y.equals(t) );
//		t.set0(1);
//		assertFalse( y.equals(t) );
//	}
	
	@Test
	public void testOther(){
//		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
//		DVector6 y = new DVector6(4, 8, -1, -7, 17, 22);
		DVector6 t = new DVector6();

		try {
			t.set(0, 0, 0, 0, 0, 0).normalize();
			fail();
		} catch (IllegalStateException e) {
			//Good!
		}
		assertEquals(new DVector6(1, 0, 0, 0, 0, 0), t);

		t.set(3, 4, -18, -6.5, 3.33, 2.77);
		t.normalize();
		assertEquals(new DVector6(0.1481573074929876, 0.19754307665731682, 
				-0.8889438449579258, -0.32100749956813984, 
				0.16445461131721625, 0.1367985805851919), t);

//		try {
//			t.set(0, 0, 0, 0).normalize();
//			//assertEquals(new dVector6(1, 0, 0), t);
//			fail(t.toString());
//		} catch (IllegalStateException e) {
//			//Ignore
//		}
//
//		t.set(3, 4, -18, -6.5);
//		t.normalize();
//		assertEquals(new dVector6(0.16058631827165676, 0.21411509102887566, -0.9635179096299405, 0.1), t);
		

		t.set(3, 4, -5, -2, 7, -11);
		assertEquals(Math.sqrt(54 + 49 + 121), t.length());
		assertEquals(224.0, t.lengthSquared());
		
//		t.set(-3, -4, -5);
//		t.eqAbs();
//		assertEquals(new dVector6(3, 4, 5), t);
//		
//		t.eqDiff(x, y);
//		assertEquals(new dVector6(-3, -6, 4), t);
	}		
	
//	@Test
//	public void testDot(){
//		dVector6 x = new dVector6(1, 2, 3, 4, 5, 6);
//		dVector6 y = new dVector6(4, 8, -1, -7, 17, 22);
//		dVector6 t = new dVector6();
//		
//		assertEquals( 4+16-3 , t.eqDot(x, y));
//	}		
	
//	@Test
//	public void testMul(){
	//TODO ?!?!?!?
//		dVector6 x = new dVector6(1, 2, 3);
//		dVector6 y = new dVector6(4, 8, -1);
//		dVector6 t = new dVector6();
//		
//		dMatrix3 B = new dMatrix3(0.10, 0.11, 0.12,   1.10, 1.11, 1.12,  2.10, 2.11, 2.22);
//		dVector6 c = new dVector6(-1, 2.5, -11.7);
//		
//		t.eqMul(B, c);
//		double x1 = ;
//		double x2 = ;
//		double x3 = ;
//	}		
}