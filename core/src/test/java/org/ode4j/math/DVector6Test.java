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
package org.ode4j.math;

import org.junit.Test;

import static org.junit.Assert.*;

public class DVector6Test {

	private static final double eps = 1e-9;

	@Test
	public void testGet(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		assertEquals(1., x.get0(), 0);
		assertEquals(2., x.get1(), 0);
		assertEquals(3., x.get2(), 0);
		assertEquals(4., x.get3(), 0);
		assertEquals(5., x.get4(), 0);
		assertEquals(6., x.get5(), 0);
		assertEquals(1., x.get(0), 0);
		assertEquals(2., x.get(1), 0);
		assertEquals(3., x.get(2), 0);
		assertEquals(4., x.get(3), 0);
		assertEquals(5., x.get(4), 0);
		assertEquals(6., x.get(5), 0);
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
		assertTrue(x.isEq(xx, 0));
		assertFalse(x.isEq(x1, 0));
		assertFalse(x.isEq(x2, 0));
		assertFalse(x.isEq(x3, 0));
		assertFalse(x.isEq(x4, 0));
		assertFalse(x.isEq(x5, 0));
		assertFalse(x.isEq(x6, 0));
	}		
		
	@Test
	public void testSet(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 x2 = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6(7, 8, 9, 10, 11, 12);
		DVector6 z = new DVector6(13, 14, 15, 16, 17, 18);

		x.set(0, 7);
		x.set(1, 8);
		x.set(2, 9);
		x.set(3, 10);
		x.set(4, 11);
		x.set(5, 12);
		assertEquals(x, y);

		x.set(1, 2, 3, 4, 5, 6);
		assertEquals(x, x2);
		
		x.set(y);
		assertEquals(x, y);

		//This ",0" should be removed at some point (?)
		x.set( new double[]{ 8, 9, 11, 7, 6, 5} );
		assertTrue(x.get0()==8 && x.get1()==9 && x.get2()==11 && x.get3()==7 && x.get4()==6 && x.get5()==5);

		assertFalse(x.isEq(x2, 0));
		assertFalse(x.isEq(y, 0));
		assertFalse(x.isEq(z, 0));
	}		
		
	@Test
	public void testInit(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6();
		DVector6 z = new DVector6(x);
		assertTrue(x.isEq(z, 0));
		assertFalse(x.isEq(y, 0));
		assertEquals(0., y.get0(), 0);
		assertEquals(0., y.get1(), 0);
		assertEquals(0., y.get2(), 0);
		assertEquals(0., y.get3(), 0);
		assertEquals(0., y.get4(), 0);
		assertEquals(0., y.get5(), 0);

		assertEquals(1., z.get0(), 0);
		assertEquals(2., z.get1(), 0);
		assertEquals(3., z.get2(), 0);
		assertEquals(4., z.get3(), 0);
		assertEquals(5., z.get4(), 0);
		assertEquals(6., z.get5(), 0);
	}		
	
	@Test
	public void testAdd(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6(4, 8, -1, -7, 17, 22);
		DVector6 t = new DVector6();
		assertFalse(x.isEq(y, 0));

		t.set(y);
		t.add(0, -3);
		t.add(1, -6);
		t.add(2, 4);
		t.add(3, 11);
		t.add(4, -12);
		t.add(5, -16);
		assertTrue(t.isEq(x, 0));
	}
	
	@Test
	public void testSub(){
		DVector6 x = new DVector6(1, 2, 3, 4, 5, 6);
		DVector6 y = new DVector6(4, 8, -1, -7, 17, 22);
		assertFalse(x.isEq(y, 0));
	}

	@Test
	public void testOther(){
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

		t.set(3, 4, -5, -2, 7, -11);
		assertEquals(Math.sqrt(54 + 49 + 121), t.length(), eps);
		assertEquals(224.0, t.lengthSquared(), eps);
	}
}