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
import org.ode4j.math.DMatrix3.DVector3ColView;
import org.ode4j.math.DMatrix3.DVector3RowTView;

import static org.junit.Assert.*;

public class DMatrix3Test {

	private static final double eps = 1e-9;

	@Test
	public void testGet(){
		DMatrix3 x = newM3();
		assertEquals(1., x.get00(), 0);
		assertEquals(2., x.get01(), 0);
		assertEquals(3., x.get02(), 0);
		assertEquals(4., x.get10(), 0);
		assertEquals(5., x.get11(), 0);
		assertEquals(6., x.get12(), 0);
		assertEquals(7., x.get20(), 0);
		assertEquals(8., x.get21(), 0);
		assertEquals(9., x.get22(), 0);
		assertEquals(1., x.get(0, 0), 0);
		assertEquals(2., x.get(0, 1), 0);
		assertEquals(3., x.get(0, 2), 0);
		assertEquals(4., x.get(1, 0), 0);
		assertEquals(5., x.get(1, 1), 0);
		assertEquals(6., x.get(1, 2), 0);
		assertEquals(7., x.get(2, 0), 0);
		assertEquals(8., x.get(2, 1), 0);
		assertEquals(9., x.get(2, 2), 0);
	}		
		
	@Test
	public void testEqual(){
		DMatrix3 x = newM3();
		DMatrix3 xx = newM3();
		DMatrix3 x1 = new DMatrix3();
		assertTrue(x.isEq(xx, 0));
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				x1.set(xx);
				x1.set(i, j, 0);
				assertFalse(x.isEq(x1, 0));
			}
		}
	}		
		
	@Test
	public void testSet(){
		DMatrix3 x = new DMatrix3();
		DMatrix3 x2 = newM3();
		
		//test setIJ()
		x.set00(1);
		assertEquals(1., x.get00(), 0);
		x.set01(2);
		assertEquals(2., x.get01(), 0);
		x.set02(3);
		assertEquals(3., x.get02(), 0);
		x.set10(4);
		assertEquals(4., x.get10(), 0);
		x.set11(5);
		assertEquals(5., x.get11(), 0);
		x.set12(6);
		assertEquals(6., x.get12(), 0);
		x.set20(7);
		assertEquals(7., x.get20(), 0);
		x.set21(8);
		assertEquals(8., x.get21(), 0);
		x.set22(9);
		assertEquals(9., x.get22(), 0);
		assertEquals(x, x2);
		
		//test set(i, j)
		x = new DMatrix3();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				x.set(i, j, 1 + j + 3*i);
				assertEquals(x.get(i, j), 1 + j + 3*i, 0);
			}
		}
		assertEquals(x, x2);

		//test set(d0, d1, ...)
		x = new DMatrix3();
		x.set(1, 2, 3, 4, 5, 6, 7, 8, 9);
		assertEquals(x, x2);
		
		//
		x = new DMatrix3();
		x.set(x2);
		assertEquals(x, x2);

		//TODO This ",0" should be removed at some point (?)
		x = new DMatrix3();
		x.set12( new double[]{ 1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 0} , 0);
		assertEquals(x, x2);

//		x = new DMatrix3();
//		x.setValues(2);
//		assertEquals(x, new DMatrix3(2, 2, 2, 2, 2, 2, 2, 2, 2));

		x = new DMatrix3();
		assertFalse(x.isEq(x2, 0));
	}		
		
	@Test
	public void testInit(){
		DMatrix3 x = newM3();
		DMatrix3 y = new DMatrix3();
		DMatrix3 z = new DMatrix3(x);
		assertTrue(x.isEq(z, 0));
		assertFalse(x.isEq(y, 0));

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				assertEquals(y.get(i, j), 0, 0);
				assertEquals(z.get(i, j), 1 + j + 3*i, 0);
			}
		}
	}		
	
	@Test
	public void testAdd(){
		DMatrix3 x = newM3();
		DMatrix3 t = new DMatrix3();
		assertFalse(x.isEq(t, 0));
		
		t.add(x);
		assertTrue(t.isEq(x, 0));

		t = new DMatrix3();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				t.add(i, j, 1 + j + 3*i);
			}
		}
		assertTrue(t.isEq(x, 0));
	}
	
	@Test
	public void testSub(){
		DMatrix3 x = newM3();
		DMatrix3 t = new DMatrix3();
		
		t.add(x);
		t.add(x);

		t.set(x);
		t.add(x);
		t.sub(0, 0, 1);
		t.sub(0, 1, 2);
		t.sub(0, 2, 3);
		t.sub(1, 0, 4);
		t.sub(1, 1, 5);
		t.sub(1, 2, 6);
		t.sub(2, 0, 7);
		t.sub(2, 1, 8);
		t.sub(2, 2, 9);
		assertEquals(t, x);
	}		
	
	@Test
	public void testScale(){
		DMatrix3 x = newM3();
		DMatrix3 t = new DMatrix3();
		
		t.set(x);
		t.scale(-2);
		assertTrue(t.isEq( new DMatrix3(-2, -4, -6, -8, -10, -12, -14, -16, -18), 0 ));
	}

	@Test
	public void testViews(){
		DMatrix3 t = newM3();

		//Column view
		DVector3ColView c0 = t.viewCol(0);
		DVector3ColView c1 = t.viewCol(1);
		DVector3ColView c2 = t.viewCol(2);
		OdeTestUtil.assertEquals(new DVector3(1, 4, 7), c0);
		OdeTestUtil.assertEquals(new DVector3(2, 5, 8), c1);
		OdeTestUtil.assertEquals(new DVector3(3, 6, 9), c2);
		t.set(0, 0, -1);
		t.set(1, 1, -5);
		t.set(2, 2, -9);
		OdeTestUtil.assertEquals(new DVector3(-1, 4, 7), c0);
		OdeTestUtil.assertEquals(new DVector3(2, -5, 8), c1);
		OdeTestUtil.assertEquals(new DVector3(3, 6, -9), c2);

		//Row view
		t = newM3();
		DVector3RowTView r0 = t.viewRowT(0);
		DVector3RowTView r1 = t.viewRowT(1);
		DVector3RowTView r2 = t.viewRowT(2);
		OdeTestUtil.assertEquals(new DVector3(1, 2, 3), r0);
		OdeTestUtil.assertEquals(new DVector3(4, 5, 6), r1);
		OdeTestUtil.assertEquals(new DVector3(7, 8, 9), r2);
		t.set(0, 0, -1);
		t.set(1, 1, -5);
		t.set(2, 2, -9);
		OdeTestUtil.assertEquals(new DVector3(-1, 2, 3), r0);
		OdeTestUtil.assertEquals(new DVector3(4, -5, 6), r1);
		OdeTestUtil.assertEquals(new DVector3(7, 8, -9), r2);

		//column clone
		t = newM3();
		DVector3 v0 = t.columnAsNewVector(0);
		DVector3 v1 = t.columnAsNewVector(1);
		DVector3 v2 = t.columnAsNewVector(2);
		OdeTestUtil.assertEquals(new DVector3(1, 4, 7), v0);
		OdeTestUtil.assertEquals(new DVector3(2, 5, 8), v1);
		OdeTestUtil.assertEquals(new DVector3(3, 6, 9), v2);
		//check changing the matrix
		t.set(0, 0, -1);
		t.set(1, 1, -5);
		t.set(2, 2, -9);
		//Check that Vectors did not change
		OdeTestUtil.assertEquals(new DVector3(1, 4, 7), v0);
		OdeTestUtil.assertEquals(new DVector3(2, 5, 8), v1);
		OdeTestUtil.assertEquals(new DVector3(3, 6, 9), v2);
		//check changing the vectors
		v0.setZero();
		v1.setZero();
		v2.setZero();
		//Check that Matrix did not change
		OdeTestUtil.assertEquals(new DMatrix3(-1, 2, 3, 4, -5, 6, 7, 8, -9), t);
	}
		
	@Test
	public void testZeroIdentity() {
		DMatrix3 t = newM3();

		t.eqIdentity();
		assertEquals(new DMatrix3(1, 0, 0, 0, 1, 0, 0, 0, 1), t);

		t.eqZero();
		assertEquals(new DMatrix3(0, 0, 0, 0, 0, 0, 0, 0, 0), t);
	}		
	
	@Test
	public void testMul(){
		DMatrix3 T = new DMatrix3();
		
		DMatrix3 B = new DMatrix3(0.10, 0.11, 0.12,   1.10, 1.11, 1.12,  2.10, 2.11, 2.12);
		DMatrix3 C = new DMatrix3(3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9);
		DMatrix3 RES = new DMatrix3(
				0.10*3.1 + 0.11*3.4 + 0.12*3.7, 
				0.10*3.2 + 0.11*3.5 + 0.12*3.8, 
				0.10*3.3 + 0.11*3.6 + 0.12*3.9,
				
				1.10*3.1 + 1.11*3.4 + 1.12*3.7, 
				1.10*3.2 + 1.11*3.5 + 1.12*3.8, 
				1.10*3.3 + 1.11*3.6 + 1.12*3.9,

				2.10*3.1 + 2.11*3.4 + 2.12*3.7, 
				2.10*3.2 + 2.11*3.5 + 2.12*3.8, 
				2.10*3.3 + 2.11*3.6 + 2.12*3.9);
		
		
		T.eqMul(B, C);
		assertEquals(RES, T);
		
		
		T = new DMatrix3();
		T.dMultiply0(B, C);
		assertEquals(RES, T);
	}
	
	@Test
	public void testTrans(){
		DMatrix3 T = new DMatrix3();
		
		DMatrix3 B = new DMatrix3(0.10, 0.11, 0.12,   1.10, 1.11, 1.12,  2.10, 2.11, 2.12);
		DMatrix3 B2 = new DMatrix3(0.10, 0.11, 0.12,   1.10, 1.11, 1.12,  2.10, 2.11, 2.12);
		DMatrix3 BT = new DMatrix3(0.10, 1.10, 2.10,   0.11, 1.11, 2.11,  0.12, 1.12, 2.12);
		
		T.set(B);
		T.eqTranspose();
		assertEquals(BT, T);
		
		T = B.reTranspose();
		assertEquals(BT, T);
		//B should not change!
		assertEquals(B2, B);
	}
	
	private DMatrix3 newM3() {
		DMatrix3 m = new DMatrix3();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				m.set(i, j, 1 + j + 3*i);
			}
		}
		return m;
	}
	
		
	@Test
	public void testDotVector() {
		DMatrix3 m = new DMatrix3(21, 22, 23, 24, 25, 26, 27, 28, 29);
		DVector3 y = new DVector3(31, 32, 33);
		double[] da = new double[]{61, 62, 71, 72, 73};
		
		double d, ex;
		
		// ************ check dotCol ************
		d = m.dotCol(0, y);
		ex = 21*31 + 24*32 + 27*33;
		assertEquals(ex, d, eps);
		
		d = m.dotCol(1, y);
		ex = 22*31 + 25*32 + 28*33;
		assertEquals(ex, d, eps);
		
		d = m.dotCol(2, y);
		ex = 23*31 + 26*32 + 29*33;
		assertEquals(ex, d, eps);
		
		try {
			d = m.dotCol(-1, y);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotCol(3, y);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		
		// ************ check dotRow Vector ************
		d = m.dotRow(0, y);
		ex = 21*31 + 22*32 + 23*33;
		assertEquals(ex, d, eps);
		
		d = m.dotRow(1, y);
		ex = 24*31 + 25*32 + 26*33;
		assertEquals(ex, d, eps);
		
		d = m.dotRow(2, y);
		ex = 27*31 + 28*32 + 29*33;
		assertEquals(ex, d, eps);
		
		try {
			d = m.dotRow(-1, y);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRow(3, y);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		
		
		
		// ************ check dotRow Array ************
		d = m.dotRow(0, da, 2);
		ex = 21*71 + 22*72 + 23*73;
		assertEquals(ex, d, eps);
		
		d = m.dotRow(1, da, 2);
		ex = 24*71 + 25*72 + 26*73;
		assertEquals(ex, d, eps);
		
		d = m.dotRow(2, da, 2);
		ex = 27*71 + 28*72 + 29*73;
		assertEquals(ex, d, eps);
		
		try {
			d = m.dotRow(-1, da, 2);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRow(3, da, 2);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRow(1, da, -1);
			fail();
		} catch (RuntimeException e) {
			//good
		}
		try {
			d = m.dotRow(1, da, 3);
			fail();
		} catch (RuntimeException e) {
			//good
		}
	}
	
	
	@Test
	public void testDotMatrix() {
		DMatrix3 m = new DMatrix3(21, 22, 23, 24, 25, 26, 27, 28, 29);
		DMatrix3 m2 = new DMatrix3(11, 12, 13, 14, 15, 16, 17, 18, 19);
		
		double d, ex;
		
		// ************ check dotColCol ************
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				d = m.dotColCol(i, m2, j);
				ex = m.get(0, i)*m2.get(0, j) + m.get(1, i)*m2.get(1, j) + m.get(2, i)*m2.get(2, j);
				assertEquals(ex, d, 0);
			}
		}
		try {
			d = m.dotColCol(-1, m2, 1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotColCol(3, m2, 1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotColCol(1, m2, -1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotColCol(1, m2, 3);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}

		
		// ************ check dotRowCol ************
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				d = m.dotRowCol(i, m2, j);
				ex = m.get(i,0)*m2.get(0,j) + m.get(i,1)*m2.get(1,j) + m.get(i,2)*m2.get(2,j);
				assertEquals(ex, d, 0);
			}
		}
		try {
			d = m.dotRowCol(-1, m2, 1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRowCol(3, m2, 1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRowCol(1, m2, -1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRowCol(1, m2, 3);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}

		
		// ************ check dotRowRow ************
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				d = m.dotRowRow(i, m2, j);
				ex = m.get(i,0)*m2.get(j,0) + m.get(i,1)*m2.get(j,1) + m.get(i,2)*m2.get(j,2);
				assertEquals(ex, d, 0);
			}
		}
		try {
			d = m.dotRowRow(-1, m2, 1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRowRow(3, m2, 1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRowRow(1, m2, -1);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
		try {
			d = m.dotRowRow(1, m2, 3);
			fail();
		} catch (IllegalArgumentException e) {
			//good
		}
	}
}