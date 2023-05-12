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

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

/**
 * 
 * @author Piotr Piastucky
 *
 */
public class TestIssue0019_BodyRemove {

	@SuppressWarnings("deprecation")
	@Test
	public void testDestroyFirst() {
		OdeHelper.initODE2(0);
		DWorld world = OdeHelper.createWorld();
		DBody body = OdeHelper.createBody(world);
		DSphere geom1 = OdeHelper.createSphere(1);
		DSphere geom2 = OdeHelper.createSphere(1);
		DSphere geom3 = OdeHelper.createSphere(1);
		geom1.setBody(body);
		geom2.setBody(body);
		geom3.setBody(body);
		geom1.destroy();
		List<DGeom> geoms = new ArrayList<DGeom>();
		DGeom g1 = body.getFirstGeom();
		while (g1 != null) {
			geoms.add(g1);
			g1 = body.getNextGeom(g1);
		}
		Assert.assertEquals(2, geoms.size()); // should be 2 remaining geoms, but there is none!!
		Assert.assertNotEquals(geoms.get(0), geom1);
		Assert.assertNotEquals(geoms.get(1), geom1);
	}

	@SuppressWarnings("deprecation")
	@Test
	public void testDestroyMiddle() {
		OdeHelper.initODE2(0);
		DWorld world = OdeHelper.createWorld();
		DBody body = OdeHelper.createBody(world);
		DSphere geom1 = OdeHelper.createSphere(1);
		DSphere geom2 = OdeHelper.createSphere(1);
		DSphere geom3 = OdeHelper.createSphere(1);
		geom1.setBody(body);
		geom2.setBody(body);
		geom3.setBody(body);
		geom2.destroy();
		List<DGeom> geoms = new ArrayList<DGeom>();
		DGeom g1 = body.getFirstGeom();
		while (g1 != null) {
			geoms.add(g1);
			g1 = body.getNextGeom(g1);
		}
		Assert.assertEquals(2, geoms.size()); // should be 2 remaining geoms, but there is only 1!!
		Assert.assertNotEquals(geoms.get(0), geom2);
		Assert.assertNotEquals(geoms.get(1), geom2);
	}

	@SuppressWarnings("deprecation")
	@Test
	public void testDestroyLast() {
		OdeHelper.initODE2(0);
		DWorld world = OdeHelper.createWorld();
		DBody body = OdeHelper.createBody(world);
		DSphere geom1 = OdeHelper.createSphere(1);
		DSphere geom2 = OdeHelper.createSphere(1);
		DSphere geom3 = OdeHelper.createSphere(1);
		geom1.setBody(body);
		geom2.setBody(body);
		geom3.setBody(body);
		geom3.destroy();
		List<DGeom> geoms = new ArrayList<DGeom>();
		DGeom g1 = body.getFirstGeom();
		while (g1 != null) {
			geoms.add(g1);
			g1 = body.getNextGeom(g1);
		}
		Assert.assertEquals(2, geoms.size());
		Assert.assertNotEquals(geoms.get(0), geom3);
		Assert.assertNotEquals(geoms.get(1), geom3);
	}
}
