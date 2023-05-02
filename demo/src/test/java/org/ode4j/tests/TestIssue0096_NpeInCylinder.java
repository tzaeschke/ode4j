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
package org.ode4j.tests;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.*;

import java.util.Random;

import static org.junit.Assert.fail;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;
import static org.ode4j.ode.OdeHelper.createCylinder;

/**
 *
 */
public class TestIssue0096_NpeInCylinder {

	private final static int WARMUP = 100 * 1000;
	// private final static int BENCHMARK = 1000 * 1000;
	private final static int BENCHMARK = 100 * 1000;
	private static final double DENSITY = 5.0;        // density of all objects
	private static final int MAX_CONTACTS = 8;

	private DWorld world;
	private DSpace space;
	private final Random r = new Random(0);

	private int cntCollisions = 0;
	private int cntContacts = 0;

	private static final float[] CUBE_POINTS = {
			0.25f, 0.25f, 0.25f, // point 0
			-0.25f, 0.25f, 0.25f, // point 1
			0.25f, -0.25f, 0.25f, // point 2
			-0.25f, -0.25f, 0.25f, // point 3
			0.25f, 0.25f, -0.25f, // point 4
			-0.25f, 0.25f, -0.25f, // point 5
			0.25f, -0.25f, -0.25f, // point 6
			-0.25f, -0.25f, -0.25f,// point 7
	};

	private static final int[] CUBE_INDICES = {
			0, 2, 6, // 0
			0, 6, 4, // 1
			1, 0, 4, // 2
			1, 4, 5, // 3
			0, 1, 3, // 4
			0, 3, 2, // 5
			3, 1, 5, // 6
			3, 5, 7, // 7
			2, 3, 7, // 8
			2, 7, 6, // 9
			5, 4, 6, // 10
			5, 6, 7  // 11
	};

	private DGeom cylinder() {
		double radius = 1; // r.nextDouble();
		double length = 1; // r.nextDouble();
		DGeom geom = OdeHelper.createCylinder(space, radius, length);

		DMass mass = OdeHelper.createMass();
		mass.setCylinder(DENSITY, r.nextInt(3) + 1, radius, length);

		return assemble(geom, body2(), mass);
	}

	private DGeom trimesh() {
		DTriMeshData data = OdeHelper.createTriMeshData();
		data.build(CUBE_POINTS, CUBE_INDICES);
		data.preprocess();
		DTriMesh geom = OdeHelper.createTriMesh(space, data, null, null, null);

		DMass mass = OdeHelper.createMass();
		mass.setTrimesh(DENSITY, geom);

		return assemble(geom, body(), mass);
	}

	private DSpace createSpace() {
		return OdeHelper.createSimpleSpace();
		// return OdeHelper.createSapSpace(DSapSpace.AXES.XZY);
		// return OdeHelper.createBHVSpace(0);
	}

	private DBody body() {
		DBody body = OdeHelper.createBody(world);

		body.setPosition(0, 0, 0);

//		DMatrix3 R = new DMatrix3();
//		DVector3 a = vector().scale(2).sub(1, 1, 1);
//		dRFromAxisAndAngle(R, a, r.nextDouble() * 10.0 - 5.0);
//		body.setRotation(R);

		return body;
	}

	private DBody body2() {
		DBody body = OdeHelper.createBody(world);

//		body.setPosition(vector().scale(25));
//
//		DMatrix3 R = new DMatrix3();
//		DVector3 a = vector().scale(2).sub(1, 1, 1);
//		dRFromAxisAndAngle(R, a, r.nextDouble() * 10.0 - 5.0);
//		body.setRotation(R);

		return body;
	}

	private DGeom assemble(DGeom geom, DBody body, DMass mass) {
		geom.setBody(body);
		body.setMass(mass);
		return geom;
	}

	private DVector3 vector() {
		return new DVector3(r.nextDouble(), r.nextDouble(), r.nextDouble());
	}

	private final DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(o1, o2);
		}
	};

	//private final DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);


	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.
	private void nearCallback(DGeom o1, DGeom o2) {
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1 != null && b2 != null && areConnectedExcluding(b1, b2, DContactJoint.class)) return;

		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
		for (int i = 0; i < MAX_CONTACTS; i++) {
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;

			contact.geom.depth = 0;
			contact.geom.pos.setZero();
			contact.geom.g1 = null;
			contact.geom.g2 = null;

			contact.fdir1.setZero();
		}
		int numc = OdeHelper.collide(o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());
		if (numc != 0) {
			cntCollisions++;
			cntContacts += numc;
		}
	}

	private void collide(DGeom geom1, DGeom geom2, int iterations) {
		//reset(geom1, geom2);

		long timer = 0;
		int prevCount = 0;
		cntCollisions = 0;
		cntContacts = 0;
		int totalCount = 0;
		for (int j = 0; j < iterations; j++) {
			long time1 = System.nanoTime();
			space.collide(0, nearCallback);
			// world.step(0.05);
			world.quickStep(0.05);

			// remove all contact joints
			long time2 = System.nanoTime();
			timer += (time2 - time1);

//			if (prevCount > 0 && j < 2) {
//				// If we have (almost) immediate collision then the geoms where to close and may have low speed.
//				// (speed is calculated from distance).
//				fail("j=" + j);
//			}

//			// abort once we have stopped getting contacts
//			if (prevCount > 0 && prevCount == cntCollisions) {
//				reset(geom1, geom2);
//				totalCount += cntCollisions;
//				prevCount = 0;
//				cntCollisions = 0;
//				cntContacts = 0;
//				continue;
//			}
			prevCount = cntCollisions;
//			if (cntCollisions > 50) {
//				fail();  // TODO why is this so high for Box? -> See also slow falling box in DemoTrimesh..?
//			}
//			if (cntContacts > MAX_CONTACTS * cntCollisions) {
//				fail();
//			}
		}
//		if (iterations == BENCHMARK) {
//			String msg = geom1.getClass().getSimpleName() + "-" + geom2.getClass().getSimpleName();
//			System.out.println("Benchmark: " + msg + " contact/iterations: " + totalCount + "/" + iterations
//					+ " time: " + timer / iterations + " ns/step");
//		}
		geom1.destroy();
		geom2.destroy();
	}

	@Before
	public void beforeTest() {
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = createSpace();
		// world.setGravity (0,0,-0.5);
		world.setCFM(1e-5);
	}

	@After
	public void afterTest() {
		space.destroy();
		world.destroy();
		OdeHelper.closeODE();
		System.gc();
	}

	@Test
	public void testIssue96() {
		for (int i = 0; i < 100; ++i) {
			DGeom g1 = cylinder();//createCylinder(1, 1);
			DGeom g2 = trimesh();//createCylinder(1, 1);
			g2.setPosition(0,0,-0.75);
			collide(g1, g2, 100);
		}
	}
}