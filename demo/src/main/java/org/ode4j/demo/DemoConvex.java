/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.demo;

import org.ode4j.drawstuff.DrawStuff;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER.DS_NONE;
import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.demo.Halton235Geom.*;

/**
 * Convex demo.
 *  Serves as a test for the convex geometry.
 * By Bram Stolk.
 */
public class DemoConvex extends DrawStuff.dsFunctions {

	// Height at which we drop the composite block.
	private static final double H = 4.20;

	private static DWorld world;
	private static DSpace space;

	private static DBody mbody;

	private static final DBody[] hbody = new DBody[halton_numc ];
	private static final DGeom[] hgeom = new DGeom[halton_numc ];

	private static DJointGroup contactgroup;

	private static boolean drawpos = false;
	private static boolean solidkernel = false;


	private final DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};

	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private void nearCallback(Object data, DGeom o1, DGeom o2) {
		assert (o1 != null);
		assert (o2 != null);
		if (o1 instanceof DSpace || o2 instanceof DSpace) {
			// colliding a space with something
			OdeHelper.spaceCollide2(o1, o2, data, nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		final int N = 32;
		DContactBuffer contacts = new DContactBuffer(N);
		int n = OdeHelper.collide(o1, o2, N, contacts.getGeomBuffer());
		if (n > 0) {
			for (int i = 0; i < n; i++) {
				DContact contact = contacts.get(i);
				contact.surface.slip1 = 0.7;
				contact.surface.slip2 = 0.7;
				contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
				contact.surface.mu = 500.0; // was: dInfinity
				contact.surface.soft_erp = 0.50;
				contact.surface.soft_cfm = 0.03;
				DJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
				c.attach(
								contact.geom.g1.getBody(),
								contact.geom.g2.getBody()
						);
			}
		}
	}


	// start simulation - set viewpoint
	@Override
	public void start() {
		//dAllocateODEDataForThread(dAllocateMaskAll);
		dsSetViewpoint(xyz, hpr);
		System.err.println("Press SPACE to reset the simulation.");
	}
	private static final float[] xyz = { // [ 3] ={
		-8, 0, 5
	};
	private static final float[] hpr = { // [ 3] ={
		0.0f, -29.5000f, 0.0000f
	};


	private static void reset() {
		DQuaternion q = new DQuaternion();
		q.setIdentity();
		mbody.setPosition(0, 0, 0 + H);
		mbody.setQuaternion(q);
		mbody.setLinearVel(0, 0, 0);
		mbody.setAngularVel(0, 0, 0);
		mbody.enable();
		for (int i = 0; i < halton_numc; ++i) {
			DBody body = hbody[i];
			if (body == null) continue;
			body.setPosition(halton_pos[i][0], halton_pos[i][1], halton_pos[i][2] + H);
			body.setQuaternion(q);
			body.setLinearVel(0, 0, 0);
			body.setAngularVel(0, 0, 0);
			body.enable();
		}
	}


	// called when a key pressed
	@Override
	public void command(char cmd) {
		switch (cmd) {
			case ' ':
				reset();
				break;
			default:
				break;
		}
	}

	@Override
	public void step(boolean pause) {
		double simstep = 1 / 240.0;
		double dt = dsElapsedTime();

		int nrofsteps = (int) Math.ceil(dt / simstep);
		nrofsteps = nrofsteps > 8 ? 8 : nrofsteps;

		for (int i = 0; i < nrofsteps && !pause; i++) {
			OdeHelper.spaceCollide(space, 0, nearCallback);
			world.quickStep(simstep);
			contactgroup.empty();
		}

		dsSetColor(1, 1, 1);
		// Draw the convex objects.
		for (int i = 0; i < halton_numc; ++i) {
			DGeom geom = hgeom[i];
			DBody body = geom.getBody();
			//const dReal *pos = dBodyGetPosition(body);
			//const dReal *rot = dBodyGetRotation(body);
			DVector3C pos = geom.getPosition();
			DMatrix3C rot = geom.getRotation();
			dsDrawConvex
					(
							pos, rot,
							halton_planes[i],
							halton_numf[i],
							halton_verts[i],
							halton_numv[i],
							halton_faces[i]
					);
		}

		if (drawpos) {
			dsSetColor(1, 0, 0.2);
			dsSetTexture(DS_NONE);
			final float l = 0.35f;
			for (int i = 0; i < halton_numc; ++i) {
				DBody body = hbody[i];
				DVector3C pos2 = body.getPosition();
				float[] pos = pos2.toFloatArray();
				float[] x0 ={
					pos[0] - l, pos[1], pos[2]
				} ;
				float[] x1 ={
					pos[0] + l, pos[1], pos[2]
				} ;
				float[] y0 ={
					pos[0], pos[1] - l, pos[2]
				} ;
				float[] y1 ={
					pos[0], pos[1] + l, pos[2]
				} ;
				float[] z0 ={
					pos[0], pos[1], pos[2] - l
				} ;
				float[] z1 ={
					pos[0], pos[1], pos[2] + l
				} ;
				dsDrawLine(x0, x1);
				dsDrawLine(y0, y1);
				dsDrawLine(z0, z1);
			}
		}
	}

	public static void main(final String[] args) {
		new DemoConvex().demo(args);
	}

	private void demo(String[] args) {
		DMass m = OdeHelper.createMass();

		// setup pointers to drawstuff callback functions
		//		dsFunctions fn;
		//		fn.version = DS_VERSION;
		//		fn.start = &start;
		//		fn.step = &simLoop;
		//		fn.command = &command;
		//		fn.stop = 0;
		//		fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace();
		((DHashSpace)space).setLevels(-3, 5);
		OdeHelper.createPlane(space, 0, 0, 1, 0);    // Add a ground plane.

		contactgroup = OdeHelper.createJointGroup();
		world.setGravity(0, 0, -9.8);
		world.setQuickStepNumIterations(32);
		world.setContactMaxCorrectingVel(40);
		world.setMaxAngularSpeed(62.8);
		world.setERP(0.7);
		world.setQuickStepW(0.75); // For increased stability.

		world.setAutoDisableFlag(true);
		world.setAutoDisableLinearThreshold(0.01);
		world.setAutoDisableAngularThreshold(0.03);
		world.setAutoDisableTime(0.15f);

		final float kernelrad = 0.7f;

		mbody = OdeHelper.createBody(world);
		mbody.setPosition(0, 0, 0 + H);
		m.setSphere(5, kernelrad );
		mbody.setMass( m );

		for (int i = 0; i < halton_numc; ++i) {
			DGeom geom = OdeHelper.createConvex(
							space,
							halton_planes[i],
							halton_numf[i],
							halton_verts[i],
							halton_numv[i],
							halton_faces[i]
					);
			hgeom[i] = geom;
			final double x = halton_pos[i][0];
			final double y = halton_pos[i][1];
			final double z = halton_pos[i][2];
			final double dsqr = x * x + y * y + z * z;

			if (dsqr < kernelrad * kernelrad && solidkernel) {
				geom.setBody(mbody);
				geom.setOffsetPosition(x, y, z);
			} else {
				DBody body = OdeHelper.createBody(world);
				hbody[i] = body;
				body.setPosition(x, y, z + H);
				double volu = halton_volu[i];
				double rad = Math.pow(volu * 3 / (4 * M_PI), (1 / 3.0));
				m.setSphere( 5, rad );
				body.setMass( m );
//#if 1
				body.setLinearDamping(0.0005);
				body.setAngularDamping(0.0300);
//#edif
				geom.setBody(body);
			}
		}

		// run simulation
		final int w = 1280;
		final int h = 720;
		dsSimulationLoop(args, w, h, this);

		contactgroup.empty ();
		contactgroup.destroy ();
		space.destroy ();
		world.destroy ();
		OdeHelper.closeODE();
	}

	@Override
	public void stop() {
		// Nothing
	}

}
