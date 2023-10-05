/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
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
package org.ode4j.demo;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.demo.WorldGeom3.*;


/**
 * Test for non-capped cylinder, by Bram Stolk.
 */
class DemoCyl extends dsFunctions {


	private static final boolean BOX = true;
	private static final boolean CYL = true;

	// some constants

	private static final float RADIUS = 0.22f;	// wheel radius
	private static final float WMASS = 0.2f;	// wheel mass
	private static final float WHEELW = 0.2f;	// wheel width
	private static final float BOXSZ = 0.4f;	// box size
	private static boolean CYL_GEOM_OFFSET = false;   // rotate cylinder using geom offset

	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;
	private static DBody boxbody;
	private static DBox boxgeom;
	private static DBody cylbody;
	private static DCylinder cylgeom;
	private static DJointGroup contactgroup;
	private static DGeom world_mesh;


	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};

	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		assert(o1!=null);
		assert(o2!=null);

		if ( o1 instanceof DSpace || o2 instanceof DSpace )
		{
			System.out.println("testing space " + o1 + "  " + o2);
			// colliding a space with something
			OdeHelper.spaceCollide2(o1,o2,data,nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		//  System.out.println("testing geoms " + o1 + "  " + o2);

		final int N = 32;
		DContactBuffer contacts = new DContactBuffer(N);
		int n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());//[0].geom),sizeof(dContact));
		if (n > 0) 
		{
			for (int i=0; i<n; i++) 
			{
				DContact contact = contacts.get(i);
				contact.surface.slip1 = 0.7;
				contact.surface.slip2 = 0.7;
				contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
				contact.surface.mu = 50.0; // was: dInfinity
				contact.surface.soft_erp = 0.96;
				contact.surface.soft_cfm = 0.04;
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
				c.attach (contact.geom.g1.getBody(), contact.geom.g2.getBody());
			}
		}
	}


	private static float[] xyz = {-8,-9,3};
	private static float[] hpr = {45.0000f,-27.5000f,0.0000f};

	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
	}



	private void reset_state()
	{
		float sx=-4, sy=-4, sz=2;
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
		if (BOX) {
			boxbody.setPosition (sx, sy+1, sz);
			boxbody.setLinearVel (0,0,0);
			boxbody.setAngularVel (0,0,0);
			boxbody.setQuaternion (q);
		}
		if (CYL) {
			cylbody.setPosition (sx, sy, sz);
			cylbody.setLinearVel (0,0,0);
			cylbody.setAngularVel (0,0,0);
			cylbody.setQuaternion (q);
		}
	}


	// called when a key pressed

	@Override
	public void command (char cmd)
	{
		switch (cmd) 
		{
		case ' ':
			reset_state();
			break;
		}
	}



	// simulation loop

	private void simLoop (boolean pause)
	{
		double simstep = 0.005; // 5ms simulation steps
		double dt = dsElapsedTime();
		int nrofsteps = (int) Math.ceil(dt/simstep);
		for (int i=0; i<nrofsteps && !pause; i++)
		{
			space.collide (null,nearCallback);
			world.quickStep (simstep);
			contactgroup.empty ();
		}

		dsSetColor (1,1,1);
		if (BOX) {
			DVector3C BPos = boxbody.getPosition();
			DMatrix3C BRot = boxbody.getRotation();
			DVector3 sides = new DVector3(BOXSZ, BOXSZ, BOXSZ);
			dsDrawBox( BPos, BRot, sides );
		}
		if (CYL) {
			dsDrawCylinder( cylbody.getPosition(), cylbody.getRotation(),
					WHEELW, RADIUS );
		}

		// draw world trimesh
		dsSetColor(0.7f,0.7f,0.4f);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_NONE);

		final DVector3C Pos = world_mesh.getPosition();

		final DMatrix3C Rot = world_mesh.getRotation();

		int numi = world_indices.length;

		for (int i=0; i<numi/3; i++)
		{
			int i0 = world_indices[i*3+0] * 3;
			int i1 = world_indices[i*3+1] * 3;
			int i2 = world_indices[i*3+2] * 3;
			dsDrawTriangle(Pos, Rot, world_vertices, i0, i1, i2, true); // single precision draw
		}
	}


	public static void main(String[] args) {
		new DemoCyl().demo(args);
	}

	private void demo(String [] args) {
		DMass m = OdeHelper.createMass();
		DMatrix3 R = new DMatrix3();

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld ();
		space = OdeHelper.createHashSpace (null);
		contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0,0,-9.8);
		world.setQuickStepNumIterations (12);


		// Create a static world using a triangle mesh that we can collide with.
		int numv = world_vertices.length;
		int numi = world_indices.length;
		System.out.println("numv=" + numv + ", numi=" + numi);
		DTriMeshData data = OdeHelper.createTriMeshData();

		data.build
		(
				world_vertices, 
				//3,// * sizeof(float), 
				//numv, 
				world_indices//, 
				//numi, 
				//3// * sizeof(dTriIndex)
		);

		world_mesh = OdeHelper.createTriMesh(space, data, null, null, null);
		world_mesh.setPosition(0, 0, 0.5);
		dRFromAxisAndAngle (R, 0,1,0, 0.0);
		world_mesh.setRotation (R);


		if (BOX) {
			boxbody = OdeHelper.createBody (world);
			m.setBox (1, BOXSZ, BOXSZ, BOXSZ);
			m.adjust (1);
			boxbody.setMass (m);
			boxgeom = OdeHelper.createBox (null, BOXSZ, BOXSZ, BOXSZ);
			boxgeom.setBody (boxbody);
			space.add (boxgeom);
		}
		if (CYL) {
			cylbody = OdeHelper.createBody (world);
			m.setSphere (1,RADIUS);
			m.adjust (WMASS);
			cylbody.setMass (m);
			cylgeom = OdeHelper.createCylinder(null, RADIUS, WHEELW);
			cylgeom.setBody (cylbody);

			if (CYL_GEOM_OFFSET) {
				DMatrix3 mat = new DMatrix3();
				dRFromAxisAndAngle(mat,1.0f,0.0f,0.0f,M_PI/2.0);
				cylgeom.setOffsetRotation(mat);
			}

			space.add (cylgeom);
		}
		reset_state();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		contactgroup.empty();
		contactgroup.destroy();

		// First destroy geoms, then space, then the world.
		if (CYL) {
			cylgeom.destroy();
		}
		if (BOX) {
			boxgeom.destroy();
		}
		world_mesh.destroy ();

		space.destroy ();
		world.destroy ();
		OdeHelper.closeODE();
	}


	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}


	@Override
	public void stop() {
		// Nothing
	}
}


