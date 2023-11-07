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
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.demo.BasketGeom.*;

/**
 * Basket ball demo.
 * Serves as a test for the sphere vs trimesh collider
 * By Bram Stolk.
 * Press the spacebar to reset the position of the ball.
 */
public class DemoBasketConvex extends dsFunctions {

	// some constants

	private static final double RADIUS = 0.3;

	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;

	private static DBody sphbody;
	private static DGeom sphgeom;

	private static DJointGroup contactgroup;
	private static DTriMesh world_mesh;


	private DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
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

		if (o1 instanceof DSpace || o2 instanceof DSpace)
		{
			//fprintf(stderr,"testing space %p %p\n", o1,o2);
			System.err.println("testing space " + o1 + " " + o2);
			// colliding a space with something
			OdeHelper.spaceCollide2(o1,o2,data,nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		//  fprintf(stderr,"testing geoms %p %p\n", o1, o2);

		final int N = 32;
		DContactBuffer contacts = new DContactBuffer(N);
		int n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());
		if (n > 0) 
		{
			for (int i=0; i<n; i++) 
			{
				DContact contact = contacts.get(i);
				// Paranoia  <-- not working for some people, temporarily removed for 0.6
				//dIASSERT(dVALIDVEC3(contact[i].geom.pos));
				//dIASSERT(dVALIDVEC3(contact[i].geom.normal));
				//dIASSERT(!dIsNan(contact[i].geom.depth));
				contact.surface.slip1 = 0.7;
				contact.surface.slip2 = 0.7;
				contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
				contact.surface.mu = 50.0; // was: dInfinity
				contact.surface.soft_erp = 0.96;
				contact.surface.soft_cfm = 0.04;
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
				c.attach( contact.geom.g1.getBody(),
						contact.geom.g2.getBody());
			}
		}
	}


	// start simulation - set viewpoint

	@Override
	public void start()
	{
		//dAllocateODEDataForThread(dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
	}
	private float[] xyz = {-8,0,5};
	private float[] hpr = {0.0f,-29.5000f,0.0000f};



	private void reset_ball()
	{
		float sx=0.0f, sy=2.973f, sz=7.51f;

//		//#if defined(_MSC_VER) && defined(dDOUBLE) 
//		//sy -= 0.01; // Cheat, to make it score under win32/double
//		//#endif
		sy += 0.033; // Windows 64 //TODO
		//sy += 0.046;  //For 'double' on Linux 64bit. //TODO !!! 

		DQuaternion q = new DQuaternion();
		q.setIdentity();
		sphbody.setPosition (sx, sy, sz);
		sphbody.setQuaternion(q);
		sphbody.setLinearVel (0,0,0);
		sphbody.setAngularVel (0,0,0);
	}


	// called when a key pressed

	@Override
	public void command (char cmd)
	{
		switch (cmd) 
		{
		case ' ':
			reset_ball();
			break;
		}
	}


	// simulation loop

	@Override
	public void step (boolean pause)
	{
		double simstep = 0.001; // 1ms simulation steps
		double dt = dsElapsedTime();

		int nrofsteps = (int) Math.ceil(dt/simstep);
		//  fprintf(stderr, "dt=%f, nr of steps = %d\n", dt, nrofsteps);

		for (int i=0; i<nrofsteps && !pause; i++)
		{
			OdeHelper.spaceCollide (space,0,nearCallback);
			world.quickStep (simstep);
			contactgroup.empty();
		}

		dsSetColor (1,1,1);
		DVector3C spos = sphbody.getPosition();
		DMatrix3C srot = sphbody.getRotation();
		dsDrawSphere ( spos, srot, RADIUS );

		// draw world trimesh
		dsSetColor(0.4f,0.7f,0.9f);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_NONE);

		DVector3C pos = world_mesh.getPosition();
		//dIASSERT(dVALIDVEC3(Pos));

		DMatrix3C rot = world_mesh.getRotation();
		//dIASSERT(dVALIDMAT3(Rot));

		int numi = world_indices.length;;

		for (int i=0; i<numi; i+=3)
		{
			int i0 = world_indices[i+0]*3;
			int i1 = world_indices[i+1]*3;
			int i2 = world_indices[i+2]*3;
			dsDrawTriangle(pos, rot, world_vertices, i0, i1, i2, true); // single precision draw
		}
	}


	/**
	 * @param args args
	 */
	public static void main(final String[] args) {
		new DemoBasketConvex().demo(args);
	}

	private void demo(String[] args)
	{
		DMass m = OdeHelper.createMass();
		DMatrix3 R = new DMatrix3();

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace(null);

		contactgroup = OdeHelper.createJointGroup();
		world.setGravity (0,0,-9.8);
		world.setQuickStepNumIterations (64);

		// Create a static world using a triangle mesh that we can collide with.
		DTriMeshData Data = OdeHelper.createTriMeshData();

		//  fprintf(stderr,"Building Single Precision Mesh\n");

		Data.build( world_vertices, world_indices );
		Data.preprocess();
		world_mesh = OdeHelper.createTriMesh(space, Data, null, null, null);
		world_mesh.enableTC(DSphere.class, false);
		world_mesh.enableTC(DBox.class, false);
		world_mesh.setPosition(0, 0, 0.5);
		R.setIdentity();
		//dIASSERT(dVALIDMAT3(R));

		world_mesh.setRotation (R);

		//float sx=0.0, sy=3.40, sz=6.80;
		sphbody = OdeHelper.createBody(world);
		m.setSphere (1,RADIUS);
		sphbody.setMass (m);
		sphgeom = OdeHelper.createConvex (null,
				IcosahedronGeom.Sphere_planes,
				IcosahedronGeom.Sphere_planecount,
				IcosahedronGeom.Sphere_points,
				IcosahedronGeom.Sphere_pointcount,
				IcosahedronGeom.Sphere_polygons);
		sphgeom.setBody (sphbody);
		reset_ball();
		space.add (sphgeom);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		// Causes segm violation? Why?
		// (because dWorldDestroy() destroys body connected to geom; must call first!)
		sphgeom.destroy();
		world_mesh.destroy ();

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
