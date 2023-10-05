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
package org.ode4j.democpp;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetLinearVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetQuaternion;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCylinder;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomIsSpace;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetOffsetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide2;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceAdd;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dCreateTriMesh;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dGeomTriMeshDataBuildSingle;
import static org.ode4j.cpp.internal.ApiCppCollisionTrimesh.dGeomTriMeshDataCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetQuickStepNumIterations;
import static org.ode4j.democpp.WorldGeom3.world_indices;
import static org.ode4j.democpp.WorldGeom3.world_vertices;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawTriangle;
import static org.ode4j.drawstuff.DrawStuff.dsElapsedTime;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DRotation.dQFromAxisAndAngle;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.cpp4j.Cmath.ceilf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fprintf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stderr;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;


/**
 * Test for non-capped cylinder, by Bram Stolk.
 */
class DemoCyl extends dsFunctions {


	//#define BOX
	//#define CYL
	private static final boolean BOX = true;
	private static final boolean CYL = true;

	// some constants

	//#define RADIUS 0.22	// wheel radius
	//#define WMASS 0.2	// wheel mass
	//#define WHEELW 0.2	// wheel width
	//#define BOXSZ 0.4	// box size
	private static final float RADIUS = 0.22f;	// wheel radius
	private static final float WMASS = 0.2f;	// wheel mass
	private static final float WHEELW = 0.2f;	// wheel width
	private static final float BOXSZ = 0.4f;	// box size
	////#define CYL_GEOM_OFFSET // rotate cylinder using geom offset
	private static boolean CYL_GEOM_OFFSET = false; 

	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;
	//#ifdef BOX
	private static DBody boxbody;
	private static DGeom boxgeom;
	//#endif
	//#ifdef CYL
	private static DBody cylbody;
	private static DGeom cylgeom;
	//#endif
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

		if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
		{
			fprintf(stderr,"testing space %s %s\n", o1,o2);
			// colliding a space with something
			dSpaceCollide2(o1,o2,data,nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		//  fprintf(stderr,"testing geoms %p %p\n", o1, o2);

		final int N = 32;
		DContactBuffer contacts = new DContactBuffer(N);
		int n = dCollide (o1,o2,N,contacts.getGeomBuffer());//[0].geom),sizeof(dContact));
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
				DJoint c = dJointCreateContact (world,contactgroup,contact);
				dJointAttach (c,
						dGeomGetBody(contact.geom.g1),
						dGeomGetBody(contact.geom.g2));
			}
		}
	}


	private static float[] xyz = {-8,-9,3};
	private static float[] hpr = {45.0000f,-27.5000f,0.0000f};

	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		//  static float xyz[3] = {-8,-9,3};
		//  static float hpr[3] = {45.0000f,-27.5000f,0.0000f};
		dsSetViewpoint (xyz,hpr);
	}



	private void reset_state()
	{
		float sx=-4, sy=-4, sz=2;
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
		if (BOX) {//#ifdef BOX
			dBodySetPosition (boxbody, sx, sy+1, sz);
			dBodySetLinearVel (boxbody, 0,0,0);
			dBodySetAngularVel (boxbody, 0,0,0);
			dBodySetQuaternion (boxbody, q);
		}//#endif
		if (CYL) {//#ifdef CYL
			dBodySetPosition (cylbody, sx, sy, sz);
			dBodySetLinearVel (cylbody, 0,0,0);
			dBodySetAngularVel (cylbody, 0,0,0);
			dBodySetQuaternion (cylbody, q);
		}//#endif
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
		int nrofsteps = (int) ceilf(dt/simstep);
		for (int i=0; i<nrofsteps && !pause; i++)
		{
			dSpaceCollide (space,0,nearCallback);
			dWorldQuickStep (world, simstep);
			dJointGroupEmpty (contactgroup);
		}

		dsSetColor (1,1,1);
		if (BOX) {//#ifdef BOX
			final DVector3C BPos = dBodyGetPosition(boxbody);
			final DMatrix3C BRot = dBodyGetRotation(boxbody);
			//		float[] bpos = {BPos[0], BPos[1], BPos[2]};
			//		float[] brot = { BRot[0], BRot[1], BRot[2], BRot[3], BRot[4], BRot[5], BRot[6], BRot[7], BRot[8], BRot[9], BRot[10], BRot[11] };
			//float[] sides = {BOXSZ, BOXSZ, BOXSZ};
			DVector3 sides = new DVector3(BOXSZ, BOXSZ, BOXSZ);
			dsDrawBox
			(
					BPos, 
					BRot, 
					sides
			); // single precision
		}//#endif
		if (CYL) {//#ifdef CYL
			final DVector3C CPos = dGeomGetPosition(cylgeom);
			final DMatrix3C CRot = dGeomGetRotation(cylgeom);
			//		float[] cpos = {CPos[0], CPos[1], CPos[2]};
			//		float[] crot = { CRot[0], CRot[1], CRot[2], CRot[3], CRot[4], CRot[5], CRot[6], CRot[7], CRot[8], CRot[9], CRot[10], CRot[11] };
			dsDrawCylinder
			( 
					//    dBodyGetPosition(cylbody),
					//    dBodyGetRotation(cylbody),
					CPos,
					CRot,
					WHEELW,
					RADIUS
			); // single precision
		}//#endif

		// draw world trimesh
		dsSetColor(0.7f,0.7f,0.4f);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_NONE);

		final DVector3C Pos = dGeomGetPosition(world_mesh);
		//float pos[3] = { Pos[0], Pos[1], Pos[2] };

		final DMatrix3C Rot = dGeomGetRotation(world_mesh);
		//float rot[12] = { Rot[0], Rot[1], Rot[2], Rot[3], Rot[4], Rot[5], Rot[6], Rot[7], Rot[8], Rot[9], Rot[10], Rot[11] };

		int numi = world_indices.length;//sizeof(world_indices)  / sizeof(dTriIndex);

		for (int i=0; i<numi/3; i++)
		{
			int i0 = world_indices[i*3+0] * 3;
			int i1 = world_indices[i*3+1] * 3;
			int i2 = world_indices[i*3+2] * 3;
			//			float *v0 = world_vertices+i0*3;
			//			float *v1 = world_vertices+i1*3;
			//			float *v2 = world_vertices+i2*3;
			dsDrawTriangle(Pos, Rot, world_vertices, i0, i1, i2, true); // single precision draw
		}
	}


	public static void main(String[] args) {
		new DemoCyl().demo(args);
	}

	private void demo(String [] args) {
		DMass m = OdeHelper.createMass();
		DMatrix3 R = new DMatrix3();

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = new DemoCyl();
		//fn.version = DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = &command;
		//  fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2(0);
		world = dWorldCreate();
		space = dHashSpaceCreate (null);
		contactgroup = dJointGroupCreate (0);
		dWorldSetGravity (world,0,0,-9.8);
		dWorldSetQuickStepNumIterations (world, 12);


		// Create a static world using a triangle mesh that we can collide with.
		int numv = world_vertices.length;//sizeof(world_vertices)/(3*sizeof(float));
		int numi = world_indices.length;//sizeof(world_indices)/ sizeof(dTriIndex);
		printf("numv=%d, numi=%d\n", numv, numi);
		DTriMeshData Data = dGeomTriMeshDataCreate();

		dGeomTriMeshDataBuildSingle
		(
				Data, 
				world_vertices, 
				3,// * sizeof(float), 
				numv, 
				world_indices, 
				numi, 
				3// * sizeof(dTriIndex)
		);

		world_mesh = dCreateTriMesh(space, Data, null, null, null);
		dGeomSetPosition(world_mesh, 0, 0, 0.5);
		dRFromAxisAndAngle (R, 0,1,0, 0.0);
		dGeomSetRotation (world_mesh, R);


		if (BOX) {//#ifdef BOX
			boxbody = dBodyCreate (world);
			dMassSetBox (m,1, BOXSZ, BOXSZ, BOXSZ);
			dMassAdjust (m, 1);
			dBodySetMass (boxbody,m);
			boxgeom = dCreateBox (null, BOXSZ, BOXSZ, BOXSZ);
			dGeomSetBody (boxgeom,boxbody);
			dSpaceAdd (space, boxgeom);
		}//#endif
		if (CYL) {//#ifdef CYL
			cylbody = dBodyCreate (world);
			dMassSetSphere (m,1,RADIUS);
			dMassAdjust (m,WMASS);
			dBodySetMass (cylbody,m);
			cylgeom = dCreateCylinder(null, RADIUS, WHEELW);
			dGeomSetBody (cylgeom,cylbody);

			if (CYL_GEOM_OFFSET) {//#if defined(CYL_GEOM_OFFSET)
				DMatrix3 mat = new DMatrix3();
				dRFromAxisAndAngle(mat,1.0f,0.0f,0.0f,M_PI/2.0);
				dGeomSetOffsetRotation(cylgeom,mat);
			} //#endif

			dSpaceAdd (space, cylgeom);
		}//#endif
		reset_state();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dJointGroupEmpty (contactgroup);
		dJointGroupDestroy (contactgroup);

		// First destroy geoms, then space, then the world.
		if (CYL) {//#ifdef CYL
			dGeomDestroy (cylgeom);
		}//#endif
		if (BOX) {//#ifdef BOX
			dGeomDestroy (boxgeom);
		}//#endif
		dGeomDestroy (world_mesh);

		dSpaceDestroy (space);
		dWorldDestroy (world);
		dCloseODE();
		//(void)world_normals; // get rid of compiler warnings
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


