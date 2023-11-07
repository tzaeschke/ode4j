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
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetQuaternion;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCylinder;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomIsSpace;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide2;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceAdd;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCylinder;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetQuickStepNumIterations;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawLine;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColorAlpha;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DRotation.dQFromAxisAndAngle;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fprintf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stderr;

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
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;


/**
 * Test for cylinder vs sphere, by Bram Stolk.
 */
class DemoCylVsSphere extends dsFunctions {


	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;

	private static DBody cylbody;
	private static DGeom cylgeom;

	private static DBody sphbody;
	private static DGeom sphgeom;

	private static DJointGroup contactgroup;

	private static boolean show_contacts = true;

	//#define CYLRADIUS    0.6
	//#define CYLLENGTH    2.0
	//#define SPHERERADIUS 0.5
	private static final float CYLRADIUS = 0.6f;
	private static final float CYLLENGTH = 2.0f;
	private static final float SPHERERADIUS = 0.5f;

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

		final int N = 32;
		DContactBuffer contacts = new DContactBuffer(N);
		int n = dCollide (o1,o2,N,contacts.getGeomBuffer());
		if (n > 0) 
		{
			for (int i=0; i<n; i++) 
			{
				DContact contact = contacts.get(i);
				contact.surface.mode = 0;
				contact.surface.mu = 50.0; // was: dInfinity
				DJoint c = dJointCreateContact (world,contactgroup,contact);
				dJointAttach (c, dGeomGetBody(contact.geom.g1), dGeomGetBody(contact.geom.g2));
				if (show_contacts) 
				{
					DMatrix3 RI = new DMatrix3();
					RI.setIdentity();
					final DVector3 ss = new DVector3(0.12,0.12,0.12);
					dsSetColorAlpha (0f,0f,1f,0.5f);
					dsDrawBox (contact.geom.pos,RI,ss);
					DVector3 pos  = contact.geom.pos;
					double depth = contact.geom.depth;
					DVector3 norm = contact.geom.normal;
					//double endp[3] = {pos[0]+depth*norm[0], pos[1]+depth*norm[1], pos[2]+depth*norm[2]};
					DVector3 endp = new DVector3();
					endp.eqSum(pos, norm, depth);
					dsSetColorAlpha (1,1,1,1);
					dsDrawLine (contact.geom.pos, endp);
				}
			}
		}
	}


	static float[] xyz = {-8,-9,3};
	static float[] hpr = {45.0000f,-27.5000f,0.0000f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
	}


	// called when a key pressed

	@Override
	public void command (char cmd)
	{
		switch (cmd) 
		{
		case ' ':
			break;
		}
	}



	// simulation loop

	private void simLoop (boolean pause)
	{
		dSpaceCollide (space,0,nearCallback);
		if (!pause)
		{
			dWorldQuickStep (world, 0.01); // 100 Hz
		}
		dJointGroupEmpty (contactgroup);

		dsSetColorAlpha (1f,1f,0f,0.5f);

		final DVector3C CPos = dBodyGetPosition(cylbody);
		final DMatrix3C CRot = dBodyGetRotation(cylbody);
		//float cpos[3] = {CPos[0], CPos[1], CPos[2]};
		//float crot[12] = { CRot[0], CRot[1], CRot[2], CRot[3], CRot[4], CRot[5], CRot[6], CRot[7], CRot[8], CRot[9], CRot[10], CRot[11] };
		
		dsDrawCylinder
		(
				CPos,
				CRot,
				CYLLENGTH,
				CYLRADIUS
		); // single precision

		final DVector3C SPos = dBodyGetPosition(sphbody);
		final DMatrix3C SRot = dBodyGetRotation(sphbody);
		//float spos[3] = {SPos[0], SPos[1], SPos[2]};
		//float srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };
		dsDrawSphere
		(
				SPos,
				SRot,
				SPHERERADIUS
		); // single precision
	}


	public static void main(String[] args) {
		new DemoCylVsSphere().demo(args);
	}
	
	@SuppressWarnings("unused")
	private void demo(String[] args) {
		DMass m = OdeHelper.createMass();

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = new DemoCylVsSphere();
		//  fn.version = DS_VERSION;
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
		dWorldSetQuickStepNumIterations (world, 32);

		dCreatePlane (space,0,0,1, 0.0);

		cylbody = dBodyCreate (world);
		DQuaternion q = new DQuaternion();
		if (false) {//#if 0
			dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
		} else { //#else
			//  dQFromAxisAndAngle (q,1,0,0, M_PI * 1.0);
			dQFromAxisAndAngle (q,1,0,0, M_PI * -0.77);
		} //#endif
		dBodySetQuaternion (cylbody,q);
		dMassSetCylinder (m,1.0,3,CYLRADIUS,CYLLENGTH);
		dBodySetMass (cylbody,m);
		cylgeom = dCreateCylinder(null, CYLRADIUS, CYLLENGTH);
		dGeomSetBody (cylgeom,cylbody);
		dBodySetPosition (cylbody, 0, 0, 3);
		dSpaceAdd (space, cylgeom);

		sphbody = dBodyCreate (world);
		dMassSetSphere (m,1,SPHERERADIUS);
		dBodySetMass (sphbody,m);
		sphgeom = dCreateSphere(null, SPHERERADIUS);
		dGeomSetBody (sphgeom,sphbody);
		dBodySetPosition (sphbody, 0, 0, 5.5);
		dSpaceAdd (space, sphgeom);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dJointGroupEmpty (contactgroup);
		dJointGroupDestroy (contactgroup);

		dGeomDestroy(sphgeom);
		dGeomDestroy (cylgeom);

		dSpaceDestroy (space);
		dWorldDestroy (world);
		dCloseODE();
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

