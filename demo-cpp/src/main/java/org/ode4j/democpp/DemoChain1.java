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

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddForce;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateBall;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetBallAnchor;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.internal.cpp4j.Cmath.sin;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.ode.DBallJoint;
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


class DemoChain1 extends dsFunctions {

	/* some constants */

	//#define NUM 10			/* number of boxes */
	//#define SIDE (0.2)		/* side length of a box */
	//#define MASS (1.0)		/* mass of a box */
	//#define RADIUS (0.1732f)	/* sphere radius */
	private static int NUM = 10;			/* number of boxes */
	private static double SIDE = (0.2);		/* side length of a box */
	private static double MASS = (1.0);		/* mass of a box */
	private static double RADIUS = (0.1732f);	/* sphere radius */


	/* dynamics and collision objects */

	private static DWorld world;
	private static DSpace space;
	private static DBody[] body = new DBody[NUM];
	private static DBallJoint[] joint = new DBallJoint[NUM-1];
	private static DJointGroup contactgroup;
	private static DGeom[] sphere=new DGeom[NUM];


	private DNearCallback myNearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};

	/** 
	 * this is called by dSpaceCollide when two objects in space are
	 * potentially colliding.
	 */
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		/* exit without doing anything if the two bodies are connected by a joint */
		DBody b1,b2;
		//dContact contact;
		DContactBuffer contacts = new DContactBuffer(1);
		DContact contact = contacts.get(0);

		b1 = dGeomGetBody(o1);
		b2 = dGeomGetBody(o2);
//		if (b1!=null && b2!=null && dAreConnected (b1,b2)) return;

		contact.surface.mode = 0;
		contact.surface.mu = 0.1;
		contact.surface.mu2 = 0;
		if (0!=dCollide (o1,o2,1,contacts.getGeomBuffer())) {//&contact.geom,sizeof(dContactGeom))) {
			DJoint c = dJointCreateContact (world,contactgroup,contact);
			dJointAttach (c,b1,b2);
		}
	}


	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};
	/**
	 *  start simulation - set viewpoint 
	 */
	@Override
	public void start()
	{

		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);
		dsSetViewpoint (xyz,hpr);
	}


	private static double angle = 0;
	/* simulation loop */
	public void simLoop (boolean pause)
	{
		int i;
		if (!pause) {
			//static double angle = 0;
			angle += 0.05;
			dBodyAddForce (body[NUM-1],0,0,1.5*(sin(angle)+1.0));

			dSpaceCollide (space,0,myNearCallback);
			dWorldStep (world,0.05);

			/* remove all contact joints */
			dJointGroupEmpty (contactgroup);
		}

		dsSetColor (1,1,0);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (i=0; i<NUM; i++) dsDrawSphere (dBodyGetPosition(body[i]),
				dBodyGetRotation(body[i]),RADIUS);
	}


	public static void main(String[] args) {
		new DemoChain1().demo(args);
	}
	
	private void demo(String[] args) {
		int i;
		double k;
		DMass m;

		/* setup pointers to drawstuff callback functions */
		//dsFunctions fn = new DemoChain1();
		//fn.version = DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = 0;
		//  fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		/* create world */
		dInitODE2(0);
		world = dWorldCreate();
		space = dHashSpaceCreate (null);
		contactgroup = dJointGroupCreate (1000000);
		dWorldSetGravity (world,0,0,-0.5);
		dCreatePlane (space,0,0,1,0);

		//TZ
		m = dMassCreate();
		for (i=0; i<NUM; i++) {
			body[i] = dBodyCreate (world);
			k = i*SIDE;
			dBodySetPosition (body[i],k,k,k+0.4);
			dMassSetBox (m,1,SIDE,SIDE,SIDE);
			dMassAdjust (m,MASS);
			dBodySetMass (body[i],m);
			sphere[i] = dCreateSphere (space,RADIUS);
			dGeomSetBody (sphere[i],body[i]);
		}
		for (i=0; i<(NUM-1); i++) {
			joint[i] = dJointCreateBall (world,null);
			dJointAttach (joint[i],body[i],body[i+1]);
			k = (i+0.5)*SIDE;
			dJointSetBallAnchor (joint[i],k,k,k+0.4);
		}

		/* run simulation */
		dsSimulationLoop (args,640,480,this);

		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
		dCloseODE();
	}

	@Override
	public void command(char cmd) {
		// Nothing
	}

	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}

	@Override
	public void stop() {
		//Nothing
	}
}