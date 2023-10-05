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
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSimpleSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceAdd;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceSetCleanup;
import static org.ode4j.cpp.internal.ApiCppExportDIF.dWorldExportDIF;
import static org.ode4j.cpp.internal.ApiCppJoint.*;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fclose;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fopen;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.cpp4j.FILE;


/**
 * buggy with suspension.
 * this also shows you how to use geom groups.
 */
class DemoBuggy extends dsFunctions {

	// some constants

	//#define LENGTH 0.7	// chassis length
	//#define WIDTH 0.5	// chassis width
	//#define HEIGHT 0.2	// chassis height
	//#define RADIUS 0.18	// wheel radius
	//#define STARTZ 0.5	// starting height of chassis
	//#define CMASS 1		// chassis mass
	//#define WMASS 0.2	// wheel mass

	private final DVector3 yunit = new DVector3( 0, 1, 0 ), zunit = new DVector3( 0, 0, 1 );

	private final double LENGTH = 0.7;	// chassis length
	private final double WIDTH = 0.5;	// chassis width
	private final double HEIGHT = 0.2;	// chassis height
	private final float RADIUS = 0.18f;	// wheel radius
	private final double STARTZ = 0.5;	// starting height of chassis
	private final double CMASS = 1;		// chassis mass
	private final double WMASS = 0.2;	// wheel mass


	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;
	private static DBody[] body = new DBody[4];
	private static DHinge2Joint[] joint = new DHinge2Joint[3];	// joint[0] is the front wheel
	private static DJointGroup contactgroup;
	private static DGeom ground;
	private static DSpace car_space;
	private static DGeom[] box = new DGeom[1];
	private static DGeom[] sphere = new DGeom[3];
	private static DBox ground_box;


	// things that the user controls

	private static double speed=0,steer=0;	// user commands



	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	static void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i,n;

		// only collide things with the ground
		boolean g1 = (o1 == ground || o1 == ground_box);
		boolean g2 = (o2 == ground || o2 == ground_box);
		if (!(g1 ^ g2)) return;

		final int N = 10;
		//dContact contact[N];
		DContactBuffer contacts = new DContactBuffer(N);
//		n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
		n = dCollide (o1,o2,N,contacts.getGeomBuffer());//[0].geom,sizeof(dContact));
		if (n > 0) {
			for (i=0; i<n; i++) {
				DContact contact = contacts.get(i);
				contact.surface.mode = dContactSlip1 | dContactSlip2 |
				dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact.surface.mu = dInfinity;
				contact.surface.slip1 = 0.1;
				contact.surface.slip2 = 0.1;
				contact.surface.soft_erp = 0.5;
				contact.surface.soft_cfm = 0.3;
				DJoint c = dJointCreateContact (world,contactgroup,contact);
				dJointAttach (c,
						dGeomGetBody(contact.geom.g1),
						dGeomGetBody(contact.geom.g2));
			}
		}
	}


	private static float[] xyz = {0.8317f,-0.9817f,0.8000f};
	private static float[] hpr = {121.0000f,-27.5000f,0.0000f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		//  static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
		//  static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
		dsSetViewpoint (xyz,hpr);
		printf ("Press:\t'a' to increase speed.\n" +
				"\t'z' to decrease speed.\n" +
				"\t',' to steer left.\n" +
				"\t'.' to steer right.\n" +
				"\t' ' to reset speed and steering.\n" +
		"\t'1' to save the current state to 'state.dif'.\n");
	}


	// called when a key pressed

	@Override
	public void command (char cmd)
	{
		switch (cmd) {
		case 'a': case 'A':
			speed += 0.3;
			break;
		case 'z': case 'Z':
			speed -= 0.3;
			break;
		case ',':
			steer -= 0.5;
			break;
		case '.':
			steer += 0.5;
			break;
		case ' ':
			speed = 0;
			steer = 0;
			break;
		case '1': {
			FILE f = fopen ("state.dif","wt");
			if (f!=null) {
				dWorldExportDIF (world,f,"");
				fclose (f);
			}
		}
		}
	}

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}};
	
	// simulation loop

	private void simLoop (boolean pause)
	{
		int i;
		if (!pause) {
			// motor
			dJointSetHinge2Param (joint[0],dParamVel2,-speed);
			dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

			// steering
			double v = steer - dJointGetHinge2Angle1 (joint[0]);
			if (v > 0.1) v = 0.1;
			if (v < -0.1) v = -0.1;
			v *= 10.0;
			dJointSetHinge2Param (joint[0],dParamVel,v);
			dJointSetHinge2Param (joint[0],dParamFMax,0.2);
			dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
			dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
			dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);

			dSpaceCollide (space,null,nearCallback);
			dWorldStep (world,0.05);

			// remove all contact joints
			dJointGroupEmpty (contactgroup);
		}

		dsSetColor (0,1,1);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		DVector3 sides = new DVector3(LENGTH,WIDTH,HEIGHT);
		dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
		dsSetColor (1,1,1);
		for (i=1; i<=3; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
				dBodyGetRotation(body[i]),0.02f,RADIUS);

		DVector3 ss = new DVector3();
		dGeomBoxGetLengths (ground_box,ss);
		dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);

		/*
  printf ("%.10f %.10f %.10f %.10f\n",
	  dJointGetHingeAngle (joint[1]),
	  dJointGetHingeAngle (joint[2]),
	  dJointGetHingeAngleRate (joint[1]),
	  dJointGetHingeAngleRate (joint[2]));
		 */
	}


	public static void main(String[] args) {
		new DemoBuggy().demo(args);
	}
	
	private void demo(String[] args) {
		int i;
		DMass m = dMassCreate();

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = this;
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
		dWorldSetGravity (world,0,0,-0.5);
		ground = dCreatePlane (space,0,0,1,0);

		// chassis body
		body[0] = dBodyCreate (world);
		dBodySetPosition (body[0],0,0,STARTZ);
		dMassSetBox (m,1,LENGTH,WIDTH,HEIGHT);
		dMassAdjust (m,CMASS);
		dBodySetMass (body[0],m);
		box[0] = dCreateBox (null,LENGTH,WIDTH,HEIGHT);
		dGeomSetBody (box[0],body[0]);

		// wheel bodies
		for (i=1; i<=3; i++) {
			body[i] = dBodyCreate (world);
			DQuaternion q = new DQuaternion();
			OdeMath.dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
			dBodySetQuaternion (body[i],q);
			dMassSetSphere (m,1,RADIUS);
			dMassAdjust (m,WMASS);
			dBodySetMass (body[i],m);
			sphere[i-1] = dCreateSphere (null,RADIUS);
			dGeomSetBody (sphere[i-1],body[i]);
		}
		dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
		dBodySetPosition (body[2],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
		dBodySetPosition (body[3],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);

		// front wheel hinge
		/*
  joint[0] = dJointCreateHinge2 (world,0);
  dJointAttach (joint[0],body[0],body[1]);
  const dReal *a = dBodyGetPosition (body[1]);
  dJointSetHinge2Anchor (joint[0],a[0],a[1],a[2]);
  dJointSetHinge2Axis1 (joint[0],0,0,1);
  dJointSetHinge2Axis2 (joint[0],0,1,0);
		 */

		// front and back wheel hinges
		for (i=0; i<3; i++) {
			joint[i] = dJointCreateHinge2 (world,null);
			dJointAttach (joint[i],body[0],body[i+1]);
			final DVector3C a = dBodyGetPosition (body[i+1]);
			dJointSetHinge2Anchor (joint[i],a.get0(),a.get1(),a.get2());
			dJointSetHinge2Axes (joint[i], zunit, yunit);
		}

		// set joint suspension
		for (i=0; i<3; i++) {
			dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
			dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
		}

		// lock back wheels along the steering axis
		for (i=1; i<3; i++) {
			// set stops to make sure wheels always stay in alignment
			dJointSetHinge2Param (joint[i],dParamLoStop,0);
			dJointSetHinge2Param (joint[i],dParamHiStop,0);
			// the following alternative method is no good as the wheels may get out
			// of alignment:
			//   dJointSetHinge2Param (joint[i],dParamVel,0);
			//   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
		}

		// create car space and add it to the top level space
		car_space = dSimpleSpaceCreate (space);
		dSpaceSetCleanup (car_space,false);
		dSpaceAdd (car_space,box[0]);
		dSpaceAdd (car_space,sphere[0]);
		dSpaceAdd (car_space,sphere[1]);
		dSpaceAdd (car_space,sphere[2]);

		// environment
		ground_box = dCreateBox (space,2,1.5,1);
		DMatrix3 R = new DMatrix3();
		OdeMath.dRFromAxisAndAngle (R,0,1,0,-0.15);
		dGeomSetPosition (ground_box,2,0,-0.34);
		dGeomSetRotation (ground_box,R);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dGeomDestroy (box[0]);
		dGeomDestroy (sphere[0]);
		dGeomDestroy (sphere[1]);
		dGeomDestroy (sphere[2]);
		dJointGroupDestroy (contactgroup);
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