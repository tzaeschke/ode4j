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
import static org.ode4j.cpp.internal.ApiCppBody.dBodyDisable;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyEnable;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetLinearVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyIsEnabled;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetLinearVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetQuaternion;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetClass;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSweepAndPruneSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.*;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphereTotal;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppOther.dAreConnected;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetCFM;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetERP;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetQuickStepNumIterations;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DMatrix.dMultiply0;
import static org.ode4j.ode.DRotation.dQFromAxisAndAngle;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.ErrorHandler.dMessage;
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
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSapSpace.AXES;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.cpp4j.java.RefInt;


/**
 * This is a demo of the QuickStep and StepFast methods,
 * originally by David Whittaker.
 */
class DemoCrash extends dsFunctions {

	// some constants

	private static final float LENGTH = 3.5f;		// chassis length
	private static final float WIDTH = 2.5f;		// chassis width
	private static final float HEIGHT = 1.0f;		// chassis height
	private static final float RADIUS = 0.5f;	// wheel radius
	private static final float STARTZ = 1.0f;	// starting height of chassis
	private static final float CMASS = 1;			// chassis mass
	private static final float WMASS = 1;			// wheel mass
	private static final float COMOFFSET = -5;		// center of mass offset
	private static final float WALLMASS = 1	;	// wall box mass
	private static final float BALLMASS = 1;		// ball mass
	private static final float FMAX = 25;			// car engine fmax
	private static final float ROWS = 1	;		// rows of cars
	private static final float COLS = 1	;		// columns of cars
	private static final int ITERS = 20;		// number of iterations
	private static final float WBOXSIZE = 1.0f;		// size of wall boxes
	private static final float WALLWIDTH = 12;		// width of wall
	private static final float WALLHEIGHT = 10;		// height of wall
	private static final float DISABLE_THRESHOLD = 0.008f;	// maximum velocity (squared) a body can have and be disabled
	private static final float DISABLE_STEPS = 10;	// number of steps a box has to have been disable-able before it will be disabled
	private static final float CANNON_X = -10;		// x position of cannon
	private static final float CANNON_Y = 5	;	// y position of cannon
	private static final float CANNON_BALL_MASS = 10;	// mass of the cannon ball
	private static final float CANNON_BALL_RADIUS = 0.5f;


	private final DVector3 xunit = new DVector3( 1, 0, 0 ), yunit = new DVector3( 0, 1, 0 ),
			zpunit = new DVector3( 0, 0, 1 ), zmunit = new DVector3( 0, 0, -1 );

	////#define BOX
	//#define CARS
	//#define WALL
	////#define BALLS
	////#define BALLSTACK
	////#define ONEBALL
	////#define CENTIPEDE
	//#define CANNON
	private static boolean BOX = false;
	private static boolean CARS = true;
	private static boolean WALL = true;
	private static boolean BALLS = false;
	private static boolean BALLSTACK = false;
	private static boolean ONEBALL = false;
	private static boolean CENTIPEDE = false;
	private static boolean CANNON = true;

	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;
	private static DBody[] body=new DBody[10000];
	private static int bodies;
	private static DHinge2Joint[] joint=new DHinge2Joint[100000];
	private static int joints;
	private static DJointGroup contactgroup;
	//private static DGeom ground;
	private static DGeom[] box=new DGeom[10000];
	private static int boxes;
	private static DGeom[] sphere=new DGeom[10000];
	private static int spheres;
	private static DBox[] wall_boxes=new DBox[10000];
	private static DBody[] wall_bodies=new DBody[10000];
	private static DGeom cannon_ball_geom;
	private static DBody cannon_ball_body;
	private static int[] wb_stepsdis=new int[10000];
	private static int wb;
	private static boolean doFast;
	private static DBody b;
	private static DMass m;


	// things that the user controls

	private static float turn = 0, speed = 0;	// user commands
	private static float cannon_angle=0,cannon_elevation=-1.2f;


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
		int i,n;

		DBody b1 = dGeomGetBody(o1);
		DBody b2 = dGeomGetBody(o2);
		if (b1!=null && b2!=null && dAreConnected(b1, b2))
			return;

		final int N = 4;
		DContactBuffer contacts = new DContactBuffer(N);
		n = dCollide (o1,o2,N,contacts.getGeomBuffer());//[0].geom,sizeof(dContact));
		if (n > 0) {
			for (i=0; i<n; i++) {
				DContact contact = contacts.get(i);
				contact.surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
				if (dGeomGetClass(o1) == DGeom.dSphereClass || dGeomGetClass(o2) == DGeom.dSphereClass)
					contact.surface.mu = 20;
				else
					contact.surface.mu = 0.5;
				contact.surface.slip1 = 0.0;
				contact.surface.slip2 = 0.0;
				contact.surface.soft_erp = 0.8;
				contact.surface.soft_cfm = 0.01;
				DJoint c = dJointCreateContact (world,contactgroup,contact);
				dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
			}
		}
	}


	private static float[] xyz = {3.8548f,9.0843f,7.5900f};
	private static float[] hpr = {-145.5f,-22.5f,0.25f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		printf ("Press:\t'a' to increase speed.\n" +
				"\t'z' to decrease speed.\n" +
				"\t',' to steer left.\n" +
				"\t'.' to steer right.\n" +
				"\t' ' to reset speed and steering.\n" +
				"\t'[' to turn the cannon left.\n" +
				"\t']' to turn the cannon right.\n" +
				"\t'1' to raise the cannon.\n" +
				"\t'2' to lower the cannon.\n" +
				"\t'x' to shoot from the cannon.\n" +
				"\t'f' to toggle fast step mode.\n" +
		"\t'r' to reset simulation.\n");
	}


	//private void makeCar(double x, double y, int &bodyI, int &jointI, int &boxI, int &sphereI)
	private void makeCar(double x, double y, RefInt bodyIr, RefInt jointIr, RefInt boxIr, RefInt sphereIr)
	{
		final int bodyI = bodyIr.get();
		final int jointI = jointIr.get();
		final int boxI = boxIr.get();
		final int sphereI = sphereIr.get();
		int i;
		DMass m = OdeHelper.createMass();

		// chassis body
		body[bodyI] = dBodyCreate (world);
		dBodySetPosition (body[bodyI],x,y,STARTZ);
		dMassSetBox (m,1,LENGTH,WIDTH,HEIGHT);
		dMassAdjust (m,CMASS/2.0);
		dBodySetMass (body[bodyI],m);
		box[boxI] = dCreateBox (space,LENGTH,WIDTH,HEIGHT);
		dGeomSetBody (box[boxI],body[bodyI]);

		// wheel bodies
		for (i=1; i<=4; i++) {
			body[bodyI+i] = dBodyCreate (world);
			DQuaternion q = new DQuaternion();
			dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
			dBodySetQuaternion (body[bodyI+i],q);
			dMassSetSphere (m,1,RADIUS);
			dMassAdjust (m,WMASS);
			dBodySetMass (body[bodyI+i],m);
			sphere[sphereI+i-1] = dCreateSphere (space,RADIUS);
			dGeomSetBody (sphere[sphereI+i-1],body[bodyI+i]);
		}
		dBodySetPosition (body[bodyI+1],x+0.4*LENGTH-0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
		dBodySetPosition (body[bodyI+2],x+0.4*LENGTH-0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
		dBodySetPosition (body[bodyI+3],x-0.4*LENGTH+0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
		dBodySetPosition (body[bodyI+4],x-0.4*LENGTH+0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);

		// front and back wheel hinges
		for (i=0; i<4; i++) {
			joint[jointI+i] = dJointCreateHinge2 (world,null);
			dJointAttach (joint[jointI+i],body[bodyI],body[bodyI+i+1]);
			final DVector3C a = dBodyGetPosition (body[bodyI+i+1]);
			dJointSetHinge2Anchor (joint[jointI+i],a.get0(),a.get1(),a.get2());
			dJointSetHinge2Axes (joint[jointI+i],(i<2 ? zpunit : zmunit), yunit);
			dJointSetHinge2Param (joint[jointI+i],dParamSuspensionERP,0.8);
			dJointSetHinge2Param (joint[jointI+i],dParamSuspensionCFM,1e-5);
			dJointSetHinge2Param (joint[jointI+i],dParamVel2,0);
			dJointSetHinge2Param (joint[jointI+i],dParamFMax2,FMAX);
		}

		//center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint)
		DBody b = dBodyCreate (world);
		dBodySetPosition (b,x,y,STARTZ+COMOFFSET);
		dMassSetBox (m,1,LENGTH,WIDTH,HEIGHT);
		dMassAdjust (m,CMASS/2.0);
		dBodySetMass (b,m);
		DFixedJoint j = dJointCreateFixed(world, null);
		dJointAttach(j, body[bodyI], b);
		dJointSetFixed(j);
		//box[boxI+1] = dCreateBox(space,LENGTH,WIDTH,HEIGHT);
		//dGeomSetBody (box[boxI+1],b);

		bodyIr.add(5);
		jointIr.add(4);
		boxIr.add(1);
		sphereIr.add(4);
	}


	private void resetSimulation()
	{
		int i;
		i = 0;
		// destroy world if it exists
		if (bodies!=0)
		{
			dJointGroupDestroy (contactgroup);
			dSpaceDestroy (space);
			dWorldDestroy (world);
		}

		for (i = 0; i < 1000; i++)
			wb_stepsdis[i] = 0;

		// recreate world

		world = dWorldCreate();

		//  space = dHashSpaceCreate( null );
		//	space = dSimpleSpaceCreate( null );
		space = dSweepAndPruneSpaceCreate( null, AXES.XYZ );//dSAP_AXES_XYZ );
		
		m = OdeHelper.createMass();

		contactgroup = dJointGroupCreate (0);
		dWorldSetGravity (world,0,0,-1.5);
		dWorldSetCFM (world, 1e-5);
		dWorldSetERP (world, 0.8);
		dWorldSetQuickStepNumIterations (world,ITERS);
		dCreatePlane (space,0,0,1,0);

		bodies = 0;
		joints = 0;
		boxes = 0;
		spheres = 0;
		wb = 0;
		RefInt rBodies = new RefInt();
		RefInt rJoints = new RefInt();
		RefInt rBoxes = new RefInt();
		RefInt rSpheres = new RefInt();
		if (CARS) {//#ifdef CARS
			for (double x = 0.0; x < COLS*(LENGTH+RADIUS); x += LENGTH+RADIUS)
				for (double y = -((ROWS-1)*(WIDTH/2+RADIUS)); y <= ((ROWS-1)*(WIDTH/2+RADIUS)); y += WIDTH+RADIUS*2)
					makeCar(x, y, rBodies, rJoints, rBoxes, rSpheres);
			bodies = rBodies.get();
			joints = rJoints.get();
			boxes = rBoxes.get();
			spheres = rSpheres.get();
		}//#endif
		if (WALL) {//#ifdef WALL
			boolean offset = false;
			for (double z = WBOXSIZE/2.0; z <= WALLHEIGHT; z+=WBOXSIZE)
			{
				offset = !offset;
				for (double y = (-WALLWIDTH+z)/2; y <= (WALLWIDTH-z)/2; y+=WBOXSIZE)
				{
					wall_bodies[wb] = dBodyCreate (world);
					dBodySetPosition (wall_bodies[wb],-20,y,z);
					dMassSetBox (m,1,WBOXSIZE,WBOXSIZE,WBOXSIZE);
					dMassAdjust (m, WALLMASS);
					dBodySetMass (wall_bodies[wb],m);
					wall_boxes[wb] = dCreateBox (space,WBOXSIZE,WBOXSIZE,WBOXSIZE);
					dGeomSetBody (wall_boxes[wb],wall_bodies[wb]);
					//dBodyDisable(wall_bodies[wb++]);
					wb++;
				}
			}
			dMessage(0,"wall boxes: %d", wb);
		}//#endif
		if (BALLS) {//#ifdef BALLS
			for (double x = -7; x <= -4; x+=1)
				for (double y = -1.5; y <= 1.5; y+=1)
					for (double z = 1; z <= 4; z+=1)
					{
						b = dBodyCreate (world);
						dBodySetPosition (b,x*RADIUS*2,y*RADIUS*2,z*RADIUS*2);
						dMassSetSphere (m,1,RADIUS);
						dMassAdjust (m, BALLMASS);
						dBodySetMass (b,m);
						sphere[spheres] = dCreateSphere (space,RADIUS);
						dGeomSetBody (sphere[spheres++],b);
					}
		}//#endif
		if (ONEBALL) {//#ifdef ONEBALL
			b = dBodyCreate (world);
			dBodySetPosition (b,0,0,2);
			dMassSetSphere (m,1,RADIUS);
			dMassAdjust (m, 1);
			dBodySetMass (b,m);
			sphere[spheres] = dCreateSphere (space,RADIUS);
			dGeomSetBody (sphere[spheres++],b);
		}//#endif
		if (BALLSTACK) {//#ifdef BALLSTACK
			for (double z = 1; z <= 6; z+=1)
			{
				b = dBodyCreate (world);
				dBodySetPosition (b,0,0,z*RADIUS*2);
				dMassSetSphere (m,1,RADIUS);
				dMassAdjust (m, 0.1);
				dBodySetMass (b,m);
				sphere[spheres] = dCreateSphere (space,RADIUS);
				dGeomSetBody (sphere[spheres++],b);
			}
		}//#endif
		if (CENTIPEDE) {//#ifdef CENTIPEDE
			DBody lastb = null;
			for (double y = 0; y < 10*LENGTH; y+=LENGTH+0.1)
			{
				// chassis body

				b = body[bodies] = dBodyCreate (world);
				dBodySetPosition (body[bodies],-15,y,STARTZ);
				dMassSetBox (m,1,WIDTH,LENGTH,HEIGHT);
				dMassAdjust (m,CMASS);
				dBodySetMass (body[bodies],m);
				box[boxes] = dCreateBox (space,WIDTH,LENGTH,HEIGHT);
				dGeomSetBody (box[boxes++],body[bodies++]);

				for (double x = -17; x > -20; x-=RADIUS*2)
				{
					body[bodies] = dBodyCreate (world);
					dBodySetPosition(body[bodies], x, y, STARTZ);
					dMassSetSphere(m, 1, RADIUS);
					dMassAdjust(m, WMASS);
					dBodySetMass(body[bodies], m);
					sphere[spheres] = dCreateSphere (space, RADIUS);
					dGeomSetBody (sphere[spheres++], body[bodies]);

					joint[joints] = dJointCreateHinge2 (world,null);
					if (x == -17)
						dJointAttach (joint[joints],b,body[bodies]);
					else
						dJointAttach (joint[joints],body[bodies-2],body[bodies]);
					final DVector3C a = dBodyGetPosition (body[bodies++]);
					dJointSetHinge2Anchor (joint[joints],a.get0(),a.get1(),a.get2());
					dJointSetHinge2Axes (joint[joints],zpunit, xunit);
					dJointSetHinge2Param (joint[joints],dParamSuspensionERP,1.0);
					dJointSetHinge2Param (joint[joints],dParamSuspensionCFM,1e-5);
					dJointSetHinge2Param (joint[joints],dParamLoStop,0);
					dJointSetHinge2Param (joint[joints],dParamHiStop,0);
					dJointSetHinge2Param (joint[joints],dParamVel2,-10.0);
					dJointSetHinge2Param (joint[joints++],dParamFMax2,FMAX);

					body[bodies] = dBodyCreate (world);
					dBodySetPosition(body[bodies], -30 - x, y, STARTZ);
					dMassSetSphere(m, 1, RADIUS);
					dMassAdjust(m, WMASS);
					dBodySetMass(body[bodies], m);
					sphere[spheres] = dCreateSphere (space, RADIUS);
					dGeomSetBody (sphere[spheres++], body[bodies]);

					joint[joints] = dJointCreateHinge2 (world,null);
					if (x == -17)
						dJointAttach (joint[joints],b,body[bodies]);
					else
						dJointAttach (joint[joints],body[bodies-2],body[bodies]);
					final DVector3C b = dBodyGetPosition (body[bodies++]);
					dJointSetHinge2Anchor (joint[joints],b.get0(),b.get1(),b.get2());
					dJointSetHinge2Axes (joint[joints],zpunit, xunit);
					dJointSetHinge2Param (joint[joints],dParamSuspensionERP,1.0);
					dJointSetHinge2Param (joint[joints],dParamSuspensionCFM,1e-5);
					dJointSetHinge2Param (joint[joints],dParamLoStop,0);
					dJointSetHinge2Param (joint[joints],dParamHiStop,0);
					dJointSetHinge2Param (joint[joints],dParamVel2,10.0);
					dJointSetHinge2Param (joint[joints++],dParamFMax2,FMAX);
				}
				if (lastb!=null)
				{
					DFixedJoint j = dJointCreateFixed(world,null);
					dJointAttach (j, b, lastb);
					dJointSetFixed(j);
				}
				lastb = b;
			}
		}//#endif
		if (BOX) {//#ifdef BOX
			body[bodies] = dBodyCreate (world);
			dBodySetPosition (body[bodies],0,0,HEIGHT/2);
			dMassSetBox (m,1,LENGTH,WIDTH,HEIGHT);
			dMassAdjust (m, 1);
			dBodySetMass (body[bodies],m);
			box[boxes] = dCreateBox (space,LENGTH,WIDTH,HEIGHT);
			dGeomSetBody (box[boxes++],body[bodies++]);	
		}//#endif
		if (CANNON) {//#ifdef CANNON
			cannon_ball_body = dBodyCreate (world);
			cannon_ball_geom = dCreateSphere (space,CANNON_BALL_RADIUS);
			dMassSetSphereTotal (m,CANNON_BALL_MASS,CANNON_BALL_RADIUS);
			dBodySetMass (cannon_ball_body,m);
			dGeomSetBody (cannon_ball_geom,cannon_ball_body);
			dBodySetPosition (cannon_ball_body,CANNON_X,CANNON_Y,CANNON_BALL_RADIUS);
		}//#endif
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
			turn += 0.1;
			if (turn > 0.3)
				turn = 0.3f;
			break;
		case '.':
			turn -= 0.1;
			if (turn < -0.3)
				turn = -0.3f;
			break;
		case ' ':
			speed = 0;
			turn = 0;
			break;
		case 'f': case 'F':
			doFast = !doFast;
			break;
		case 'r': case 'R':
			resetSimulation();
			break;
		case '[':
			cannon_angle += 0.1;
			break;
		case ']':
			cannon_angle -= 0.1;
			break;
		case '1':
			cannon_elevation += 0.1;
			break;
		case '2':
			cannon_elevation -= 0.1;
			break;
		case 'x': case 'X': {
			DMatrix3 R2 = new DMatrix3(), R3 = new DMatrix3(), R4 = new DMatrix3();
			dRFromAxisAndAngle (R2,0,0,1,cannon_angle);
			dRFromAxisAndAngle (R3,0,1,0,cannon_elevation);
			dMultiply0 (R4,R2,R3);
			double[] cpos = {CANNON_X,CANNON_Y,1};
			for (int i=0; i<3; i++) cpos[i] += 3*R4.get(i, 2);//[i*4+2];
			dBodySetPosition (cannon_ball_body,cpos[0],cpos[1],cpos[2]);
			double force = 10;
			dBodySetLinearVel (cannon_ball_body,force*R4.get(0, 2),force*R4.get(1,2),force*R4.get(2,2));
			dBodySetAngularVel (cannon_ball_body,0,0,0);
			break;
		}
		}
	}


	// simulation loop

	private void simLoop (boolean pause)
	{
		int i, j;

		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

		if (!pause) {
			if (BOX) {
				//dBodyAddForce(body[bodies-1],lspeed,0,0);
				dBodyAddForce(body[bodies-1],speed,0,0);
			}
			for (j = 0; j < joints; j++)
			{
				double curturn = dJointGetHinge2Angle1 (joint[j]);
				//dMessage (0,"curturn %e, turn %e, vel %e", curturn, turn, (turn-curturn)*1.0);
				dJointSetHinge2Param(joint[j],dParamVel,(turn-curturn)*1.0);
				dJointSetHinge2Param(joint[j],dParamFMax,dInfinity);
				dJointSetHinge2Param(joint[j],dParamVel2,speed);
				dJointSetHinge2Param(joint[j],dParamFMax2,FMAX);
				dBodyEnable(dJointGetBody(joint[j],0));
				dBodyEnable(dJointGetBody(joint[j],1));
			}		
			if (doFast)
			{
				dSpaceCollide (space,0,nearCallback);
				dWorldQuickStep (world,0.05);
				dJointGroupEmpty (contactgroup);
			}
			else
			{
				dSpaceCollide (space,0,nearCallback);
				dWorldStep (world,0.05);
				dJointGroupEmpty (contactgroup);
			}

			for (i = 0; i < wb; i++)
			{
				b = dGeomGetBody(wall_boxes[i]);
				if (dBodyIsEnabled(b)) 
				{
					boolean disable = true;
					final DVector3C lvel = dBodyGetLinearVel(b);
					double lspeed = lvel.get0()*lvel.get0()+lvel.get(1)*lvel.get1()+lvel.get2()*lvel.get2();
					if (lspeed > DISABLE_THRESHOLD)
						disable = false;
					final DVector3C avel = dBodyGetAngularVel(b);
					double aspeed = avel.get0()*avel.get0()+avel.get1()*avel.get1()+avel.get2()*avel.get2();
					if (aspeed > DISABLE_THRESHOLD)
						disable = false;

					if (disable)
						wb_stepsdis[i]++;
					else
						wb_stepsdis[i] = 0;

					if (wb_stepsdis[i] > DISABLE_STEPS)
					{
						dBodyDisable(b);
						dsSetColor(0.5f,0.5f,1);
					}
					else
						dsSetColor(1,1,1);

				}
				else
					dsSetColor(0.4f,0.4f,0.4f);
				DVector3 ss = new DVector3();
				dGeomBoxGetLengths (wall_boxes[i], ss);
				dsDrawBox(dGeomGetPosition(wall_boxes[i]), dGeomGetRotation(wall_boxes[i]), ss);
			}
		}
		else
		{
			for (i = 0; i < wb; i++)
			{
				b = dGeomGetBody(wall_boxes[i]);
				if (dBodyIsEnabled(b))
					dsSetColor(1,1,1);
				else
					dsSetColor(0.4f,0.4f,0.4f);
				DVector3 ss = new DVector3();
				dGeomBoxGetLengths (wall_boxes[i], ss);
				dsDrawBox(dGeomGetPosition(wall_boxes[i]), dGeomGetRotation(wall_boxes[i]), ss);
			}
		}

		dsSetColor (0,1,1);
		DVector3 sides = new DVector3(LENGTH,WIDTH,HEIGHT);
		for (i = 0; i < boxes; i++)
			dsDrawBox (dGeomGetPosition(box[i]),dGeomGetRotation(box[i]),sides);
		dsSetColor (1,1,1);
		for (i=0; i< spheres; i++) dsDrawSphere (dGeomGetPosition(sphere[i]),
				dGeomGetRotation(sphere[i]),RADIUS);

		// draw the cannon
		dsSetColor (1,1,0);
		DMatrix3 R2 = new DMatrix3(), R3 = new DMatrix3(), R4 = new DMatrix3();
		dRFromAxisAndAngle (R2,0,0,1,cannon_angle);
		dRFromAxisAndAngle (R3,0,1,0,cannon_elevation);
		dMultiply0 (R4,R2,R3);
		DVector3 cpos = new DVector3(CANNON_X,CANNON_Y,1);
		DVector3 csides = new DVector3(2,2,2);
		dsDrawBox (cpos,R2,csides);
		for (i=0; i<3; i++) cpos.add(i,  1.5*R4.get(i, 2));//[i*4+2]);
		dsDrawCylinder (cpos,R4,3f,0.5f);

		// draw the cannon ball
		dsDrawSphere (dBodyGetPosition(cannon_ball_body),dBodyGetRotation(cannon_ball_body),
				CANNON_BALL_RADIUS);
	}

	public static void main(String[] args) {
		new DemoCrash().demo(args);
	}

	private void demo(String[] args) {
		doFast = true;

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = this;
		//fn.version = DS_VERSION;
		//	fn.start = &start;
		//	fn.step = &simLoop;
		//	fn.command = &command;
		//	fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		dInitODE2(0);

		bodies = 0;
		joints = 0;
		boxes = 0;
		spheres = 0;

		resetSimulation();

		// run simulation
		dsSimulationLoop (args,640,480,this);

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