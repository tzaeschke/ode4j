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
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DSapSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;


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

	private static final DVector3C xunit = new DVector3(1, 0, 0);
	private static final DVector3C yunit = new DVector3(0, 1, 0);
	private static final DVector3C zpunit = new DVector3(0, 0, 1);
	private static final DVector3C zmunit = new DVector3(0, 0, -1);

//	private static boolean BOX = false;
//	private static boolean CARS = true;
//	private static boolean WALL = true;
//	private static boolean BALLS = false;
//	private static boolean BALLSTACK = false;
//	private static boolean ONEBALL = false;
//	private static boolean CENTIPEDE = false;
//	private static boolean CANNON = true;
	private static boolean BOX = false;
	private static boolean CARS = true;
	private static boolean WALL = true;
	private static boolean BALLS = false;
	private static boolean BALLSTACK = false;
	private static boolean ONEBALL = true;
	private static boolean CENTIPEDE = true;
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
	private static DBox[] box=new DBox[10000];
	private static int boxes;
	private static DSphere[] sphere=new DSphere[10000];
	private static int spheres;
	private static DBox[] wall_boxes=new DBox[10000];
	private static DBody[] wall_bodies=new DBody[10000];
	private static DSphere cannon_ball_geom;
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

		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null && OdeHelper.areConnected(b1, b2))
			return;

		final int N = 4;
		DContactBuffer contacts = new DContactBuffer(N);
		n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());//[0].geom,sizeof(dContact));
		if (n > 0) {
			for (i=0; i<n; i++) {
				DContact contact = contacts.get(i);
				contact.surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
				if ( o1 instanceof DSphere || o2 instanceof DSphere )
					contact.surface.mu = 20;
				else
					contact.surface.mu = 0.5;
				contact.surface.slip1 = 0.0;
				contact.surface.slip2 = 0.0;
				contact.surface.soft_erp = 0.8;
				contact.surface.soft_cfm = 0.01;
				DJoint c = OdeHelper.createContactJoint(world,contactgroup,contact);
				c.attach (o1.getBody(), o2.getBody());
			}
		}
	}


	private static final float[] xyz = {3.8548f,9.0843f,7.5900f};
	private static final float[] hpr = {-145.5f,-22.5f,0.25f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
		System.out.println("Press:\t'a' to increase speed.\n" +
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

	private static class IrContainer {
		int bodyIr, jointIr, boxIr, sphereIr;
	}

	//private void makeCar(double x, double y, int &bodyI, int &jointI, int &boxI, int &sphereI)
	private void makeCar(double x, double y, IrContainer ir)
	{
		final int bodyI = ir.bodyIr;
		final int jointI = ir.jointIr;
		final int boxI = ir.boxIr;
		final int sphereI = ir.sphereIr;
		int i;
		DMass m = OdeHelper.createMass();

		// chassis body
		body[bodyI] = OdeHelper.createBody(world);
		body[bodyI].setPosition (x,y,STARTZ);
		m.setBox (1,LENGTH,WIDTH,HEIGHT);
		m.adjust (CMASS/2.0);
		body[bodyI].setMass (m);
		box[boxI] = OdeHelper.createBox (space,LENGTH,WIDTH,HEIGHT);
		box[boxI].setBody (body[bodyI]);

		// wheel bodies
		for (i=1; i<=4; i++) {
			body[bodyI+i] = OdeHelper.createBody(world);
			DQuaternion q = new DQuaternion();
			dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
			body[bodyI+i].setQuaternion (q);
			m.setSphere (1,RADIUS);
			m.adjust (WMASS);
			body[bodyI+i].setMass (m);
			sphere[sphereI+i-1] = OdeHelper.createSphere (space,RADIUS);
			sphere[sphereI+i-1].setBody (body[bodyI+i]);
		}
		body[bodyI+1].setPosition (x+0.4*LENGTH-0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
		body[bodyI+2].setPosition (x+0.4*LENGTH-0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
		body[bodyI+3].setPosition (x-0.4*LENGTH+0.5*RADIUS,y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
		body[bodyI+4].setPosition (x-0.4*LENGTH+0.5*RADIUS,y-WIDTH*0.5,STARTZ-HEIGHT*0.5);

		// front and back wheel hinges
		for (i=0; i<4; i++) {
			joint[jointI+i] = OdeHelper.createHinge2Joint (world,null);
			DHinge2Joint j = joint[jointI+i]; 
			j.attach (body[bodyI],body[bodyI+i+1]);
			DVector3C a = body[bodyI+i+1].getPosition ();
			j.setAnchor (a);
			j.setAxes ((i<2 ? zpunit : zmunit), yunit);
			j.setParamSuspensionERP (0.8);
			j.setParamSuspensionCFM (1e-5);
			j.setParamVel2 (0);
			j.setParamFMax2 (FMAX);
		}

		//center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint)
		DBody b = OdeHelper.createBody (world);
		b.setPosition (x,y,STARTZ+COMOFFSET);
		m.setBox (1,LENGTH,WIDTH,HEIGHT);
		m.adjust (CMASS/2.0);
		b.setMass (m);
		DFixedJoint j = OdeHelper.createFixedJoint(world, null);
		j.attach(body[bodyI], b);
		j.setFixed();
		//box[boxI+1] = dCreateBox(space,LENGTH,WIDTH,HEIGHT);
		//dGeomSetBody (box[boxI+1],b);

		ir.bodyIr += 5;
		ir.jointIr += 4;
		ir.boxIr += 1;
		ir.sphereIr += 4;
	}


	private void shutdownSimulation()
	{
		// destroy world if it exists
		if (bodies!=0)
		{
			//TODO
//		    threading.shutdownProcessing();//dThreadingImplementationShutdownProcessing(threading);
//		    pool.freeThreadPool();
//		    world.setStepThreadingImplementation(null, null);
//		    threading.free();

			contactgroup.destroy ();
			space.destroy ();
			world.destroy ();

	        bodies = 0;
		}
	}
	
	private void setupSimulation()
	{
		int i;
		for (i = 0; i < 1000; i++)
			wb_stepsdis[i] = 0;

		// recreate world

		world = OdeHelper.createWorld();

		//  space = dHashSpaceCreate( null );
		//	space = dSimpleSpaceCreate( null );
		space = OdeHelper.createSapSpace( null, DSapSpace.AXES.XYZ );

		m = OdeHelper.createMass();

		contactgroup = OdeHelper.createJointGroup();
		world.setGravity (0,0,-1.5);
		world.setCFM (1e-5);
		world.setERP (0.8);
		world.setQuickStepNumIterations (ITERS);

		//TODO
//	    DThreadingImplementation threading = OdeHelper.allocateMultiThreaded();
//	    DThreadingThreadPool pool = OdeHelper.allocateThreadPool(4, 0, /*dAllocateFlagBasicData,*/ null);
//	    pool.serveMultiThreadedImplementation(threading);
//	    // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
//	    world.setStepThreadingImplementation(threading.dThreadingImplementationGetFunctions(), threading);

		OdeHelper.createPlane (space,0,0,1,0);

		bodies = 0;
		joints = 0;
		boxes = 0;
		spheres = 0;
		wb = 0;
		IrContainer ir = new IrContainer();
		if (CARS) {//#ifdef CARS
			for (double x = 0.0; x < COLS*(LENGTH+RADIUS); x += LENGTH+RADIUS)
				for (double y = -((ROWS-1)*(WIDTH/2+RADIUS)); y <= ((ROWS-1)*(WIDTH/2+RADIUS)); y += WIDTH+RADIUS*2)
					makeCar(x, y, ir);
			bodies = ir.bodyIr;
			joints = ir.jointIr;
			boxes = ir.boxIr;
			spheres = ir.sphereIr;
		}//#endif
		if (WALL) {//#ifdef WALL
			boolean offset = false;
			for (double z = WBOXSIZE/2.0; z <= WALLHEIGHT; z+=WBOXSIZE)
			{
				offset = !offset;
				for (double y = (-WALLWIDTH+z)/2; y <= (WALLWIDTH-z)/2; y+=WBOXSIZE)
				{
					wall_bodies[wb] = OdeHelper.createBody (world);
					wall_bodies[wb].setPosition (-20,y,z);
					m.setBox (1,WBOXSIZE,WBOXSIZE,WBOXSIZE);
					m.adjust (WALLMASS);
					wall_bodies[wb].setMass (m);
					wall_boxes[wb] = OdeHelper.createBox (space,WBOXSIZE,WBOXSIZE,WBOXSIZE);
					wall_boxes[wb].setBody (wall_bodies[wb]);
					//dBodyDisable(wall_bodies[wb++]);
					wb++;
				}
			}
			System.out.println("wall boxes: " + wb);
		}//#endif
		if (BALLS) {//#ifdef BALLS
			for (double x = -7; x <= -4; x+=1)
				for (double y = -1.5; y <= 1.5; y+=1)
					for (double z = 1; z <= 4; z+=1)
					{
						b = OdeHelper.createBody (world);
						b.setPosition (x*RADIUS*2,y*RADIUS*2,z*RADIUS*2);
						m.setSphere (1,RADIUS);
						m.adjust (BALLMASS);
						b.setMass (m);
						sphere[spheres] = OdeHelper.createSphere (space,RADIUS);
						sphere[spheres++].setBody (b);
					}
		}//#endif
		if (ONEBALL) {//#ifdef ONEBALL
			b = OdeHelper.createBody (world);
			b.setPosition (0,0,2);
			m.setSphere (1,RADIUS);
			m.adjust (1);
			b.setMass (m);
			sphere[spheres] = OdeHelper.createSphere (space,RADIUS);
			sphere[spheres++].setBody (b);
		}//#endif
		if (BALLSTACK) {//#ifdef BALLSTACK
			for (double z = 1; z <= 6; z+=1)
			{
				b = OdeHelper.createBody (world);
				b.setPosition (0,0,z*RADIUS*2);
				m.setSphere (1,RADIUS);
				m.adjust (0.1);
				b.setMass (m);
				sphere[spheres] = OdeHelper.createSphere (space,RADIUS);
				sphere[spheres++].setBody (b);
			}
		}//#endif
		if (CENTIPEDE) {//#ifdef CENTIPEDE
			DBody lastb = null;
			for (double y = 0; y < 10*LENGTH; y+=LENGTH+0.1)
			{
				// chassis body

				b = body[bodies] = OdeHelper.createBody (world);
				body[bodies].setPosition (-15,y,STARTZ);
				m.setBox (1,WIDTH,LENGTH,HEIGHT);
				m.adjust (CMASS);
				body[bodies].setMass (m);
				box[boxes] = OdeHelper.createBox (space,WIDTH,LENGTH,HEIGHT);
				box[boxes++].setBody (body[bodies++]);

				for (double x = -17; x > -20; x-=RADIUS*2)
				{
					body[bodies] = OdeHelper.createBody (world);
					body[bodies].setPosition(x, y, STARTZ);
					m.setSphere(1, RADIUS);
					m.adjust(WMASS);
					body[bodies].setMass(m);
					sphere[spheres] = OdeHelper.createSphere (space, RADIUS);
					sphere[spheres++].setBody (body[bodies]);

					joint[joints] = OdeHelper.createHinge2Joint (world,null);
					if (x == -17)
						joint[joints].attach (b,body[bodies]);
					else
						joint[joints].attach (body[bodies-2],body[bodies]);
					DVector3C a = body[bodies++].getPosition ();
					DHinge2Joint j = joint[joints++];
					j.setAnchor (a);
					j.setAxes (zpunit, xunit);
					j.setParamSuspensionERP (1.0);
					j.setParamSuspensionCFM (1e-5);
					j.setParamLoStop (0);
					j.setParamHiStop (0);
					j.setParamVel2 (-10.0);
					j.setParamFMax2 (FMAX);

					body[bodies] = OdeHelper.createBody (world);
					body[bodies].setPosition(-30 - x, y, STARTZ);
					m.setSphere(1, RADIUS);
					m.adjust(WMASS);
					body[bodies].setMass(m);
					sphere[spheres] = OdeHelper.createSphere (space, RADIUS);
					sphere[spheres++].setBody (body[bodies]);

					joint[joints] = OdeHelper.createHinge2Joint (world,null);
					if (x == -17)
						joint[joints].attach (b,body[bodies]);
					else
						joint[joints].attach (body[bodies-2],body[bodies]);
					DVector3C b = body[bodies++].getPosition ();
					j = joint[joints++];
					j.setAnchor (b);
					j.setAxes (zpunit, xunit);
					j.setParamSuspensionERP (1.0);
					j.setParamSuspensionCFM (1e-5);
					j.setParamLoStop (0);
					j.setParamHiStop (0);
					j.setParamVel2 (10.0);
					j.setParamFMax2 (FMAX);
				}
				if (lastb!=null)
				{
					DFixedJoint j = OdeHelper.createFixedJoint(world,null);
					j.attach (b, lastb);
					j.setFixed();
				}
				lastb = b;
			}
		}//#endif
		if (BOX) {//#ifdef BOX
			body[bodies] = OdeHelper.createBody (world);
			body[bodies].setPosition (0,0,HEIGHT/2);
			m.setBox (1,LENGTH,WIDTH,HEIGHT);
			m.adjust (1);
			body[bodies].setMass (m);
			box[boxes] = OdeHelper.createBox (space,LENGTH,WIDTH,HEIGHT);
			box[boxes++].setBody (body[bodies++]);	
		}//#endif
		if (CANNON) {//#ifdef CANNON
			cannon_ball_body = OdeHelper.createBody (world);
			cannon_ball_geom = OdeHelper.createSphere (space,CANNON_BALL_RADIUS);
			m.setSphereTotal (CANNON_BALL_MASS,CANNON_BALL_RADIUS);
			cannon_ball_body.setMass (m);
			cannon_ball_geom.setBody (cannon_ball_body);
			cannon_ball_body.setPosition (CANNON_X,CANNON_Y,CANNON_BALL_RADIUS);
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
			shutdownSimulation();
			setupSimulation();
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
			cannon_ball_body.setPosition (cpos[0],cpos[1],cpos[2]);
			double force = 10;
			cannon_ball_body.setLinearVel (force*R4.get(0, 2),force*R4.get(1,2),force*R4.get(2,2));
			cannon_ball_body.setAngularVel (0,0,0);
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
				body[bodies-1].addForce(speed,0,0);
			}
			for (j = 0; j < joints; j++)
			{
				DHinge2Joint j2 = joint[j];
				double curturn = j2.getAngle1 ();
				//dMessage (0,"curturn %e, turn %e, vel %e", curturn, turn, (turn-curturn)*1.0);
				j2.setParamVel((turn-curturn)*1.0);
				j2.setParamFMax(dInfinity);
				j2.setParamVel2(speed);
				j2.setParamFMax2(FMAX);
				j2.getBody(0).enable();
				j2.getBody(1).enable();
			}		
			if (doFast)
			{
				space.collide (null,nearCallback);
				world.quickStep (0.05);
				contactgroup.empty ();
			}
			else
			{
				space.collide (null,nearCallback);
				world.step (0.05);
				contactgroup.empty ();
			}

			for (i = 0; i < wb; i++)
			{
				b = wall_boxes[i].getBody();
				if (b.isEnabled()) 
				{
					boolean disable = true;
					DVector3C lvel = b.getLinearVel();
					double lspeed = lvel.lengthSquared();
					if (lspeed > DISABLE_THRESHOLD)
						disable = false;
					DVector3C avel = b.getAngularVel();
					double aspeed = avel.lengthSquared();
					if (aspeed > DISABLE_THRESHOLD)
						disable = false;

					if (disable)
						wb_stepsdis[i]++;
					else
						wb_stepsdis[i] = 0;

					if (wb_stepsdis[i] > DISABLE_STEPS)
					{
						b.disable();
						dsSetColor(0.5f,0.5f,1);
					}
					else
						dsSetColor(1,1,1);

				}
				else
					dsSetColor(0.4f,0.4f,0.4f);
				DVector3 ss = new DVector3();
				wall_boxes[i].getLengths (ss);
				dsDrawBox(wall_boxes[i].getPosition(), wall_boxes[i].getRotation(), ss);
			}
		}
		else
		{
			for (i = 0; i < wb; i++)
			{
				b = wall_boxes[i].getBody();
				if (b.isEnabled())
					dsSetColor(1,1,1);
				else
					dsSetColor(0.4f,0.4f,0.4f);
				DVector3 ss = new DVector3();
				wall_boxes[i].getLengths (ss);
				dsDrawBox(wall_boxes[i].getPosition(), wall_boxes[i].getRotation(), ss);
			}
		}

		dsSetColor (0,1,1);
		DVector3 sides = new DVector3(LENGTH,WIDTH,HEIGHT);
		for (i = 0; i < boxes; i++)
			dsDrawBox (box[i].getPosition(),box[i].getRotation(),sides);
		dsSetColor (1,1,1);
		for (i=0; i< spheres; i++) dsDrawSphere (sphere[i].getPosition(),
				sphere[i].getRotation(),RADIUS);

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
		dsDrawSphere (cannon_ball_body.getPosition(),cannon_ball_body.getRotation(),
				CANNON_BALL_RADIUS);
	}

	public static void main(String[] args) {
		new DemoCrash().demo(args);
	}

	private void demo(String[] args) {
		doFast = true;

		OdeHelper.initODE2(0);

		bodies = 0;
		joints = 0;
		boxes = 0;
		spheres = 0;

		setupSimulation();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		shutdownSimulation();
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