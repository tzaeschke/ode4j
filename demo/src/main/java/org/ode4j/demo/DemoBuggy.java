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
import org.ode4j.ode.DPlane;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHinge2Joint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;


/**
 * buggy with suspension.
 * this also shows you how to use geom groups.
 */
class DemoBuggy extends dsFunctions {

	// some constants

	private final double LENGTH = 0.7;	// chassis length
	private final double WIDTH = 0.5;	// chassis width
	private final double HEIGHT = 0.2;	// chassis height
	private final float RADIUS = 0.18f;	// wheel radius
	private final double STARTZ = 0.5;	// starting height of chassis
	private final double CMASS = 1;		// chassis mass
	private final double WMASS = 0.2;	// wheel mass

	private static final DVector3C yunit = new DVector3(0, 1, 0);
	private static final DVector3C zunit = new DVector3(0, 0, 1);


	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;
	private static DBody[] body = new DBody[4];
	private static DHinge2Joint[] joint = new DHinge2Joint[3];	// joint[0] is the front wheel
	private static DJointGroup contactgroup;
	private static DPlane ground;
	private static DSpace car_space;
	private static DBox[] box = new DBox[1];
	private static DSphere[] sphere = new DSphere[3];
	private static DBox ground_box;


	// things that the user controls

	private static double speed=0,steer=0;	// user commands



	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private static void nearCallback (Object data, DGeom o1, DGeom o2)
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
		n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());//[0].geom,sizeof(dContact));
		if (n > 0) {
			for (i=0; i<n; i++) {
				DContact contact = contacts.get(i);
				contact.surface.mode = OdeConstants.dContactSlip1 | OdeConstants.dContactSlip2 |
				OdeConstants.dContactSoftERP | OdeConstants.dContactSoftCFM | OdeConstants.dContactApprox1;
				contact.surface.mu = OdeConstants.dInfinity;
				contact.surface.slip1 = 0.1;
				contact.surface.slip2 = 0.1;
				contact.surface.soft_erp = 0.5;
				contact.surface.soft_cfm = 0.3;
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
				c.attach(
						contact.geom.g1.getBody(),
						contact.geom.g2.getBody());
			}
		}
	}


	private static float[] xyz = {0.8317f,-0.9817f,0.8000f};
	private static float[] hpr = {121.0000f,-27.5000f,0.0000f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
		System.out.println("Press:\t'a' to increase speed.");
		System.out.println("\t'z' to decrease speed.");
		System.out.println("\t',' to steer left.");
		System.out.println("\t'.' to steer right.");
		System.out.println("\t' ' to reset speed and steering.");
		System.out.println("\t'1' to save the current state to 'state.dif'.");
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
//			FILE f = fopen ("state.dif","wt");
//			if (f!=null) {
//				OdeHelper.dWorldExportDIF (world,f,"");
//				fclose (f);
//			}
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
			joint[0].setParamVel2 (-speed);
			joint[0].setParamFMax2 (0.1);

			// steering
			double v = steer - joint[0].getAngle1();
			if (v > 0.1) v = 0.1;
			if (v < -0.1) v = -0.1;
			v *= 10.0;
			joint[0].setParamVel (v);
			joint[0].setParamFMax (0.2);
			joint[0].setParamLoStop (-0.75);
			joint[0].setParamHiStop (0.75);
			joint[0].setParamFudgeFactor (0.1);

			space.collide(null,nearCallback);
			world.step(0.05);

			// remove all contact joints
			contactgroup.empty();
		}

		dsSetColor (0,1,1);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		DVector3 sides = new DVector3(LENGTH,WIDTH,HEIGHT);
		dsDrawBox (body[0].getPosition(), body[0].getRotation(), sides);
		dsSetColor (1,1,1);
		for (i=1; i<=3; i++) dsDrawCylinder (body[i].getPosition(),
				body[i].getRotation(),0.02f,RADIUS);

		DVector3C ss = ground_box.getLengths();
		dsDrawBox (ground_box.getPosition(), ground_box.getRotation(), ss);
	}


	public static void main(String[] args) {
		new DemoBuggy().demo(args);
	}
	
	private void demo(String[] args) {
		int i;
		DMass m = OdeHelper.createMass();

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace(null);
		contactgroup = OdeHelper.createJointGroup();
		world.setGravity (0,0,-0.5);
		ground = OdeHelper.createPlane(space,0,0,1,0);

		// chassis body
		body[0] = OdeHelper.createBody(world);
		body[0].setPosition(0, 0, STARTZ);
		m.setBox(1, LENGTH, WIDTH, HEIGHT);
		m.adjust(CMASS);
		body[0].setMass(m);
		box[0] = OdeHelper.createBox (null,LENGTH,WIDTH,HEIGHT);
		box[0].setBody(body[0]);

		// wheel bodies
		for (i=1; i<=3; i++) {
			body[i] = OdeHelper.createBody(world);
			DQuaternion q = new DQuaternion();
			OdeMath.dQFromAxisAndAngle (q,1,0,0,Math.PI*0.5);
			body[i].setQuaternion(q);
			m.setSphere(1,RADIUS);
			m.adjust(WMASS);
			body[i].setMass(m);
			sphere[i-1] = OdeHelper.createSphere (null,RADIUS);
			sphere[i-1].setBody(body[i]);
		}
		body[1].setPosition(0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
		body[2].setPosition(-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
		body[3].setPosition(-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);

		// front and back wheel hinges
		for (i=0; i<3; i++) {
			joint[i] = OdeHelper.createHinge2Joint (world,null);
			joint[i].attach(body[0],body[i+1]);
			final DVector3C a = body[i+1].getPosition();
			DHinge2Joint h2 = joint[i];
			h2.setAnchor (a);
			h2.setAxes (zunit, yunit);
		}

		// set joint suspension
		for (i=0; i<3; i++) {
			joint[i].setParamSuspensionERP (0.4);
			joint[i].setParamSuspensionCFM (0.8);
		}

		// lock back wheels along the steering axis
		for (i=1; i<3; i++) {
			// set stops to make sure wheels always stay in alignment
			joint[i].setParamLoStop (0);
			joint[i].setParamHiStop (0);
			// the following alternative method is no good as the wheels may get out
			// of alignment:
			//   dJointSetHinge2Param (joint[i],dParamVel,0);
			//   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
		}

		// create car space and add it to the top level space
		car_space = OdeHelper.createSimpleSpace(space);
		car_space.setCleanup(false);
		car_space.add (box[0]);
		car_space.add (sphere[0]);
		car_space.add (sphere[1]);
		car_space.add (sphere[2]);

		// environment
		ground_box = OdeHelper.createBox (space,2,1.5,1);
		DMatrix3 R = new DMatrix3();
		OdeMath.dRFromAxisAndAngle (R,0,1,0,-0.15);
		ground_box.setPosition(2,0,-0.34);
		ground_box.setRotation(R);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		box[0].destroy();
		sphere[0].destroy();
		sphere[1].destroy();
		sphere[2].destroy();
		contactgroup.destroy();
		space.destroy();
		world.destroy();
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