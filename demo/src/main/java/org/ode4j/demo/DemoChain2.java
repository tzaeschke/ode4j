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
import org.ode4j.math.DVector3;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSimpleSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;


class DemoChain2 extends dsFunctions {

	// some constants

	//#define NUM 10			// number of boxes
	//#define SIDE (0.2)		// side length of a box
	//#define MASS (1.0)		// mass of a box
	//#define RADIUS (0.1732f)	// sphere radius
	private static int NUM = 10;			// number of boxes
	private static float SIDE = (0.2f);		// side length of a box
	private static float MASS = (1.0f);		// mass of a box
	//private static float RADIUS = (0.1732f);	// sphere radius

	//using namespace ode;

	// dynamics and collision objects

	private DWorld world;
	private DSimpleSpace space;
	private DBody[] body=new DBody[NUM];
	private DBallJoint[] joint=new DBallJoint[NUM-1];
	private DJointGroup contactgroup;
	private DBox[] box=new DBox[NUM];


	// this is called by space.collide when two objects in space are
	// potentially colliding.

	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null && OdeHelper.areConnected (b1,b2)) return;

		// @@@ it's still more convenient to use the C interface here.

		DContactBuffer contacts = new DContactBuffer(1);
		DContact contact = contacts.get(0);
		contact.surface.mode = 0;
		contact.surface.mu = Double.POSITIVE_INFINITY;
		if (OdeHelper.collide (o1,o2,1,contacts.getGeomBuffer())!=0) {//&contact.geom,sizeof(dContactGeom))) {
			DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
			c.attach(b1, b2);
		}
	}

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};
	
	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};

	// start simulation - set viewpoint

	@Override
	public void start()
	{
		//TODO dAllocateODEDataForThread(dAllocateMaskAll);

		//  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
		//  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
		dsSetViewpoint (xyz,hpr);
	}

	private double angle = 0;

	// simulation loop

	@Override
	public void step (boolean pause)
	{
		if (!pause) {
			//    static double angle = 0;
			angle += 0.05;
			body[NUM-1].addForce (0,0,1.5*(Math.sin(angle)+1.0));

			space.collide (0,nearCallback);
			world.step (0.05);

			// remove all contact joints
			contactgroup.empty();
		}

		DVector3 sides = new DVector3(SIDE,SIDE,SIDE);
		dsSetColor (1,1,0);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<NUM; i++)
			dsDrawBox (body[i].getPosition(),body[i].getRotation(),sides);
	}


	public static void main(String[] args) {
		new DemoChain2().demo(args);
	}
	
	private void demo(String[] args) {
		// create world
		OdeHelper.initODE2(0);

		int i;
		contactgroup = OdeHelper.createJointGroup();
		world = OdeHelper.createWorld();
		world.setGravity (0,0,-0.5);
		world.setCFM (1e-5);
		space = OdeHelper.createSimpleSpace(null);
		OdeHelper.createPlane(space,0,0,1,0);

		for (i=0; i<NUM; i++) {
			body[i] = OdeHelper.createBody(world);//.create (world);
			double k = i*SIDE;
			body[i].setPosition (k,k,k+0.4);
			DMass m = OdeHelper.createMass();
			m.setBox (1,SIDE,SIDE,SIDE);
			m.adjust (MASS);
			body[i].setMass (m);
			body[i].setData (i);

			box[i]= OdeHelper.createBox(space, SIDE,SIDE,SIDE);
			box[i].setBody (body[i]);
		}
		for (i=0; i<(NUM-1); i++) {
			joint[i] = OdeHelper.createBallJoint(world); //.create (world);
			joint[i].attach (body[i],body[i+1]);
			double k = (i+0.5)*SIDE;
			joint[i].setAnchor (k,k,k+0.4);
		}

		// run simulation
		dsSimulationLoop (args,640,480,this);

		OdeHelper.closeODE();
	}

	@Override
	public void command(char cmd) {
		// Nothing
	}

	@Override
	public void stop() {
		// Nothing
	}
}