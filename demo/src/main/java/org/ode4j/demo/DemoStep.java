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

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.drawstuff.DrawStuff.*;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DWorld;

/**
 * test the step function by comparing the output of the fast and the slow
 * version, for various systems. currently you have to define COMPARE_METHODS
 * in step.cpp for this to work properly.
 *  
 * report MAX error
 */
class DemoStep extends dsFunctions {

	// some constants

	private static int NUM = 10;	// number of bodies
	private static int NUMJ = 9;			// number of joints
	//private static double SIDE = 0.2;		// side length of a box
	//private static double MASS = 1.0;		// mass of a box
	private static float RADIUS = 0.1732f;	// sphere radius


	// dynamics and collision objects

	static DWorld world=null;
	static DBody[] body=new DBody[NUM];
	static DBallJoint[] joint=new DBallJoint[NUMJ];


	// create the test system

	private static void createTest()
	{
		int i,j;
		if (world != null) world.destroy();

		world = OdeHelper.createWorld();

		// create random bodies
		for (i=0; i<NUM; i++) {
			// create bodies at random position and orientation
			body[i] = OdeHelper.createBody (world);
			body[i].setPosition (dRandReal()*2-1,dRandReal()*2-1,
					dRandReal()*2+RADIUS);
			DQuaternion q = new DQuaternion();
			for (j=0; j<4; j++) q.set(j, dRandReal()*2-1);
			body[i].setQuaternion (q);

			// set random velocity
			body[i].setLinearVel (dRandReal()*2-1,dRandReal()*2-1,
					dRandReal()*2-1);
			body[i].setAngularVel (dRandReal()*2-1,dRandReal()*2-1,
					dRandReal()*2-1);

			// set random mass (random diagonal mass rotated by a random amount)
			DMass m = OdeHelper.createMass();
			DMatrix3 R = new DMatrix3();
			m.setBox (1,dRandReal()+0.1,dRandReal()+0.1,dRandReal()+0.1);
			m.adjust (dRandReal()+1);
			for (j=0; j<4; j++) q.set(j, dRandReal()*2-1);
			OdeMath.dRfromQ(R, q);
			m.rotate (R);
			body[i].setMass (m);
		}

		// create ball-n-socket joints at random positions, linking random bodies
		// (but make sure not to link the same pair of bodies twice)
		char[] linked=new char[NUM*NUM];
		for (i=0; i<NUM*NUM; i++) linked[i] = 0;
		for (i=0; i<NUMJ; i++) {
			int b1,b2;
			do {
				b1 = dRandInt (NUM);
				b2 = dRandInt (NUM);
			} while (linked[b1*NUM + b2]!=0 || b1==b2);
			linked[b1*NUM + b2] = 1;
			linked[b2*NUM + b1] = 1;
			joint[i] = OdeHelper.createBallJoint (world,null);
			joint[i].attach (body[b1],body[b2]);
			joint[i].setAnchor (dRandReal()*2-1,
					dRandReal()*2-1,dRandReal()*2+RADIUS);
		}

		for (i=0; i<NUM; i++) {
			// move bodies a bit to get some joint error
			DVector3C pos = body[i].getPosition(); 
			body[i].setPosition (pos.get(0)+dRandReal()*0.2-0.1,
					pos.get(1)+dRandReal()*0.2-0.1,pos.get(2)+dRandReal()*0.2-0.1);
		}
	}


	private static float[] xyz = {2.6117f,-1.4433f,2.3700f};
	private static float[] hpr = {151.5000f,-30.5000f,0.0000f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
	}


	// simulation loop

	@Override
	public void step (boolean pause)
	{
		if (!pause) {
			// add random forces and torques to all bodies
			int i;
			final double scale1 = 5;
			final double scale2 = 5;
			for (i=0; i<NUM; i++) {
				body[i].addForce (
						scale1*(dRandReal()*2-1),
						scale1*(dRandReal()*2-1),
						scale1*(dRandReal()*2-1));
				body[i].addTorque (
						scale2*(dRandReal()*2-1),
						scale2*(dRandReal()*2-1),
						scale2*(dRandReal()*2-1));
			}

			world.step (0.05);
			createTest();
		}

		// float sides[3] = {SIDE,SIDE,SIDE};
		dsSetColor (1,1,0);
		for (int i=0; i<NUM; i++)
			dsDrawSphere (body[i].getPosition(), body[i].getRotation(), RADIUS);
	}


	public static void main(String[] args) {
		new DemoStep().demo(args);
	}
	
	private void demo(String[] args) {
		OdeHelper.initODE2(0);
		dRandSetSeed (0); // System.currentTimeMillis()/1000);
		createTest();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		world.destroy();
		OdeHelper.closeODE();
	}


	@Override
	public void command(char cmd) {
		//Nothing
	}


	@Override
	public void stop() {
		// Nothing
	}
}
