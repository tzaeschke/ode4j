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

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

import static org.ode4j.ode.OdeMath.*;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

import static org.ode4j.drawstuff.DrawStuff.*;

class DemoSlider extends dsFunctions {

	// some constants
	private static final float SIDE = 0.5f; // side length of a box
	private static final float MASS = 1.0f; // mass of a box

	// dynamics and collision objects
	private static DWorld world;
	private static DBody[] body = new DBody[2];
	private static DSliderJoint slider;


	// state set by keyboard commands
	private static int occasional_error = 0;


	// start simulation - set viewpoint
	private static float[] xyz= {1.0382f,-1.0811f,1.4700f};
	private static float[] hpr= {135.0000f,-19.5000f,0.0000f};
	
	@Override
	public void start()	{
		dsSetViewpoint (xyz,hpr);
		System.out.println ("Press 'e' to start/stop occasional error.");
	}


	// called when a key pressed

	@Override
	public void command (char cmd) {
		if (cmd == 'e' || cmd == 'E') {
			occasional_error ^= 1;
		}
	}


	// simulation loop
	private static double a=0;
	private static int count = 0;

	@Override
	public void step (boolean pause)	{
		final double kd = -0.3;	// angular damping constant
		final double ks = 0.5;	// spring constant
		if (!pause) {
			// add an oscillating torque to body 0, and also damp its rotational motion
			//TZ    static double a=0;
			DVector3C w = body[0].getAngularVel();
			body[0].addTorque (kd*w.get0(),kd*w.get1()+0.1*Math.cos(a),kd*w.get2()+0.1*Math.sin(a));
			a += 0.01;

			// add a spring force to keep the bodies together, otherwise they will
			// fly apart along the slider axis.
			DVector3C p1 = body[0].getPosition();
			DVector3C p2 = body[1].getPosition();
			DVector3 tmp = new DVector3();
//			dBodyAddForce (body[0],ks*(p2[0]-p1[0]),ks*(p2[1]-p1[1]),
//					ks*(p2[2]-p1[2]));
//			dBodyAddForce (body[1],ks*(p1[0]-p2[0]),ks*(p1[1]-p2[1]),
//					ks*(p1[2]-p2[2]));
			tmp.eqDiff(p2, p1).scale(ks);
			body[0].addForce(tmp);
			tmp.eqDiff(p1, p2).scale(ks);
			body[1].addForce(tmp);

			// occasionally re-orient one of the bodies to create a deliberate error.
			if (occasional_error!=0) {
				//TZ      static int count = 0;
				if ((count % 20)==0) {
					// randomly adjust orientation of body[0]
					//TZ final double *R1;
					DMatrix3 R2 = new DMatrix3(), R3 = new DMatrix3();
					DMatrix3C R1 = body[0].getRotation();
					dRFromAxisAndAngle (R2,dRandReal()-0.5,dRandReal()-0.5,
							dRandReal()-0.5,dRandReal()-0.5);
					R3.eqMul(R1, R2);//dMultiply0 (R3,R1,R2,3,3,3);
					body[0].setRotation (R3);

					// randomly adjust position of body[0]
					DVector3C pos = body[0].getPosition();
					body[0].setPosition (
							pos.get0()+0.2*(dRandReal()-0.5),
							pos.get1()+0.2*(dRandReal()-0.5),
							pos.get2()+0.2*(dRandReal()-0.5));
				}
				count++;
			}

			world.step (0.05);
		}

		DVector3 sides1= new DVector3(SIDE,SIDE,SIDE);
		DVector3 sides2= new DVector3(SIDE*0.8f,SIDE*0.8f,SIDE*2.0f);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		dsSetColor (1,1,0);
		dsDrawBox (body[0].getPosition(), body[0].getRotation(), sides1);
		dsSetColor (0,1,1);
		dsDrawBox (body[1].getPosition(), body[1].getRotation(), sides2);
	}


	public static void main (String[] args) {
		new DemoSlider().demo(args);
	}
	
	private void demo(String[] args) {
		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		DMass m = OdeHelper.createMass();
		m.setBox (1,SIDE,SIDE,SIDE);
		m.adjust (MASS);

		body[0] = OdeHelper.createBody (world);
		body[0].setMass (m);
		body[0].setPosition (0,0,1);
		body[1] = OdeHelper.createBody (world);
		body[1].setMass (m);
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q,-1,1,0,0.25*M_PI);
		body[1].setPosition (0.2,0.2,1.2);
		body[1].setQuaternion (q);

		slider = OdeHelper.createSliderJoint (world,null);
		slider.attach (body[0], body[1]);
		slider.setAxis (1,1,1);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		world.destroy ();
		OdeHelper.closeODE();
	}


	@Override
	public void stop() {
	}
}
