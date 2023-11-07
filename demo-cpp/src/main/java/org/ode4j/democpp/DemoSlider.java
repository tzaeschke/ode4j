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

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddTorque;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetQuaternion;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateSlider;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetSliderAxis;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dQFromAxisAndAngle;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.cpp4j.Cmath.cos;
import static org.ode4j.ode.internal.cpp4j.Cmath.sin;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DWorld;

class DemoSlider extends dsFunctions {

	// some constants
	//#define SIDE (0.5f)	// side length of a box
	//#define MASS (1.0)	// mass of a box
	private static final float SIDE = 0.5f;
	private static final float MASS = 1.0f;

	// dynamics and collision objects
	private static DWorld world;
	private static DBody[] body=new DBody[2];
	private static DSliderJoint slider;


	// state set by keyboard commands
	private static int occasional_error = 0;


	// start simulation - set viewpoint

	private static float[] xyz= {1.0382f,-1.0811f,1.4700f};
	private static float[] hpr= {135.0000f,-19.5000f,0.0000f};
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		//  static float xyz[3] = {1.0382f,-1.0811f,1.4700f};
		//  static float hpr[3] = {135.0000f,-19.5000f,0.0000f};
		dsSetViewpoint (xyz,hpr);
		printf ("Press 'e' to start/stop occasional error.\n");
	}


	// called when a key pressed

	@Override
	public void command (char cmd)
	{
		if (cmd == 'e' || cmd == 'E') {
			occasional_error ^= 1;
		}
	}


	// simulation loop
	private static double a=0;
	private static int count = 0;

	private static void simLoop (boolean pause)
	{
		final double kd = -0.3;	// angular damping constant
		final double ks = 0.5;	// spring constant
		if (!pause) {
			// add an oscillating torque to body 0, and also damp its rotational motion
			//TZ    static double a=0;
			final DVector3C w = dBodyGetAngularVel (body[0]);
			dBodyAddTorque (body[0],kd*w.get0(),kd*w.get1()+0.1*cos(a),kd*w.get2()+0.1*sin(a));
			a += 0.01;

			// add a spring force to keep the bodies together, otherwise they will
			// fly apart along the slider axis.
			final DVector3C p1 = dBodyGetPosition (body[0]);
			final DVector3C p2 = dBodyGetPosition (body[1]);
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
					DMatrix3C R1 = new DMatrix3();
					DMatrix3 R2 = new DMatrix3(), R3 = new DMatrix3();
					R1 = dBodyGetRotation (body[0]);
					dRFromAxisAndAngle (R2,dRandReal()-0.5,dRandReal()-0.5,
							dRandReal()-0.5,dRandReal()-0.5);
					R3.eqMul(R1, R2);//dMultiply0 (R3,R1,R2,3,3,3);
					dBodySetRotation (body[0],R3);

					// randomly adjust position of body[0]
					final DVector3C pos = dBodyGetPosition (body[0]);
					dBodySetPosition (body[0],
							pos.get0()+0.2*(dRandReal()-0.5),
							pos.get1()+0.2*(dRandReal()-0.5),
							pos.get2()+0.2*(dRandReal()-0.5));
				}
				count++;
			}

			dWorldStep (world,0.05);
		}

		DVector3 sides1= new DVector3(SIDE,SIDE,SIDE);
		DVector3 sides2= new DVector3(SIDE*0.8f,SIDE*0.8f,SIDE*2.0f);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		dsSetColor (1,1,0);
		dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides1);
		dsSetColor (0,1,1);
		dsDrawBox (dBodyGetPosition(body[1]),dBodyGetRotation(body[1]),sides2);
	}


	public static void main (String[] args)
	{
		new DemoSlider().demo(args);
	}
	
	private void demo(String[] args) {
		// setup pointers to drawstuff callback functions
		//dsFunctions fn = new DemoSlider();
		//fn.version = DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = &command;
		//  fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2(0);
		world = dWorldCreate();
		DMass m = dMassCreate();
		dMassSetBox (m,1,SIDE,SIDE,SIDE);
		dMassAdjust (m,MASS);

		body[0] = dBodyCreate (world);
		dBodySetMass (body[0],m);
		dBodySetPosition (body[0],0,0,1);
		body[1] = dBodyCreate (world);
		dBodySetMass (body[1],m);
		DQuaternion q = new DQuaternion();
		dQFromAxisAndAngle (q,-1,1,0,0.25*M_PI);
		dBodySetPosition (body[1],0.2,0.2,1.2);
		dBodySetQuaternion (body[1],q);

		slider = dJointCreateSlider (world,null);
		dJointAttach (slider,body[0],body[1]);
		dJointSetSliderAxis (slider,1,1,1);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dWorldDestroy (world);
		dCloseODE();
	}


	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}


	@Override
	public void stop() {
	}
}
