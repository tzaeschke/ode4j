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
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSimpleSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateAMotor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateLMotor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetAMotorAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetAMotorNumAxes;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetAMotorParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetLMotorAxis;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetLMotorNumAxes;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetLMotorParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax3;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamVel;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamVel2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamVel3;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DLMotorJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;


class DemoMotor extends dsFunctions {

	// some constants
	//#define SIDE (0.5f)	// side length of a box
	//#define MASS (1.0)	// mass of a box
	private static final float SIDE = 0.5f;
	private static final float MASS = 1.0f;


	// dynamics and collision objects
	static DWorld world;
	static DBody[] body = new DBody[2];
	static DBox[] geom = new DBox[2];
	static DLMotorJoint[] lmotor = new DLMotorJoint[2];
	static DAMotorJoint[] amotor = new DAMotorJoint[2];
	static DSpace space;
	static DJointGroup contactgroup;


	// start simulation - set viewpoint

	static float[] xyz = {1.0382f,-1.0811f,1.4700f};
	static float[] hpr = {135.0000f,-19.5000f,0.0000f};
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		//  static float xyz[3] = {1.0382f,-1.0811f,1.4700f};
		//  static float hpr[3] = {135.0000f,-19.5000f,0.0000f};
		dsSetViewpoint (xyz,hpr);
		printf ("Press 'q,a,z' to control one axis of lmotor connectiong two bodies. (q is +,a is 0, z is -)\n");
		printf ("Press 'w,e,r' to control one axis of lmotor connectiong first body with world. (w is +,e is 0, r is -)\n");
	}


	// called when a key pressed
	@Override
	public void command (char cmd)
	{
		if (cmd == 'q' || cmd == 'Q') {
			dJointSetLMotorParam(lmotor[0],dParamVel,0);
			dJointSetLMotorParam(lmotor[0],dParamVel2,0);
			dJointSetLMotorParam(lmotor[0],dParamVel3,0.1);
		} else if (cmd == 'a' || cmd == 'A') {
			dJointSetLMotorParam(lmotor[0],dParamVel,0);
			dJointSetLMotorParam(lmotor[0],dParamVel2,0);
			dJointSetLMotorParam(lmotor[0],dParamVel3,0);
		} else if (cmd == 'z' || cmd == 'Z') {
			dJointSetLMotorParam(lmotor[0],dParamVel,0);
			dJointSetLMotorParam(lmotor[0],dParamVel2,0);
			dJointSetLMotorParam(lmotor[0],dParamVel3,-10.1);
		} else if (cmd == 'w' || cmd == 'W') {
			dJointSetLMotorParam(lmotor[1],dParamVel,10.1);
			dJointSetLMotorParam(lmotor[1],dParamVel2,0);
			dJointSetLMotorParam(lmotor[1],dParamVel3,0);
		} else if (cmd == 'e' || cmd == 'E') {
			dJointSetLMotorParam(lmotor[1],dParamVel,0);
			dJointSetLMotorParam(lmotor[1],dParamVel2,0);
			dJointSetLMotorParam(lmotor[1],dParamVel3,0);
		} else if (cmd == 'r' || cmd == 'R') {
			dJointSetLMotorParam(lmotor[1],dParamVel,-10.1);
			dJointSetLMotorParam(lmotor[1],dParamVel2,0);
			dJointSetLMotorParam(lmotor[1],dParamVel3,0);
		}
	}



	static void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = dGeomGetBody(o1);
		DBody b2 = dGeomGetBody(o2);

		DContactBuffer cb = new DContactBuffer(1);
		DContact contact = cb.get(0);
		contact.surface.mode = 0;
		contact.surface.mu = dInfinity;
		if (0!=dCollide (o1,o2,1,cb.getGeomBuffer()/* sizeof(dContactGeom) */ )) {
			DJoint c = dJointCreateContact (world,contactgroup,contact);
			dJointAttach (c,b1,b2);
		}
	}

	// simulation loop
	private static DNearCallback nearCallback = new DNearCallback() {

		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);

		}

	};
	static void simLoop (boolean pause)
	{
		if (!pause) {
			dSpaceCollide(space,null,nearCallback);
			dWorldQuickStep (world,0.05);
			dJointGroupEmpty(contactgroup);
		}

		DVector3 sides1 = new DVector3();
		dGeomBoxGetLengths(geom[0], sides1);
		DVector3 sides2 = new DVector3();
		dGeomBoxGetLengths(geom[1], sides2);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		dsSetColor (1,1,0);
		dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides1);
		dsSetColor (0,1,1);
		dsDrawBox (dBodyGetPosition(body[1]),dBodyGetRotation(body[1]),sides2);
	}


	public static void main (String[] args)
	{
		new DemoMotor().demo(args);
	}
	
	private void demo(String[] args) {
		// setup pointers to drawstuff callback functions
		//dsFunctions fn = new DemoMotor();
		//fn.version = DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = &command;
		//fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2(0);
		contactgroup = dJointGroupCreate(0);
		world = dWorldCreate();
		space = dSimpleSpaceCreate(null);
		DMass m = dMassCreate();
		dMassSetBox (m,1,SIDE,SIDE,SIDE);
		dMassAdjust (m,MASS);

		body[0] = dBodyCreate (world);
		dBodySetMass (body[0],m);
		dBodySetPosition (body[0],0,0,1);
		geom[0] = dCreateBox(space,SIDE,SIDE,SIDE);
		body[1] = dBodyCreate (world);
		dBodySetMass (body[1],m);
		dBodySetPosition (body[1],0,0,2);
		geom[1] = dCreateBox(space,SIDE,SIDE,SIDE); 

		dGeomSetBody(geom[0],body[0]);
		dGeomSetBody(geom[1],body[1]);

		lmotor[0] = dJointCreateLMotor (world,null);
		dJointAttach (lmotor[0],body[0],body[1]);
		lmotor[1] = dJointCreateLMotor (world,null);
		dJointAttach (lmotor[1],body[0],null);
		amotor[0] = dJointCreateAMotor(world,null);
		dJointAttach(amotor[0], body[0],body[1]);
		amotor[1] = dJointCreateAMotor(world,null);
		dJointAttach(amotor[1], body[0], null);

		for (int i=0; i<2; i++) {
			dJointSetAMotorNumAxes(amotor[i], 3);
			dJointSetAMotorAxis(amotor[i],0,1,1,0,0);
			dJointSetAMotorAxis(amotor[i],1,1,0,1,0);
			dJointSetAMotorAxis(amotor[i],2,1,0,0,1);
			dJointSetAMotorParam(amotor[i],dParamFMax,0.00001);
			dJointSetAMotorParam(amotor[i],dParamFMax2,0.00001);
			dJointSetAMotorParam(amotor[i],dParamFMax3,0.00001);

			dJointSetAMotorParam(amotor[i],dParamVel,0);
			dJointSetAMotorParam(amotor[i],dParamVel2,0);
			dJointSetAMotorParam(amotor[i],dParamVel3,0);

			dJointSetLMotorNumAxes(lmotor[i],3);
			dJointSetLMotorAxis(lmotor[i],0,1,1,0,0);
			dJointSetLMotorAxis(lmotor[i],1,1,0,1,0);
			dJointSetLMotorAxis(lmotor[i],2,1,0,0,1);

			dJointSetLMotorParam(lmotor[i],dParamFMax,0.0001);
			dJointSetLMotorParam(lmotor[i],dParamFMax2,0.0001);
			dJointSetLMotorParam(lmotor[i],dParamFMax3,0.0001);
		}

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dJointGroupDestroy(contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
		dCloseODE();
		//return 0;
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