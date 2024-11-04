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

import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAMotorJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DLMotorJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;


/**
 * Motor demo.  Creates two boxes.
 * The first box is connected to the world with an angular motor (amotor).
 * The second box is connected to the first box with a linear motor (lmotor).
 */
public class DemoMotor extends dsFunctions {
	// some constants
	//#define SIDE (0.5f)	// side length of a box
	//#define MASS (1.0)	// mass of a box
	private static final float SIDE = 0.5f;
	private static final float MASS = 1.0f;

	// maximum force that can be applied in a given frame.
	private final double FORCE_MAX_ANGULAR = 0.01;
	private final double FORCE_MAX_LINEAR = 0.1;
	// force applied by the user each frame when keys are pressed.  (not held, not released.)
	private final int ACCELERATION = 1;

	// dynamics and collision objects
	private static DWorld world;
	private static final DBody[] body = new DBody[2];
	private static final DBox[] geom = new DBox[2];
	private static final DLMotorJoint[] lmotor = new DLMotorJoint[2];
	private static final DAMotorJoint[] amotor = new DAMotorJoint[2];
	private static DSpace space;
	private static DJointGroup contactgroup;

	// viewpoint at start
	private static float[] xyz = {1.0382f,-1.0811f,1.4700f};
	private static float[] hpr = {135.0000f,-19.5000f,0.0000f};

	@Override
	public void start() {
		dsSetViewpoint (xyz,hpr);
		System.out.println ("Press 'q,a,z' to control one axis of lmotor connectiong two bodies. (q is +,a is 0, z is -)");
		System.out.println ("Press 'w,e,r' to control one axis of amotor connectiong first body with world. (w is +,e is 0, r is -)");
	}

	// called when a key pressed
	@Override
	public void command (char cmd) {
		// get the forces
		double linear = lmotor[0].getParam(DJoint.PARAM_N.dParamVel3);
		double angular = amotor[1].getParam(DJoint.PARAM_N.dParamVel1);
		// change the forces based on the key
		switch(cmd) {
		case 'q': case 'Q':  linear+= ACCELERATION;  break;
		case 'a': case 'A':  linear=0;  break;
		case 'z': case 'Z':  linear-= ACCELERATION;  break;
		case 'w': case 'W':  angular+= ACCELERATION;  break;
		case 'e': case 'E':  angular=0;  break;
		case 'r': case 'R':  angular-= ACCELERATION;  break;
		default:  break;
		}
		// report the new forces
		System.out.println("linear="+linear+" angular="+angular);
		// apply the new forces
		lmotor[0].setParamVel3(linear);
		amotor[1].setParamVel(angular);
	}

	private static void nearCallback (Object data, DGeom o1, DGeom o2) {
		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();

		DContactBuffer cb = new DContactBuffer(1);
		DContact contact = cb.get(0);
		contact.surface.mode = 0;
		contact.surface.mu = OdeConstants.dInfinity;
		if (0!=OdeHelper.collide (o1,o2,1,cb.getGeomBuffer()/* sizeof(dContactGeom) */ )) {
			DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
			c.attach (b1,b2);
		}
	}

	@Override
	public void step(boolean pause) {
		if (!pause) {
			OdeHelper.spaceCollide(space,null, DemoMotor::nearCallback);
			world.quickStep (0.05);
			contactgroup.empty ();
		}

		// draw the two boxes
		DVector3C sides1 = geom[0].getLengths();
		DVector3C sides2 = geom[1].getLengths();
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		dsSetColor (1,1,0);
		dsDrawBox (body[0].getPosition(), body[0].getRotation(), sides1);
		dsSetColor (0,1,1);
		dsDrawBox (body[1].getPosition(), body[1].getRotation(), sides2);
	}

	/**
	 * @param args args
	 */
	public static void main (String[] args) {
		new DemoMotor().demo(args);
	}
	
	private void demo(String[] args) {
		// create world
		OdeHelper.initODE2(0);
		contactgroup = OdeHelper.createJointGroup();
		world = OdeHelper.createWorld();
		space = OdeHelper.createSimpleSpace (null);

		// create a mass descriptor
		DMass m = OdeHelper.createMass();
		m.setBox (1,SIDE,SIDE,SIDE);
		m.adjust (MASS);

		// box 0
		body[0] = OdeHelper.createBody (world);
		body[0].setMass (m);
		body[0].setPosition (0,0,1);
		geom[0] = OdeHelper.createBox(space,SIDE,SIDE,SIDE);
		geom[0].setBody (body[0]);

		// box 1
		body[1] = OdeHelper.createBody (world);
		body[1].setMass (m);
		body[1].setPosition (0,0,2);
		geom[1] = OdeHelper.createBox(space,SIDE,SIDE,SIDE);
		geom[1].setBody (body[1]);

		// linear motor 0 connects two boxes together
		lmotor[0] = OdeHelper.createLMotorJoint (world,null);
		lmotor[0].attach (body[0],body[1]);

		// linear motor 1 connects first box with world
		lmotor[1] = OdeHelper.createLMotorJoint (world,null);
		lmotor[1].attach (body[0],null);

		// angular motor 0 connects two boxes together
		amotor[0] = OdeHelper.createAMotorJoint (world,null);
		amotor[0].attach (body[0],body[1]);

		// angular motor 1 connects first box with world
		amotor[1] = OdeHelper.createAMotorJoint (world,null);
		amotor[1].attach (body[0], null);

		// set the axies and forces for each motor.
		// axies are in world space, regardless of the body they are attached to.

		for (int i=0; i<2; i++) {
			amotor[i].setNumAxes(3);
			amotor[i].setAxis(0,1,1,0,0);
			amotor[i].setAxis(1,1,0,1,0);
			amotor[i].setAxis(2,1,0,0,1);
			amotor[i].setParamFMax (FORCE_MAX_ANGULAR);
			amotor[i].setParamFMax2(FORCE_MAX_ANGULAR);
			amotor[i].setParamFMax3(FORCE_MAX_ANGULAR);

			amotor[i].setParamVel(0);
			amotor[i].setParamVel2(0);
			amotor[i].setParamVel3(0);

			lmotor[i].setNumAxes(3);
			lmotor[i].setAxis(0,1,1,0,0);
			lmotor[i].setAxis(1,1,0,1,0);
			lmotor[i].setAxis(2,1,0,0,1);

			lmotor[i].setParamFMax (FORCE_MAX_LINEAR);
			lmotor[i].setParamFMax2(FORCE_MAX_LINEAR);
			lmotor[i].setParamFMax3(FORCE_MAX_LINEAR);

			lmotor[i].setParamVel(0);
			lmotor[i].setParamVel2(0);
			lmotor[i].setParamVel3(0);
		}

		// run simulation
		dsSimulationLoop (args,640,480,this);

		contactgroup.destroy();
		space.destroy ();
		world.destroy ();
		OdeHelper.closeODE();
	}

	@Override
	public void stop() {
		//Nothing
	}
}