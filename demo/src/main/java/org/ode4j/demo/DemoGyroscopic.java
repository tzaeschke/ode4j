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
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DPlane;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;


/**
 *
 * Gyroscopic demo.
 */
public class DemoGyroscopic extends dsFunctions {

	//private boolean write_world = false;
	private boolean show_contacts = false;
	private DWorld world;
	private DBody top1, top2;
	private DSpace space;
	private DJointGroup contactgroup;

	private final double pinradius = 0.05f;
	private final double pinlength = 1.5f;
	private final double topradius = 1.0f;
	private final double toplength = 0.25f;
	//private final double topmass = 1.0f;

	private static final int MAX_CONTACTS = 4;

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};

	private void nearCallback (Object data, DGeom o1, DGeom o2) {
		// for drawing the contact points
		DMatrix3 RI = new DMatrix3();
		RI.setIdentity ();
		DVector3C ss = new DVector3(0.02,0.02,0.02);

		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();

		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS); 
		int numc = OdeHelper.collide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer());

		for (int i=0; i<numc; i++) {
			contacts.get(i).surface.mode = OdeConstants.dContactApprox1;
			contacts.get(i).surface.mu = 2;

			DJoint c = OdeHelper.createContactJoint (world,contactgroup,contacts.get(i));
			c.attach (b1,b2);
			if (show_contacts)
				dsDrawBox (contacts.get(i).geom.pos, RI, ss);
		}
	}


	private static float[] xyz = {4.777f, -2.084f, 2.18f};
	private static float[] hpr = {153.0f, -14.5f, 0.0f};
	// start simulation - set viewpoint

	@Override
	public void start() {
		dsSetViewpoint (xyz,hpr);
		System.out.println ("Orange top approximates conservation of angular momentum");
		System.out.println ("Green top uses conservation of angular velocity");
		System.out.println ("---");
		System.out.println ("SPACE to reset");
		System.out.println ("A to tilt the tops.");
		System.out.println ("T to toggle showing the contact points.");
		System.out.println ("1 to save the current state to 'state.dif'.");
	}


	// called when a key pressed

	@Override
	public void command (char cmd) {
		cmd = Character.toLowerCase (cmd);
		if (cmd == ' ')
		{
			reset();       
		}
		else if (cmd == 'a') {
			tilt();
		}
		else if (cmd == 't') {
			show_contacts = !show_contacts;
		}
//		else if (cmd == '1') {
//			write_world = true;
//		}
	}

	// simulation loop

	private void simLoop (boolean pause) {
		dsSetColor (0,0,2);
		space.collide(0,nearCallback);
		if (!pause) {
			//world.quickStep(0.02);
			world.step(0.02);
		}

		// remove all contact joints
		contactgroup.empty();

		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

		dsSetColor (1,0.5f,0);
		dsDrawCylinder(top1.getPosition(),
				top1.getRotation(),
				toplength, topradius);
		dsDrawCapsule(top1.getPosition(),
				top1.getRotation(),
				pinlength, pinradius);

		dsSetColor (0.5f,1,0);
		dsDrawCylinder(top2.getPosition(),
				top2.getRotation(),
				toplength, topradius);
		dsDrawCapsule(top2.getPosition(),
				top2.getRotation(),
				pinlength, pinradius);

	}


	private void reset() {
		DMatrix3 R = new DMatrix3();
		R.setIdentity();

		top1.setRotation(R);
		top2.setRotation(R);

		top1.setPosition(0.8f, -2, 2);
		top2.setPosition(0.8f, 2, 2);

		top1.setAngularVel(1,0,7);
		top2.setAngularVel(1,0,7);

		top1.setLinearVel(0,0.2f,0);
		top2.setLinearVel(0,0.2f,0);
	}

	private void tilt() {
		top1.addTorque(0, 10, 0);
		top2.addTorque(0, 10, 0);
	}

	/**
	 * @param args args
	 */
	public static void main(String[] args) {
		new DemoGyroscopic().demo(args);

	}

	private void demo(String[] args) {
		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		world.setGravity(0,0,-0.5f);
		world.setCFM(1e-5f);
		world.setLinearDamping(0.00001f);
		world.setAngularDamping(0.0001f);

		space = OdeHelper.createSimpleSpace(null);
		contactgroup = OdeHelper.createJointGroup();

		DPlane floor = OdeHelper.createPlane(space, 0,0,1,0);

		top1 = OdeHelper.createBody(world);
		top2 = OdeHelper.createBody(world);

		DMass m = OdeHelper.createMass();
		m.setCylinderTotal(1, 3, topradius, toplength);
		top1.setMass(m);
		top2.setMass(m);

		DGeom g1, g2, pin1, pin2;
		g1 = OdeHelper.createCylinder(space, topradius, toplength);
		g1.setBody(top1);
		g2 = OdeHelper.createCylinder(space, topradius, toplength);
		g2.setBody(top2);

		pin1 = OdeHelper.createCapsule(space, pinradius, pinlength);
		pin1.setBody(top1);
		pin2 = OdeHelper.createCapsule(space, pinradius, pinlength);
		pin2.setBody(top2);

		top2.setGyroscopicMode(false);

		reset();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		g1.DESTRUCTOR();//delete g1;
		g2.DESTRUCTOR();//delete g2;
		pin1.DESTRUCTOR();//delete pin1;
		pin2.DESTRUCTOR();//delete pin2;
		floor.DESTRUCTOR();//delete floor;
		contactgroup.empty();
		top1.DESTRUCTOR();//delete top1;
		top2.DESTRUCTOR();//delete top2;
		space.DESTRUCTOR();//delete space;
		world.DESTRUCTOR();//delete world;
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
