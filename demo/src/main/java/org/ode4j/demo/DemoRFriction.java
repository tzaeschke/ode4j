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

import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactRolling;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DQuaternion;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;


/**
 * Angular friction demo:
 * 
 * A bunch of ramps of different pitch.
 * A bunch of spheres with rolling friction.
 * 
 */
public class DemoRFriction extends dsFunctions {

	// some constants
	private static final double GRAVITY = 10;	// the global gravity to use
	private static final int RAMP_COUNT = 8;

	private final double rampX = 6.0f;
	private final double rampY = 0.5f;
	private final double rampZ = 0.25f;
	private final double sphereRadius = 0.25f;
	private final double maxRamp = OdeMath.M_PI/4.0f; // Needs to be less than pi/2
	private final double rampInc = maxRamp/RAMP_COUNT;

	// dynamics and collision objects
	private DWorld world = null;
	private DSpace space = null;
	private DJointGroup contactgroup = null;

	private double mu = 0.37; // the global mu to use
	private double rho = 0.1; // the global rho to use
	private double omega = 25.0;

	private DBox[] rampGeom = new DBox[RAMP_COUNT];
	private DBody[] sphereBody = new DBody[RAMP_COUNT];
	private DSphere[] sphereGeom = new DSphere[RAMP_COUNT];

	private DNearCallback nearCallback = new DNearCallback(){
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}};

	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i;

		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();

		if (b1==null && b2==null) {
			return;
		}

		DContactBuffer contacts = new DContactBuffer(3);
		for (DContact contact: contacts) {
			contact.surface.mode = dContactApprox1 | dContactRolling;
			contact.surface.mu = mu;
			contact.surface.rho = rho;
		}
		int numc = OdeHelper.collide (o1,o2,3,contacts.getGeomBuffer());
		if (numc != 0) {
			for (i=0; i<numc; i++) {
				DJoint c = OdeHelper.createContactJoint(world,contactgroup,contacts.get(i));
				c.attach (b1,b2);
			}
		}
	}


	// start simulation - set viewpoint
	private static double[] xyz = {0,-3.0f,3.0f};
	private static double[] hpr = {90.0000,-15.0000,0.0000};

	@Override
	public void start() {
		//dAllocateODEDataForThread(dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		System.out.println("Press:");
		System.out.println("\t'[' or ']' to change initial angular velocity");
		System.out.println("\t'm' to increase sliding friction");
		System.out.println("\t'n' to decrease sliding friction");
		System.out.println("\t'j' to increase rolling friction");
		System.out.println("\t'h' to decrease rolling friction");
		System.out.println("\t'r' to reset simulation.");
	}

	/**
	 * Delete the bodies, etc.
	 */
	private void clear() {
		if (contactgroup != null) { 
			contactgroup.destroy();
		}
		if (space != null) { 
			space.destroy();
		}
		if (world != null) { 
			world.destroy();
		}
	}



	/**
	 * Cleanup if necessary and rebuild the
	 * world.
	 */
	private void reset() {
		clear();

		// create world
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace();
		contactgroup = OdeHelper.createJointGroup();
		world.setGravity (0,0,-GRAVITY);
		OdeHelper.createPlane (space,0,0,1,0);

		// Calculate mass for sphere a capsule with water density.
		DMass sphereMass = OdeHelper.createMass();
		sphereMass.setSphere(1000,sphereRadius);

		for (int ii=0;ii<RAMP_COUNT;++ii) {
			DQuaternion q = new DQuaternion();

			double angle = (ii+1)*rampInc;
			double cosA = Math.cos(angle);
			double sinA = Math.sin(angle);
			double rampW = rampX/cosA; // Box width that preserves ground distance
			double zPos = 0.5*(sinA*rampW-cosA*rampZ); // Position that makes end meet ground
			double yPos = ii*1.25*rampY;
			double xPos = 0;


			// Create the ramp
			rampGeom[ii] = OdeHelper.createBox(space,rampW,rampY,rampZ);
			OdeMath.dQFromAxisAndAngle(q,0,1,0,angle);
			rampGeom[ii].setQuaternion(q);
			rampGeom[ii].setPosition(xPos,yPos,zPos);

			// Create the spheres
			xPos = -(0.5)*rampX + sphereRadius;
			zPos = sinA*rampW + sphereRadius;
			sphereBody[ii] = OdeHelper.createBody(world);
			sphereBody[ii].setMass(sphereMass);
			sphereGeom[ii] = OdeHelper.createSphere(space,sphereRadius);
			sphereGeom[ii].setBody(sphereBody[ii]);
			sphereBody[ii].setPosition(xPos,yPos,zPos);
			sphereBody[ii].setAngularVel(0,omega,0);
		}
	}


	@Override
	public void command (char cmd) {
		switch (cmd) {
		case 'h': case 'H':
			rho-=0.02; 
			if (rho<0) rho=0;
			break;	
		case 'j': case 'J':
			rho+=0.02;
			if (rho>1) rho=1;
			break;
		case 'n': case 'N':
			mu-=0.02;
			if (mu<0) mu=0;
			break;
		case 'm': case 'M':
			mu+=0.02;
			if (mu>1) mu=1;
			break;
		case 'r': case 'R':
			reset();
			break;
		case ']':
			omega+=1;
			break;
		case '[':
			omega-=1;
			break;
		}
	}

	// simulation loop

	@Override
	public void step (boolean pause)	{
		if (!pause) {
			space.collide (null, nearCallback);
			world.step (0.017); // 60 fps
			// remove all contact joints
			contactgroup.empty();
		}

		// Render ramps and spheres
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		
		dsSetColor (1,0.5,0);
		for (DBox g: rampGeom) {
			dsDrawBox (g.getPosition(), g.getRotation(), g.getLengths());
		}

		dsSetColor(0,0,1);
		for (DSphere g: sphereGeom) {
			dsDrawSphere (g.getPosition(), g.getRotation(), sphereRadius);
		}
	}


	/**
	 * @param args args
	 */
	public static void main(String[] args) {
		new DemoRFriction().demo(args);
	}

	private void demo(String[] args) {
		OdeHelper.initODE2(0);
		reset();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		clear();
		OdeHelper.closeODE();
	}


	@Override
	public void stop() {
		// Nothing
	}

}
