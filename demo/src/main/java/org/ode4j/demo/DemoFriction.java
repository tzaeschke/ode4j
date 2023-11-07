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
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;


/**
 * test the Coulomb friction approximation.
 * 
 * a 10x10 array of boxes is made, each of which rests on the ground.
 * a horizantal force is applied to each box to try and get it to slide.
 * box[i][j] has a mass (i+1)*MASS and a force (j+1)*FORCE. by the Coloumb
 * friction model, the box should only slide if the force is greater than MU
 * times the contact normal force, i.e.
 * 
 * f > MU * body_mass * GRAVITY
 *  (j+1)*FORCE > MU * (i+1)*MASS * GRAVITY
 *  (j+1) > (i+1) * (MU*MASS*GRAVITY/FORCE)
 *  (j+1) > (i+1) * k
 *  
 * this should be independent of the number of contact points, as N contact
 * points will each have 1/N'th the normal force but the pushing force will
 * have to overcome N contacts. the constants are chosen so that k=1.
 * thus you should see a triangle made of half the bodies in the array start to
 * slide.
 */
class DemoFriction extends dsFunctions {

	// some constants

	private static final float LENGTH = 0.2f;	// box length & width
	private static final float HEIGHT = 0.05f;	// box height
	private static final float MASS = 0.2f;	// mass of box[i][j] = (i+1) * MASS
	private static final float FORCE = 0.05f;	// force applied to box[i][j] = (j+1) * FORCE
	private static final float MU = 0.5f;		// the global mu to use
	private static final float GRAVITY = 0.5f;	// the global gravity to use
	private static final int N1 = 10;		// number of different forces to try
	private static final int N2 = 10;		// number of different masses to try


	// dynamics and collision objects

	private static DWorld world;
	private static DSpace space;
	private static DBody[][] body = new DBody[N1][N2];
	private static DJointGroup contactgroup;
	private static DGeom ground;
	private static DGeom[][] box = new DGeom[N1][N2];

	
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
		int i;

		// only collide things with the ground
		boolean g1 = (o1 == ground);
		boolean g2 = (o2 == ground);
		if (!(g1 ^ g2)) return;

		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();

		DContactBuffer contacts = new DContactBuffer(3);		// up to 3 contacts per box
		for (i=0; i<3; i++) {
			contacts.get(i).surface.mode = 
				OdeConstants.dContactSoftCFM | OdeConstants.dContactApprox1;
			contacts.get(i).surface.mu = MU;
			contacts.get(i).surface.soft_cfm = 0.01;
		}
		int numc = OdeHelper.collide (o1,o2,3,contacts.getGeomBuffer());
		if (numc!= 0) {//[0].geom,sizeof(dContact))) {
			for (i=0; i<numc; i++) {
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contacts.get(i));
				c.attach (b1,b2);
			}
		}
	}


	private static float[] xyz = {1.7772f,-0.7924f,2.7600f};
	private static float[] hpr = {90.0000f,-54.0000f,0.0000f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
	}


	// simulation loop

	private void simLoop (boolean pause)
	{
		int i;
		if (!pause) {
			// apply forces to all bodies
			for (i=0; i<N1; i++) {
				for (int j=0; j<N2; j++) {
					body[i][j].addForce (FORCE*(i+1),0,0);
				}
			}

			OdeHelper.spaceCollide (space,0,nearCallback);
			world.step (0.05);

			// remove all contact joints
			contactgroup.empty();
		}

		dsSetColor (1,0,1);
		DVector3 sides = new DVector3(LENGTH,LENGTH,HEIGHT);
		for (i=0; i<N1; i++) {
			for (int j=0; j<N2; j++) {
				dsDrawBox (box[i][j].getPosition(), box[i][j].getRotation(),
						sides);
			}
		}
	}


	public static void main(String[] args) {
		new DemoFriction().demo(args);
	}
	
	private void demo(String[] args) {
		int i,j;
		DMass m = OdeHelper.createMass();

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace (null);
		contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0,0,-GRAVITY);
		ground = OdeHelper.createPlane (space,0,0,1,0);

		// bodies
		for (i=0; i<N1; i++) {
			for (j=0; j<N2; j++) {
				body[i][j] = OdeHelper.createBody (world);
				m.setBox (1,LENGTH,LENGTH,HEIGHT);
				m.adjust (MASS*(j+1));
				body[i][j].setMass (m);
				body[i][j].setPosition (i*2*LENGTH,j*2*LENGTH,HEIGHT*0.5);

				box[i][j] = OdeHelper.createBox (space,LENGTH,LENGTH,HEIGHT);
				box[i][j].setBody (body[i][j]);
			}
		}

		// run simulation
		dsSimulationLoop (args,640,480,this);

		contactgroup.destroy();
		space.destroy();
		world.destroy();
		OdeHelper.closeODE();
	}


	@Override
	public void command(char cmd) {
		// Nothing
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