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
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;


/**
 *  Test for breaking joints, by Bram Stolk
 */
class DemoFeedback extends dsFunctions {

	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;

	private static final int   STACKCNT=10;	// nr of weights on bridge
	private static final int   SEGMCNT=16;	// nr of segments in bridge
	private static final float[] SEGMDIM = { 0.9f, 4, 0.1f };

	//private static DGeom  groundgeom;
	private static DBody[]  segbodies = new DBody[SEGMCNT];
	private static DGeom[]  seggeoms = new DGeom[SEGMCNT];
	private static DBody[]  stackbodies = new DBody[STACKCNT];
	private static DGeom[]  stackgeoms = new DGeom[STACKCNT];
	private static DHingeJoint[] hinges = new DHingeJoint[SEGMCNT-1];
	private static DSliderJoint[] sliders = new DSliderJoint[2];
	private static DJoint.DJointFeedback[] jfeedbacks = new DJoint.DJointFeedback[SEGMCNT-1];
	private static double[] colours = new double[SEGMCNT];
	private static int[] stress = new int[SEGMCNT-1];

	private static DJointGroup contactgroup;

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}};
	
	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		assert(o1!=null);
		assert(o2!=null);

		if ( o1 instanceof DSpace || o2 instanceof DSpace )
		{
			// colliding a space with something
			OdeHelper.spaceCollide2(o1,o2,data,nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		final int N = 32;
		//dContact contact[N];
		DContactBuffer contacts = new DContactBuffer(N);
		int n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());//,sizeof(dContact));
		if (n > 0) 
		{
			for (int i=0; i<n; i++) 
			{
				DContact contact = contacts.get(i);
				contact.surface.mode = OdeConstants.dContactSoftERP 
					| OdeConstants.dContactSoftCFM | OdeConstants.dContactApprox1;
				contact.surface.mu = 100.0;
				contact.surface.soft_erp = 0.96;
				contact.surface.soft_cfm = 0.02;
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
				c.attach (
						contact.geom.g1.getBody(),
						contact.geom.g2.getBody());
			}
		}
	}


	private static final float[] xyz = { -6, 8, 6};
	private static final float[] hpr = { -65.0f, -27.0f, 0.0f};
	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
	}


	// called when a key pressed

	@Override
	public void command (char cmd)
	{
	}


	private void drawGeom (DGeom g)
	{
		DVector3C pos = g.getPosition();
		DMatrix3C R   = g.getRotation();

		//int type = dGeomGetClass (g);
		if (g instanceof DBox)
		{
			DVector3C sides = ((DBox)g).getLengths ();
			dsDrawBox (pos,R,sides);
		}
		if (g instanceof DCylinder)
		{
			double r = ((DCylinder)g).getRadius();
			double l = ((DCylinder)g).getLength();
			dsDrawCylinder (pos, R, l, r);
		}
	}


	private static void inspectJoints()
	{
		final double forcelimit = 4000.0;
		int i;
		for (i=0; i<SEGMCNT-1; i++)
		{
			if (hinges[i].getBody(0)!=null)
			{
				// This joint has not snapped already... inspect it.
				double l0 = jfeedbacks[i].f1.length();// dLENGTH(jfeedbacks[i].f1);
				double l1 = jfeedbacks[i].f2.length();// dLENGTH(jfeedbacks[i].f2);
				colours[i+0] = 0.95*colours[i+0] + 0.05 * l0/forcelimit;
				colours[i+1] = 0.95*colours[i+1] + 0.05 * l1/forcelimit;
				if (l0 > forcelimit || l1 > forcelimit)
					stress[i]++;
				else
					stress[i]=0;
				if (stress[i]>4)
				{
					// Low-pass filter the noisy feedback data.
					// Only after 4 consecutive timesteps with excessive load, snap.
					System.err.println("SNAP! (that was the sound of joint " + i + " breaking)");
					hinges[i].attach (null, null);
				}
			}
		}
	}


	// simulation loop

	public void simLoop (boolean pause)
	{
		int i;

		float simstep = 0.002f; // 2ms simulation steps
		float dt = (float) dsElapsedTime();
		int nrofsteps = (int) Math.ceil(dt/simstep);
		for (i=0; i<nrofsteps && !pause; i++)
		{
			OdeHelper.spaceCollide (space,null,nearCallback);
			world.quickStep (simstep);
			contactgroup.empty ();
			inspectJoints();
		}

		for (i=0; i<SEGMCNT; i++)
		{
			float r=0,g=0,b=0.2f;
			float v = (float) colours[i];
			if (v>1.0) v=1.0f;
			if (v<0.5) 
			{
				r=2*v;
				g=1.0f;
			}
			else
			{
				r=1.0f;
				g=2*(1.0f-v);
			}
			dsSetColor (r,g,b);
			drawGeom(seggeoms[i]);
		}
		dsSetColor (1,1,1);
		for (i=0; i<STACKCNT; i++)
			drawGeom(stackgeoms[i]);
	}



	public static void main(String[] args) {
		new DemoFeedback().demo(args);
	}
	
	private void demo(String[] args) {
		DMass m = OdeHelper.createMass();

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace (null);
		contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0,0,-9.8);
		world.setQuickStepNumIterations (20);

		int i;
		for (i=0; i<SEGMCNT; i++)
		{
			segbodies[i] = OdeHelper.createBody (world);
			segbodies[i].setPosition(i - SEGMCNT/2.0, 0, 5);
			m.setBox (1, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
			segbodies[i].setMass (m);
			seggeoms[i] = OdeHelper.createBox (null, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
			seggeoms[i].setBody (segbodies[i]);
			space.add (seggeoms[i]);
		}

		for (i=0; i<SEGMCNT-1; i++)
		{
			//TZ
			jfeedbacks[i] = OdeHelper.createJointFeedback();
			hinges[i] = OdeHelper.createHingeJoint (world,null);
			hinges[i].attach (segbodies[i],segbodies[i+1]);
			hinges[i].setAnchor (i + 0.5 - SEGMCNT/2.0, 0, 5);
			hinges[i].setAxis (0,1,0);
			hinges[i].setParamFMax (8000.0);
			// NOTE:
			// Here we tell ODE where to put the feedback on the forces for this hinge
			hinges[i].setFeedback (jfeedbacks[i]);
			stress[i]=0;
		}

		for (i=0; i<STACKCNT; i++)
		{
			stackbodies[i] = OdeHelper.createBody(world);
			m.setBox (2.0, 2, 2, 0.6);
			stackbodies[i].setMass(m);

			stackgeoms[i] = OdeHelper.createBox(null, 2, 2, 0.6);
			stackgeoms[i].setBody(stackbodies[i]);
			stackbodies[i].setPosition(0,0,8+2*i);
			space.add(stackgeoms[i]);
		}

		sliders[0] = OdeHelper.createSliderJoint (world,null);
		sliders[0].attach(segbodies[0], null);
		sliders[0].setAxis (1,0,0);
		sliders[0].setParamFMax (  4000.0);
		sliders[0].setParamLoStop (   0.0);
		sliders[0].setParamHiStop (   0.2);

		sliders[1] = OdeHelper.createSliderJoint (world,null);
		sliders[1].attach(segbodies[SEGMCNT-1], null);
		sliders[1].setAxis (1,0,0);
		sliders[1].setParamFMax (  4000.0);
		sliders[1].setParamLoStop (   0.0);
		sliders[1].setParamHiStop (  -0.2);

		OdeHelper.createPlane(space, 0,0,1,0);

		for (i=0; i<SEGMCNT; i++)
			colours[i]=0.0;

		// run simulation
		dsSimulationLoop (args,1280,720,this);

		contactgroup.empty();
		contactgroup.destroy();

		// First destroy seggeoms, then space, then the world.
		for (i=0; i<SEGMCNT; i++)
			seggeoms[i].destroy();
		for (i=0; i<STACKCNT; i++)
			stackgeoms[i].destroy();

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
