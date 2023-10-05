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
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeMath;

import static org.ode4j.drawstuff.DrawStuff.*;


/**
 * Test for cylinder vs sphere, by Bram Stolk.
 */
class DemoCylVsSphere extends dsFunctions {


	// dynamics and collision objects (chassis, 3 wheels, environment)

	private static DWorld world;
	private static DSpace space;

	private static DBody cylbody;
	private static DCylinder cylgeom;

	private static DBody sphbody;
	private static DSphere sphgeom;

	private static DJointGroup contactgroup;

	private static boolean show_contacts = true;

	//#define CYLRADIUS    0.6
	//#define CYLLENGTH    2.0
	//#define SPHERERADIUS 0.5
	private static final float CYLRADIUS = 0.6f;
	private static final float CYLLENGTH = 2.0f;
	private static final float SPHERERADIUS = 0.5f;

	private DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};
	
	
	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		assert(o1!=null);
		assert(o2!=null);

		if ( o1 instanceof DSpace || o2 instanceof DSpace )
		{
			System.err.println("testing space " + o1 + " " + o2);
			// colliding a space with something
			OdeHelper.spaceCollide2(o1,o2,data,nearCallback);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		final int N = 32;
		DContactBuffer contacts = new DContactBuffer(N);
		int n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());
		if (n > 0) 
		{
			for (int i=0; i<n; i++) 
			{
				DContact contact = contacts.get(i);
				contact.surface.mode = 0;
				contact.surface.mu = 50.0; // was: dInfinity
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
				c.attach (contact.geom.g1.getBody(), contact.geom.g2.getBody());
				if (show_contacts) 
				{
					DMatrix3 RI = new DMatrix3();
					RI.setIdentity ();
					final DVector3 ss = new DVector3(0.12,0.12,0.12);
					dsSetColorAlpha (0f,0f,1f,0.5f);
					dsDrawBox (contact.geom.pos,RI,ss);
					DVector3 pos  = contact.geom.pos;
					double depth = contact.geom.depth;
					DVector3 norm = contact.geom.normal;
					//double endp[3] = {pos[0]+depth*norm[0], pos[1]+depth*norm[1], pos[2]+depth*norm[2]};
					DVector3 endp = new DVector3();
					endp.eqSum(pos, norm, depth);
					dsSetColorAlpha (1,1,1,1);
					dsDrawLine (contact.geom.pos, endp);
				}
			}
		}
	}


	private static float[] xyz = {-8,-9,3};
	private static float[] hpr = {45.0000f,-27.5000f,0.0000f};
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
		switch (cmd) 
		{
		case ' ':
			break;
		}
	}



	// simulation loop

	private void simLoop (boolean pause)
	{
		OdeHelper.spaceCollide (space,0,nearCallback);
		if (!pause)
		{
			world.quickStep (0.01); // 100 Hz
		}
		contactgroup.empty ();

		dsSetColorAlpha (1f,1f,0f,0.5f);

		dsDrawCylinder
		(
				cylbody.getPosition(),
				cylbody.getRotation(),
				CYLLENGTH,
				CYLRADIUS
		);

		dsDrawSphere
		(
				sphbody.getPosition(),
				sphbody.getRotation(),
				SPHERERADIUS
		);
	}


	public static void main(String[] args) {
		new DemoCylVsSphere().demo(args);
	}
	
	@SuppressWarnings("unused")
	private void demo(String[] args) {
		DMass m = OdeHelper.createMass();

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace (null);
		contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0,0,-9.8);
		world.setQuickStepNumIterations (32);

		OdeHelper.createPlane (space,0,0,1, 0.0);

		cylbody = OdeHelper.createBody (world);
		DQuaternion q = new DQuaternion();
		if (false) {//#if 0
			OdeMath.dQFromAxisAndAngle (q,1,0,0,Math.PI*0.5);
		} else { //#else
			//  dQFromAxisAndAngle (q,1,0,0, M_PI * 1.0);
			OdeMath.dQFromAxisAndAngle (q,1,0,0, Math.PI * -0.77);
		} //#endif
		cylbody.setQuaternion (q);
		m.setCylinder (1.0,3,CYLRADIUS,CYLLENGTH);
		cylbody.setMass (m);
		cylgeom = OdeHelper.createCylinder(null, CYLRADIUS, CYLLENGTH);
		cylgeom.setBody (cylbody);
		cylbody.setPosition (0, 0, 3);
		space.add (cylgeom);

		sphbody = OdeHelper.createBody(world);
		m.setSphere (1,SPHERERADIUS);
		sphbody.setMass (m);
		sphgeom = OdeHelper.createSphere(null, SPHERERADIUS);
		sphgeom.setBody (sphbody);
		sphbody.setPosition (0, 0, 5.5);
		space.add (sphgeom);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		contactgroup.empty();
		contactgroup.destroy();

		sphgeom.destroy();
		cylgeom.destroy();

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

