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
import static org.ode4j.drawstuff.DrawStuff.dsDrawCapsule;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetColorAlpha;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactBounce;
import static org.ode4j.ode.OdeConstants.dContactFDir1;
import static org.ode4j.ode.OdeConstants.dContactMotion1;
import static org.ode4j.ode.OdeConstants.dContactMotion2;
import static org.ode4j.ode.OdeConstants.dContactMotionN;
import static org.ode4j.ode.OdeMath.dPlaneSpace;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;


/**
 * This demo shows how to use dContactMotionN in a lifting platform.
 */
public class DemoMotion extends dsFunctions {

	// some constants

	private static final int NUM = 100;			// max number of objects
	private static final double DENSITY = 5.0;		// density of all objects
	private static final int GPB = 3;			// maximum number of geometries per body
	private static final int MAX_CONTACTS = 8;		// maximum number of contact points per body
	//private static final int USE_GEOM_OFFSET = 1;

	// dynamics and collision objects

	private class MyObject {
		DBody body;			// the body
		DGeom[] geom = new DGeom[GPB];		// geometries representing this body
	}

	private static int num=0;		// number of objects in simulation
	private static int nextobj=0;		// next object to recycle if num==NUM
	private static DWorld world;
	private static DSpace space;
	private static MyObject[] obj = new MyObject[NUM];
	private static DJointGroup contactgroup;
	private static boolean show_aabb = false;	// show geom AABBs?
	private static boolean show_contacts = false;	// show contact points?
	private static boolean random_pos = true;	// drop objects from random position?
	private static boolean show_body = false;

	private static DGeom platform, ground;

	private DVector3 platpos = new DVector3(0, 0, 0);
	private int mov_type = 2;
	private double mov_time = 0;


	private final double mov1_speed = 0.2;

	private DVector3 mov2_vel = new DVector3( 0.2, 0.1, 0.25);




	/****************************************************************
	 *  Movement 1: move platform up, reset every 80 units of time. *
	 *      This is the simplest case                               *
	 ****************************************************************/
	private void moveplat_1(double stepsize)
	{
		mov_time += stepsize;
		if (mov_time > 80)
			mov_time = 0;

//		platpos.v[0] = platpos.v[1] = 0;
		// the platform moves up (Z) at constant speed: mov1_speed
//		platpos.v[2] = mov1_speed * mov_time;
		platpos.set(0, 0, mov1_speed * mov_time);
	}

	// Generate contact info for movement 1
	private void contactplat_1(DContact contact)
	{
		contact.surface.mode |= dContactMotionN;
		contact.surface.motionN = mov1_speed;
	}



	/****************************************************************
	 *  Movement 2: move platform along direction mov2_vel, reset   *
	 *  every 80 units of time.                                     *
	 *      This is the most general case: the geom moves along     *
	 *      an arbitrary direction.                                 *
	 ****************************************************************/
	private void moveplat_2(double stepsize)
	{
		mov_time += stepsize;
		if (mov_time > 80)
			mov_time = 0;

		// the platform moves at constant speed: mov2_speed
//		platpos.v[0] = mov2_vel.v[0] * mov_time;
//		platpos.v[1] = mov2_vel.v[1] * mov_time;
//		platpos.v[2] = mov2_vel.v[2] * mov_time;
		platpos.set(mov2_vel).scale(mov_time);
	}

	// Generate contact info for movement 1
	private void contactplat_2(DContact contact)
	{
		/*
      For arbitrary contact directions we need to project the moving
      geom's velocity against the contact normal and fdir1, fdir2
      (obtained with dPlaneSpace()). Assuming moving geom=g2
      (so the contact joint is in the moving geom's reference frame):
      motion1 = dDOT(fdir1, vel);
      motion2 = dDOT(fdir2, vel);
      motionN = dDOT(normal, vel);

      For geom=g1 just negate motionN and motion2. fdir1 is an arbitrary
      vector, so there's no need to negate motion1.

		 */
		contact.surface.mode |= 
			dContactMotionN |                   // velocity along normal
			dContactMotion1 | dContactMotion2 | // and along the contact plane
			dContactFDir1;                      // don't forget to set the direction 1


		// This is a convenience function: given a vector, it finds other 2 perpendicular vectors
		DVector3 motiondir1 = new DVector3(), motiondir2 = new DVector3();
		dPlaneSpace(contact.geom.normal, motiondir1, motiondir2);
//		for (int i=0; i<3; ++i)
//			contact.fdir1.v[i] = motiondir1.v[i];
		contact.fdir1.set(motiondir1);


		double inv = 1;
		if (contact.geom.g1 == platform)
			inv = -1;

		contact.surface.motion1 = mov2_vel.dot(motiondir1);
		contact.surface.motion2 = inv * mov2_vel.dot(motiondir2);
		contact.surface.motionN = inv * mov2_vel.dot(contact.geom.normal);

	}



	private static final DVector3 ss = new DVector3(0.02,0.02,0.02);


	//static void nearCallback (Object[] data, dGeom o1, dGeom o2)
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		DMatrix3 RI = new DMatrix3();
		//    static final double ss[3] = {0.02,0.02,0.02};

		//dContact[] contact = new dContact[MAX_CONTACTS];
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
		//    int numc = dCollide (o1, o2, MAX_CONTACTS, contact[0].geom, sizeof(dContact));
		int numc = OdeHelper.collide (o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());

		if (numc != 0)
			RI.setIdentity();

		boolean isplatform = (o1 == platform) || (o2 == platform);

		for (int i=0; i< numc; i++) {
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce;
			contact.surface.mu = 1;
			contact.surface.bounce = 0.25;
			contact.surface.bounce_vel = 0.01;

			if (isplatform) {
				switch (mov_type) {
				case 1:
					contactplat_1(contact);
					break;
				case 2:
					contactplat_2(contact);
					break;
				}
			}

			//        dxJoint c = dJointCreateContact (world,contactgroup,contact+i);
			DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
			c.attach (o1.getBody(), o2.getBody());
			if (show_contacts) 
				dsDrawBox (contact.geom.pos, RI, ss);
		}
	}


	// start simulation - set viewpoint

	private static float[] xyz = {2.1106f,-1.3007f,2.f};
	private static float[] hpr = {150.f,-13.5000f,0.0000f};

	@Override
	public void start()
	{
		//dAllocateODEDataForThread(dAllocateMaskAll);
		dsSetViewpoint (xyz,hpr);
		System.out.println ("To drop another object, press:");
		System.out.println ("   b for box.");
		System.out.println ("   s for sphere.");
		System.out.println ("   c for capsule.");
		System.out.println ("   y for cylinder.");
		System.out.println ("Press m to change the movement type");
		System.out.println ("Press space to reset the platform");
		System.out.println ("To toggle showing the geom AABBs, press a.");
		System.out.println ("To toggle showing the contact points, press t.");
		System.out.println ("To toggle dropping from random position/orientation, press r.");
		System.out.println ("To save the current state to 'state.dif', press 1.");
	}


	// called when a key pressed

	//void command (int cmd)
	@Override
	public void command (char cmd2)
	{
		char cmd = cmd2;
		int i;
		int k;
		double[] sides = new double[3];
		DMass m = OdeHelper.createMass();
		boolean setBody;

		cmd = Character.toLowerCase(cmd);
		if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'y')
		{
			setBody = false;
			if (num < NUM) {
				i = num;
				num++;
			}
			else {
				i = nextobj;
				nextobj++;
				if (nextobj >= num) nextobj = 0;

				// destroy the body and geoms for slot i
				if (obj[i].body!=null) {
				    obj[i].body.destroy();
				}
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!= null) {
					    obj[i].geom[k].destroy();
					}
				}
				//memset (obj[i],0);//,sizeof(obj[i]));
				obj[i] = null;
			}

			//TZ
			if (obj[i]!=null) throw new IllegalStateException("" + i);
			obj[i] = new MyObject();
			
			obj[i].body = OdeHelper.createBody (world);
			for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if (random_pos) 
			{
				obj[i].body.setPosition (
						dRandReal()*2-1 + platpos.get0(),
						dRandReal()*2-1 + platpos.get1(),
						dRandReal()+2 + platpos.get2());
				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			}
			else 
			{
				obj[i].body.setPosition ( 
						platpos.get0(),
						platpos.get1(),
						platpos.get2()+2);
				R.setIdentity();
			}
			obj[i].body.setRotation (R);
			//            obj[i].body.dBodySetData (obj[i].body,(void*) i);
			obj[i].body.setData (i);

			if (cmd == 'b') {
				m.setBox (DENSITY,sides[0],sides[1],sides[2]);
				obj[i].geom[0] = OdeHelper.createBox (space,sides[0],sides[1],sides[2]);
			}
			else if (cmd == 'c') {
				sides[0] *= 0.5;
				m.setCapsule (DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCapsule (space,sides[0],sides[1]);
			}
			else if (cmd == 'y') {
				m.setCylinder (DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = OdeHelper.createCylinder (space,sides[0],sides[1]);
			}
			else if (cmd == 's') {
				sides[0] *= 0.5;
				m.setSphere (DENSITY,sides[0]);
				obj[i].geom[0] = OdeHelper.createSphere (space,sides[0]);
			}

			if (!setBody)
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k] != null) {
					    obj[i].geom[k].setBody (obj[i].body);
					}
				}

			obj[i].body.setMass (m);
		}
		else if (cmd == 'a') {
			show_aabb ^= true;
		}
		else if (cmd == 't') {
			show_contacts ^= true;
		}
		else if (cmd == 'r') {
			//        random_pos ^= 1;
			random_pos ^= true;
		}
		else if (cmd == ' ') {
			mov_time = 0;
		}
		else if (cmd == 'm') {
			mov_type = mov_type==1 ? 2 : 1;
			mov_time = 0;
		}
	}


	// draw a geom

	//void drawGeom (dxGeom g, final double[] pos, final double[] R, boolean show_aabb)
	private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb)
	{
		if (g == null) return;
		if (pos == null) pos = g.getPosition ();
		if (R == null) R = g.getRotation ();

		if (g instanceof DBox) {
			DVector3C sides = ((DBox)g).getLengths();
			dsDrawBox (pos,R,sides);
		}
		else if (g instanceof DSphere) {
			dsDrawSphere (pos,R, ((DSphere)g).getRadius() );
		}
		else if (g instanceof DCapsule) {
			DCapsule cap = (DCapsule) g; 
			dsDrawCapsule (pos,R,cap.getLength(),cap.getRadius());
		}
		else if (g instanceof DCylinder) {
			DCylinder cyl = (DCylinder) g;
			dsDrawCylinder (pos,R,cyl.getLength(),cyl.getRadius());
		}
		if (show_body) {
			DBody body = g.getBody ();
			if (body != null) {
				DVector3C bodypos = body.getPosition (); 
				DMatrix3C bodyr = body.getRotation (); 
				DVector3 bodySides = new DVector3( 0.1, 0.1, 0.1 );
				dsSetColorAlpha(0,1,0,1);
				dsDrawBox(bodypos,bodyr,bodySides); 
			}
		}
		if (show_aabb) {
			// draw the bounding box for this geom
			DAABBC aabb = g.getAABB();
			DVector3 bbpos = aabb.getCenter();
			DVector3 bbsides = aabb.getLengths();
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			dsSetColorAlpha (1f,0f,0f,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}
	}


	// simulation loop

	private void updatecam()
	{
		xyz[0] = (float) (platpos.get0() + 3.3f);
		xyz[1] = (float) (platpos.get1() - 1.8f);
		xyz[2] = (float) (platpos.get2() + 2f);
		dsSetViewpoint (xyz, hpr);
	}

	private void simLoop (boolean pause)
	{
		final double stepsize = 0.02;

		dsSetColor (0,0,2);
		OdeHelper.spaceCollide (space,null,new DNearCallback() {
			@Override
			public void call(Object data, DGeom o1, DGeom o2) {
				nearCallback(data, o1, o2);
			}} );
		if (!pause) {

			if (mov_type == 1)
				moveplat_1(stepsize);
			else
				moveplat_2(stepsize);

			//dGeomSetPosition(platform, platpos.v[0], platpos.v[1], platpos.v[2]);
			platform.setPosition(platpos);
			updatecam();
			world.quickStep (stepsize);
			//dWorldStep (world,stepsize);
		}

		// remove all contact joints
		contactgroup.empty ();

		dsSetColor (1,1,0);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<num; i++) {
			for (int j=0; j < GPB; j++) {
				if ( !obj[i].body.isEnabled() ) {
					dsSetColor (1f,0.8f,0f);
				}
				else {
					dsSetColor (1,1,0);
				}
				drawGeom (obj[i].geom[j],null,null,show_aabb);
			}
		}
		dsSetColor (1,0,0);
		drawGeom (platform,null,null,show_aabb);
		//usleep(5000);
	}


	public static void main (String[] args) {
		new DemoMotion().demo(args);
	}
	
	
	private void demo(String[] args) {

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();
		//#if 1
		space = OdeHelper.createHashSpace();
		//#elif 0
		//DVector3 center = new DVector3(0,0,0), extents = new DVector3( 100, 100, 100);
		//space = OdeHelper.createQuadTreeSpace(null, center, extents, 5);
		//#elif 0
		//space = OdeHelper.createSapSpace(DSapSpace.AXES.XYZ);
		//#else
		//space = OdeHelper.createSimpleSpace();
		//#endif
		
		contactgroup = OdeHelper.createJointGroup();
		world.setGravity (0,0,-0.5);
		world.setCFM (1e-5);

		world.setLinearDamping(0.00001);
		world.setAngularDamping(0.005);
		world.setMaxAngularSpeed(200);

		world.setContactSurfaceLayer (0.001);
		ground = OdeHelper.createPlane (space,0,0,1,0);
		//TZ not required memset (obj,0,sizeof(obj));

		// create lift platform
		platform = OdeHelper.createBox(space, 4, 4, 1);

		ground.setCategoryBits(1l);
		platform.setCategoryBits(2l);
		ground.setCollideBits(~2l);
		platform.setCollideBits(~1l);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		contactgroup.destroy ();
		space.destroy ();
		world.destroy ();
		OdeHelper.closeODE();
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
