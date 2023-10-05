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
import static org.ode4j.cpp.internal.ApiCppBody.dBodyDestroy;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyIsEnabled;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetData;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCapsule;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCylinder;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateSphere;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomCapsuleGetParams;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomCylinderGetParams;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomDestroy;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetAABB;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetClass;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetCategoryBits;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetCollideBits;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSphereGetRadius;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppExportDIF.dWorldExportDIF;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCapsule;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetCylinder;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetSphere;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetAngularDamping;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetCFM;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetContactSurfaceLayer;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetLinearDamping;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetMaxAngularSpeed;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCapsule;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetColorAlpha;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DGeom.dBoxClass;
import static org.ode4j.ode.DGeom.dCapsuleClass;
import static org.ode4j.ode.DGeom.dCylinderClass;
import static org.ode4j.ode.DGeom.dSphereClass;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactBounce;
import static org.ode4j.ode.OdeConstants.dContactFDir1;
import static org.ode4j.ode.OdeConstants.dContactMotion1;
import static org.ode4j.ode.OdeConstants.dContactMotion2;
import static org.ode4j.ode.OdeConstants.dContactMotionN;
import static org.ode4j.ode.OdeMath.dCalcVectorDot3;
import static org.ode4j.ode.OdeMath.dPlaneSpace;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fclose;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fopen;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABB;
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
import org.ode4j.ode.internal.cpp4j.FILE;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;


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
	private static boolean write_world = false;
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

		contact.surface.motion1 = dCalcVectorDot3(mov2_vel, motiondir1);
		contact.surface.motion2 = inv * dCalcVectorDot3(mov2_vel, motiondir2);
		contact.surface.motionN = inv * dCalcVectorDot3(mov2_vel, contact.geom.normal);

	}



	private static final DVector3 ss = new DVector3(0.02,0.02,0.02);


	//static void nearCallback (Object[] data, dGeom o1, dGeom o2)
	void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		DMatrix3 RI = new DMatrix3();
		//    static final double ss[3] = {0.02,0.02,0.02};

		//dContact[] contact = new dContact[MAX_CONTACTS];
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
		//    int numc = dCollide (o1, o2, MAX_CONTACTS, contact[0].geom, sizeof(dContact));
		int numc = dCollide (o1, o2, MAX_CONTACTS, contacts.getGeomBuffer());

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
			DJoint c = dJointCreateContact (world,contactgroup,contact);
			dJointAttach (c, dGeomGetBody(o1), dGeomGetBody(o2));
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
		printf ("To drop another object, press:\n");
		printf ("   b for box.\n");
		printf ("   s for sphere.\n");
		printf ("   c for capsule.\n");
		printf ("   y for cylinder.\n");
		printf ("Press m to change the movement type\n");
		printf ("Press space to reset the platform\n");
		printf ("To toggle showing the geom AABBs, press a.\n");
		printf ("To toggle showing the contact points, press t.\n");
		printf ("To toggle dropping from random position/orientation, press r.\n");
		printf ("To save the current state to 'state.dif', press 1.\n");
	}


	//char locase (char c)
	//{
	//    if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
	//    else return c;
	//}


	// called when a key pressed

	//void command (int cmd)
	@Override
	public void command (char cmd2)
	{
		char cmd = cmd2;
		int i; //size_t i;
		int k;
		double[] sides = new double[3];
		DMass m = dMassCreate();
		boolean setBody;

		cmd = Character.toLowerCase(cmd);//locase (cmd);
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
				    dBodyDestroy (obj[i].body);
				}
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!= null) {
					    dGeomDestroy (obj[i].geom[k]);
					}
				}
				//memset (obj[i],0);//,sizeof(obj[i]));
				obj[i] = null;
			}

			//TZ
			if (obj[i]!=null) throw new IllegalStateException("" + i);
			obj[i] = new MyObject();
			
			obj[i].body = dBodyCreate (world);
			for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if (random_pos) 
			{
				dBodySetPosition (obj[i].body,
						dRandReal()*2-1 + platpos.get0(),
						dRandReal()*2-1 + platpos.get1(),
						dRandReal()+2 + platpos.get2());
				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			}
			else 
			{
				dBodySetPosition (obj[i].body, 
						platpos.get0(),
						platpos.get1(),
						platpos.get2()+2);
				R.setIdentity();
			}
			dBodySetRotation (obj[i].body,R);
			//            obj[i].body.dBodySetData (obj[i].body,(void*) i);
			dBodySetData (obj[i].body, i);

			if (cmd == 'b') {
				dMassSetBox (m,DENSITY,sides[0],sides[1],sides[2]);
				obj[i].geom[0] = dCreateBox (space,sides[0],sides[1],sides[2]);
			}
			else if (cmd == 'c') {
				sides[0] *= 0.5;
				dMassSetCapsule (m,DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = dCreateCapsule (space,sides[0],sides[1]);
			}
			else if (cmd == 'y') {
				dMassSetCylinder (m,DENSITY,3,sides[0],sides[1]);
				obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
			}
			else if (cmd == 's') {
				sides[0] *= 0.5;
				dMassSetSphere (m,DENSITY,sides[0]);
				obj[i].geom[0] = dCreateSphere (space,sides[0]);
			}

			if (!setBody)
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k] != null) {
					    dGeomSetBody (obj[i].geom[k], obj[i].body);
					}
				}

			dBodySetMass (obj[i].body,m);
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
		else if (cmd == '1') {
			//write_world = 1;
			write_world = true;
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
		if (pos == null) pos = dGeomGetPosition (g);
		if (R == null) R = dGeomGetRotation (g);

		int type = dGeomGetClass (g);
		if (type == dBoxClass) {
			DVector3 sides = new DVector3();
			dGeomBoxGetLengths ((DBox)g,sides);
			dsDrawBox (pos,R,sides);
		}
		else if (type == dSphereClass) {
			dsDrawSphere (pos,R,(float)dGeomSphereGetRadius ((DSphere)g));
		}
		else if (type == dCapsuleClass) {
			RefDouble radius = new RefDouble(0),length = new RefDouble(0);
			dGeomCapsuleGetParams ((DCapsule)g,radius,length);
			dsDrawCapsule (pos,R,length.getF(),radius.getF());
		}
		else if (type == dCylinderClass) {
			RefDouble radius = new RefDouble(0),length = new RefDouble(0);
			dGeomCylinderGetParams ((DCylinder)g,radius,length);
			dsDrawCylinder (pos,R,length.getF(),radius.getF());
		}
		if (show_body) {
			DBody body = dGeomGetBody(g);
			if (body != null) {
				final DVector3C bodypos = dBodyGetPosition (body); 
				final DMatrix3C bodyr = dBodyGetRotation (body); 
				DVector3 bodySides = new DVector3( 0.1, 0.1, 0.1 );
				dsSetColorAlpha(0,1,0,1);
				dsDrawBox(bodypos,bodyr,bodySides); 
			}
		}
		if (show_aabb) {
			// draw the bounding box for this geom
			DAABB aabb = new DAABB();
			dGeomGetAABB (g,aabb);
			DVector3 bbpos = aabb.getCenter();
			//for (i=0; i<3; i++) bbpos.v[i] = 0.5*(aabb.v[i*2] + aabb.v[i*2+1]);
			DVector3 bbsides = aabb.getLengths();
			//for (i=0; i<3; i++) bbsides.v[i] = aabb.v[i*2+1] - aabb.v[i*2];
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
		dSpaceCollide (space,null,new DNearCallback() {
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
			dWorldQuickStep (world, stepsize);
			//dWorldStep (world,stepsize);
		}

		if (write_world) {
			FILE f = fopen ("state.dif","wt");
			if (f != null) {
				dWorldExportDIF (world,f,"X");
				fclose (f);
			}
			write_world = false;
		}

		// remove all contact joints
		dJointGroupEmpty (contactgroup);

		dsSetColor (1,1,0);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<num; i++) {
			for (int j=0; j < GPB; j++) {
				if (! dBodyIsEnabled (obj[i].body)) {
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


	//int main (int argc, char **argv)
	public static void main (String[] args) {
		new DemoMotion().demo(args);
	}
	
	private void demo(String[] args) {
		// setup pointers to drawstuff callback functions
		//dsFunctions fn = new DemoMotion();
		//fn.version = DS_VERSION;
		//    fn.start = start;
		//    fn.step = simLoop;
		//    fn.command = command;
		//    fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2(0);
		world = dWorldCreate();
		
		//#if 1
	    space = dHashSpaceCreate (null);
	    //#elif 0
	    //DVector3 center = new DVector3(0,0,0), extents = new DVector3( 100, 100, 100);
        //space = dQuadTreeSpaceCreate(null, center, extents, 5);
	    //#elif 0
	    //space = dSweepAndPruneSpaceCreate (null, DSapSpace.AXES.XYZ);
	    //#else
	    //space = dSimpleSpaceCreate(null);
	    //#endif
	    
		contactgroup = dJointGroupCreate (0);
		dWorldSetGravity (world, 0,0,-0.5);
		dWorldSetCFM (world, 1e-5);

		dWorldSetLinearDamping(world, 0.00001);
		dWorldSetAngularDamping(world, 0.005);
		dWorldSetMaxAngularSpeed(world, 200);

		dWorldSetContactSurfaceLayer (world,0.001);
		ground = dCreatePlane (space,0,0,1,0);
		//TZ not required memset (obj,0,sizeof(obj));

		// create lift platform
		platform = dCreateBox(space, 4, 4, 1);

		dGeomSetCategoryBits(ground, 1l);
		dGeomSetCategoryBits(platform, 2l);
		dGeomSetCollideBits(ground, ~2l);
		dGeomSetCollideBits(platform, ~1l);

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
		dCloseODE();
		//    return 0;
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
