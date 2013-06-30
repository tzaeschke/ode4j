/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DGeomTransform;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeConstants;
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
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;


class DemoSpaceStress extends dsFunctions {
	// some constants

	private static final int NUM = 10000;			// max number of objects
	private static final float DENSITY = 5.0f;		// density of all objects
	private static final int GPB = 3;			// maximum number of geometries per body
	private static final int MAX_CONTACTS = 4;		// maximum number of contact points per body
	private static final int WORLD_SIZE = 100;


	// dynamics and collision objects

	private class MyObject {
		DBody body;			// the body
		DGeom[] geom=new DGeom[GPB];		// geometries representing this body
	};

	private static int num=0;		// number of objects in simulation
	private static int nextobj=0;		// next object to recycle if num==NUM
	private static DWorld world;
	private static DSpace space;
	private static MyObject[] obj=new MyObject[NUM];
	private static DJointGroup contactgroup;
	private static int selected = -1;	// selected object
	private static boolean show_aabb = false;	// show geom AABBs?
	private static boolean show_contacts = false;	// show contact points?
	private static boolean random_pos = true;	// drop objects from random position?
	private static boolean draw_geom = true;


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
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null && OdeHelper.areConnectedExcluding (b1,b2,DContactJoint.class)) return;

		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
		for (i=0; i<MAX_CONTACTS; i++) {
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;
		}
		int numc = OdeHelper.collide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer());
		if (numc != 0) {
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			final DVector3 ss = new DVector3(0.02,0.02,0.02);
			for (i=0; i<numc; i++) {
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contacts.get(i));
				c.attach (b1,b2);
				if (show_contacts) dsDrawBox (contacts.get(i).geom.pos,RI,ss);
			}
		}
	}


	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};
	// start simulation - set viewpoint
	public void start()
	{
		OdeHelper.allocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		System.out.println ("To drop another object, press:");
		System.out.println ("   o to disable rendering.");
		System.out.println ("   b for box.");
		System.out.println ("   s for sphere.");
		System.out.println ("   c for cylinder.");
		System.out.println ("   x for a composite object.");
		System.out.println ("To select an object, press space.");
		System.out.println ("To disable the selected object, press d.");
		System.out.println ("To enable the selected object, press e.");
		System.out.println ("To toggle showing the geom AABBs, press a.");
		System.out.println ("To toggle showing the contact points, press t.");
		System.out.println ("To toggle dropping from random position/orientation, press r.");
	}


	//private char locase (char c)
	//{
	//  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
	//  else return c;
	//}


	// called when a key pressed

	public void command (char cmd)
	{
		int i,j,k;
		double[] sides = new double[3];
		DMass m = OdeHelper.createMass();

		cmd = Character.toLowerCase(cmd);//locase (cmd);
		if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x'
		/* || cmd == 'l' */) {
			if (num < NUM) {
				i = num;
				num++;
			}
			else {
				i = nextobj;
				nextobj++;
				if (nextobj >= num) nextobj = 0;

				// destroy the body and geoms for slot i
				obj[i].body.destroy();
				for (k=0; k < GPB; k++) {
					if (obj[i].geom[k]!=null) obj[i].geom[k].destroy();
				}
				//memset (&obj[i],0,sizeof(obj[i]));
				obj[i] = new MyObject();
			}

			obj[i].body = OdeHelper.createBody (world);
			for (k=0; k<3; k++) sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if (random_pos) {
				obj[i].body.setPosition (
						dRandReal()*WORLD_SIZE-(WORLD_SIZE/2),dRandReal()*WORLD_SIZE-(WORLD_SIZE/2),dRandReal()+1);
				dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
			}
			else {
				double maxheight = 0;
				for (k=0; k<num; k++) {
					DVector3C pos = obj[k].body.getPosition();
					if (pos.get2() > maxheight) maxheight = pos.get2();
				}
				obj[i].body.setPosition (0,0,maxheight+1);
				dRFromAxisAndAngle (R,0,0,1,dRandReal()*10.0-5.0);
			}
			obj[i].body.setRotation (R);
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
			/*
    // cylinder option not yet implemented
    else if (cmd == 'l') {
      sides[1] *= 0.5;
      dMassSetCapsule (&m,DENSITY,3,sides[0],sides[1]);
      obj[i].geom[0] = dCreateCylinder (space,sides[0],sides[1]);
    }
			 */
			else if (cmd == 's') {
				sides[0] *= 0.5;
				m.setSphere (DENSITY,sides[0]);
				obj[i].geom[0] = OdeHelper.createSphere (space,sides[0]);
			}
			else if (cmd == 'x') {
				DGeom[] g2=new DGeom[GPB];		// encapsulated geometries
				DVector3[] dpos = new DVector3[GPB];	// delta-positions for encapsulated geometries

				// start accumulating masses for the encapsulated geometries
				DMass m2 = OdeHelper.createMass();
				m.setZero ();

				// set random delta positions
				for (j=0; j<GPB; j++) {
					dpos[j] = new DVector3();
					for (k=0; k<3; k++) dpos[j].set(k, dRandReal()*0.3-0.15 );
				}

				for (k=0; k<GPB; k++) {
					DGeomTransform gt = OdeHelper.createGeomTransform (space); 
					obj[i].geom[k] = gt;
					gt.setCleanup (true);
					if (k==0) {
						double radius = dRandReal()*0.25+0.05;
						g2[k] = OdeHelper.createSphere (null,radius);
						m2.setSphere (DENSITY,radius);
					}
					else if (k==1) {
						g2[k] = OdeHelper.createBox (null,sides[0],sides[1],sides[2]);
						m2.setBox (DENSITY,sides[0],sides[1],sides[2]);
					}
					else {
						double radius = dRandReal()*0.1+0.05;
						double length = dRandReal()*1.0+0.1;
						g2[k] = OdeHelper.createCapsule (null,radius,length);
						m2.setCapsule (DENSITY,3,radius,length);
					}
					gt.setGeom (g2[k]);

					// set the transformation (adjust the mass too)
					//dGeomSetPosition (g2[k],dpos[k][0],dpos[k][1],dpos[k][2]);
					g2[k].setPosition(dpos[k]);
					//dMassTranslate (m2,dpos[k][0],dpos[k][1],dpos[k][2]);
					m2.translate(dpos[k]);
					DMatrix3 Rtx = new DMatrix3();
					dRFromAxisAndAngle (Rtx,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
							dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
					g2[k].setRotation (Rtx);
					m2.rotate (Rtx);

					// add to the total mass
					m.add (m2);
				}

				// move all encapsulated objects so that the center of mass is (0,0,0)
				DVector3C negC = m.getC().clone().scale(-1);
				for (k=0; k<2; k++) {
//					dGeomSetPosition (g2[k],
//							dpos[k][0]-m.c[0],
//							dpos[k][1]-m.c[1],
//							dpos[k][2]-m.c[2]);
					g2[k].setPosition(dpos[k].reAdd(negC));
				}
				//dMassTranslate (m,-m.c[0],-m.c[1],-m.c[2]);
				m.translate(negC);
			}

			for (k=0; k < GPB; k++) {
				if (obj[i].geom[k]!=null) obj[i].geom[k].setBody (obj[i].body);
			}

			obj[i].body.setMass (m);
		}

		if (cmd == ' ') {
			selected++;
			if (selected >= num) selected = 0;
			if (selected < 0) selected = 0;
		}
		else if (cmd == 'd' && selected >= 0 && selected < num) {
			obj[selected].body.disable();
		}
		else if (cmd == 'e' && selected >= 0 && selected < num) {
			obj[selected].body.enable();
		}
		else if (cmd == 'a') {
			show_aabb ^= true;
		}
		else if (cmd == 't') {
			show_contacts ^= true;
		}
		else if (cmd == 'r') {
			random_pos ^= true;
		}
		else if (cmd == 'o') {
			draw_geom ^= true;
		}
	}


	// draw a geom

	private void drawGeom (DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb)
	{
		if (!draw_geom){
			return;
		}

		if (g==null) return;
		if (pos==null) pos = g.getPosition();
		if (R==null) R = g.getRotation();

		if (g instanceof DBox) {
			DVector3C sides = ((DBox)g).getLengths();
			dsDrawBox (pos,R,sides);
		}
		else if (g instanceof DSphere) {
			dsDrawSphere (pos,R,((DSphere)g).getRadius());
		}
		else if (g instanceof DCapsule) {
			DCapsule cap = (DCapsule) g; 
			dsDrawCapsule (pos,R,cap.getLength(), cap.getRadius());
		}
		/*
  // cylinder option not yet implemented
  else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
		 */
		else if (g instanceof DGeomTransform) {
			DGeom g2 = ((DGeomTransform)g).getGeom ();
			DVector3C pos2 = g2.getPosition();
			DMatrix3C R2 = g2.getRotation();
			DVector3 actual_pos = new DVector3();
			DMatrix3 actual_R = new DMatrix3();
			dMULTIPLY0_331 (actual_pos,R,pos2);
			//    actual_pos[0] += pos[0];
			//    actual_pos[1] += pos[1];
			//    actual_pos[2] += pos[2];
			actual_pos.add(pos);
			dMULTIPLY0_333 (actual_R,R,R2);
			drawGeom (g2,actual_pos,actual_R,false);
		}

		if (show_aabb) {
			// draw the bounding box for this geom
			DAABBC aabb = g.getAABB();
			DVector3 bbpos = aabb.getCenter();
			DVector3 bbsides = aabb.getLengths();
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			dsSetColorAlpha (1,0,0,0.5f);
			dsDrawBox (bbpos,RI,bbsides);
		}
	}


	// simulation loop

	private void simLoop (boolean pause)
	{
		dsSetColor (0,0,2);
		OdeHelper.spaceCollide (space,0,nearCallback);
		//if (!pause) dWorldStep (world,0.05);
		//if (!pause) dWorldStepFast (world,0.05, 1);

		// remove all contact joints
		contactgroup.empty();

		dsSetColor (1,1,0);
		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		for (int i=0; i<num; i++) {
			for (int j=0; j < GPB; j++) {
				if (i==selected) {
					dsSetColor (0,0.7f,1);
				}
				else if ( !obj[i].body.isEnabled() ) {
					dsSetColor (1,0,0);
				}
				else {
					dsSetColor (1,1,0);
				}
				drawGeom (obj[i].geom[j],null,null,show_aabb);
			}
		}
	}


	public static void main(String[] args) {
		new DemoSpaceStress().demo(args);
	}

	private void demo(String[] args) {

		// create world
		OdeHelper.initODE2(0);
		world = OdeHelper.createWorld();


		DVector3 Center = new DVector3();//{0, 0, 0, 0};
		DVector3 Extents = new DVector3(WORLD_SIZE * 0.55, WORLD_SIZE * 0.55, WORLD_SIZE * 0.55);//, 0};

		//space = dSimpleSpaceCreate(0);
		//space = dHashSpaceCreate (0);
		space = OdeHelper.createQuadTreeSpace (null, Center, Extents, 6);

		contactgroup = OdeHelper.createJointGroup();
		world.setGravity (0,0,-0.5);
		world.setCFM (1e-5);
		OdeHelper.createPlane (space,0,0,1,0);
		//memset (obj,0,sizeof(obj));
		for (int i = 0; i < obj.length; i++) {
			obj[i] = new MyObject();
		}

		for (int i = 0; i < NUM; i++){
			command('s');
		}

		// run simulation
		dsSimulationLoop (args,352,288,this);

		contactgroup.destroy();
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
